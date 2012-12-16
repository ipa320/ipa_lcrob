#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_teleop')
import rospy

from sensor_msgs.msg import Joy
from evdev import InputDevice, list_devices, categorize, ecodes, events
import time

class evjoy:
    def __init__(self):
	rospy.init_node("evjoy")

        self.dev = None
	self.pub = rospy.Publisher('/joy', Joy)
	
	axes = rospy.get_param("~axes",[])
	self.axes_sign = [1.0] * len(axes)
	self.axes_factor = [True] * len(axes)
	for i in range(len(axes)):
	    if axes[i].startswith('!'):
		self.axes_sign[i] = -1.0
		axes[i] = axes[i][1:]	 
	    if axes[i].startswith('~'):
		self.axes_factor[i] = False
		axes[i] = axes[i][1:]	
	self.axes = dict( [(ecodes.ecodes[axes[i]],i) for i in range(len(axes)) ])
	self.axes_data = [0.0] * len(self.axes)
	buttons = rospy.get_param("~buttons",[])
	self.buttons = dict( [(ecodes.ecodes[buttons[i]],i) for i in range(len(buttons)) ])
	self.button_data = [0] * len(self.buttons)
	self.nullzone = rospy.get_param("~nullzone",0.0)
	self.debug = rospy.get_param("~debug",False)
	
	self.enumerate()

    def enumerate(self):
        self.dev = None
        print "enumerating joystick..."
	while not rospy.is_shutdown() and self.dev==None:
                devices = map(InputDevice, list_devices())
                for d in devices:
                    print d.name
                    if "pad" in d.name.lower():
                        self.dev = d
                        break
                if self.dev!=None: break
                time.sleep(2)

        assert(self.dev != None)
	
    def loop(self):
	self.dev.grab()
	for event in self.dev.read_loop():
	    self.handle_event(event)
	    if rospy.is_shutdown(): exit()
	
    def factor(self,axis, v):
	if not self.axes_factor[axis]:
		return v
	if v < 128: value = (v-128)/128.0
	else: value = (v-128)/127.0
	if value < -1.0: value = -1.0
	elif value > 1.0: value = 1.0
	elif abs(value) < self.nullzone: value = 0.0
	return value

    def handle_event(self, event):
	try:
	    if event.type == ecodes.EV_ABS:
		axis = self.axes[event.code]
		self.axes_data[axis] = self.axes_sign[axis] * self.factor(axis, event.value)
		self.publish()
	    elif event.type == ecodes.EV_KEY:
		self.button_data[self.buttons[event.code]] = 1 if event.value != 0 else 0
		self.publish()
	except KeyError:
	    if self.debug: print categorize(event)

    def publish(self):
	cmd = Joy()
	cmd.header.stamp = rospy.Time.now()
	cmd.axes = self.axes_data
	cmd.buttons = self.button_data
	if self.debug: print cmd
	self.pub.publish(cmd)	

if __name__ == "__main__":
    ev = evjoy()
    ev.loop()
