#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_teleop')
import rospy

from sensor_msgs.msg import Joy
#from geometry_msgs.msg import Twist
#from asyncore import file_dispatcher, loop
from evdev import InputDevice, list_devices, categorize, ecodes, events, resolve_ecodes, AbsInfo

devices = map(InputDevice, list_devices())

dev = None
for d in devices:
    print d.name
    if "pad" in d.name.lower():
	dev = d
	break

assert(dev != None)

caps_z = dev.capabilities(False,True)[ecodes.EV_ABS][ecodes.ABS_Z][1].max
caps_y = dev.capabilities(False,True)[ecodes.EV_ABS][ecodes.ABS_Y][1].max

def factor(v, caps):
    if v < 128: value = (v-128)/128.0
    else: value = (v-128)/127.0
    if value < -1.0: value = -1.0
    elif value > 1.0: value = 1.0
    return value
    

dev.grab()
#class InputDeviceDispatcher(file_dispatcher):
class InputDeviceDispatcher():
    def __init__(self, device):
	rospy.init_node("evjoy_vel")
        self.device = device
        #file_dispatcher.__init__(self, device)
        self.deadman = False
        self.absz = 0.0
        self.absy = 0.0
        self.pub = rospy.Publisher('/joy', Joy)

    def recv(self, ign=None):
	if rospy.is_shutdown(): exit()
        return self.device.read()
    def handle_read(self):
        for event in self.recv():
            self.handle_event(categorize(event))
    def handle_event(self, event):
	if isinstance(event, events.AbsEvent) and self.deadman:
	    if event.event.code == ecodes.ABS_Z:
		self.absz = -factor(event.event.value,caps_z)
		self.publish()
	    elif event.event.code == ecodes.ABS_Y:
		self.absy = -factor(event.event.value,caps_y)
		self.publish()
	elif isinstance(event, events.KeyEvent):
	    if event.keycode == "BTN_Z":
		self.deadman = event.keystate != event.key_up
		self.publish()
	    else: print event	
    def publish(self):
	cmd = Joy()
	cmd.header.stamp = rospy.Time.now()
	cmd.axes = [self.absy, self.absz]
	cmd.buttons = [1 if self.deadman else 0 ]
	self.pub.publish(cmd)	

idd = InputDeviceDispatcher(dev)
#loop()
for event in dev.read_loop():
	idd.handle_event(categorize(event))
