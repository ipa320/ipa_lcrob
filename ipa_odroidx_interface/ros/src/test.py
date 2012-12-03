#!/usr/bin/python
import roslib; 
roslib.load_manifest('ipa_odroidx_interface')
import rospy

from avr_interface import AVRInterface

intf = AVRInterface()
intf.setup()

print "Inputs are ", intf.get_input()
for i in range(0,4):
	print "Analog ch. ",i," is ", intf.get_analog(i)

cnt=0
d=1
while True:
	#print "Inputs are ", intf.get_input()
	intf.set_output(0,cnt)
	cnt+=d
	if cnt>=255: d=-1
	elif cnt==0: d=1
	#intf.set_output(0,0)
intf.set_motor(0,1,127)
intf.set_motor(1,1,0)
