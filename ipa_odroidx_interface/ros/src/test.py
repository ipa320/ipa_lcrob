#!/usr/bin/python
import roslib; 
roslib.load_manifest('ipa_odroidx_interface')
import rospy
import sys

from avr_interface import AVRInterface

intf = AVRInterface()
intf.setup()

print "Inputs are ", intf.get_input()
for i in range(0,4):
	print "Analog ch. ",i," is ", intf.get_analog(i)

cnt=0
d=1
while False:#True:
	#print "Inputs are ", intf.get_input()
	intf.set_output(0,cnt)
	cnt+=d
	if cnt>=255: d=-1
	elif cnt==0: d=1
	#intf.set_output(0,0)
intf.set_motor(0,0,int(sys.argv[1]))
#intf.set_motor(1,1,0)
