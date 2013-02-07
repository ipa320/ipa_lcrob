#!/usr/bin/python

import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach

from mobina_interface.srv import *
from turtlebot_node.srv import SetTurtlebotMode


def commandTablet(self, cmd, srv_name = '/tablet/command'):
    rospy.wait_for_service(srv_name)
    try:
	add_two_ints = rospy.ServiceProxy(srv_name, StringSrv)
	resp1 = add_two_ints(cmd)
    except rospy.ServiceException, e:
	print "Service call failed: %s"%e


class Tablet_StartLinphone(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('startLinphone')

class Tablet_DisableScreensaver(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('on')

class Tablet_EnableScreensaver(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('off')

class Tablet_Play(smach.State):
	def __init__(self, f):
		self.file = f
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('play'+self.file+';')

class Tablet_Start(smach.State):
	def __init__(self, f):
		self.file = f
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('start'+self.file+';')



class Turtlebot_SetMode(smach.State):
	def __init__(self, mode):
		self.mode = mode
		smach.State.__init__(self, 
			outcomes=['succeeded','failed'])

	def execute(self, userdata):
	    set_operation_mode = "/turtlebot/set_operation_mode"
	    rospy.wait_for_service(srv_name)
	    try:
		add_two_ints = rospy.ServiceProxy(srv_name, SetTurtlebotMode)
		if add_two_ints(self.mode).valid_mode: return 'succeeded'
	    except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	    return 'failed'

