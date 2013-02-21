#!/usr/bin/python

import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach

from mobina_interface.srv import *
from turtlebot_node.srv import SetTurtlebotMode


def commandTablet(cmd, srv_name = '/tablet/command'):
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
		return 'succeeded'

class Tablet_KillLinphone(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		os.system("adb shell am force-stop org.linphone")
		return 'succeeded'

class Tablet_DisableScreensaver(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('on')
		return 'succeeded'

class Tablet_EnableScreensaver(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('off')
		return 'succeeded'

class Tablet_Play(smach.State):
	def __init__(self, f):
		self.file = f
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('play'+self.file+';')
		return 'succeeded'

class Tablet_Start(smach.State):
	def __init__(self, f):
		self.file = f
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		commandTablet('start'+self.file+';')
		return 'succeeded'



class Turtlebot_SetMode(smach.State):
	def __init__(self, mode):
		self.mode = mode
		smach.State.__init__(self, 
			outcomes=['succeeded','failed'])

	def execute(self, userdata):
	    srv_name = "/turtlebot_node/set_operation_mode"
	    rospy.wait_for_service(srv_name)
	    try:
		add_two_ints = rospy.ServiceProxy(srv_name, SetTurtlebotMode)
		if add_two_ints(self.mode).valid_mode: return 'succeeded'
	    except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	    return 'failed'


class Exit(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
	    exit(0)
	    return 'succeeded'


