#!/usr/bin/python
import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach

from ApproachPose import *
from mobina_states import *
from BasicIO import *
from cob_object_detection_msgs.srv import *
from turtlebot_node.msg import TurtlebotSensorState


class CheckTablet(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded'])

		with self:
		    	self.add('SCREEN_ON',Tablet_DisableScreensaver(),
		                           transitions={'succeeded':'PLAY_MOVIE'})

		    	self.add('PLAY_MOVIE',Tablet_Start('/sdcard/Video/Mayer.mp4'),
		                           transitions={'succeeded':'succeeded'})

class CheckLED(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded'])

		with self:
		    	self.add('LED_R',Light('red'), transitions={'succeeded':'WAIT_LED_R'})
		    	self.add('WAIT_LED_R',Sleep(5),
		                           transitions={'succeeded':'LED_G'})
		    	self.add('LED_G',Light('green'), transitions={'succeeded':'WAIT_LED_G'})
		    	self.add('WAIT_LED_G',Sleep(5),
		                           transitions={'succeeded':'LED_B'})
		    	self.add('LED_B',Light('blue'), transitions={'succeeded':'WAIT_LED_B'})
		    	self.add('WAIT_LED_B',Sleep(5),
		                           transitions={'succeeded':'succeeded'})


class CheckTray(smach.StateMachine):
	def __init__(self):
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded'])

		with self:
		    	self.add('TRAY1',sss_wrapper('move','tray', 'home'),
			                   transitions={'succeeded':'WAIT1','failed':'LED_R'})
                        self.add('WAIT1',Sleep(5),
                                           transitions={'succeeded':'TRAY2'})

		    	self.add('TRAY2',sss_wrapper('move','tray', 'video'),
			                   transitions={'succeeded':'WAIT2','failed':'LED_R'})
                        self.add('WAIT2',Sleep(5),
                                           transitions={'succeeded':'TRAY3'})

		    	self.add('TRAY3',sss_wrapper('move','tray', 'home'),
			                   transitions={'succeeded':'LED_G','failed':'LED_R'})

		    	self.add('LED_R',Light('red'), transitions={'succeeded':'WAIT_LED_R'})
		    	self.add('WAIT_LED_R',Sleep(5),
		                           transitions={'succeeded':'succeeded'})

		    	self.add('LED_G',Light('green'), transitions={'succeeded':'WAIT_LED_G'})
		    	self.add('WAIT_LED_G',Sleep(5),
		                           transitions={'succeeded':'succeeded'})


class Scenario(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded'])
        with self:

            	self.add('CHECK_LED',CheckLED(),
                                   transitions={'succeeded':'CHECK_TRAY'})

            	self.add('CHECK_TRAY',CheckTray(),
                                   transitions={'succeeded':'CHECK_TABLET'})

            	self.add('CHECK_TABLET',CheckTablet(),
                                   transitions={'succeeded':'succeeded'})


if __name__=='__main__':
	rospy.init_node('Check')
	sm = Scenario()
	
	sm.execute()
	rospy.spin()

