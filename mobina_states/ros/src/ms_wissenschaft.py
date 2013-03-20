#!/usr/bin/python
import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach

from ApproachPose import *
from mobina_states import *
from BasicIO import *



class CheckLocked(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['locked','unlocked'])

	def execute(self, userdata):
		#TODO:
		return 'unlocked'

class CheckSlump(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['slump','nothing'])

	def execute(self, userdata):
		#TODO:
		return 'slump'


class Slump(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:

            	self.add('LED_START',Light('red_fast_pulse'),
                                   transitions={'succeeded':'MOVE_TO_PERSON'})

            	self.add('MOVE_TO_PERSON',sss_wrapper('move_base_rel','base', [0.1,0,0]),
                                   transitions={'succeeded':'MOVE_TO_PERSON2','failed':'LED_NOT_REACHED'})

            	self.add('MOVE_TO_PERSON2',sss_wrapper('move_base_rel','base', [0,0,1.57]),
                                   transitions={'succeeded':'LED_REACHED','failed':'LED_NOT_REACHED'})

            	self.add('LED_REACHED',Light('white'),
                                   transitions={'succeeded':'PLAY_MOVIE'})

            	self.add('PLAY_MOVIE',Sleep(1),#Tablet_Start('/sdcard/Video/Mayer.mp4'),
                                   transitions={'succeeded':'WAIT_FOR_MOVIE'})

            	self.add('WAIT_FOR_MOVIE',Sleep(30),
                                   transitions={'succeeded':'LED_FINISHED'})

            	self.add('LED_FINISHED',Light('green_medium_pulse'),
                                   transitions={'succeeded':'MOVE_TO_HOME'})

            	self.add('MOVE_TO_HOME',sss_wrapper('move_base_rel','base', [0,0,-1.57]),
                                   transitions={'succeeded':'MOVE_TO_HOME2','failed':'LED_NOT_REACHED'})

            	self.add('MOVE_TO_HOME2',sss_wrapper('move_base_rel','base', [-0.1,0,0]),
                                   transitions={'succeeded':'succeeded','failed':'LED_NOT_REACHED'})

		#error case
            	self.add('LED_NOT_REACHED',Light('red'),
                                   transitions={'succeeded':'failed'})

class Scenario(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        with self:

            	self.add('CHECK_LOCKED',CheckLocked(),
                                   transitions={'locked':'SLEEP_CHECK_LOCKED',
                                                'unlocked':'SLUMP_DETECTION'})

            	self.add('SLEEP_CHECK_LOCKED',Sleep(0.2),
                                   transitions={'succeeded':'CHECK_LOCKED'})

            	self.add('SLUMP_DETECTION',CheckSlump(),
                                   transitions={'nothing':'SLEEP_CHECK_LOCKED',
                                                'slump':'SLUMP'})

            	self.add('SLUMP',Slump(),
                                   transitions={'succeeded':'SLEEP_CHECK_LOCKED',
                                                'failed':'SLEEP_CHECK_LOCKED'})

            	self.add('SLEEP_SLUMP_DETECTION',Sleep(0.2),
                                   transitions={'succeeded':'SLUMP_DETECTION'})


if __name__=='__main__':
	rospy.init_node('MsWissenschaft')
	sm = Scenario()
	outcome = sm.execute()
	rospy.spin()
