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
from brics_actuator.msg import JointVelocities, JointValue

class CheckLocked(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['locked','unlocked'])
		rospy.Subscriber("/turtlebot_node/sensor_state", TurtlebotSensorState, self.callback)
		self.lock = True

	def callback(self, data):
		self.lock = data.charging_sources_available>0

	def execute(self, userdata):
		if self.lock: return 'locked'
		return 'unlocked'

class CheckSlump(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['slump1','slump2','slump_pos','nothing'], output_keys=['position'])

	def execute(self, userdata):
		rospy.wait_for_service('/fall_detection')
		try:
		        fall_detection = rospy.ServiceProxy('/fall_detection', cob_object_detection_msgs.srv.DetectObjects)
			req = cob_object_detection_msgs.srv.DetectObjectsRequest()
		        resp1 = fall_detection(req)
    		except rospy.ServiceException, e:
		        print "Service call failed: %s"%e
			return 'nothing'
		if len(resp1.object_list.detections)>0:
			if resp1.object_list.detections[0].label=="1":
				userdata.position = "person_fake1"
				return 'slump1'
			elif resp1.object_list.detections[0].label=="2":
				userdata.position = "person_fake2"
				return 'slump2'
			userdata.position = "person_fake"
			return "slump_pos"
		
		return 'nothing'

class MoveToUPosition(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded','failed'], input_keys=['position'])

	def execute(self, userdata):
		ah = sss.move('base', userdata.position)
		if ah.get_state() == 3:
			return 'succeeded'
		else:
			return 'failed'

class MoveToPosition(smach.State):
	def __init__(self, pos):
		smach.State.__init__(self, 
			outcomes=['succeeded','failed'])
		self.pos = pos

	def execute(self, userdata):
		ah = sss.move('base', self.pos)
		if ah.get_state() == 3:
			return 'succeeded'
		else:
			return 'failed'

class StillTrayController(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded'])

	def execute(self, userdata):
		pub = rospy.Publisher('/tray_controller/command_vel', JointVelocities)
		rospy.sleep(0.2)
		vel = JointVelocities()
		vel.velocities = [JointValue()]
		vel.velocities[0].value = 1
		pub.publish(vel)
		vel.velocities[0].value = 0
		pub.publish(vel)
		return 'succeeded'


class Slump(smach.StateMachine):
    def __init__(self, mode):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'], input_keys=['position'])

        with self:
            	self.add('LED_START',Light('red_fast_pulse'),
                                   transitions={'succeeded':'MOVE_TO_PERSON'})

		if mode==1:
		    	self.add('MOVE_TO_PERSON',sss_wrapper('move_base_rel','base', 'pos1_1'),
		                           transitions={'succeeded':'MOVE_TO_PERSON2','failed':'LED_NOT_REACHED'})

		    	self.add('MOVE_TO_PERSON2',sss_wrapper('move_base_rel','base', 'pos1_2'),
		                           transitions={'succeeded':'MOVE_TO_PERSON3','failed':'LED_NOT_REACHED'})

		    	self.add('MOVE_TO_PERSON3',sss_wrapper('move_base_rel','base', 'pos1_3'),
		                           transitions={'succeeded':'LED_REACHED','failed':'LED_NOT_REACHED'})
		elif mode==2:
		    	self.add('MOVE_TO_PERSON',sss_wrapper('move_base_rel','base', 'pos2_1'),
		                           transitions={'succeeded':'MOVE_TO_PERSON2','failed':'LED_NOT_REACHED'})

		    	self.add('MOVE_TO_PERSON2',sss_wrapper('move_base_rel','base', 'pos2_2'),
		                           transitions={'succeeded':'MOVE_TO_PERSON3','failed':'LED_NOT_REACHED'})

		    	self.add('MOVE_TO_PERSON3',sss_wrapper('move_base_rel','base', 'pos2_3'),
		                           transitions={'succeeded':'LED_REACHED','failed':'LED_NOT_REACHED'})
		elif mode==3:
            		self.add('MOVE_TO_PERSON',MoveToUPosition(),
                                   transitions={'succeeded':'LED_REACHED',
                                                'failed':'LED_NOT_REACHED'})


            	self.add('LED_REACHED',Light('white'),
                                   transitions={'succeeded':'MOVE_TRAY_MOVIE'})

	    	self.add('MOVE_TRAY_MOVIE',sss_wrapper('move','tray', 'video'),
	                           transitions={'succeeded':'STILL_TRAY','failed':'STILL_TRAY'})

	    	self.add('STILL_TRAY',StillTrayController(),
	                           transitions={'succeeded':'PLAY_MOVIE'})

            	self.add('PLAY_MOVIE',Tablet_Start('/sdcard/Video/Mayer.mp4'),
                                   transitions={'succeeded':'WAIT_FOR_MOVIE'})

            	self.add('WAIT_FOR_MOVIE',Sleep(28),
                                   transitions={'succeeded':'MOVE_TRAY_HOME'})

	    	self.add('MOVE_TRAY_HOME',sss_wrapper('move','tray', 'home'),
	                           transitions={'succeeded':'LED_FINISHED','failed':'LED_FINISHED'})

            	self.add('LED_FINISHED',Light('green_medium_pulse'),
                                   transitions={'succeeded':'MOVE_TO_HOME'})

		if mode==1:
		    	self.add('MOVE_TO_HOME',sss_wrapper('move_base_rel','base', 'pos1_1_back'),
		                           transitions={'succeeded':'MOVE_TO_HOME2','failed':'LED_NOT_REACHED'})

		    	self.add('MOVE_TO_HOME2',sss_wrapper('move_base_rel','base', 'pos1_2_back'),
		                           transitions={'succeeded':'MOVE_TO_HOME3','failed':'LED_NOT_REACHED'})

		    	self.add('MOVE_TO_HOME3',sss_wrapper('move_base_rel','base', 'pos1_3_back'),
		                           transitions={'succeeded':'succeeded','failed':'LED_NOT_REACHED'})
		elif mode==2:
		    	self.add('MOVE_TO_HOME',sss_wrapper('move_base_rel','base', 'pos2_1_back'),
		                           transitions={'succeeded':'MOVE_TO_HOME2','failed':'LED_NOT_REACHED'})

		    	self.add('MOVE_TO_HOME2',sss_wrapper('move_base_rel','base', 'pos2_2_back'),
		                           transitions={'succeeded':'MOVE_TO_HOME3','failed':'LED_NOT_REACHED'})

		    	self.add('MOVE_TO_HOME3',sss_wrapper('move_base_rel','base', 'pos2_3_back'),
		                           transitions={'succeeded':'succeeded','failed':'LED_NOT_REACHED'})
		elif mode==3:
            		self.add('MOVE_TO_HOME',MoveToPosition('home'),
                                   transitions={'succeeded':'succeeded',
                                                'failed':'LED_NOT_REACHED'})

		#error case
		if mode==3:
		    	self.add('LED_NOT_REACHED',Light('red'),
		                           transitions={'succeeded':'MOVE_TO_HOME_EM'})
            		self.add('MOVE_TO_HOME_EM',MoveToPosition('home'),
                                   transitions={'succeeded':'failed',
                                                'failed':'failed'})
		else:
		    	self.add('LED_NOT_REACHED',Light('red'),
		                           transitions={'succeeded':'failed'})

class Scenario(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        with self:

	    	self.add('MOVE_TRAY_HOME',sss_wrapper('move','tray', 'home'),
	                           transitions={'succeeded':'CHECK_LOCKED','failed':'CHECK_LOCKED'})

            	self.add('CHECK_LOCKED',CheckLocked(),
                                   transitions={'locked':'SLEEP_CHECK_LOCKED',
                                                'unlocked':'SLUMP_DETECTION'})

            	self.add('SLEEP_CHECK_LOCKED',Sleep(0.2),
                                   transitions={'succeeded':'CHECK_LOCKED'})

            	self.add('SLUMP_DETECTION',CheckSlump(),
                                   transitions={'nothing':'SLEEP_SLUMP_DETECTION',
                                                'slump1':'SLUMP1', 'slump2':'SLUMP2', 'slump_pos':'SLUMP_POS'})

            	self.add('SLEEP_SLUMP_DETECTION',Sleep(0.2),
                                   transitions={'succeeded':'SLEEP_CHECK_LOCKED'})

            	self.add('SLUMP1',Slump(1),
                                   transitions={'succeeded':'SLEEP_CHECK_LOCKED',
                                                'failed':'SLEEP_CHECK_LOCKED'})

            	self.add('SLUMP2',Slump(2),
                                   transitions={'succeeded':'SLEEP_CHECK_LOCKED',
                                                'failed':'SLEEP_CHECK_LOCKED'})

            	self.add('SLUMP_POS',Slump(3),
                                   transitions={'succeeded':'SLEEP_CHECK_LOCKED',
                                                'failed':'SLEEP_CHECK_LOCKED'})


if __name__=='__main__':
	rospy.init_node('MsWissenschaft')
	sm = Scenario()

	if True:	
		sm.execute()
		rospy.spin()
	else:
		# Start SMACH viewer
		smach_viewer = smach_ros.IntrospectionServer('MsWissenschaft', sm, '/MsWissenschaft')
		smach_viewer.start()

		outcome = sm.execute()

		# stop SMACH viewer
		rospy.spin()
		# smach_thread.stop()
		smach_viewer.stop()

