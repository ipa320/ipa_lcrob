#!/usr/bin/python
import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach

from ApproachPose import *
from mobina_states import *
from BasicIO import *


class DeliveryTask(smach.StateMachine):
    def __init__(self, path_name):
        smach.StateMachine.__init__(self,
								outcomes=['succeeded','failed'])
        with self:

            	self.add('LIGHT2',Light('blue'),
                                   transitions={'succeeded':'MOVE_TO_DELIVERY'})


            	self.add('MOVE_TO_DELIVERY',ApproachPose(path_name),
                                   transitions={'reached':'PLAY_HAPPY_BIRTHDAY',
                                                'not_reached':'MOVE_TO_DELIVERY',
                                                'failed':'failed'})

            	self.add('PLAY_HAPPY_BIRTHDAY',Tablet_Start("/mnt/sdcard/ad.mp4"),
                                   transitions={'succeeded':'WAIT2'})

            	self.add('WAIT2',Sleep(25),
                                   transitions={'succeeded':'LIGHT3'})

            	self.add('LIGHT3',Light('green'),
                                   transitions={'succeeded':'succeeded'})


class DeliverCake(smach.StateMachine):
    def __init__(self, path_name="path"):
        smach.StateMachine.__init__(self, outcomes=['succeeded','not_reached','failed'])

        with self:

            self.add('SAFE_MODE', Turtlebot_SetMode(2),
			transitions={'succeeded':'DELIVERY0', 'failed':'failed'})

            no = 0
            #for i in range(0,3):
            while rospy.has_param("/script_server/base/"+path_name+str(no)):
            	#last = i>1n
		last = not rospy.has_param("/script_server/base/"+path_name+str(no+1))
		next = 'DELIVERY'+str(no+1)
		if last:
			next = 'succeeded'
            	self.add('DELIVERY'+str(no),DeliveryTask(path_name+str(no)),
                                   transitions={'succeeded':next,
                                                'failed':'failed'})

            	no += 1

            self.add('MOVE_BACK',ApproachPose("charge_pose"),
                                   transitions={'reached':'succeeded',
                                                'not_reached':'not_reached',
                                                'failed':'failed'})


if __name__=='__main__':
	rospy.init_node('DeliverCake')
	sm = DeliverCake()
	outcome = sm.execute()
	rospy.spin()
