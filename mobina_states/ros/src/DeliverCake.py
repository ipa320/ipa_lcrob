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

            	self.add('MOVE_TO_CHARGE',ApproachPose("charge_pose"),
                                   transitions={'reached':'LIGHT1',
                                                'not_reached':'MOVE_TO_CHARGE',
                                                'failed':'failed'})

            	self.add('LIGHT1',Light('green'),
                                   transitions={'succeeded':'WAIT'})

            	self.add('WAIT',Sleep(15),
                                   transitions={'succeeded':'LIGHT2'})

            	self.add('LIGHT2',Light('blue'),
                                   transitions={'succeeded':'MOVE_TO_DELIVERY'})


            	self.add('MOVE_TO_DELIVERY',ApproachPose(path_name),
                                   transitions={'reached':'LIGHT3',
                                                'not_reached':'MOVE_TO_DELIVERY',
                                                'failed':'failed'})

            	self.add('LIGHT3',Light('red'),
                                   transitions={'succeeded':'PLAY_HAPPY_BIRTHDAY'})

            	self.add('PLAY_HAPPY_BIRTHDAY',Tablet_Start("happy_birthday.avi"),
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
