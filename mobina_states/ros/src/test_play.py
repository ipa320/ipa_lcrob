#!/usr/bin/python
import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach

from mobina_states import *
import sys


class DeliverCake(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded'])

        with self:
		if len(sys.argv)>5:
		    	self.add('PLAY_HAPPY_BIRTHDAY6',Tablet_Play("/mnt/sdcard/destruct.mp3"),
		                           transitions={'succeeded':'succeeded'})
		elif len(sys.argv)>4:
		    	self.add('PLAY_HAPPY_BIRTHDAY5',Tablet_Start("/sdcard/Video/Mayer.mp4"),
		                           transitions={'succeeded':'succeeded'})
		elif len(sys.argv)>3:
		    	self.add('PLAY_HAPPY_BIRTHDAY4',Tablet_StartLinphone(),
		                           transitions={'succeeded':'succeeded'})
		elif len(sys.argv)>2:
		    	self.add('PLAY_HAPPY_BIRTHDAY1',Tablet_Start("http://www.golem.de/"),
		                           transitions={'succeeded':'succeeded'})
		elif len(sys.argv)>1:
		    	self.add('PLAY_HAPPY_BIRTHDAY2',Tablet_Play("/mnt/sdcard/musik.mp3"),
		                           transitions={'succeeded':'succeeded'})
		else:
            		self.add('PLAY_HAPPY_BIRTHDAY3',Tablet_Start("/mnt/sdcard/Mimi.mp4"),
                	                   transitions={'succeeded':'succeeded'})

            	#self.add('EXIT', Exit(),
                #                   transitions={'succeeded':'succeeded'})



if __name__=='__main__':
	rospy.init_node('test')
	sm = DeliverCake()
	outcome = sm.execute()
	rospy.spin()
