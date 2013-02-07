#!/usr/bin/python

import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach

from mobina_interface.srv import *
from turtlebot_node.srv import SetTurtlebotMode

from mobina_states import Turtlebot_SetMode


# main
def main():
	rospy.init_node('search_dock')

	# create a SMACH state machine
	SM_REHACARE = smach.StateMachine(outcomes=['succeeded','failed'])

	# open the container
	with SM_REHACARE:
		#TODO: move to some pose

		# add states to the container
		smach.StateMachine.add('PASSIVE', Turtlebot_SetMode(1),
			transitions={'succeeded':'SEARCH_DOCK', 'failed':'failed'})

		smach.StateMachine.add('SEARCH_DOCK', Turtlebot_SetMode(4),
			transitions={'succeeded':'succeeded', 'failed':'failed'})

#------------------------------------------------------------------------------------------#
#-----	EXECUTE SMACH				-------------------------------------------------------#


	# Execute SMACH tree in a separate thread so that we can ctrl-c the script
	# smach_thread = threading.Thread(target = SM_WIMICARE.execute)
	# smach_thread.start()

	SM_REHACARE.execute()

#------------------------------------------------------------------------------------------#
#-----	EXECUTE MAIN FUNCTION		-------------------------------------------------------#

if __name__ == '__main__':
	main()
