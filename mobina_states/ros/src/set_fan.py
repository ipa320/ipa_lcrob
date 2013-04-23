#!/usr/bin/python

import roslib
roslib.load_manifest('mobina_states')
import rospy
import smach
import sys

from mobina_states import Turtlebot_SetFan


# main
def main():
	rospy.init_node('set_fan')

	# create a SMACH state machine
	SM_REHACARE = smach.StateMachine(outcomes=['succeeded'])

	# open the container
	with SM_REHACARE:
		smach.StateMachine.add('SET_MODE', Turtlebot_SetFan(float(sys.argv[1])),
			transitions={'succeeded':'succeeded'})

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
