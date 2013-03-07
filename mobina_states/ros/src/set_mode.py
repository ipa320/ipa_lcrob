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
	mode = -1

	if len(sys.argv)==2:
		s = sys.argv[1]
		if s=="passive": mode = 1
		elif s=="safe": mode = 2
		elif s=="full": mode = 3
		elif s=="dock": mode = 4
		elif s=="reboot": mode = 5
	if mode==-1:
		print "please specify mode as argument"
		print "possible modes are:"
		print "\tpassive"
		print "\tsafe"
		print "\tfull"
		print "\tdock"
		exit()

	rospy.init_node('set_mode')

	# create a SMACH state machine
	SM_REHACARE = smach.StateMachine(outcomes=['succeeded','failed'])

	# open the container
	with SM_REHACARE:
		smach.StateMachine.add('SET_MODE', Turtlebot_SetMode(mode),
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
