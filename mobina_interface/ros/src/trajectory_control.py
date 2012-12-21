#!/usr/bin/python
#***************************************************************
#
# Copyright (c) 2010
#
# Fraunhofer Institute for Manufacturing Engineering	
# and Automation (IPA)
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Project name: mobina
# ROS stack name: mobina
# ROS package name: mobina
#								
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#			
# Author: Joshua Hampp
#
# Date of creation: Nov 2012
# ToDo:
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fraunhofer Institute for Manufacturing 
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#****************************************************************

import roslib; 
roslib.load_manifest('mobina_interface')
import rospy
import actionlib
import yaml
import math

from pr2_controllers_msgs.msg import *
from std_srvs.srv import *
from trajectory_msgs.msg import *

from thread import start_new_thread

class TrajectoryControl(object):
	# create messages that are used to publish feedback/result
	_feedback = JointTrajectoryFeedback()
	_result   = JointTrajectoryResult()

	def __init__(self, name, intf, conf_out, conf_in):
		self.intf = intf
		self.conf_out = conf_out
		self.conf_in  = conf_in
		self.calibration = [ [0,0.0006], [1,0.0006] ]
		self.tolerance = rospy.get_param(name+'/tolerance', 0.005)

		if rospy.has_param(name+'/calibration'):
			self.calibration = yaml.load(open(rospy.get_param(name+'/calibration')))

		self.calibration_srv = rospy.Service(name+'/calibration', Empty, self.handle_calibration)
		self._action_name = name+"/follow_joint_trajectory"
		self._as = actionlib.SimpleActionServer(self._action_name, JointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)
		self._as.start()

	def start_test(self, pos):
		goal = JointTrajectoryGoal()
		pt = JointTrajectoryPoint()
		pt.positions=[pos]
		goal.trajectory = JointTrajectory()
		goal.trajectory.points = [pt]
		start_new_thread(self.execute_cb, (goal,))

	def handle_calibration(self, req):
		pos  = [0.1, 0.9]
		pos2 = [0.2, 0.8]
		speed = [x * 0.1 for x in range(1, 11)]

		self.calibration = [ [0,0] ]
		self.moveTo(pos[1], speed[0], pos2)	#start position
		for i in range(0,len(speed)):
			self.calibration.append( [speed, self.moveTo(pos[(i%2)], speed[i]), pos2] )

		print self.calibration

		print "Save? (Y/n) ", 
		if read().lower()!="n":
			open(rospy.get_param(name+'/calibration'),'w').write(str(self.calibration))

		return EmptyResponse()

	def move(self, pos, speed, pos2):
		start = False
		end = False
		while abs(self.intf.get(self.conf_in)-pos)>self.tolerance:
			if self._as.is_preempt_requested():
				rospy.loginfo('%s: Preempted' % self._action_name)
				self._as.set_preempted()
				success = False
				break

			if self.intf.get(self.conf_in)>=pos2[0] and self.intf.get(self.conf_in)<=pos2[1]:
				if not start:
					start = [rospy.get_time(),self.intf.get(self.conf_in)]
				else:
					end = [rospy.get_time(),self.intf.get(self.conf_in)]
			f = 1
			if self.intf.get(self.conf_in)-pos>0:
				f=-1
			self.intf.set_val(self.conf_out, f*speed)

		self.intf.set_val(self.conf_out, 0)

		print start, end

		return abs(end[1]-start[1])/(end[0]-start[0])

	def changed(self):
		return
	
	def rad2pos(self, rad):
		return rad/(2*math.pi)		#TODO:
		

	def speed2val(self, speed):
		for i in range(0, len(self.calibration)-1):
			if speed>=self.calibration[i][0] and speed<=self.calibration[i+1][0]:
				f = (speed-self.calibration[i][0])/(self.calibration[i+1][0]-self.calibration[i][0])
				return f*(self.calibration[i+1][1]-self.calibration[i][1]) + self.calibration[i][1]
		return self.calibration[len(self.calibration)-1][1]
    
	def execute_cb(self, goal):
		# helper variables
		r = rospy.Rate(50)
		success = True

		assert( len(goal.trajectory.points[0].positions)==1 )
		#assert( len(goal.trajectory.points[0].velocities)==1 )

		pos = self.rad2pos(goal.trajectory.points[0].positions[0])
		speed = 0
		if len(goal.trajectory.points[0].velocities)==1:
			max_vel = goal.trajectory.points[0].velocities[0]
		else:
			max_vel = 1/float(255)
		if max_vel<1/float(255):
			max_vel = 1
    
		while abs(self.intf.get(self.conf_in)-pos)>self.tolerance and not rospy.is_shutdown():
			if (self.speed2val(speed)/abs(self.intf.get(self.conf_in)-pos))>=max_vel:
				s = speed - max_vel
				if s<0:
					s = 1/float(255)
			else:
				s = speed + max_vel
				if s>1:
					s = 1
			f = 1
			if self.intf.get(self.conf_in)-pos>0:
				f=-1

			if s!=speed:
				speed = s
				self.intf.set_val(self.conf_out, f*speed)
			r.sleep()
		self.intf.set_val(self.conf_out, 0)
      
		if success:
			self._as.set_succeeded(self._result)
      
