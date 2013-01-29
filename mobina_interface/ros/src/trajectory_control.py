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

from control_msgs.msg import *
from std_srvs.srv import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointVelocities
from thread import start_new_thread

class TrajectoryControl(object):
	# create messages that are used to publish feedback/result
	_feedback = FollowJointTrajectoryActionFeedback()
	_result   = FollowJointTrajectoryResult()

	def __init__(self, name, joint_name, intf, conf_out, conf_in):
		self.intf = intf
		self.conf_out = conf_out
		self.conf_in  = conf_in
		self.calibration = [ [0,0.0006], [1,0.0006] ]
		self.calibration_pos = [ [0,0], [1,1] ]
		self.tolerance = rospy.get_param(name+'/tolerance', 0.005)

		if rospy.has_param(name+'/calibration'):
			y = yaml.load(open(rospy.get_param(name+'/calibration')))
			if 'calibration_vel' in y: self.calibration = y['calibration_vel']
			if 'calibration_pos' in y: self.calibration_pos = y['calibration_pos']

		self.joint_msg = JointState()
		self.joint_msg.name = [joint_name]
		self.joint_msg.position = [0.0]
		self.joint_msg.velocity = [0.0]

		rospy.Subscriber(name+'/command_vel', JointVelocities, self.on_vel)

		self.joint_pub = rospy.Publisher('joint_states', JointState)
		self.calibration_srv = rospy.Service(name+'/calibration', Empty, self.handle_calibration)
		self._action_name = name+"/follow_joint_trajectory"
		self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)
		self._as.start()

	def start_test(self, pos):
		goal = FollowJointTrajectoryActionGoal()
		pt = FollowJointTrajectoryActionPoint()
		pt.positions=[pos]
		goal.trajectory = FollowJointTrajectoryAction()
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
		while abs(self.getpos()-pos)>self.tolerance:
			if self._as.is_preempt_requested():
				rospy.loginfo('%s: Preempted' % self._action_name)
				self._as.set_preempted()
				success = False
				break

			if self.getpos()>=pos2[0] and self.getpos()<=pos2[1]:
				if not start:
					start = [rospy.get_time(),self.getpos()]
				else:
					end = [rospy.get_time(),self.getpos()]
			f = 1
			if self.getpos()-pos>0:
				f=-1
			self.intf.set_val(self.conf_out, f*speed)

		self.intf.set_val(self.conf_out, 0)

		print start, end

		return abs(end[1]-start[1])/(end[0]-start[0])

	def changed(self):
		return
	
	def rad2pos(self, rad):
		j = 0
		for c in self.calibration_pos:
			if rad>=c[0]: break
			j+=1
		if j>=len(self.calibration_pos): j-=1
		return (self.calibration_pos[j+1][1]-self.calibration_pos[j][1])*(rad-self.calibration_pos[j][0])/(self.calibration_pos[j+1][0]-self.calibration_pos[j][0])+self.calibration_pos[j][1]

	def getpos(self):
		p = self.intf.get(self.conf_in)
		j = 0
		for c in self.calibration_pos:
			if p>=c[1]: break
			j+=1
		if j>=len(self.calibration_pos): j-=1
		return (self.calibration_pos[j+1][0]-self.calibration_pos[j][0])*(p-self.calibration_pos[j][1])/(self.calibration_pos[j+1][1]-self.calibration_pos[j][1])+self.calibration_pos[j][0]
		

	def speed2val(self, speed):
		for i in range(0, len(self.calibration)-1):
			if speed>=self.calibration[i][0] and speed<=self.calibration[i+1][0]:
				f = (speed-self.calibration[i][0])/(self.calibration[i+1][0]-self.calibration[i][0])
				return f*(self.calibration[i+1][1]-self.calibration[i][1]) + self.calibration[i][1]
		return self.calibration[len(self.calibration)-1][1]

	def on_vel(self, vel):
		if len(vel.velocities)==1: self.intf.set_val(self.conf_out, vel.velocities[0].value)
		else: rospy.logwarn("only one joint in "+self._action_name)

	def execute_cb(self, goal):
		# helper variables
		r = rospy.Rate(50)
		success = True

		assert( len(goal.trajectory.points[0].positions)==1 )
		#assert( len(goal.trajectory.points[0].velocities)==1 )

		pos = self.rad2pos(goal.trajectory.points[0].positions[0])
		#print "desired pos", pos
		speed = 0
		if len(goal.trajectory.points[0].velocities)==1:
			max_vel = goal.trajectory.points[0].velocities[0]
		else:
			max_vel = 1/float(255)
		if max_vel<1/float(255):
			max_vel = 1
    
		while abs(self.getpos()-pos)>self.tolerance and not rospy.is_shutdown():
			#print "pos ",self.getpos()
			if (self.speed2val(speed)/abs(self.getpos()-pos))>=max_vel:
				s = speed - max_vel
				if s<0:
					s = 1/float(255)
			else:
				s = speed + max_vel
				if s>1:
					s = 1
			f = -1
			if self.getpos()-pos>0:
				f=1

			if s!=speed:
				speed = s
				self.joint_msg.velocity = [f*speed]
				self.intf.set_val(self.conf_out, f*speed)
			r.sleep()

		self.intf.set_val(self.conf_out, 0)
		self.joint_msg.velocity = [0.0]
		self._result.error_code = 0
		if success:
			self._as.set_succeeded(self._result)

	def publish(self):
		self.joint_msg.header.stamp = rospy.Time.now()
		self.joint_msg.position = [self.getpos()]
		self.joint_pub.publish(self.joint_msg)
