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
# Project name: 
# ROS stack name: cob_driver
# ROS package name: cob_odroidx_interface
#								
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#			
# Author: Joshua Hampp
#
# Date of creation: June 2010
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
roslib.load_manifest('ipa_odroidx_interface')
import rospy

from sensor_msgs.msg import ChannelFloat32
from ipa_odroidx_interface.srv import *
from avr_interface import AVRInterface

class AVRControl:
	def __init__(self):
		self.ns_global_prefix = "/odroidx_interface"

		if not rospy.has_param(self.ns_global_prefix + "/polling_interval"):
			self.polling_interval = 1
		else:
			self.polling_interval = rospy.get_param(self.ns_global_prefix + "/polling_interval")

		if not rospy.has_param(self.ns_global_prefix + "/analog_channels"):
			self.analog_ch = []
		else:
			self.analog_ch = map(int, str(rospy.get_param(self.ns_global_prefix + "/analog_channels")).split(","))

		self._connect()

		self.old_values = [-1]*14
		self.old_values[0] = 0	#prevent of motor stop
		self.old_values[1] = 0

		self.pub_marker = rospy.Publisher("state", ChannelFloat32)
		rospy.Subscriber("command", ChannelFloat32, self.setCallback, queue_size=1)

		s = rospy.Service('mot0', MotorAim, self.cb_mot0)

	def _connect(self):
		while True:
			try:
				self.intf = AVRInterface()
				self.intf.setup()
				break
			except Exception, e:
				print "AVR Error:", e
				rospy.sleep(1)


	def _check(self):
		try:
			self.intf.setup()
		except Exception, e:
			return False
		return True


	def setCallback(self, data):
		assert(len(data.values)==14)
		for i in range(0,2):
			assert (data.values[i]>=-1 and data.values[i]<=1)
			if self.old_values[i]!=data.values[i]:
				self.intf.set_motor(i, data.values[i]<0, int(abs(data.values[i])*255))
				self.old_values[i] = data.values[i]
		for i in range(2,8):
			assert (data.values[i]>=0 and data.values[i]<=1)
			if self.old_values[i]!=data.values[i] or self.old_values[i+6]!=data.values[i+6]:
				self.intf.set_output(i-2, int(data.values[i]*255))
				self.old_values[i] = data.values[i]
				self.old_values[i+6] = 0
		for i in range(8,14):
			assert (data.values[i]>=0 and data.values[i]<=1)
			if self.old_values[i]!=data.values[i]:
				self.intf.set_pulse(i-8, int(data.values[i]*255))
				self.old_values[i] = data.values[i]

	def cb_mot0(self, data):
		self.intf.set_motoraim0(data.value)
		return MotorAimResponse()

	def publish(self):
		data = map(float, self.intf.get_input() )
		for a in self.analog_ch:
			data[a] = self.intf.get_analog(a)/float(1023)
		
		ch = ChannelFloat32()
		ch.name = "avr_measurements"
		ch.values = data
		self.pub_marker.publish(ch)
			

if __name__ == '__main__':
	rospy.init_node('odroidx_interface')
	ac = AVRControl()

	r = rospy.Rate(ac.polling_interval)
	n = 0
	while not rospy.is_shutdown():
		n+=1
		if n%100==0:
			if not ac._check(): ac._connect()
		ac.publish()
		r.sleep()
	
