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
# Project name: care-o-bot
# ROS stack name: cob_driver
# ROS package name: cob_light
#								
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#			
# Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
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
roslib.load_manifest('mobina_interface')
import rospy
from trajectory_control import TrajectoryControl
from light_control import LightControl
from sensor_msgs.msg import ChannelFloat32


class MobinaInterface:
	def __init__(self):
		self.pub_out = rospy.Publisher("command", ChannelFloat32, tcp_nodelay=True)

		self.output = ChannelFloat32()
		self.input = ChannelFloat32()

		self.output.name = "output"
		self.output.values = [0,0, 0,0,0, 0,0,0]

		self.input.name = "input"
		self.input.values = [0,0, 0,0]

		self.lights = [LightControl("light_controller", self, [3,6,2])]
		self.motors = [TrajectoryControl("tray_controller", "tray_joint", self, 0, 0)]

                rospy.Subscriber("state", ChannelFloat32, self.InputCallback)

	def set_val(self, pin, value):
		#print "setval ",pin,value
		self.output.values[pin] = value
		self.pub_out.publish(self.output)

	def get(self, pin):
		return self.input.values[pin]

	def InputCallback(self, inp):
		self.input = inp
		for m in self.motors:	#inform all motors
			m.changed()

	def publish_marker(self):
		for l in self.lights:
			l.publish_marker()
		for m in self.motors:
			m.publish()

if __name__ == '__main__':
	rospy.init_node('mobina')
	mi = MobinaInterface()
		
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		mi.publish_marker()
		r.sleep()
