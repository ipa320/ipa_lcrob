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
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker



class LightControl:
	def __init__(self, ns, intf, conf):
		self.pub_marker = rospy.Publisher(ns+"/marker", Marker)

		self.intf = intf
		self.conf = conf

		# set default color to green rgba = [0,1,0,1]
		self.color = ColorRGBA()
		self.color.r = 0
		self.color.g = 0
		self.color.b = 0
		self.color.a = 1
		self.setRGB( self.color )
		rospy.Subscriber(ns+"/command", ColorRGBA, self.LightCallback)

	def setRGB(self, color):
		self.color = color
		self.intf.set_val(self.conf[0], self.color.r)
		self.intf.set_val(self.conf[1], self.color.g)
		self.intf.set_val(self.conf[2], self.color.b)
		

	def publish_marker(self):
		# create marker
		marker = Marker()
		marker.header.frame_id = "/base_link"
		marker.header.stamp = rospy.Time.now()
		marker.ns = "color"
		marker.id = 0
		marker.type = 2 # SPHERE
		marker.action = 0 # ADD
		marker.pose.position.x = 0
		marker.pose.position.y = 0
		marker.pose.position.z = 1.5
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = self.color.a #Transparency
		marker.color.r = self.color.r
		marker.color.g = self.color.g
		marker.color.b = self.color.b
		# publish marker
		self.pub_marker.publish(marker)

	def LightCallback(self,color):
		rospy.loginfo("Received new color: rgb = [%d, %d, %d] a = [%d]", color.r, color.g, color.b, color.a)
		self.setRGB(color)

