#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_environment_perception_intern
# \note
#   ROS package name: cob_3d_mapping_demonstrator
#
# \author
#   Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
# \author
#   Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
#
# \date Date of creation: 03/2012
#
# \brief
#   Implementation of ROS node for script_server.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import time

import roslib
roslib.load_manifest('cob_script_server')
import rospy
import actionlib
import sys

from simple_script_server import *

sss = simple_script_server()

## Script server class which inherits from script class.
#
# Implements actionlib interface for the script server.
#
class test_motor():
  ## Initializes the actionlib interface of the script server.
  #
  def __init__(self):
    self.actor = "motor"

  def move_to(self, pos):
    sss.move(self.actor,[[pos]])


## Main routine for running the script server
#
if __name__ == '__main__':
  rospy.init_node('test_motor')
  tm = test_motor()
  tm.move_to(float(sys.argv[1]))
  rospy.spin()
