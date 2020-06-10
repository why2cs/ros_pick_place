#!/usr/bin/env python
# -*- coding: utf_8 -*-

from __future__ import print_function
import rospy
from robot_jaka.srv import *
from robot_jaka.srv import *

rospy.init_node("robot_client_test")
rospy.loginfo("****** Starting Robot (JAKA) Client Test Node ... ... ******")

rospy.wait_for_service("robot_service/robot_end_position")
rospy.wait_for_service("robot_service/robot_end_move")
handle_position = rospy.ServiceProxy("robot_service/robot_end_position",RobotEndPosition)
handle_move = rospy.ServiceProxy("robot_service/robot_end_move",RobotEndMove)

current_position=handle_position()
print("robot current position ==>",current_position.pos)

target_pos=(current_position.pos[0]+20,current_position.pos[1],current_position.pos[2],\
                current_position.pos[3],current_position.pos[4],current_position.pos[5])
response_move=handle_move(target_pos,10)
print("robot move status ==>",response_move.status)

rospy.loginfo("****** Stopped Robot (JAKA) Client Test Node ! ******")
