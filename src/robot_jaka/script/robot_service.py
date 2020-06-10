#!/usr/bin/env python
# -*- coding: utf_8 -*-

from __future__ import print_function
from robot_interface import Robot
import rospy
from robot_jaka.srv import RobotEndPosition, RobotEndPositionResponse
from robot_jaka.srv import RobotEndMove,RobotEndMoveResponse
from robot_jaka.msg import RobotEndMoveTarget

class Robot_Service:
    def __init__(self,ip_address,port):
        rospy.on_shutdown(self.clean_up)

        self.ip_address=ip_address
        self.port=port
        self.robot=Robot(ip_address,port)
        self.service_end_position=rospy.Service("robot_service/robot_end_position",RobotEndPosition,self.robot_end_position_service_callback,1)
        self.service_end_move=rospy.Service("robot_service/robot_end_move",RobotEndMove,self.robot_end_move_service_callback,1)
        self.subscriber_end_move_target=rospy.Subscriber("robot_topic/robot_end_move",RobotEndMoveTarget,self.robot_end_move_subsciber_callback,1)


    def clean_up(self):
        rospy.loginfo("****** Stopped Robot (JAKA) Service Node ! ******")
        rospy.sleep(2)

    def robot_end_position_service_callback(self,req):
        pos = self.robot.robotEndPosition()
        return RobotEndPositionResponse(pos)

    def robot_end_move_service_callback(self,req):
        errorCode = self.robot.robotEndMove(req.pos[0],req.pos[1],req.pos[2],req.pos[3],req.pos[4],req.pos[5],req.speed)
        return RobotEndMoveResponse(errorCode == "0")

    def robot_end_move_subsciber_callback(self,msg):
        errorCode = self.robot.robotEndMove(msg.pos[0],msg.pos[1],msg.pos[2],msg.pos[3],msg.pos[4],msg.pos[5],msg.speed)
        if errorCode != "0":
            rospy.loginfo("Robot end move ERROR: %s !",errorCode)


if __name__ == "__main__":

    rospy.init_node("robot_service")
    robot_service=Robot_Service("192.168.3.150",10001)
    rospy.loginfo("****** Starting Robot (JAKA) Service Node ... ... ******")
    rospy.spin()


