#!/usr/bin/env python
# -*- coding: utf_8 -*-

from __future__ import print_function
import rospy
import socket
import math
import pickle
import numpy.matlib
from std_msgs.msg import Int8
from rocr6_msgs.msg import Feedback,Goal
from g3p_msgs.msg import Gripper
import time

class RobotVisionWrapper:
    def __init__(self):
        rospy.loginfo("==========>START Robot Vision Wrapper, press [Ctrl+C]: exit<==========")
        rospy.on_shutdown(self.cleanUp)

        self.visionSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.visionSocket.settimeout(5)
        self.visionIsConnected = False

        self.inputResponding = False
        self.robotStatus = Feedback()
        self.objectCompensationXYZ = None
        self.useInputRPY = None
        self.objectRotateRPY = None
        self.objectPrePoseOffsetXYZRPY = None
        self.pickPrePose1Joint=None
        self.pickPrePose2Joint=None
        self.pickLatePose1Joint=None
        self.pickLatePose2Joint=None
        self.placeLatePose1Joint=None
        self.placeLatePose2Joint=None

        self.wrapperInputSub = rospy.Subscriber("/vision_rocr6/input",Int8,self.inputCallback)
        self.robotFeedbackSub = rospy.Subscriber("/rocr6_msgs/feedback",Feedback,self.updateRobotStatus)
        self.robotGoalPub = rospy.Publisher("/rocr6_msgs/goal",Goal,queue_size=5)
        self.robotGripper3Pub = rospy.Publisher("/g3p_msgs/gripper",Gripper,queue_size=5)

    def cleanUp(self):
        rospy.loginfo("==========>Robot Vision Wrapper STOPPED!<==========")
        # rospy.sleep(5)

    def initParam(self, objectCompensationXYZ, useInputRPY, objectRotateRPY, objectPrePoseOffsetXYZRPY,
                    pickPrePose1Joint, pickPrePose2Joint, pickLatePose1Joint, pickLatePose2Joint):
        self.objectCompensationXYZ = tuple(map(float,objectCompensationXYZ.split(",")))
        self.useInputRPY = useInputRPY
        self.objectRotateRPY = tuple(map(float,objectRotateRPY.split(",")))
        self.objectPrePoseOffsetXYZRPY = tuple(map(float,objectPrePoseOffsetXYZRPY.split(",")))
        self.pickPrePose1Joint = tuple(map(float,pickPrePose1Joint.split(",")))
        self.pickPrePose2Joint = tuple(map(float,pickPrePose2Joint.split(",")))
        self.pickLatePose1Joint = tuple(map(float,pickLatePose1Joint.split(",")))
        self.pickLatePose2Joint = tuple(map(float,pickLatePose2Joint.split(",")))
        self.placeLatePose1Joint = tuple(map(float,placeLatePose1Joint.split(",")))
        self.placeLatePose2Joint = tuple(map(float,placeLatePose2Joint.split(",")))

    def inputCallback(self, msg):
        if msg.data == 0:
            self.robotJointMove(0,0,0,0,0,0)
            return
            
        if not self.inputResponding:
            self.inputResponding = True
            if self.visionIsConnected:
                matrix = self.visionObjectRequest(str(msg.data))
                if numpy.all(matrix == 0):
                    rospy.logwarn("The target object is not detected !")
                elif numpy.all(matrix == -1):
                    rospy.logwarn("Illegal input, please re-enter!")
                else:
                    self.robotPickAndPlace(matrix)
            else:
                rospy.logfatal("Need to connect the VisionServer first, please try to restart this node !")
            self.inputResponding = False
        else:
            rospy.logwarn("Input is responding, please wait until the response is over !")

    def updateRobotStatus(self, msg):
        self.robotStatus = msg

    def connectVision(self, visionIPAddress, visionPort):
        rospy.loginfo("Connect to VisionServer ......")
        while not self.visionIsConnected and not rospy.is_shutdown():
            try:
                self.visionSocket.connect((visionIPAddress,visionPort))
                self.visionIsConnected = True
                rospy.loginfo("Connect to VisionServer Success !")
            except Exception as e:
                rospy.logwarn("Connect to VisionServer Failed! %s",e)
            rospy.sleep(5)

    def disconnectVision(self):
        while self.visionIsConnected and not rospy.is_shutdown():
            self.visionSocket.close()
            self.visionIsConnected = False
            rospy.loginfo("Disconnect to VisionServer Success !")
            rospy.sleep(5)

    def visionAction(self,commandString):
        self.visionSocket.sendall(commandString.encode("ascii"))
        recvData=self.visionSocket.recv(1024).decode("ascii")
        rospy.loginfo("tx:[%s]    rx:[%s]",commandString,recvData)
        return recvData

    def startVision(self):
        rospy.loginfo(self.visionAction("start"))

    def stopVision(self):
        rospy.loginfo(self.visionAction("stop"))

    def visionObjectRequest(self,objectNo):
        self.visionSocket.sendall(objectNo.encode("ascii"))
        # test=pickle.dumps(matrix,protocol=0)
        matrix = pickle.loads(self.visionSocket.recv(1024))
        rospy.loginfo("objectNo:[%s]\nobjectPosMatrix: %s", objectNo, matrix)
        return matrix

    def robotJointMove(self, j1, j2, j3, j4, j5, j6):   #单位： 弧度
        pos = Goal()
        pos.cmd, pos.a1, pos.a2, pos.a3, pos.a4, pos.a5, pos.a6 = 0, j1, j2, j3, j4, j5, j6
        self.robotGoalPub.publish(pos)
        rospy.loginfo("robotJointMoveTo: %f,%f,%f,%f,%f,%f",pos.a1, pos.a2, pos.a3, pos.a4, pos.a5, pos.a6)

    def robotJointMoveL(self, valueList):
        while self.robotStatus.status != 3 and not rospy.is_shutdown():
            rospy.sleep(1)
        if self.robotStatus.status == 3 and valueList != (-1,-1,-1,-1,-1,-1):
            self.robotJointMove(valueList[0], valueList[1], valueList[2], valueList[3], valueList[4], valueList[5])
            rospy.sleep(1)

    def robotCartesianMove(self, x, y, z, Rr, Rp, Ry): # 单位：米
        pos = Goal()
        pos.cmd, pos.a1, pos.a2, pos.a3, pos.a4, pos.a5, pos.a6 = 1, x, y, z, Rr, Rp, Ry
        self.robotGoalPub.publish(pos)
        rospy.loginfo("robotCartesianMoveTo: %f,%f,%f,%f,%f,%f",pos.a1, pos.a2, pos.a3, pos.a4, pos.a5, pos.a6)

    def robotCartesianMoveL(self, valueList):
        while self.robotStatus.status != 3 and not rospy.is_shutdown():
            rospy.sleep(1)
        if self.robotStatus.status == 3 and valueList != (-1,-1,-1,-1,-1,-1):
            self.robotCartesianMove(valueList[0], valueList[1], valueList[2], valueList[3], valueList[4], valueList[5])
            rospy.sleep(1)

    def robotMatrix2Pose(self, matrix):
        x = matrix[0,3] + self.objectCompensationXYZ[0]
        y = matrix[1,3] + self.objectCompensationXYZ[1]
        z = matrix[2,3] + self.objectCompensationXYZ[2]

        Rr, Rp, Ry = None, None, None
        if self.useInputRPY:
            Rr = self.objectRotateRPY[0]
            Rp = self.objectRotateRPY[1]
            Ry = self.objectRotateRPY[2]
        else:
            if abs(abs(matrix[0,2])-1)<1e-6:
                Rr=0
                if matrix[0,2]>0:
                    Ry=math.atan2(matrix[2,1],matrix[1,1])
                else:
                    Ry=-math.atan2(matrix[1,0],matrix[2,0])
                Rp=math.asin(matrix[0,2])
            else:
                Rr=-math.atan2(matrix[0,1],matrix[0,0])
                Ry=-math.atan2(matrix[1,2],matrix[2,2])
                Rp=math.atan(matrix[0,2]*math.cos(Rr)/matrix[0,0])
        return (x, y, z, Rr, Rp, Ry)

    def robotMatrix2PrePose(self, pose):
        return (pose[0]+self.objectPrePoseOffsetXYZRPY[0], pose[1]+self.objectPrePoseOffsetXYZRPY[1],
                pose[2]+self.objectPrePoseOffsetXYZRPY[2], pose[3]+self.objectPrePoseOffsetXYZRPY[3],
                pose[4]+self.objectPrePoseOffsetXYZRPY[4], pose[5]+self.objectPrePoseOffsetXYZRPY[5])

    def robotGripper3ChangeState(self, state):
        gripper3State = Gripper()
        gripper3State.state = state
        self.robotGripper3Pub.publish(gripper3State)
        rospy.loginfo("gripper3ChangeState: %f",gripper3State.state)

    def robotGripper3ChangeStateL(self, state):
        while self.robotStatus.status != 3 and not rospy.is_shutdown():
            rospy.sleep(1)
        if self.robotStatus.status == 3:
            self.robotGripper3ChangeState(state)
            rospy.sleep(2)

    def robotPickAndPlace(self, matrix):
        pose = self.robotMatrix2Pose(matrix)
        self.robotJointMoveL(self.pickPrePose1Joint)
        self.robotJointMoveL(self.pickPrePose2Joint)
        self.robotCartesianMoveL(self.robotMatrix2PrePose(pose))
        self.robotCartesianMoveL(pose)
        self.robotGripper3ChangeStateL(0)
        self.robotCartesianMoveL(self.robotMatrix2PrePose(pose))
        self.robotJointMoveL(self.pickLatePose1Joint)
        self.robotJointMoveL(self.pickLatePose2Joint)
        self.robotGripper3ChangeStateL(10)
        self.robotJointMoveL(self.placeLatePose1Joint)
        self.robotJointMoveL(self.placeLatePose2Joint)


if __name__ == "__main__":
    rospy.init_node("robot_vision_wrapper")

    visionServerIP = rospy.get_param("robot_vision_wrapper/vision_server_ip")
    visionServerPort = rospy.get_param("robot_vision_wrapper/vision_server_port")
    objectCompensationXYZ = rospy.get_param("robot_vision_wrapper/object_compensation_XYZ")
    useInputRPY = rospy.get_param("robot_vision_wrapper/use_input_RPY")
    objectRotateRPY = rospy.get_param("robot_vision_wrapper/object_rotate_RPY")
    objectPrePoseOffsetXYZRPY = rospy.get_param("robot_vision_wrapper/object_pre_pose_offset_XYZRPY")
    pickPrePose1Joint = rospy.get_param("robot_vision_wrapper/pick_pre_pose_1_joint")
    pickPrePose2Joint = rospy.get_param("robot_vision_wrapper/pick_pre_pose_2_joint")
    pickLatePose1Joint = rospy.get_param("robot_vision_wrapper/pick_late_pose_1_joint")
    pickLatePose2Joint = rospy.get_param("robot_vision_wrapper/pick_late_pose_2_joint")
    placeLatePose1Joint = rospy.get_param("robot_vision_wrapper/place_late_pose_1_joint")
    placeLatePose2Joint = rospy.get_param("robot_vision_wrapper/place_late_pose_2_joint")

    wrapper = RobotVisionWrapper()
    wrapper.initParam(objectCompensationXYZ, useInputRPY, objectRotateRPY, objectPrePoseOffsetXYZRPY,
                    pickPrePose1Joint, pickPrePose2Joint, pickLatePose1Joint, pickLatePose2Joint)

    wrapper.connectVision(visionServerIP, visionServerPort)
    wrapper.startVision()
    rospy.sleep(10)

    rospy.spin()

    wrapper.stopVision()
    wrapper.disconnectVision()
