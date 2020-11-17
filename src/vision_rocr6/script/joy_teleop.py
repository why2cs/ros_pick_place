#!/usr/bin/env python
# -*- coding: utf_8 -*-
from __future__ import print_function
import importlib

import rospy
from sensor_msgs.msg import Joy
import time
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg
from std_msgs.msg import Int8
from rocr6_msgs.msg import Goal,Feedback
# from rocr6_msgs.msg import Goal,Gripper,xJoint,Feedback
# from zeus_s2_msgs.msg import joy_control
from g3p_msgs.msg import Gripper



class JoyTeleop:
	def __init__(self):
		self.active = 0
		self.flag = 0
		self.x_speed_scale = rospy.get_param('~x_speed_scale')
		# self.y_speed_scale = rospy.get_param('~y_speed_scale')
		self.w_speed_scale = rospy.get_param('~w_speed_scale')
		self.velocity = Twist()
		self.rocr6 = Goal()
		self.jiazhua = Gripper()
		# self.joyps3 = joy_control()
		self.rate = rospy.Rate(20)
        
		# self.cmdVelPublisher = rospy.Publisher('/cmd_moment', joy_control, queue_size = 3)
		self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 3)
		self.joySubscriber = rospy.Subscriber('joy', Joy, self.buttonCallback)
		self.rocr6Publisher = rospy.Publisher('/rocr6_msgs/goal', Goal, queue_size = 3)
		self.jiazhuaPublisher = rospy.Publisher('/g3p_msgs/gripper', Gripper, queue_size = 3)
		self.visionPulisher = rospy.Publisher("/vision_rocr6/input",Int8, queue_size = 10)
		self.loop()
	def buttonCallback(self, joy_data):
		# print "X : %f  ,Y : %f  ,Z: %f  ,joy_data.axes[0]: %f ,joy_data.axes[1]: %f  ,joy_data.axes[2]: %f  ,\
		# joy_data.axes[3]: %f ,joy_data.axes[4]: %f ,joy_data.axes[5]: %f,joy_data.buttons[4]:%f,joy_data.buttons[5]:%f" %(self.velocity.linear.x , self.velocity.linear.y,self.velocity.angular.z,joy_data.axes[0],\
		# joy_data.axes[1],joy_data.axes[2], joy_data.axes[3] , joy_data.axes[4], joy_data.axes[5],joy_data.buttons[4] ,joy_data.buttons[5])

		# if(joy_data.buttons[6] == 1):
    	# 		self.velocity.linear.x = self.x_speed_scale * joy_data.axes[3]
		# 	self.velocity.angular.z = self.w_speed_scale * joy_data.axes[0]
		# 	self.active = 1		
		# else:
		# 	self.velocity.linear = Vector3(0.,0.,0.)
		# 	self.velocity.angular = Vector3(0.,0.,0.)
		# 	self.active = 0
		# 	self.cmdVelPublisher.publish(self.velocity)					
							
		# if(joy_data.axes[6]==1.0):
    	# 		self.rocr6.shoulder_pan_joint.position = 0.0
		# 	self.rocr6.shoulder_pan_joint.velocity = 0.0
		# 	self.rocr6.shoulder_pan_joint.effort = 0.0

		# 	self.rocr6.shoulder_lift_joint.position = 1.6935
		# 	self.rocr6.shoulder_lift_joint.velocity = 0.0
		# 	self.rocr6.shoulder_lift_joint.effort = 0.0

		# 	self.rocr6.elbow_joint.position = -2.5235
		# 	self.rocr6.elbow_joint.velocity = 0.0
		# 	self.rocr6.elbow_joint.effort = 0.0

		# 	self.rocr6.wrist_1_joint.position = 0.0
		# 	self.rocr6.wrist_1_joint.velocity = 0.0
		# 	self.rocr6.wrist_1_joint.effort = 0.0

		# 	self.rocr6.wrist_2_joint.position = 0.0
		# 	self.rocr6.wrist_2_joint.velocity = 0.0
		# 	self.rocr6.wrist_2_joint.effort = 0.0

		# 	self.rocr6.wrist_3_joint.position = 0.830
		# 	self.rocr6.wrist_3_joint.velocity = 0.0
		# 	self.rocr6.wrist_3_joint.effort = 0.0
		# 	self.rocr6Publisher.publish(self.rocr6)
		# if(joy_data.axes[6]== -1.0):
		# 	self.rocr6.shoulder_pan_joint.position = 0.0
		# 	self.rocr6.shoulder_pan_joint.velocity = 0.0
		# 	self.rocr6.shoulder_pan_joint.effort = 0.0

		# 	self.rocr6.shoulder_lift_joint.position = 0.0
		# 	self.rocr6.shoulder_lift_joint.velocity = 0.0
		# 	self.rocr6.shoulder_lift_joint.effort = 0.0

		# 	self.rocr6.elbow_joint.position = 0.0
		# 	self.rocr6.elbow_joint.velocity = 0.0 
		# 	self.rocr6.elbow_joint.effort = 0.0

		# 	self.rocr6.wrist_1_joint.position = 0.0
		# 	self.rocr6.wrist_1_joint.velocity = 0.0
		# 	self.rocr6.wrist_1_joint.effort = 0.0

		# 	self.rocr6.wrist_2_joint.position = 0.0
		# 	self.rocr6.wrist_2_joint.velocity = 0.0
		# 	self.rocr6.wrist_2_joint.effort = 0.0

		# 	self.rocr6.wrist_3_joint.position = 0.0
		# 	self.rocr6.wrist_3_joint.velocity = 0.0
		# 	self.rocr6.wrist_3_joint.effort = 0.0
		# 	self.rocr6Publisher.publish(self.rocr6)
		if(joy_data.axes[7]==1.0):
			self.jiazhua.state = 11
			self.jiazhuaPublisher.publish(self.jiazhua)
		if(joy_data.axes[7]== -1.0):
			self.jiazhua.state = 0
			self.jiazhuaPublisher.publish(self.jiazhua)							
		# 	self.active = 1
		# 	self.joyps3.priority_flag = 0.0
		# 	self.joyps3.parking_mode = 0.0
		# else:
		# 	self.joyps3.left_wheel_torque_value = 0.0
		# 	self.joyps3.right_wheel_torque_value = 0.0
		# 	self.active = 0
		# 	# self.joyps3.priority_flag = 0.0
		# 	self.cmdVelPublisher.publish(self.joyps3)
		# # if(joy_data.buttons[5] == 1.0):
    	# # 		self.joyps3.priority_flag = 1.0
    	# # 		self.active = 0
		# # 	self.cmdVelPublisher.publish(self.joyps3)
		# # if(joy_data.axes[2] == -1.0):
		# # 		self.joyps3.parking_mode = 1.0
    	# # 	self.active = 1
		# # if(joy_data.axes[5] == -1.0):
    	# # 		self.joyps3.parking_mode = 2.0
    	# # 	self.active = 1


		if(joy_data.buttons[0]==1.0):
			msg = Int8()
			msg.data = 1
			self.visionPulisher.publish(msg)

		if(joy_data.buttons[1]==1.0):
			msg = Int8()
			msg.data = 2
			self.visionPulisher.publish(msg)

		if(joy_data.buttons[3]==1.0):
			msg = Int8()
			msg.data = 3
			self.visionPulisher.publish(msg)

		if(joy_data.buttons[4]==1.0):
			msg = Int8()
			msg.data = 4
			self.visionPulisher.publish(msg)

		if(joy_data.buttons[7]==1.0):
			msg = Int8()
			msg.data = 0
			self.visionPulisher.publish(msg)

		if(joy_data.buttons[9]==1.0):
			msg = Int8()
			msg.data = 99
			self.visionPulisher.publish(msg)

                            
	def loop(self):
		while not rospy.is_shutdown():
						if(self.active == 1):
			    				# self.cmdVelPublisher.publish(self.joyps3)
							self.cmdVelPublisher.publish(self.velocity)
						# print("X : %f  Y : %f  Z: %f ", self.velocity.linear.x , self.velocity.linear.y,self.velocity.angular.z)
						self.rate.sleep()
			

	
if __name__ == '__main__':
	rospy.init_node('joy_teleop')
	joy = JoyTeleop()
	try:
		rospy.spin()
	except	rospy.ROSInterruptException:
		print('exception')			

