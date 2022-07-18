#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on July 6 2022

@author: Giovanni Franzese, Cognitive Robotics, TU Delft
Run this node in a terminal. 
Remember to run the spacenav launch file 
$ roslaunch spacenav_node classic.launch 
and to install the space nav before with 
$ sudo apt install spacenavd
$ sudo apt install ros-indigo-spacenav-node
"""

import rospy
import numpy as np
import quaternion # pip install numpy-quaternion
from quaternion import from_euler_angles
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Joy
from pynput.keyboard import Listener, KeyCode
from scipy.spatial.transform import Rotation


class Teleoperation():
    def __init__(self):
        self.control_freq=rospy.Rate(100)
        self.offset = [0, 0, 0, 0, 0, 0]
        self.curr_pos = None
        self.joint_pos = None
        self.joy_sub=rospy.Subscriber("/spacenav/joy", Joy, self.spacenav_callback)
        self.pos_sub=rospy.Subscriber("/cartesian_pose_elbow", PoseStamped, self.ee_pos_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose_elbow', PoseStamped, queue_size=0)  
        #self.curr_pos = np.array([0.5, 0, 0.5])
        #self.curr_ori = np.array([1, 0, 0, 0])


    def spacenav_callback(self, data):
        self.offset[0]= 0.002*data.axes[0]
        self.offset[1]= 0.002*data.axes[1]
        self.offset[2]= 0.002*data.axes[2]
        self.offset[3]= 0.02*data.axes[3]
        self.offset[4]= 0.02*data.axes[4] 
        self.offset[5]= 0.02*data.axes[5]

    def ee_pos_callback(self, data):
        self.curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.curr_ori = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])


  
    def spacenav(self):
        # robot = rtb.models.DH.Panda()
        # local_pos = curr_pos.reshape(1, 3)
        # x_new = local_pos[0][0]
        # y_new = local_pos[0][1]
        # z_new = local_pos[0][2]
        # joint=self.curr_orijoint_pos[0:7].reshape(1, -1)
        # fw_pose = robot.fkine(joint)
        # goal_ori = fw_pose.R
        # rot = Rotation.from_quat(self.curr_ori)
        # goal_ori_eul = rot.as_euler('xyz')
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x =  self.curr_pos[0]
        goal.pose.position.y =  self.curr_pos[1]
        goal.pose.position.z =  self.curr_pos[2]
            #print("quat", quat) 
        goal.pose.orientation.w = self.curr_ori[0]    
        goal.pose.orientation.x = self.curr_ori[1] 
        goal.pose.orientation.y = self.curr_ori[2] 
        goal.pose.orientation.z = self.curr_ori[3] 
        self.goal_pub.publish(goal)  
        quat_goal=np.quaternion(self.curr_ori[0],self.curr_ori[1],self.curr_ori[2],self.curr_ori[3])

        while not rospy.is_shutdown(): 
   
            goal.pose.position.x = goal.pose.position.x + self.offset[0]
            goal.pose.position.y = goal.pose.position.y + self.offset[1]
            goal.pose.position.z = goal.pose.position.z + self.offset[2]
            alpha_beta_gamma=np.array([self.offset[3], self.offset[4], self.offset[5]])
            q_delta=from_euler_angles(alpha_beta_gamma)  
            quat_goal=q_delta*quat_goal

            goal.pose.orientation.w = quat_goal.w   
            goal.pose.orientation.x = quat_goal.x
            goal.pose.orientation.y = quat_goal.y
            goal.pose.orientation.z = quat_goal.z
        
            self.goal_pub.publish(goal)            
        
            self.control_freq.sleep()
        
    
#%%    
if __name__ == '__main__':
    rospy.init_node('Teleoperation', anonymous=True)

#%%
    teleoperation=Teleoperation()
    rospy.sleep(2)
    teleoperation.spacenav()