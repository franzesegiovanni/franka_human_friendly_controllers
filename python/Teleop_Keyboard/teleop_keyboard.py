import rospy
import math
import numpy as np
import time
from geometry_msgs.msg import PoseStamped, Pose, Twist
from pynput.keyboard import Listener
from pose_transform_functions import  array_quat_2_pose, list_2_quaternion, position_2_array, transform_pose
from panda import Panda



class TeleopKeyboard(Panda):
    def __init__(self):
        rospy.init_node("teleop_node")
        super(TeleopKeyboard, self).__init__()
        self.r=rospy.Rate(20)

        self.key_value = None
        # Subscriber to keyboard from teleop_twist_keyboard package
        self.key_sub=rospy.Subscriber("/key_vel", Twist, self.keyboard_read_callback)
        
        rospy.spin()



    def keyboard_read_callback(self, key_input):
        self.key_value = key_input
        print(self.key_value) 
        if self.key_value.linear.x == 0.8:
            # Set this to the current value as it needs some orientation value to publish to /equilibrium_pose
            quat_goal = list_2_quaternion(self.curr_ori)
            # Get current postion and add 5 mm to the z axis
            goal = array_quat_2_pose(self.curr_pos + np.array([0.0, 0.0, 0.005]), quat_goal)
            self.goal_pub.publish(goal)
            self.key_value.linear.x = 0.0 
            print("moved Robot up 5 mm")

        elif self.key_value.linear.x == -0.5:
            quat_goal = list_2_quaternion(self.curr_ori)
            goal = array_quat_2_pose(self.curr_pos + np.array([0.0, 0.0, -0.005]), quat_goal)
            self.goal_pub.publish(goal)
            self.key_value.linear.x = 0.0 
            print("moved Robot down 5 mm")


        if self.key_value.angular.z == 1.0:
            quat_goal = list_2_quaternion(self.curr_ori)
            goal = array_quat_2_pose(self.curr_pos + np.array([0.0, 0.005, 0.0]), quat_goal)
            self.goal_pub.publish(goal)
            self.key_value.angular.z = 0.0
            print("moved Robot left 5 mm")
        
        elif self.key_value.angular.z == -1.0:
            quat_goal = list_2_quaternion(self.curr_ori)
            goal = array_quat_2_pose(self.curr_pos + np.array([0.0, -0.005, 0.0]), quat_goal)
            self.goal_pub.publish(goal)
            self.key_value.angular.z = 0.0
            print("moved Robot right 5 mm")
        # rospy.sleep(1)


