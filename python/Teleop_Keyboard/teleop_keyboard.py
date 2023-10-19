import rospy
import math
import numpy as np
import time
from geometry_msgs.msg import PoseStamped, Pose, Twist
from pynput.keyboard import Listener
from pose_transform_functions import  array_quat_2_pose, list_2_quaternion, position_2_array, transform_pose
from panda import Panda



class TeleopKeyboard(Panda):
    """
    A class that allows a user to control the cartesian pose of the Franka Emika robots through 
    a key board, that builds on top of the funcionality provided by the `Panda` class,
    which provides an interface to the Franka Emika robots. This is done through the `teleop_twist_keyboard`
    package.
    """
    def __init__(self):
        # start up a ROS node
        rospy.init_node("teleop_node")
        super(TeleopKeyboard, self).__init__()

        # Set the node frequency - twice the frequency of the /key_vel topic
        self.r=rospy.Rate(20)
        
        # Variable to store the current key value.
        self.key_value = None

        # Subscriber to keyboard from teleop_twist_keyboard package
        self.key_sub=rospy.Subscriber("/cmd_vel", Twist, self.keyboard_read_callback)
        
        # Amount to move the robot 
        self.move_distance = 0.005 
        # This funciton is needed to keep the node running 
        rospy.spin()



    def keyboard_read_callback(self, key_input):
        """
        Callback function that changes the robots end effector cartesian pose when 
        one of the arrow keys are pressed. It changes the equilibrium pose by 1mm, given the key pressed
        and direction assigned to that key.
        
        Key - Action         
        'i' = + z-axis
        ',' = - z-axis
        'j' = + y-axis
        'l' = - y-axis
        'u' = + x-axis
        'o' = - x-axis
        """
        self.key_value = key_input
        print(self.key_value) 

        # + Z axis
        if self.key_value.linear.x == 0.5 and self.key_value.angular.z == 0.0:
            # Set this to the current value as it needs some orientation value to publish to /equilibrium_pose
            quat_goal = list_2_quaternion(self.curr_ori)
            # Get current postion and add 1 mm to the z axis
            goal = array_quat_2_pose(self.curr_pos + np.array([0.0, 0.0, self.move_distance]), quat_goal)
            self.goal_pub.publish(goal)
            self.key_value.linear.x = 0.0 
            print("moved Robot up 5 mm")

        # - Z axis
        elif self.key_value.linear.x == -0.5 and self.key_value.angular.z == 0.0:
            # Set this to the current value as it needs some orientation value to publish to /equilibrium_pose
            quat_goal = list_2_quaternion(self.curr_ori)
            # Get current postion and add 1 mm to the z axis
            goal = array_quat_2_pose(self.curr_pos + np.array([0.0, 0.0, -self.move_distance]), quat_goal)
            self.goal_pub.publish(goal)
            self.key_value.linear.x = 0.0 
            print("moved Robot down 1 mm")


        # + Y axis
        elif self.key_value.linear.x == 0.0 and self.key_value.angular.z == 1.0:
            # Set this to the current value as it needs some orientation value to publish to /equilibrium_pose
            quat_goal = list_2_quaternion(self.curr_ori)
            # Get current postion and add 1 mm to the z axis
            goal = array_quat_2_pose(self.curr_pos + np.array([0.0, self.move_distance, 0.0]), quat_goal)
            self.goal_pub.publish(goal)
            self.key_value.angular.z = 0.0
            print("moved Robot left 1 mm")
        
        # - Y axis
        elif self.key_value.linear.x == 0.0 and self.key_value.angular.z == -1.0:
            # Set this to the current value as it needs some orientation value to publish to /equilibrium_pose
            quat_goal = list_2_quaternion(self.curr_ori)
            # Get current postion and add 1 mm to the z axis
            goal = array_quat_2_pose(self.curr_pos + np.array([0.0, -self.move_distance, 0.0]), quat_goal)
            self.goal_pub.publish(goal)
            self.key_value.angular.z = 0.0
            print("moved Robot right 1 mm")


        # + X axis
        elif self.key_value.linear.x == 0.5 and self.key_value.angular.z == 1.0:
            # Set this to the current value as it needs some orientation value to publish to /equilibrium_pose
            quat_goal = list_2_quaternion(self.curr_ori)
            # Get current postion and add 1 mm to the z axis
            goal = array_quat_2_pose(self.curr_pos + np.array([self.move_distance, 0.0, 0.0]), quat_goal)
            self.goal_pub.publish(goal)
            self.key_value.angular.z = 0.0
            print("moved Robot left 1 mm")
        
        # - X axis
        elif self.key_value.linear.x == 0.0 and self.key_value.angular.z == -1.0:
            # Set this to the current value as it needs some orientation value to publish to /equilibrium_pose
            quat_goal = list_2_quaternion(self.curr_ori)
            # Get current postion and add 1 mm to the z axis
            goal = array_quat_2_pose(self.curr_pos + np.array([-self.move_distance, 0.0, 0.0]), quat_goal)
            self.goal_pub.publish(goal)
            self.key_value.angular.z = 0.0
            print("moved Robot right 1 mm")


