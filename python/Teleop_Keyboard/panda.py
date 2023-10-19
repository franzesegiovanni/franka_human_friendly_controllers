#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import quaternion # pip install numpy-quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped, Twist
from std_msgs.msg import Float32MultiArray

class Panda():
    """
    Base class to interact with Franka Emika robot, through the 
    nodes and topics provided by franka ROS and the 
    franka_human_friendly_controller package
    """
    def __init__(self):
        super(Panda, self).__init__()

        # variables to store the cartesian pose of the robot end effector 
        self.curr_pos=None
        self.curr_ori=None
        
        # Subscriber to get robot end effector pose in cartesian space
        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)
        # x = PoseStamped.
        # Publisher to send a new robot end effector pose goal/attractor in cartesian space
        self.goal_pub = rospy.Publisher("/equilibrium_pose", PoseStamped, queue_size=0)

        rospy.sleep(1)

    def ee_pos_callback(self, curr_conf):
        """
        Callback function that updates the cartesian position of the Robots 
        end effector, as it changes, from the '/cartesian_pose' topic provided
        by franka_ros
        """
        # Set the current position 
        self.curr_pos = np.array([curr_conf.pose.position.x, curr_conf.pose.position.y, curr_conf.pose.position.z])
        # Set the current orientation 
        self.curr_ori = np.array([curr_conf.pose.orientation.w, curr_conf.pose.orientation.x, curr_conf.pose.orientation.y, curr_conf.pose.orientation.z])


        
