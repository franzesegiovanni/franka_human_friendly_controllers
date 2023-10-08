#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import quaternion # pip install numpy-quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped, Twist
from std_msgs.msg import Float32MultiArray
import dynamic_reconfigure.client


# from pose_transform_functions import  array_quat_2_pose
# from franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal, MoveActionGoal
class Panda():
    def __init__(self):
        super(Panda, self).__init__()
        # self.r=rospy.Rate(20)
        self.curr_pos=None
        self.curr_ori=None
        
        # Subscriber to get robot end effector pose in cartesian space
        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)

        # Publisher to send a new robot end effector pose goal/attractor in cartesian space
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        

        rospy.sleep(1)

    def ee_pos_callback(self, curr_conf):
        self.curr_pos = np.array([curr_conf.pose.position.x, curr_conf.pose.position.y, curr_conf.pose.position.z])
        self.curr_ori = np.array([curr_conf.pose.orientation.w, curr_conf.pose.orientation.x, curr_conf.pose.orientation.y, curr_conf.pose.orientation.z])


        
