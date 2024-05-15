#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import time
import os
from geometry_msgs.msg import PoseStamped, Pose
from pynput.keyboard import Listener, KeyCode, Key
from pose_transform_functions import  array_quat_2_pose, list_2_quaternion, position_2_array, transform_pose
from panda import Panda

class LfD(Panda):
    def __init__(self):
        rospy.init_node("learning_node")
        super(LfD, self).__init__()
        self.r=rospy.Rate(20)
        self.pose = Pose()
        self.recorded_traj = None 
        self.recorded_ori=None
        self.recorded_gripper= None
        self.end=False
        self.attractor_distance_threshold=0.05 # If the attractor is larger than the treshold, the robot will wait
        self.max_gripper_force= 2

        self.gripper_close_width = 0
        self.gripper_open_width  = 0.06
        self.gripper_sensitivity= 0.03

        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == Key.esc:
            self.end = True

    def traj_rec(self, trigger=0.005, rec_position=True, rec_orientation=True):
        # trigger for starting the recording
        if rec_position: 
            self.set_K.update_configuration({"translational_stiffness_X": 0})
            self.set_K.update_configuration({"translational_stiffness_Y": 0})
            self.set_K.update_configuration({"translational_stiffness_Z": 0})
        if rec_orientation: 
            self.set_K.update_configuration({"rotational_stiffness_X": 0})
            self.set_K.update_configuration({"rotational_stiffness_Y": 0})
            self.set_K.update_configuration({"rotational_stiffness_Z": 0})
        self.set_K.update_configuration({"nullspace_stiffness": 0})  
        self.end=False
        init_pos = self.curr_pos
        robot_perturbation = 0
        print("Move robot to start recording.")
        while robot_perturbation < trigger:
            robot_perturbation = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
        
        self.recorded_traj = self.curr_pos
        self.recorded_ori = self.curr_ori
        self.recorded_gripper = self.gripper_open_width

        
        print("Recording started. Press ESC to stop.")
        while not self.end:
            if self.gripper_width < (self.gripper_open_width - self.gripper_sensitivity):
                # print("Close gripper")
                self.grip_value = 0 #Close the gripper
            else:
                # print("Open gripper")
                self.grip_value = self.gripper_open_width #Open the gripper

            self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
            self.recorded_ori  = np.c_[self.recorded_ori, self.curr_ori]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.grip_value]

            
            self.r.sleep()

        quat_goal = list_2_quaternion(self.curr_ori)
        goal = array_quat_2_pose(self.curr_pos, quat_goal)
        self.goal_pub.publish(goal)
        self.set_stiffness(100, 100, 100, 5, 5, 5, 0)
        rospy.loginfo("Ending trajectory recording")
    
    def execute(self):
        self.set_stiffness(1000, 1000, 1000, 30, 30, 30, 0)
        start = PoseStamped()
        self.grasp_gripper(self.recorded_gripper[0][0], self.max_gripper_force)
        quat_start = list_2_quaternion(self.recorded_ori[:, 0])
        start = array_quat_2_pose(self.recorded_traj[:, 0], quat_start)
        
        self.go_to_pose(start)

        self.time_index=0
        while self.time_index <( self.recorded_traj.shape[1]):

            quat_goal = list_2_quaternion(self.recorded_ori[:, self.time_index])
            goal = array_quat_2_pose(self.recorded_traj[:, self.time_index], quat_goal)
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()

                   
            if (self.recorded_gripper[0][self.time_index]-self.recorded_gripper[0][max([0,self.time_index-1])]) < -self.gripper_sensitivity:
                print("closing gripper")
                self.grasp_gripper(self.recorded_gripper[0][self.time_index], self.max_gripper_force)
                time.sleep(0.1)

            if (self.recorded_gripper[0][self.time_index]-self.recorded_gripper[0][max([0,self.time_index-1])]) > self.gripper_sensitivity:
                print("open gripper")
                self.move_gripper(self.recorded_gripper[0][self.time_index])
                time.sleep(0.1)

            self.goal_pub.publish(goal)

            # Safety feature in case somebody is touching the robot during execution
            goal_pos_array = position_2_array(goal.pose.position)
            if np.linalg.norm(self.curr_pos-goal_pos_array) <= self.attractor_distance_threshold:
                self.time_index=self.time_index+1
            self.r.sleep()

    def save(self, name='demo'):
        curr_dir=os.getcwd()
        np.savez(curr_dir+ '/data/' + str(name) + '.npz',
                 traj=self.recorded_traj,
                 ori=self.recorded_ori,
                 grip=self.recorded_gripper)

    def load(self, name='demo'):
        curr_dir=os.getcwd()
        data = np.load(curr_dir+ '/data/' + str(name) + '.npz')
        self.recorded_traj = data['traj']
        self.recorded_ori = data['ori']
        self.recorded_gripper = data['grip']
        self.filename=str(name)
