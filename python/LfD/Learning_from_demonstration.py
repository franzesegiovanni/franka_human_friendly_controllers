#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import quaternion # pip install numpy-quaternion
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32
import dynamic_reconfigure.client
from pynput.keyboard import Listener, KeyCode
class LfD():
    def __init__(self):
        self.r=rospy.Rate(100)
        self.K_pos=600
        self.K_ori=30
        self.K_ns=10
        self.curr_pos=None
        self.curr_ori=None
        self.width=None
        self.recorded_traj = None 
        self.recorded_ori=None
        self.recorded_gripper= None
        self.end=False
        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)
        self.gripper_sub=rospy.Subscriber("/joint_states", JointState, self.gripper_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.grip_pub = rospy.Publisher('/gripper_online', Float32, queue_size=0)
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True  
    def ee_pos_callback(self, data):
        self.curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.curr_ori = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
        #rospy.loginfo([data.x, data.y, data.z])
    def gripper_callback(self, data):
        self.width =data.position[7]+data.position[8]

    
    def set_stiffness(self, k_t1, k_t2, k_t3,k_r1,k_r2,k_r3, k_ns):
        
        set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        set_K.update_configuration({"translational_stiffness_X": k_t1})
        set_K.update_configuration({"translational_stiffness_Y": k_t2})
        set_K.update_configuration({"translational_stiffness_Z": k_t3})        
        set_K.update_configuration({"rotational_stiffness_X": k_r1}) 
        set_K.update_configuration({"rotational_stiffness_Y": k_r2}) 
        set_K.update_configuration({"rotational_stiffness_Z": k_r3})
        set_K.update_configuration({"nullspace_stiffness": k_ns})   

    def traj_rec(self, trigger=0.005, rec_position=True, rec_orientation=True):
        # trigger for starting the recording
        if rec_position==True: 
            set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
            set_K.update_configuration({"translational_stiffness_X": 0})
            set_K.update_configuration({"translational_stiffness_Y": 0})
            set_K.update_configuration({"translational_stiffness_Z": 0})  
        if rec_orientation==True: 
            set_K.update_configuration({"rotational_stiffness_X": 0}) 
            set_K.update_configuration({"rotational_stiffness_Y": 0}) 
            set_K.update_configuration({"rotational_stiffness_Z": 0})

        init_pos = self.curr_pos
        vel = 0
        print("Move robot to start recording.")
        while vel < trigger:
            vel = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
        
        self.recorded_traj = self.curr_pos
        self.recorded_ori = self.curr_ori
        self.recorded_gripper= self.width
        key_pressed = False
        print("Recording started. Press e to stop.")
        self.end=False
        while not self.end:
            now = time.time()            # get the time

            self.recorded_traj = np.c_[self.recorded_traj, self.curr_pos]
            self.recorded_ori  = np.c_[self.recorded_ori, self.curr_ori]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.width]

            self.r.sleep()

    # control robot to desired goal position
    def go_to_pose(self, goal_pose):
        # the goal pose should be of type PoseStamped. E.g. goal_pose=PoseStampled()
        start = self.curr_pos
        start_ori=self.curr_ori
        goal_=np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])
        # interpolate from start to goal with attractor distance of approx 1 cm
        squared_dist = np.sum(np.subtract(start, goal_)**2, axis=0)
        dist = np.sqrt(squared_dist)
        print("dist", dist)
        interp_dist = 0.001  # [m]
        step_num_lin = math.floor(dist / interp_dist)

        
        print("num of steps linear", step_num_lin)
        
        
        q_start=np.quaternion(start_ori[0], start_ori[1], start_ori[2], start_ori[3])
        print("q_start", q_start)
        q_goal=np.quaternion(goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z)
        inner_prod=q_start.x*q_goal.x+q_start.y*q_goal.y+q_start.z*q_goal.z+q_start.w*q_goal.w
        if inner_prod < 0:
            q_start.x=-q_start.x
            q_start.y=-q_start.y
            q_start.z=-q_start.z
            q_start.w=-q_start.w
        inner_prod=q_start.x*q_goal.x+q_start.y*q_goal.y+q_start.z*q_goal.z+q_start.w*q_goal.w
        theta= np.arccos(np.abs(inner_prod))
        print(theta)
        interp_dist_polar = 0.001 
        step_num_polar = math.floor(theta / interp_dist_polar)

        
        print("num of steps polar", step_num_polar)
        
        step_num=np.max([step_num_polar,step_num_lin])
        
        print("num of steps", step_num)
        x = np.linspace(start[0], goal_pose.pose.position.x, step_num)
        y = np.linspace(start[1], goal_pose.pose.position.y, step_num)
        z = np.linspace(start[2], goal_pose.pose.position.z, step_num)
        
        goal = PoseStamped()
        
        goal.pose.position.x = x[0]
        goal.pose.position.y = y[0]
        goal.pose.position.z = z[0]
        
        
        quat=np.slerp_vectorized(q_start, q_goal, 0.0)
        goal.pose.orientation.x = quat.x
        goal.pose.orientation.y = quat.y
        goal.pose.orientation.z = quat.z
        goal.pose.orientation.w = quat.w

        self.goal_pub.publish(goal)

        self.set_stiffness(self.K_pos, self.K_pos, self.K_pos ,self.K_ori,self.K_ori,self.K_ori, 0.0)

        goal = PoseStamped()
        for i in range(step_num):
            now = time.time()         
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = x[i]
            goal.pose.position.y = y[i]
            goal.pose.position.z = z[i]
            quat=np.slerp_vectorized(q_start, q_goal, i/step_num)
            #print("quat", quat) 
            goal.pose.orientation.x = quat.x
            goal.pose.orientation.y = quat.y
            goal.pose.orientation.z = quat.z
            goal.pose.orientation.w = quat.w
            self.goal_pub.publish(goal)
            self.r.sleep()   


    def execute(self):
        for i in range (self.recorded_traj.shape[1]):
            goal = PoseStamped()

            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = self.recorded_traj[0][i] 
            goal.pose.position.y = self.recorded_traj[1][i]
            goal.pose.position.z = self.recorded_traj[2][i]

            goal.pose.orientation.w = self.recorded_ori[0][i] 
            goal.pose.orientation.x = self.recorded_ori[1][i] 
            goal.pose.orientation.y = self.recorded_ori[2][i] 
            goal.pose.orientation.z = self.recorded_ori[3][i] 

            self.goal_pub.publish(goal)
            
            grip_command = Float32()

            grip_command.data = self.recorded_gripper[0][i]
            #print("Gripper:", grip_command.data)

            self.grip_pub.publish(grip_command) 

            self.r.sleep()

#%%
if __name__ == '__main__':
    rospy.init_node('LfD', anonymous=True)
#%%    
    LfD=LfD()

#%%

    LfD.traj_rec() 

#%%
    start = PoseStamped()
    
    start.pose.position.x = LfD.recorded_traj[0][0]
    start.pose.position.y = LfD.recorded_traj[1][0]
    start.pose.position.z = LfD.recorded_traj[2][0]

    start.pose.orientation.w = LfD.recorded_ori[0][0] 
    start.pose.orientation.x = LfD.recorded_ori[1][0] 
    start.pose.orientation.y = LfD.recorded_ori[2][0] 
    start.pose.orientation.z = LfD.recorded_ori[3][0] 
    LfD.go_to_pose(start)

#%%
    LfD.execute()
#%%
