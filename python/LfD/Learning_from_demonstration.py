#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32
from pynput.keyboard import Listener, KeyCode
class LfD():
    def __init__(self):
        self.r=rospy.Rate(10)
        self.K_pos=600
        self.K_ori=30
        self.K_ns=10
        self.curr_pos=None
        self.width=None
        self.recorded_traj = None 
        self.recorded_ori=None
        self.recorded_gripper= None

        self.pos_sub=rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)
        self.gripper_sub=rospy.Subscriber("/joint_states", JointState, self.gripper_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.grip_pub = rospy.Publisher('/gripper_online', Float32, queue_size=0)
        self.stiff_pub = rospy.Publisher('/stiffness', Float32MultiArray, queue_size=0)
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True  
    def ee_pos_callback(self, data):
        self.curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.curr_ori = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
        #rospy.loginfo([data.x, data.y, data.z])
    def gripper_callback(self, data):
        self.width =data.position[7]+data.position[8]


    def traj_rec(self, rec_position=True, rec_orientation=True):
        # trigger for starting the recording
        stiff=[self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, self.K_ns]
        if rec_position==True: stiff[0:3]=0
        if rec_orientation==True: stiff[3:6]=0

        trigger = 0.05
        stiff_des = Float32MultiArray()
        stiff_des.data = np.array(stiff).astype(np.float32)
        self.stiff_pub.publish(stiff_des) 

        init_pos = self.curr_pos
        vel = 0
        print("Move robot to start recording.")
        while vel > trigger:
            vel = math.sqrt((self.curr_pos[0]-init_pos[0])**2 + (self.curr_pos[1]-init_pos[1])**2 + (self.curr_pos[2]-init_pos[2])**2)
        
        self.recorded_traj = self.curr_pos


        self.recorded_gripper= self.width
        key_pressed = False
        print("Recording started. Press e to stop.")
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
        interp_dist = 0.01  # [m]
        step_num = math.floor(dist / interp_dist)
        print("num of steps", step_num)
        x = np.linspace(start[0], goal_pose.pose.position.x, step_num)
        y = np.linspace(start[1], goal_pose.pose.position.y, step_num)
        z = np.linspace(start[2], goal_pose.pose.position.z, step_num)
        
        rot_x= np.linspace(start_ori[0], goal_pose.pose.orientation.x , step_num)
        rot_y= np.linspace(start_ori[1], goal_pose.pose.orientation.y , step_num)
        rot_z= np.linspace(start_ori[2], goal_pose.pose.orientation.z , step_num)
        rot_w= np.linspace(start_ori[3], goal_pose.pose.orientation.w, step_num)
        goal = PoseStamped()
        
        goal.pose.position.x = x[0]
        goal.pose.position.y = y[0]
        goal.pose.position.z = z[0]

        goal.pose.orientation.x = rot_x[0]
        goal.pose.orientation.y = rot_y[0]
        goal.pose.orientation.z = rot_z[0]
        goal.pose.orientation.w = rot_w[0]

        self.goal_pub.publish(goal)

        
        stiff_des = Float32MultiArray()

        stiff_des.data = np.array([self.K_pos, self.K_pos, self.K_pos, self.K_ori, self.K_ori, self.K_ori, self.K_ns]).astype(np.float32)
    #    print(stiff_des)
        self.stiff_pub.publish(stiff_des)
        goal = PoseStamped()
        for i in range(step_num):
            now = time.time()            # get the time
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = x[i]
            goal.pose.position.y = y[i]
            goal.pose.position.z = z[i]

            goal.pose.orientation.x = rot_x[i]
            goal.pose.orientation.y = rot_y[i]
            goal.pose.orientation.z = rot_z[i]
            goal.pose.orientation.w = rot_w[i]
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

            goal.pose.orientation.x = self.recorded_ori[0][i] 
            goal.pose.orientation.y = self.recorded_ori[1][i] 
            goal.pose.orientation.z = self.recorded_ori[2][i] 
            goal.pose.orientation.w = self.recorded_ori[3][i] 

            self.goal_pub.publish(goal)
            
            grip_command = Float32()

            grip_command.data = self.recorded_gripper[0][i]
            #print("Gripper:", grip_command.data)

            self.grip_pub.publish(grip_command) 

            self.r.sleep()

    #def start_ros(self):

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

        start.pose.orientation.x = LfD.recorded_ori[0][0] 
        start.pose.orientation.y = LfD.recorded_ori[1][0] 
        start.pose.orientation.z = LfD.recorded_ori[2][0] 
        start.pose.orientation.w = LfD.recorded_ori[3][0] 
        LfD.go_to_pose(start)

#%%
        LfD.execute()
#%%
