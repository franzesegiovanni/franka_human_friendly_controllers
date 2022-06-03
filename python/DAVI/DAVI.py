import numpy as np
import time
import quaternion as Q

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

from SoftReferences import PiecewiseLinearReference

class VariableImpedanceController:
  POS_MARGIN = 1e-3         # acceptable goal error [m]
  QUAT_MARGIN = np.pi/180   # acceptable goal error [rad]
  K_LIN_FIX = 1000          # linear stiffness if DoF is fixed [N/m]
  K_LIN_MAX = 600           # maximum linear stiffness [N/m]
  K_ANG_FIX = 25            # angular stiffness if DoF is fixed [Nm/rad]
  K_ANG_MAX = 15            # maximum angular stiffness [Nm/rad]

  CONTROL_FREQ = 30         # impedance control frequency [Hz]
  DT = 1/CONTROL_FREQ       # impedance control time step [s]
  DT_TRANSITION = 1.        # time of human resistance to reduce stiffness [s]
  DK_LIN_MAX = K_LIN_MAX*DT/DT_TRANSITION
  DK_ANG_MAX = K_ANG_MAX*DT/DT_TRANSITION

  NMA = 50                  # size of force/torque moving average filter

  last_forces_ = np.zeros((NMA,3))
  last_torques_ = np.zeros((NMA,3))
  stiffness = np.array([400, 400, 400, 30, 30, 30, 0])

  # For disagreement detection
  forceThreshold = 3.*np.ones(3)      # intraction force recognized as gR =/ gH [N]
  torqueThreshold = 1.5*np.ones(3)    # intraction torque recognized as gR =/ gH [Nm]
  posThreshold = 0.05                 # trajectory error recognized as gR =/ gH [m]
  angThreshold = np.pi/8              # angular error recognized as gR =/ gH [rad]


  def ee_pos_callback(self, data): 
    self.curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    curr_ori_arr = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
    if curr_ori_arr[0] < 0:
      curr_ori_arr *= -1
    self.curr_ori = Q.from_float_array(curr_ori_arr)

  def read_conf(self, data):
    self.curr_conf = np.array(data.position[:7])
    self.curr_cVel = np.array(data.velocity[:7])
      
  def read_force(self, data):
    self.last_forces_ = np.append([[data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]], self.last_forces_[:-1], axis=0)
    self.force = np.mean(self.last_forces_, axis=0)

    self.last_torques_ = np.append([[data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]], self.last_torques_[:-1], axis=0)
    self.torque = np.mean(self.last_torques_, axis=0)

  def __init__(self, eeCtrlDOF=3) -> None:
    self.eeDOF = eeCtrlDOF

    self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
    self.stiff_pub = rospy.Publisher('/stiffness', Float32MultiArray, queue_size=0) #in this vector we can send [x, y, z, rotX, rotY, rotZ, ns] stiffness
    self.activeControl = True

    rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)
    rospy.Subscriber("/joint_states", JointState, self.read_conf)
    rospy.Subscriber("/force_torque_ext", WrenchStamped , self.read_force)
    time.sleep(1)

    self.setForceTransform(lambda f,t:f)

  def setForceTransform(self, transform:np.ndarray):
    self.forceAtHuman = transform

  def reach_goal(self, trajectory:PiecewiseLinearReference, varStiff:bool=True, debugLevel:int=1):
    trajectory.initialize(self.curr_pos, self.curr_ori)

    def isAtGoal(relPrecision:float=1.):
      diffX, diffQ = trajectory.getDiff2Goal(self.curr_pos, self.curr_ori)
      return diffX < self.POS_MARGIN*relPrecision and diffQ < self.QUAT_MARGIN*relPrecision

    def getDeltaK(disagreementMode:str='force'):
      deltaK_lin = self.DK_LIN_MAX
      deltaK_ang = self.DK_ANG_MAX

      if 'force' in disagreementMode:
        forceThreshold = np.linalg.norm(self.forceThreshold)
        torqueThreshold = np.linalg.norm(self.torqeThreshold)
        normForce = np.linalg.norm(self.force)
        normTorque = np.linalg.norm(self.torque)
        if normForce > forceThreshold or normTorque > torqueThreshold:
          if debugLevel >= 1 and normForce > forceThreshold:
            print('High force detected: {0} N'.format(normForce))
          if debugLevel >= 1 and normTorque > torqueThreshold:
            print('High torque detected: {0} Nm'.format(normTorque))
          deltaK_lin = min(-self.DK_LIN_MAX, deltaK_lin)
          deltaK_ang = min(-self.DK_ANG_MAX, deltaK_ang)

      if 'position' in disagreementMode:
        deltaX = np.linalg.norm(xSetpoint-self.curr_pos)
        deltaAng = Q.rotation_intrinsic_distance(qSetpoint, self.curr_ori)
        if deltaX > self.posThreshold or deltaAng > self.angThreshold:
          if debugLevel >= 1 and deltaX > self.posThreshold:
            print('Large distance error: {0} m'.format(deltaX))
          if debugLevel >= 1 and deltaAng > self.angThreshold:
            print('Large angular error: {0} rad'.format(deltaAng))
          deltaK_lin = min(-self.DK_LIN_MAX, deltaK_lin)
          deltaK_ang = min(-self.DK_ANG_MAX, deltaK_ang)

      if 'energy' in disagreementMode:
        if self.eeDOF > 3:
          print('Warning: energy-based disagreement detection responding to force only')

        forceThreshold = 2*self.forceThreshold*np.ones(3)   # using a higher force threshold(!) w.r.t. force disagreement
        K_inv = np.diag([1/s if s > 0 else 0 for s in self.stiffness[:3]])
        energyThreshold = np.dot(forceThreshold, np.dot(K_inv, forceThreshold))
        force = self.force
        energy = np.dot(force, np.dot(K_inv, force))
        relEnergy = (energy-energyThreshold)/energyThreshold if energyThreshold > 0 else 0
        if np.linalg.norm(self.force) > np.linalg.norm(forceThreshold):
          if debugLevel >= 1:
            print('Energy level {:.2f} times above threshold; force = {:.2f} N (K={:.0f})'.format(relEnergy, np.linalg.norm(self.force), self.stiffness[0]))
          deltaK_lin = -relEnergy*self.DK_LIN_MAX
          deltaK_ang = -relEnergy*self.DK_ANG_MAX

      return deltaK_lin, deltaK_ang

    # control loop
    r = rospy.Rate(self.CONTROL_FREQ)

    goal = PoseStamped()
    stiff_des = Float32MultiArray()

    noMotionIter = 0
    old_pose = [self.curr_pos, self.curr_ori]

    self.activeControl = True
    stiff_lin = self.K_LIN_MAX
    stiff_ang = self.K_ANG_MAX
    if debugLevel >= 1:
      print("Active control switched on")

    while not isAtGoal():
      # compute setpoint
      xSetpoint, qSetpoint = trajectory.getSetpoint(self.DT, self.curr_pos, self.curr_ori)

      if varStiff:
        deltaK_lin, deltaK_ang = getDeltaK()

        stiff_lin = min(max(0., stiff_lin+deltaK_lin), self.K_LIN_MAX)
        stiff_ang = min(max(0., stiff_lin+deltaK_ang), self.K_ANG_MAX)
      
      # Correct stiffness to avoid sudden increase
      if stiff_lin > self.stiffness[0] + self.DK_LIN_MAX:
        stiff_lin = self.stiffness[0] + self.DK_LIN_MAX
      if stiff_ang > self.stiffness[5] + self.DK_ANG_MAX:
        stiff_ang = self.stiffness[5] + self.DK_ANG_MAX

      # send goal position, angle
      goal.pose.position.x = xSetpoint[0]
      goal.pose.position.y = xSetpoint[1]
      goal.pose.position.z = xSetpoint[2]

      goal.pose.orientation.x = qSetpoint.x
      goal.pose.orientation.y = qSetpoint.y
      goal.pose.orientation.z = qSetpoint.z
      goal.pose.orientation.w = qSetpoint.w

      if debugLevel >= 2:
        print("Goal:")
        print(goal.pose)
      self.goal_pub.publish(goal)

      if self.eeDOF < 4:
        stiff_ang = 25.
      self.stiffness = np.array([stiff_lin, stiff_lin, stiff_lin, self.K_ANG_FIX, self.K_ANG_FIX, stiff_ang, 0.])
      stiff_des.data = self.stiffness.astype(np.float32)
      if debugLevel >= 2:
        print("Stiffness:")
        print(stiff_des)
      self.stiff_pub.publish(stiff_des)

      #use rate of ROS
      r.sleep()

      # exit when stiffness is dropped
      if stiff_lin == 0 and (self.eeDOF < 4 or stiff_ang == 0):
        self.activeControl = False
        if debugLevel >= 1:
          print("Active control switched off because of zero stiffness")
        # self.stiffness = np.array([0., 0., 0., 25., 25., 0., 0.])
        return 1

      # exit when standing still
      if np.linalg.norm(self.curr_pos - old_pose[0]) < self.POS_MARGIN and Q.rotation_intrinsic_distance(self.curr_ori, old_pose[1]) < self.QUAT_MARGIN:
        noMotionIter += 1
        if noMotionIter > 0.5/self.DT: # 0.5 sec
          return 0 if isAtGoal(10) else 2
      else:
        noMotionIter = 0
      old_pose = [self.curr_pos, self.curr_ori]

    return 0

  def setPassive(self, debugLevel=0):
    self.activeControl = False
    if debugLevel >= 1:
      print("Active control switched off in passive mode")

    self.stiffness = np.array([0., 0., 0., self.K_ANG_FIX, self.K_ANG_FIX, self.K_ANG_FIX if self.eeDOF<4 else 0., 0.])
    stiff_des = Float32MultiArray()
    stiff_des.data = self.stiffness.astype(np.float32)
    if debugLevel >= 1:
      print("Stiffness:")
      print(stiff_des)
    self.stiff_pub.publish(stiff_des)
    return 0

  def get_current_pose(self) -> np.array:
    return self.curr_pos, Q.as_float_array(self.curr_ori)

#%%