import numpy as np
import time
import quaternion as Q

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

from SoftReferences import PiecewiseLinearReference

class VariableImpedanceController:
  NMA = 50
  last_forces_ = np.zeros((NMA,3))
  last_torques_ = np.zeros((NMA,3))
  stiffness = np.array([400, 400, 400, 30, 30, 30, 0])


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
    xMargin = 1e-3        # acceptable goal error [m]
    qMargin = np.pi/180   # acceptable goal error [rad]
    xErr = 0.05           # trajectory error recognized as gR =/ gH [m]
    angErr = np.pi/4      # angular error recognized as gR =/ gH [rad]
    fErr = 5.             # intraction force recognized as gR =/ gH [N]
    tErr = 2.5            # intraction torque recognized as gR =/ gH [N]
    K_lin_max=600
    K_ang_max=15
    T_reduce_stiff = 1.   # time of human resistance to drop stiffness [s]

    trajectory.initialize(self.curr_pos, self.curr_ori)

    def isAtGoal(relPrecision:float=1.):
      diffX, diffQ = trajectory.getDiff2Goal(self.curr_pos, self.curr_ori)
      return diffX < xMargin*relPrecision and diffQ < qMargin*relPrecision

    # control loop
    control_frequency = 30
    dt = 1/control_frequency
    r = rospy.Rate(control_frequency)

    maxDeltaK = K_lin_max*dt/T_reduce_stiff
    maxDeltaKappa = K_ang_max*dt/T_reduce_stiff
    
    goal = PoseStamped()
    stiff_des = Float32MultiArray()

    interactionStiffness=1.
    noMotionIter = 0
    old_pose = [self.curr_pos, self.curr_ori]

    self.activeControl = True
    if debugLevel >= 1:
      print("Active control switched on")

    while not isAtGoal():
      # compute setpoint
      xSetpoint, qSetpoint = trajectory.getSetpoint(dt, self.curr_pos, self.curr_ori)

      if debugLevel >= 1:
        normForce = np.linalg.norm(self.force)
        if (normForce) > fErr:
          print('High force detected: {0} N'.format(normForce))

        normTorque = np.linalg.norm(self.torque)
        if (normTorque) > tErr:
          print('High torque detected: {0} Nm'.format(normTorque))

        deltaX = np.linalg.norm(xSetpoint-self.curr_pos)
        if (deltaX) > xErr:
          print('Large distance error: {0} m'.format(deltaX))

        deltaAng = Q.rotation_intrinsic_distance(qSetpoint, self.curr_ori)
        if (deltaAng) > angErr:
          print('Large angular error: {0} rad'.format(deltaAng))

      if varStiff:
        # check if stiffness needs to be reduced
        # reduce_stiff = np.linalg.norm(self.force) > fErr or np.linalg.norm(xSetpoint-self.curr_pos) > xErr or np.linalg.norm(self.torque) > tErr or Q.rotation_intrinsic_distance(qSetpoint, self.curr_ori) > angErr
        if np.linalg.norm(self.force) > fErr or np.linalg.norm(self.torque) > tErr:
          interactionStiffness = max(interactionStiffness-dt/T_reduce_stiff, 0)
        elif interactionStiffness > 0:
          interactionStiffness = min(interactionStiffness+dt/T_reduce_stiff, 1)

        stiff_lin = K_lin_max*interactionStiffness
        stiff_ang = K_ang_max*interactionStiffness
      
      # Correct stiffness to avoid sudden increase
      if stiff_lin > self.stiffness[0] + maxDeltaK:
        stiff_lin = self.stiffness[0] + maxDeltaK
      if stiff_ang > self.stiffness[5] + maxDeltaKappa:
        stiff_ang = self.stiffness[5] + maxDeltaKappa

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
      self.stiffness = np.array([stiff_lin, stiff_lin, stiff_lin, 25., 25., stiff_ang, 0.])
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
      if np.linalg.norm(self.curr_pos - old_pose[0]) < xMargin and Q.rotation_intrinsic_distance(self.curr_ori, old_pose[1]) < qMargin:
        noMotionIter += 1
        if noMotionIter > 0.5/dt: # 0.5 sec
          return 0 if isAtGoal(10) else 2
      else:
        noMotionIter = 0
      old_pose = [self.curr_pos, self.curr_ori]

    return 0

  def setPassive(self, debugLevel=0):
    self.activeControl = False
    if debugLevel >= 1:
      print("Active control switched off in passive mode")

    self.stiffness = np.array([0.0, 0.0, 0.0, 25.0, 25.0, 25.0 if self.eeDOF<4 else 0.0, 0.0])
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