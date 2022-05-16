import numpy as np
import quaternion as Q

# General class for soft reference tracking
class SoftReference:
  alpha_err_x = 10.
  alpha_err_q = 50.

  tRefTraj = None
  xRefTraj = None
  qRefTraj = None

  def __init__(self, tRef:np.ndarray, xRef:np.ndarray, qRef:np.array):
    self.tRefTraj = tRef
    self.xRefTraj = xRef
    self.qRefTraj = qRef

    self.xGoal = xRef[-1]
    self.qGoal = qRef[-1]

  def getSetpoint(self, dt:float, x:np.array, q:Q.quaternion):
    correctionPQ = [1+self.alpha_err_x*np.linalg.norm(x-self.xRef),
                    1+self.alpha_err_q*Q.rotation_intrinsic_distance(q, self.qRef)]
    self.tRef += dt/(np.prod(correctionPQ)/np.mean(correctionPQ))
    if self.tRef < self.tRefTraj[-1]:
      self.xRef = np.array([np.interp(self.tRef, self.tRefTraj, self.xRefTraj[:,j]) for j in range(3)])

      tNextIdx = np.where(self.tRefTraj > self.tRef)[0][0]
      tPrevIdx = tNextIdx-1
      self.qRef = Q.slerp(self.qRefTraj[tPrevIdx], self.qRefTraj[tNextIdx], self.tRefTraj[tPrevIdx], self.tRefTraj[tNextIdx], self.tRef)
    else:
      self.xRef = self.xRefTraj[-1]
      self.qRef = self.qRefTraj[-1]
    return self.xRef, self.qRef


# Defining trajectories as straight lines connecting a small number of via-points
class PiecewiseLinearReference(SoftReference):
  v = 0.3               # desired linear velocity [m/s]
  w = np.pi/2           # desired angular velocity [rad/s]

  # Define trajectory by the points the robot should pass through
  def __init__(self, tWaypoints:np.ndarray, xyzWaypoints:np.ndarray, oriWaypoints:np.ndarray):
    tRefTraj = np.array([0])
    xRefTraj = xyzWaypoints
    qRefTraj = np.array([Q.from_float_array(o) for o in oriWaypoints])

    lenX = len(xRefTraj)
    lenQ = len(qRefTraj)
    if lenX != lenQ:
      raise Exception("Trajectory positions and orientations mut have same length. Found: len(pos) = {}, len(ori) = {}".format(lenX, lenQ))

    for i in range(1,lenX):
      xDist = np.linalg.norm(xRefTraj[i-1]-xRefTraj[i])
      angDist = Q.rotation_intrinsic_distance(qRefTraj[i-1], qRefTraj[i])
      tRefTraj = np.append(tRefTraj, [tRefTraj[-1] + max(xDist/self.v, angDist/self.w)])

    super().__init__(tRefTraj, xRefTraj, qRefTraj)

  # Initialize reference trajectory to start from the latest observed robot pose
  def initialize(self, x0:np.ndarray, q0:Q.quaternion):
    xDist = np.linalg.norm(x0-self.xRefTraj[0])
    angDist = Q.rotation_intrinsic_distance(q0, self.qRefTraj[0])
    dt = max(xDist/self.v, angDist/self.w)

    self.tRefTraj = np.append([0], dt+self.tRefTraj)
    self.xRefTraj = np.append([x0], self.xRefTraj, axis=0)
    self.qRefTraj = np.append([q0], self.qRefTraj, axis=0)

    self.tRef = 0
    self.xRef = x0
    self.qRef = q0

  def getDiff2Goal(self, xyzRobot, qRobot):
    return np.linalg.norm(xyzRobot-self.xGoal), Q.rotation_intrinsic_distance(qRobot,self.qGoal)

  def setVelocities(self, v:float, w:float):
    self.v = v
    self.w = w
