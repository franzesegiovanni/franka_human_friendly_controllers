import time
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class GripperController:
  # rosrun franka_gripper franka_gripper_online
  def read_pos(self, data):
    self.pos = data.position[0]

  def getState(self) -> int:
    return 0 if self.pos > 0.01 else 1 if self.pos > 0.001 else -1

  def open(self, dOpen=0.1): # open distance [m]
    self.gripper_pub.publish(dOpen)

    while True:
      if self.getState() == 0:
        time.sleep(0.5)
        return 0  # exit flag
      time.sleep(0.05)

  def close(self, dClose=0.0): # close distance [m]
    self.gripper_pub.publish(dClose)

    while True:
      if self.getState():
        time.sleep(0.5)
        return 0  # exit flag
      time.sleep(0.05)

  def __init__(self) -> None:
    rospy.Subscriber("/franka_gripper/joint_states", JointState, self.read_pos)
    self.gripper_pub = rospy.Publisher('/gripper_online', Float32, queue_size=10)
