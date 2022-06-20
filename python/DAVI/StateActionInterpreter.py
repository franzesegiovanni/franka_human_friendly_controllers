from DAVI import VariableImpedanceController
from GripperController import GripperController
from SoftReferences import PiecewiseLinearReference
from StateActionModel import StateActionModel
import numpy as np
import time, copy, rospy
from std_msgs.msg import Float32, Int16, String

class CupGrasper(GripperController):
  def getState(self) -> int:
    return 1 if self.pos < 0.001 else 0 if self.pos > 0.005 else -1

  def open(self, dOpen=0.02):
    return super().open(dOpen)


class ActionInterpreter():
  ori = np.array([0, np.sqrt(.5), np.sqrt(.5), 0])

  def __init__(self) -> None:
    # Make sure a ROS node is running!
    self.hapticPub = rospy.Publisher('/vibration', Float32, queue_size=1)
    self.controller = VariableImpedanceController()
    self.grasp = CupGrasper()
    time.sleep(1)

    xyz,_ = self.controller.get_current_pose()
    self.controller.reach_goal(PiecewiseLinearReference(None,[xyz],[self.ori]))
    self.grasp.open()
    self.controller.setPassive()

    print("Move robot to initial position and press Enter.")
    time.sleep(0.1)
    input()

    xyz0,_ = self.controller.get_current_pose()
    self.model = StateActionModel([xyz0])

    # Just for data recording
    self.statePub = rospy.Publisher('/scenario_abstract_state', Int16, queue_size=1)
    self.actionPub = rospy.Publisher('/scenario_abstract_action', String, queue_size=1)


  def waitForNextState(self, currState):
    dt = 1/30         # s time before checking again
    timeoutTime = 60  # s before stopping to check, > dt
    stateMargin = .1  # m distance considered close to a state
    eps = 1e-9        # floatingpoint error considered 0

    waitedTime = 0
    pose = self.controller.get_current_pose()[0]
    t = time.time()
    for _ in range(int(timeoutTime/dt)):
      state, d = self.model.findClosestState(self.controller.get_current_pose()[0])
      stateIdx = self.model.getStateIdx(state)

      if np.linalg.norm(state-currState) > eps and d <= stateMargin:
        self.model.updateActionImitationList(currState, 'go to {}'.format(stateIdx))
        self.hapticPub.publish(0.25)
        return state, stateIdx

      time.sleep(max(0,dt + t - time.time()))
      t = time.time()
      
      waitedTime = (waitedTime + dt) if np.linalg.norm(self.controller.get_current_pose()[0]-pose) < 1e-3 else 0
      pose = self.controller.get_current_pose()[0]

      # after quarter of a second of inactivity, assume arrival at unknown state
      if waitedTime > 0.25 and d > stateMargin:
        stateIdx = self.model.addNewState(pose)
      
      # after 5 seconds of inactivity, assume arrived at goal and ungrasp
      if 0<=(waitedTime-3.5)<dt or 0<=(waitedTime-4.)<dt or 0<=(waitedTime-4.5)<dt:
        self.hapticPub.publish(0.25)
      if waitedTime > 5.:
        self.model.addGoalState(state)
        break

    return state, stateIdx

  def doAction(self, action):
    state, sIdx = self.getStateAndIdx()
    self.statePub.publish(sIdx)
    self.grasp.close()
    print(action)
    self.actionPub.publish(action)
    if action == 'wait':
      self.controller.setPassive()
      state, sIdx = self.waitForNextState(state)
    else:
      goalPos = self.model.statesSet.data[int(action.split()[-1])][:3]
      exitflag = self.controller.reach_goal(PiecewiseLinearReference(None,[goalPos],[self.ori]), debugLevel=0)
      if exitflag == 1:
        print("Dropped to passive")
        self.controller.setPassive()
        state, sIdx = self.waitForNextState(state)
      else:
        state, sIdx = self.getStateAndIdx()
    self.statePub.publish(sIdx)

    if np.linalg.norm(state-self.controller.get_current_pose()[0]) > 2e-2:
      self.doAction('go to {}'.format(sIdx))

    exitCriterion = self.model.isGoalState(state)
    if exitCriterion:
      print('ungrasp')
      self.grasp.open()

    return state, self.model.getStateIdx(state), exitCriterion


  def getState(self):
    state = copy.deepcopy(self.model.findClosestState(self.controller.get_current_pose()[0])[0])
    return state

  def getStateIdx(self) -> int:
    return self.model.getStateIdx(self.getState())

  def getStateAndIdx(self):
    state = self.getState()
    return state, self.model.getStateIdx(state)