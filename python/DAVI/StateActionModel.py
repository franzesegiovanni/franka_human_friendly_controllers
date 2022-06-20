import numpy as np
from scipy.spatial import KDTree

class StateActionModel():
  # Initializing model
  # In: initial set of know states        - numpy array [[x,y,z], ...]
  #     (optional) initial set of actions - list of tuples [(stateKey, action)]
  #     (optional) initial set of terminal states
  def __init__(self, statesList:np.ndarray, actionsList=None, supportList=None) -> None:
    self.statesSet = KDTree(statesList)

    self.stateActionImitationList = {}
    if actionsList is not None:
      for stateKey, action in enumerate(actionsList):
        self.stateActionImitationList[stateKey] = action

    self.goalSet = None
    if supportList is not None:
      self.goalSet = KDTree(supportList)

  # Check if state is in terminal states set
  def isGoalState(self, state) -> bool:
    return self.goalSet is not None and self.goalSet.query(state)[0] < 1e-2 and int(round(state[-1]))==0

  # Get index of closest state in list of known states
  def getStateIdx(self, state) -> int:
    _, stateIdx = self.statesSet.query(state)
    return stateIdx

  # Get state point of closest state in list of known states
  def findClosestState(self, state:np.array) -> np.array:
    distance, closestStateIdx = self.statesSet.query(state)
    return self.statesSet.data[closestStateIdx], distance

  # Add a state to the list of known states
  def addNewState(self, state:np.ndarray) -> int:
    self.statesSet = KDTree(np.append(self.statesSet.data, [state], axis=0))
    return self.statesSet.n-1

  # Add known state to list of terminal states
  def addGoalState(self, state:np.ndarray) -> None:
    stateIdx = self.getStateIdx(state)
    if self.goalSet is None:
      self.goalSet = KDTree([self.statesSet.data[stateIdx]])
    else:
      self.goalSet = KDTree(np.append(self.goalSet.data, [self.statesSet.data[stateIdx]], axis=0))


  def getPassiveAction(self):
    return 'wait'

  # Get action in state if known, otherwise return passive
  def getImitatedAction(self, state):
    _, stateKey = self.statesSet.query(state)
    if stateKey not in self.stateActionImitationList:
      return self.getPassiveAction()
    else:
      return self.stateActionImitationList[stateKey]

  # Add observed action to imitation list
  # In: state
  #     action
  def updateActionImitationList(self, state, ah):
    if ah != 'wait':
      stateKey = self.statesSet.query(state)[1]
      self.stateActionImitationList[stateKey] = ah