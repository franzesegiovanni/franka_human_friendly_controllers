#####################################################################
## Cup placing test scenario 
#####################################################################

import time
import rospy
import numpy as np
from StateActionInterpreter import ActionInterpreter


rospy.init_node('CupTest', anonymous=True)

actionController = ActionInterpreter()


print("Press Enter to start episode")
input()

while True:
  time.sleep(1)
  exitCriterion = False
  actionController.grasp.close()
  state, sIdx = actionController.getStateAndIdx()
  stateTrace = [sIdx]
  while not exitCriterion:
    action = actionController.model.getImitatedAction(state)
    state, sIdx, exitCriterion = actionController.doAction(action)
    stateTrace.append(sIdx)

  # time.sleep(1)
  actionController.controller.setPassive()

  print("Press Enter to do another episode episode, enter 'q' to quit")
  txt = input()
  if 'q' in txt:
    break


print("Do you want to save the model states and goals? (y/n)")
confirmSave = input()
if 'y' in confirmSave:
  fileName = 'states_goals.npz'
  np.savez(fileName, states=actionController.model.statesSet.data,
                    goals=actionController.model.goalSet.data)
