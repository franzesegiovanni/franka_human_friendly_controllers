# %%
from Learning_from_demonstration import LfD
import rospy
#%%
LfD=LfD()
LfD.home_gripper() # homeing the gripper allows to kinestheicall move it. 
rospy.sleep(5)
#%%
LfD.traj_rec()
#%%
LfD.save()
#%%
LfD.load()
#%%
LfD.execute()
# %%
