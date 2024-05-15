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
LfD.save(name="demo")
#%%
LfD.load(name="demo")
#%%
LfD.execute()
# %%
