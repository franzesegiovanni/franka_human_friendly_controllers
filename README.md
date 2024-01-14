# Franka Human Friendly Controllers

If you found this repo useful for your research, please cite it as:

```
@inproceedings{franzese2021ilosa,
  title={ILoSA: Interactive learning of stiffness and attractors},
  author={Franzese, Giovanni and M{\'e}sz{\'a}ros, Anna and Peternel, Luka and Kober, Jens},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={7778--7785},
  year={2021},
  organization={IEEE}
}
```
-Install Franka ROS from [here](https://frankaemika.github.io/docs/installation_linux.html)

- Go the the catkin_ws where you install franka_ros
```
cd /path/to/catkin_ws
```
- Install the human-friendly controller:

```
cd catkin_ws/src
git clone https://github.com/franzesegiovanni/franka_human_friendly_controllers.git
cd .. 
catkin_make -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

To run the controller:
- Switch on your Panda robot (make sure the gripper is initialized correctly), unlock its joints (and activate the FCI if necessary).
- Open a terminal, in every terminal: ```source devel/setup.bash```
- ```roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP load_gripper:=True```

# Run this in Gazebo simulation
The first time you run it, set up Gazebo with 
``` python3 setup_gazebo.py ```. 

Compile again: 
``` catkin_make -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build```. 

To lunch the cartesian impedance controller in simulation: 

``` roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=cartesian_variable_impedance_controller rviz:=true ```

To kill gazebo run:
``` killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 rosmaster & killall -9 roscore killall -9 rviz```

# DAVI controller
The independent Disagreement-Aware Variable Impedance (DAVI) controller that interfaces with the Franka Human Friendly Controllers has moved to it's own separate repo: https://github.com/LindavdSpaa/DAVI_controller.
  
