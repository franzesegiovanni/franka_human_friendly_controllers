# Franka Human Friendly Controllers

### What is this repository useful for? 
This code is build on top of [Franka Cartesian Control](https://github.com/frankaemika/franka_ros/blob/develop/franka_example_controllers/src/cartesian_impedance_example_controller.cpp). 
This code has some other desirable features for human-robot interaction.

Features:

- **Joint limit repulsion**: The controller has a joint limit repulsion feature that allows the robot to avoid joint limits. This is useful when the robot is controlled by a human operator.

- **Anisotropic stiffness**: The controller allows the user to set different stiffness values for each axis, both linear and angular.

- **Publish the desired attractor**: the toipc /equilibrium_pose is the topic where you can publish the deried pose of the robot. The message type is geometry_msgs/PoseStamped.

- **Safety feature**: The attractor distance are clipped inside the controller. You can set the clipping value in the rqt_reconfigure. This enusre that the robot will not move too fast when a too far attractor is published.

- **Haptic feedback**: You can publish the haptic feedback in the topic /haptic_feedback. The message type is std_msgs/Float32. The value that is published is the time that the last joint will vibrate. For example 
`
rostopic pub /haptic_feedback std_msgs/Float32 "data: 0.5"
`
will make the last joint vibrate for 0.5 seconds.
- **Read external forces**: The controller reads the external forces and torques from the robot and publish them in the topic /force_torque_ext. The message type is geometry_msgs/WrenchStamped. This value is already filtered as it is also compensating for the gravity, Coriolis and friction forces.

### Installation 
- Install Franka ROS from [here](https://frankaemika.github.io/docs/installation_linux.html)

- Go the the catkin_ws where you install franka_ros
```
cd /path/to/catkin_ws
```
- Install the human-friendly controller:

```
cd catkin_ws/src
git clone https://github.com/franzesegiovanni/franka_human_friendly_controllers.git
cd .. 
catkin build -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

To run the controller:
- Switch on your Panda robot (make sure the gripper is initialized correctly), unlock its joints (and activate the FCI).
- Open a terminal and source the evnironment:
 ```
 source devel/setup.bash
 ```
 If you have a Panda, run the controller as:
 ```
 roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP load_gripper:=True arm_id:=panda
 ```

If you have an FR3 then run 

 ```
 roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP load_gripper:=True arm_id:=fr3
 ```

 selecting the right arm_id is important for set correctly the joint limit repulsion.

# Run this in Gazebo simulation
After building the catkin_ws and sourced the environment, you can run the following python code to set up the files such that to be able to run the code in simulation.
``` python3 setup_gazebo.py ```. 

Compile again: 
``` catkin build ```

To lunch the cartesian impedance controller in simulation: 

``` roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=cartesian_variable_impedance_controller rviz:=true ```

To kill gazebo run:
``` killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 rosmaster & killall -9 roscore killall -9 rviz```


# Learning from Demonstration in Python

In a terminal with the workspace sourced, open vscode, be sure that you have the extension Jupyter installed and then run one by one the cells in python/LfD/main_lfd.py. You can record kinesthetic demonstration, save them, load them and then play them back. Every time you execute a skill, the robot goes back to the starting position of the demonstration.


# More cool stuff
- Do you want to control two arms at the same time using similar controllers? Check out [franka_bimanual_controllers](https://github.com/franzesegiovanni/franka_bimanual_controllers).

- Do you want to read the franka buttons in ROS? Check out [franka_buttons](https://github.com/franzesegiovanni/franka_buttons).

- Do you want to do learning from demonstration? Check out [ILoSA](https://github.com/franzesegiovanni/ILoSA), [SIMPLe](https://github.com/franzesegiovanni/SIMPLe) and [franka_learning_from_demonstration](https://github.com/platonics-delft/franka_learning_from_demonstrations).

# Cite us! 
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
# Acknowledgements

This work has received funding from the European Union’s ERC starting grant TERI "Teaching Robots Interactively", number 804907. 