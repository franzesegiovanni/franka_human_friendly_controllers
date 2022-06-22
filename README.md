# Franka Human Friendly Controllers
To install:
- Create a workspace containing a src directory.
- Inside the src directory, clone the franka_ros repository by frankaemika.
- Inside the repository, clone this repository.
- Return to the workspace main directory (cd ../..).
- Source your version of ROS (e.g. ```source /opt/ros/melodic/setup.bash```).
- Build the project, calling: 
 ```
 catkin_make -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build 
 ```

To run the controller:
- Switch on your Panda robot (make sure the gripper is initialized correctly), unlock its joints (and activate the FCI if necessary).
- Open a terminal, in every terminal: ```source devel/setup.bash```
- ```roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP load_gripper:=True```
- In a different terminal (don't forget to source again): ``` rosrun franka_human_friendly_controllers franka_gripper_online ```

# Run this in gazebo simulation
in franka_gazebo/CMakeList.txt
    in catkin_package(...)  **add**  ```franka_human_friendly_controllers```

in franka_gazebo/CMakeList.txt
    in find_package(..)  **add**  ```franka_human_friendly_controllers```

in franka_gazebo/package.xml  **add**  ```<depend>franka_human_friendly_controllers</depend>```

in franka_gazebo/config/sim_controllers.yaml **add**

```cartesian_variable_impedance_controller:
  type: franka_human_friendly_controllers/CartesianVariableImpedanceController 
  arm_id: $(arg arm_id)
  joint_names:
    - $(arg arm_id)_joint1
    - $(arg arm_id)_joint2
    - $(arg arm_id)_joint3
    - $(arg arm_id)_joint4
    - $(arg arm_id)_joint5
    - $(arg arm_id)_joint6
    - $(arg arm_id)_joint7 
```
To lunch the cartesian impedance controller in simulation: 

``` roslaunch franka_gazebo panda.launch x:=-0.5 world:=$(rospack find franka_gazebo)/world/stone.sdf controller:=cartesian_variable_impedance_controller rviz:=true ```

# Disagreement-aware Variable Impedance controller (DAVI)
To run the demo of teaching the robot where and how to place a paper cup:
- Open a new terminal in src/franka_human_friendly_controllers/python/DAVI
- Run the main file: ```python3 main.py```

The robot will move to make the gripper face down, and then switch to gravity compensation.
The robot can now be moved to the desired initial end-effector position.
Upon pressing enter, the model is further initialized and the Episodes can be started and quit via the keyboard.

Watch our demo here: https://youtu.be/toIUuFguFgM

  
  
