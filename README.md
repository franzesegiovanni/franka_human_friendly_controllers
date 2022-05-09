# Franka Human Friendly Controllers
To install:
- Create a workspace containing a src directory.
- Inside the src directory, clone the franka_ros repository by frankaemika.
- Inside the repository, clone this repository.
- Return to the workspace main directory (cd ../..).
- Source your version of ROS.
- Call catkin_make to build the project.

To run the controller:
- Switch on your Panda robot (make sure the gripper is initialized correctly), unlock its joints (and activate the FCI if necessary).
- Open a terminal, in every terminal: source devel/setup.bash
- roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=<ROBOT_IP> load_gripper:=True
- In a different terminal (don't forget to source again): rosrun franka_gripper franka_gripper_online

To run the demo of teaching the robot where and how to place a paper cup, open a new terminal.
In src/franka_human_friendly_controllers/python, run the main file: python3 main.py

Watch our demo here: https://youtu.be/A9cvBl_3fYc
