# Teleoperating the Franka Emika Robots through the keyboard

The aim of this example is give a stripped down example of to interact with the Franka Emika robots through ROS and a python script. The `Panda` class in this folder is a subset of the `Panda` class in the other folders.

## Installing dependencies

This guide assumes you have the `franka_ros` and the `franka_human_friendly_controllers` package running on the real robot or Gazebo. 

The only other dependency is the `teleop_twist_keyboard` package, which can be installed with

```bash 
sudo apt install ros-noetic-teleop-twist-keyboard
```

## Running the script

With the `franka_human_friendly_controllers` running on the robot or Gazebo, start the `teleop_twist_keyboard` node in another terminal. 
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Then run the main script
```bash
python3 main_teleop.py
```

Then to move the robot, press the following keys:


Key - Action         
'i' = + z-axis
',' = - z-axis
'j' = + y-axis
'l' = - y-axis
'u' = + x-axis
'o' = - x-axis

**Note:** make sure the terminal running the 'teleop_twist_keyboard' node is active by clicking on it
