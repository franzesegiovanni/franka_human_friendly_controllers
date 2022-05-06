# FrankaAdvancedControllers
Extra Controllers for Control of Franka Emika Panda

To make it working with the simulator, please add: 
in franka_gazebo/CMakeList.txt  
in catkin_package(...)
add
franka_advanced_controllers

in franka_gazebo/CMakeList.txt  
in find_package(..)
add 
franka_advanced_controllers

in franka_gazebo/package.xml
add
<depend>franka_advanced_controllers</depend>

in franka_gazebo/config/sim_controllers.yaml
add
cartesian_impedance_advanced_controller:
  type: franka_advanced_controllers/CartesianImpedanceAdvancedController 
  arm_id: $(arg arm_id)
  joint_names:
    - $(arg arm_id)_joint1
    - $(arg arm_id)_joint2
    - $(arg arm_id)_joint3
    - $(arg arm_id)_joint4
    - $(arg arm_id)_joint5
    - $(arg arm_id)_joint6
    - $(arg arm_id)_joint7

