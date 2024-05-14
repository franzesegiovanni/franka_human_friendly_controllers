import rospkg
def add_text(file_path, text_to_add):
    with open(file_path, 'r+') as file:
        content = file.read()
        if text_to_add not in content:
            file.seek(0, 2)  # Move the file pointer to the end
            file.write(text_to_add)

def search_and_paste(file_path, search_word, paste_word):
    with open(file_path, 'r+') as file:
        lines = file.readlines()
        file.seek(0)  # Reset file pointer to the beginning

        for line in lines:
            file.write(line)  # Write the original line to the file

            if search_word in line:
                if not(paste_word + '\n' in lines):
                    file.write(paste_word + '\n')

def replace_line(file_path, search_line, replace_line):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    with open(file_path, 'w') as file:
        for line in lines:
            if line.strip() == search_line:
                file.write(replace_line + '\n')
            else:
                file.write(line)

# Print the path to the franka_gazebo

ros_pack = rospkg.RosPack()
franka_gazebo = ros_pack.get_path('franka_gazebo')
franka_hfc = ros_pack.get_path('franka_human_friendly_controllers')

new_text_cartesian=''' 
cartesian_variable_impedance_controller:
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
'''

new_text_joint=''' 
joint_variable_impedance_controller:
  type: franka_human_friendly_controllers/JointVariableImpedanceController 
  arm_id: $(arg arm_id)
  joint_names:
    - $(arg arm_id)_joint1
    - $(arg arm_id)_joint2
    - $(arg arm_id)_joint3
    - $(arg arm_id)_joint4
    - $(arg arm_id)_joint5
    - $(arg arm_id)_joint6
    - $(arg arm_id)_joint7 
'''

# # Perform the replacement
# new_text = new_text.replace("franka_advanced_controllers", input_string)
file_path=franka_gazebo + '/config/sim_controllers.yaml'
print("Change files")
print(file_path)
add_text(file_path, new_text_cartesian)
add_text(file_path, new_text_joint)

# MODIFY THE PACKAGE
file_path = franka_gazebo + '/package.xml'
print("Change files")
print(file_path)
existing_depend = '<depend>franka_example_controllers</depend>'
new_depend = '  <depend>franka_human_friendly_controllers</depend>'

# new_depend = new_depend.replace("franka_advanced_controllers", input_string)
search_and_paste(file_path, existing_depend, new_depend)

# MODIFY CMAKE
file_path = franka_gazebo + '/CMakeLists.txt'
print("Change files in directory")
print(file_path)
existing_depend = 'franka_example_controllers'
new_depend = '  franka_human_friendly_controllers'

# new_depend = new_depend.replace("franka_advanced_controllers", input_string)
search_and_paste(file_path, existing_depend, new_depend)

# Modify the controller to not have the joint repulsion that generated obscillations in sumulation 
file_path = franka_hfc +  '/src/cartesian_variable_impedance_controller.cpp'
print("Change files in directory")
print(file_path)
search_line= 'tau_d << tau_task + tau_nullspace + coriolis+ tau_joint_limit;'
new_line='tau_d << tau_task + tau_nullspace + coriolis;'
replace_line(file_path, search_line, new_line)

# Modify the joint impedance control also
file_path = franka_hfc + '/src/joint_variable_impedance_controller.cpp'
print("Change files in directory")
print(file_path)
search_line= 'tau_d << tau_joint + coriolis + tau_joint_limit;'
new_line='tau_d << tau_joint + coriolis;'
replace_line(file_path, search_line, new_line)


# MODIFY THE CLOCK
file_path = franka_gazebo +  '/launch/robot.launch'
search_line = '<arg name="use_sim_time" value="true"/>'
new_line = '    <arg name="use_sim_time" value="false"/>'
print("Change files in directory")
print(file_path)
replace_line(file_path, search_line, new_line)
# add the rqt in the launch file
file_path = franka_gazebo + '/launch/panda.launch'
print("Change files")
print(file_path)
existing_depend='</include>'
new_depend='<node name="rqt_reconfigure" pkg="rqt_reconfigure" type="rqt_reconfigure" required="false" />'
search_and_paste(file_path, existing_depend, new_depend)
