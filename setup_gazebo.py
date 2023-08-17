import os
# import sys
# input_string = sys.argv[1]
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
# input_string = sys.argv[1]

# MAIN
# MODIFY THE YAML FILE
# Get the current directory of the Python file
current_dir = os.path.dirname(os.path.abspath(__file__))
# Go one directory up
parent_dir = os.path.dirname(current_dir)
# Access the directory of a subdirectory
subdirectory = os.path.join(parent_dir, 'franka_gazebo')
# Print the path to the subdirectory


new_text=''' 
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

# # Perform the replacement
# new_text = new_text.replace("franka_advanced_controllers", input_string)
file_path=os.path.join(subdirectory, 'config/sim_controllers.yaml')
print("Change files")
print(file_path)
add_text(file_path, new_text)

# MODIFY THE PACKAGE
file_path = os.path.join(subdirectory, 'package.xml')
print("Change files")
print(file_path)
existing_depend = '<depend>franka_example_controllers</depend>'
new_depend = '  <depend>franka_human_friendly_controllers</depend>'

# new_depend = new_depend.replace("franka_advanced_controllers", input_string)
search_and_paste(file_path, existing_depend, new_depend)

# MODIFY CMAKE
file_path = os.path.join(subdirectory, 'CMakeLists.txt')
print("Change files in directory")
print(file_path)
existing_depend = 'franka_example_controllers'
new_depend = '  franka_human_friendly_controllers'

# new_depend = new_depend.replace("franka_advanced_controllers", input_string)
search_and_paste(file_path, existing_depend, new_depend)

# MODIFY THE CLOCK
file_path = os.path.join(subdirectory, 'launch/robot.launch')
search_line = '<arg name="use_sim_time" value="true"/>'
new_line = '    <arg name="use_sim_time" value="false"/>'
print("Change files in directory")
print(file_path)
replace_line(file_path, search_line, new_line)
# add the rqt in the launch file
file_path = os.path.join(subdirectory, 'launch/panda.launch')
print("Change files")
print(file_path)
existing_depend='</include>'
new_depend='<node name="rqt_reconfigure" pkg="rqt_reconfigure" type="rqt_reconfigure" required="false" />'
search_and_paste(file_path, existing_depend, new_depend)
