import os
import rospkg

def replace_line(file_path, search_string, replace_string):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    with open(file_path, 'w') as file:
        for line in lines:
            if search_string in line:
                line = line.replace(search_string, replace_string)
            file.write(line)

ros_pack = rospkg.RosPack()
package_path = ros_pack.get_path('franka_description')
# Modify the controller to not have the joint repulsion that generated obscillations in sumulation 
file_path = package_path + '/robots/common/franka_robot.xacro'
print("Change files in directory")
print(file_path)
search_line= 'safety_distance="0.03"'
new_line='safety_distance="-0.02"'
replace_line(file_path, search_line, new_line)
replace_line(file_path, search_line, new_line)