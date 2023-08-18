import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import rospy
from desk import Desk
from controller import LaunchManager
import sys
import subprocess

# Check if the correct number of arguments are provided
if len(sys.argv) != 4:
    print("Usage: python gui.py <hostname> <username> <password>")
    sys.exit(1)

# Retrieve command-line arguments
hostname = str(sys.argv[1])
username = str(sys.argv[2])
password = str(sys.argv[3])

# Now you can use the variables hostname, username, and password in your script
print("Hostname:", hostname)
print("Username:", username)
print("Password:", password)


# hostname = "172.16.0.3"
# username = "franka_hri"
# password = "Panda2022"

launch_manager = LaunchManager(
    package='franka_human_friendly_controllers',
    launch_file='cartesian_variable_impedance_controller.launch',
    robot_ip=hostname
)

#Start ROS node
rospy.init_node('Desk', anonymous=True)
rospy.sleep(1)

desk=Desk(hostname, username, password)
rospy.sleep(1)

desk.listen(desk.button_callback)

def fun1(event):
    print("Unlock the robot!")
    desk.take_control(force=True)
    desk.unlock(force=True)
def fun2(event):
    print("Lock the robot!")
    launch_manager.stop()
    desk.lock()
    desk.deactivate_fci()
def fun3(event):
    print("Starting the cartesian impedance control!")
    desk.activate_fci()
    launch_manager.start()
def fun4(event):
    print("Stopping the cartesian impedance control!")
    desk.deactivate_fci()
    launch_manager.stop()


def check_roscore_running():
    try:
        subprocess.check_output(['rosnode', 'list'])
        return True
    except subprocess.CalledProcessError:
        return False

def start_roscore():
    subprocess.Popen(['roscore'])

if not check_roscore_running():
    print("ROS core is not running. Starting ROS core...")
    start_roscore()
else:
    print("ROS core is already running.")

    
# Create buttons
button1_ax = plt.axes([0.4, 0.7, 0.2, 0.1])  # [x, y, width, height]
button1 = Button(button1_ax, 'Lock')
button1.on_clicked(fun1)

button2_ax = plt.axes([0.4, 0.5, 0.2, 0.1])
button2 = Button(button2_ax, 'Unlock')
button2.on_clicked(fun2)

button3_ax = plt.axes([0.4, 0.3, 0.2, 0.1])
button3 = Button(button3_ax, 'Start Controller')
button3.on_clicked(fun3)

button4_ax = plt.axes([0.4, 0.1, 0.2, 0.1])
button4 = Button(button4_ax, 'Stop Controller')
button4.on_clicked(fun4)



# Display the plot
plt.show()
