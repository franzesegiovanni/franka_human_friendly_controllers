import roslaunch
import rospy

class LaunchManager:
    def __init__(self, package, launch_file, robot_ip):
        self.package = package
        self.launch_file = launch_file
        self.robot_ip = robot_ip
        self.launch_api = None

    def start(self):
        rospy.init_node('launch_example', anonymous=True)

        # Create a ROS launch API object
        self.launch_api = roslaunch.scriptapi.ROSLaunch()

        # Start the launch file
        self.launch_api.start()

        # Load the specified package and launch file
        launch_file_path = roslaunch.rlutil.resolve_launch_arguments([self.package, self.launch_file])

        launch_args = 'robot_ip:= ' + self.robot_ip  # Replace 'arg_name' and 'arg_value' with your argument name and value

        launch_file=[(launch_file_path[0], [launch_args])]
        self.launch_api.parent = roslaunch.parent.ROSLaunchParent(
            rospy.get_param("/run_id"), launch_file
        )

        # Set the robot IP as a parameter for the launch file
        self.launch_api.parent.start()

        ###

        # parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        # parent.start()

    def stop(self):
        if self.launch_api is not None:
            self.launch_api.parent.shutdown()

def main():
    robot_ip = '172.16.0.3'  # Replace with the actual robot IP

    launch_manager = LaunchManager(
        package='franka_human_friendly_controllers',
        launch_file='cartesian_variable_impedance_controller.launch',
        robot_ip=robot_ip
    )

    try:
        launch_manager.start()

        rospy.sleep(20)
        
        launch_manager.stop()

    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
