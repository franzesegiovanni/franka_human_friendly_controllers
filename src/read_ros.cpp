#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <franka/exception.h>
#include <franka_human_friendly_controllers/cartesian_variable_impedance_controller.h>
#include <franka_human_friendly_controllers/franka_model.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <franka_human_friendly_controllers/pseudo_inversion.h>
#include "franka_msgs/FrankaState.h"

// creat global variables
std::array<double, 3> Position;
std::array<double, 4> Orientation;
Eigen::Matrix<double, 6, 1> force_torque;
std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

void state_callback(franka_msgs::FrankaState robot_state)
{
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation()); 
  Eigen::Quaterniond orientation(transform.linear());

  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  
  Position[0]= position[0];
  Position[1]= position[1];
  Position[2]= position[2];

  Orientation[0] = orientation.x();  //x
  Orientation[1] = orientation.y();  //y
  Orientation[2] = orientation.z();  //z
  Orientation[3] = orientation.w();  //w

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Position_EE");


  ros::NodeHandle read_state;
  ros::Rate loop_rate(100);

  ros::Subscriber sub = read_state.subscribe("/franka_state_controller/franka_states", 1, state_callback);

  ros::Publisher pub_cartesian_pose_= read_state.advertise<geometry_msgs::PoseStamped>("/cartesian_pose",1);


  geometry_msgs::PoseStamped msg;
  geometry_msgs::WrenchStamped force_torque_msg;


  while (ros::ok())
  {

    msg.pose.position.x=Position[0];
    msg.pose.position.y=Position[1];
    msg.pose.position.z=Position[2];

    msg.pose.orientation.x=Orientation[0];
    msg.pose.orientation.y=Orientation[1];
    msg.pose.orientation.z=Orientation[2];
    msg.pose.orientation.w=Orientation[3];

    pub_cartesian_pose_.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}