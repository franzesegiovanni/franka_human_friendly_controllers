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
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

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

  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_ext(robot_state.tau_ext_hat_filtered.data());
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  //std::array<double, 7> gravity = model_handle_->getGravity();
  Eigen::Matrix<double, 7, 1>  tau_f;
  Eigen::MatrixXd jacobian_transpose_pinv;
  franka_human_friendly_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  
  // Compute the value of the friction
  tau_f(0) =  FI_11/(1+exp(-FI_21*(dq(0)+FI_31))) - TAU_F_CONST_1;
  tau_f(1) =  FI_12/(1+exp(-FI_22*(dq(1)+FI_32))) - TAU_F_CONST_2;
  tau_f(2) =  FI_13/(1+exp(-FI_23*(dq(2)+FI_33))) - TAU_F_CONST_3;
  tau_f(3) =  FI_14/(1+exp(-FI_24*(dq(3)+FI_34))) - TAU_F_CONST_4;
  tau_f(4) =  FI_15/(1+exp(-FI_25*(dq(4)+FI_35))) - TAU_F_CONST_5;
  tau_f(5) =  FI_16/(1+exp(-FI_26*(dq(5)+FI_36))) - TAU_F_CONST_6;
  tau_f(6) =  FI_17/(1+exp(-FI_27*(dq(6)+FI_37))) - TAU_F_CONST_7;

  float iCutOffFrequency=10.0;
  force_torque+=(-jacobian_transpose_pinv*(tau_ext-tau_f)-force_torque)*(1-exp(-0.001 * 2.0 * M_PI * iCutOffFrequency));
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
  ros::Publisher pub_force_torque_= read_state.advertise<geometry_msgs::WrenchStamped>("/force_torque_ext",1);


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

    force_torque_msg.wrench.force.x=force_torque[0];
    force_torque_msg.wrench.force.y=force_torque[1];
    force_torque_msg.wrench.force.z=force_torque[2];
    force_torque_msg.wrench.torque.x=force_torque[3];
    force_torque_msg.wrench.torque.y=force_torque[4];
    force_torque_msg.wrench.torque.z=force_torque[5];

    pub_force_torque_.publish(force_torque_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}