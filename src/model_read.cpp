// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka_human_friendly_controllers/model_read.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>
#include <memory>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <franka_human_friendly_controllers/franka_model.h>
#include <franka_human_friendly_controllers/pseudo_inversion.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"

namespace franka_human_friendly_controllers {


  
bool ModelRead::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  
  pub_cartesian_pose_= node_handle.advertise<geometry_msgs::PoseStamped>("/cartesian_pose",1);

  pub_force_torque_= node_handle.advertise<geometry_msgs::WrenchStamped>("/force_torque_ext",1);
  
  franka_state_interface_ = robot_hw->get<franka_hw::FrankaStateInterface>();
  
//   if (franka_state_interface_ == nullptr) {
//     ROS_ERROR("ModelExampleController: Could not get Franka state interface from hardware");
//     return false;
//   }
//   std::string arm_id;
//   if (!node_handle.getParam("arm_id", arm_id)) {
//     ROS_ERROR("ModelExampleController: Could not read parameter arm_id");
//     return false;
//   }
//   model_interface_ = robot_hw->get<franka_hw::FrankaModelInterface>();
//   if (model_interface_ == nullptr) {
//     ROS_ERROR_STREAM("ModelExampleController: Error getting model interface from hardware");
//     return false;
//   }

//   try {
//     state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
//         franka_state_interface_->getHandle(arm_id + "_robot"));
//   } catch (const hardware_interface::HardwareInterfaceException& ex) {
//     ROS_ERROR_STREAM(
//         "ModelExampleController: Exception getting franka state handle: " << ex.what());
//     return false;
//   }

//   try {
//     model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
//         model_interface_->getHandle(arm_id + "_model"));
//   } catch (hardware_interface::HardwareInterfaceException& ex) {
//     ROS_ERROR_STREAM(
//         "ModelExampleController: Exception getting model handle from interface: " << ex.what());
//     return false;
//   }
//   return true;
// }

   if (franka_state_interface_ == nullptr) {
     ROS_ERROR("ModelExampleController: Could not get Franka state interface from hardware");
     return false;
    }
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianVariableImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianVariableImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianVariableImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianVariableImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianVariableImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianVariableImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }
                                  }
                                  
void ModelRead::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

    // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 49> mass_array = model_handle_->getMass();
  Eigen::Map<Eigen::Matrix<double, 7, 7> > mass(mass_array.data());
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());

  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_ext(robot_state.tau_ext_hat_filtered.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Matrix<double, 7, 1>  tau_f;
  Eigen::MatrixXd jacobian_transpose_pinv;

  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  tau_f(0) =  FI_11/(1+exp(-FI_21*(dq(0)+FI_31))) - TAU_F_CONST_1;
  tau_f(1) =  FI_12/(1+exp(-FI_22*(dq(1)+FI_32))) - TAU_F_CONST_2;
  tau_f(2) =  FI_13/(1+exp(-FI_23*(dq(2)+FI_33))) - TAU_F_CONST_3;
  tau_f(3) =  FI_14/(1+exp(-FI_24*(dq(3)+FI_34))) - TAU_F_CONST_4;
  tau_f(4) =  FI_15/(1+exp(-FI_25*(dq(4)+FI_35))) - TAU_F_CONST_5;
  tau_f(5) =  FI_16/(1+exp(-FI_26*(dq(5)+FI_36))) - TAU_F_CONST_6;
  tau_f(6) =  FI_17/(1+exp(-FI_27*(dq(6)+FI_37))) - TAU_F_CONST_7;
  force_torque=force_torque-jacobian_transpose_pinv*(tau_ext-tau_f);
  // publish force, torque
  int filter_step=filter_step+1;
  int filter_step_=10;
  int alpha=1;
  if (filter_step==filter_step_){
    geometry_msgs::WrenchStamped force_torque_msg;
    force_torque_msg.wrench.force.x=force_torque_old[0]*(1-alpha)+force_torque[0]*alpha/(filter_step_);
    force_torque_msg.wrench.force.y=force_torque_old[1]*(1-alpha)+ force_torque[1]*alpha/(filter_step_);
    force_torque_msg.wrench.force.z=force_torque_old[2]*(1-alpha)+force_torque[2]*alpha/(filter_step_);
    force_torque_msg.wrench.torque.x=force_torque_old[3]*(1-alpha)+force_torque[3]*alpha/(filter_step_);
    force_torque_msg.wrench.torque.y=force_torque_old[4]*(1-alpha)+force_torque[4]*alpha/(filter_step_);
    force_torque_msg.wrench.torque.z=force_torque_old[5]*(1-alpha)+force_torque[5]*alpha/(filter_step_);
    pub_force_torque_.publish(force_torque_msg);
    force_torque_old=force_torque/(filter_step_);
    force_torque.setZero();
    filter_step=0;
    }
  geometry_msgs::PoseStamped pose_msg;

  pose_msg.pose.position.x=position[0];
  pose_msg.pose.position.y=position[1];
  pose_msg.pose.position.z=position[2];
  pose_msg.pose.orientation.x=orientation.x();
  pose_msg.pose.orientation.y=orientation.y();
  pose_msg.pose.orientation.z=orientation.z();
  pose_msg.pose.orientation.w=orientation.w();
  pub_cartesian_pose_.publish(pose_msg);
  
}

}
PLUGINLIB_EXPORT_CLASS(franka_human_friendly_controllers::ModelRead,
                       controller_interface::ControllerBase)
