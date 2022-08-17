// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_human_friendly_controllers/compliance_joint_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

// #include <panda_utils/panda_trac_ik.h>


namespace franka_human_friendly_controllers {

class JointVariableImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double nullspace_stiffness_{0.0};
  double nullspace_stiffness_target_{0.0};
  double dt{0.001};
  double time_old;
  int alpha;
  int filter_step{0};
  int filter_step_;
  const double delta_tau_max_{1.0};
  double damping_ratio{1};
  Eigen::Matrix<double, 7, 1> q_d_;
  Eigen::Matrix<double, 7, 7> joint_stiffness_target_;
  Eigen::Matrix<double, 7, 7> joint_damping_target_;
  Eigen::Matrix<double, 6, 1> force_torque;
  Eigen::Matrix<double, 6, 1> force_torque_old;
  Eigen::Matrix<float, 7, 1> stiff_;
  std::array<double, 7> q_start_ik;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::array<double,7> goal;//{0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
  Eigen::Matrix<double, 7, 1> goal_;
  std::array<double, 49> mass_goal_;
  std::array<double, 9> total_inertia_ = {{0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017}}; 
  //total_inertia_ << 0.001, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017; // dummie parameter to get goal mass matrix

  double total_mass_ = {0.73}; // dummie parameter to get goal mass matrix
  //double total_mass_ = {0.9}; // dummie parameter to get goal mass matrix
  std::array<double, 3> F_x_Ctotal_ = {{-0.01, 0.0, 0.03}}; // dummie parameter to get goal mass matrix

  Eigen::Matrix<double, 7, 7> K_;
  Eigen::Matrix<double, 7, 7> D_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_human_friendly_controllers::compliance_joint_paramConfig>>
      dynamic_server_compliance_joint_param_;

  ros::NodeHandle dynamic_reconfigure_compliance_joint_param_node_;
  void complianceJointParamCallback(franka_human_friendly_controllers::compliance_joint_paramConfig& config,
                               uint32_t level);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // Configuration pose subscriber
  ros::Subscriber sub_equilibrium_config_;
  void equilibriumConfigurationCallback( const sensor_msgs::JointState::ConstPtr& joint);


  // Equilibrium pose subscriber IK
  ros::Subscriber sub_equilibrium_pose_ik;
  //void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
//   void equilibriumConfigurationIKCallback( const geometry_msgs::PoseStampedConstPtr& msg);
  // Multi directional stiffness stiffnes
  ros::Subscriber sub_stiffness_;
  void equilibriumStiffnessCallback(const std_msgs::Float32MultiArray::ConstPtr& stiffness_);

  ros::Publisher pub_stiff_update_;
  ros::Publisher pub_cartesian_pose_;
  ros::Publisher pub_force_torque_;

//   PandaTracIK _panda_ik_service;
//   KDL::JntArray _joints_result;

  hardware_interface::PositionJointInterface *_position_joint_interface;
  std::vector<hardware_interface::JointHandle> _position_joint_handles;

};


}  // namespace franka_advanced_controllers
