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

  double dt{0.001};
  int alpha;
  int filter_step{0};
  int filter_step_;
  const double delta_tau_max_{1.0};
  double damping_ratio{1};
  Eigen::Matrix<double, 7, 1> q_d_;
  Eigen::Matrix<double, 7, 7> joint_stiffness_target_;
  Eigen::Matrix<double, 7, 7> joint_damping_target_;

  // Dynamic reconfigure of stiffness 
  std::unique_ptr<dynamic_reconfigure::Server<franka_human_friendly_controllers::compliance_joint_paramConfig>>
      dynamic_server_compliance_joint_param_;

  ros::NodeHandle dynamic_reconfigure_compliance_joint_param_node_;
  void complianceJointParamCallback(franka_human_friendly_controllers::compliance_joint_paramConfig& config,
                               uint32_t level);

  // Configuration joint configuration subscriber
  ros::Subscriber sub_equilibrium_config_;
  void equilibriumConfigurationCallback( const sensor_msgs::JointState::ConstPtr& joint);


  Eigen::Matrix<double, 6, 1> force_torque;
  Eigen::Matrix<double, 6, 1> force_torque_old;

  ros::Publisher pub_stiff_update_;
  ros::Publisher pub_cartesian_pose_;
  ros::Publisher pub_force_torque_;

  hardware_interface::PositionJointInterface *_position_joint_interface;
  std::vector<hardware_interface::JointHandle> _position_joint_handles;

};


}  // namespace franka_advanced_controllers
