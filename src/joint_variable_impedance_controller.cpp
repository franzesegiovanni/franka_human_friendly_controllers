// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_human_friendly_controllers/joint_variable_impedance_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <franka_human_friendly_controllers/franka_model.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka_human_friendly_controllers/pseudo_inversion.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
namespace franka_human_friendly_controllers {

bool JointVariableImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

  sub_equilibrium_config_ = node_handle.subscribe(
      "/equilibrium_configuration", 20, &JointVariableImpedanceController::equilibriumConfigurationCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  pub_stiff_update_ = node_handle.advertise<dynamic_reconfigure::Config>(
    "/dynamic_reconfigure_compliance_param_node/parameter_updates", 5);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("JointVariableImpedanceControllers: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointVariableImpedanceControllers: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointVariableImpedanceControllers: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointVariableImpedanceControllers: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointVariableImpedanceControllers: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointVariableImpedanceControllers: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointVariableImpedanceControllers: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVariableImpedanceControllers: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_joint_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_joint_param_.reset(
      new dynamic_reconfigure::Server<franka_human_friendly_controllers::compliance_joint_paramConfig>(
          dynamic_reconfigure_compliance_joint_param_node_));

  dynamic_server_compliance_joint_param_->setCallback(
      boost::bind(&JointVariableImpedanceController::complianceJointParamCallback, this, _1, _2));

  return true;
}

void JointVariableImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  q_d_ = q_initial;  // this command sets the goal of the robot to the starting position. It allows to not have strange behavious of the controller
  force_torque_old.setZero();
}

void JointVariableImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());

  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d( 
      robot_state.tau_J_d.data());

  Eigen::VectorXd tau_joint(7), tau_d(7), error_vect(7), tau_joint_limit(7);

  error_vect.setZero();
  error_vect(0)=(q_d_(0) - q(0));
  error_vect(1)=(q_d_(1) - q(1));
  error_vect(2)=(q_d_(2) - q(2));
  error_vect(3)=(q_d_(3) - q(3));
  error_vect(4)=(q_d_(4) - q(4));
  error_vect(5)=(q_d_(5) - q(5));
  error_vect(6)=(q_d_(6) - q(6));
  
  tau_joint << joint_stiffness_target_ * (error_vect) -  joint_damping_target_ * (dq);

  // calculation of a repulsor when going close to a joint limit
  tau_joint_limit.setZero();
  if (q(0)>2.85)     { tau_joint_limit(0)=-10; }
  if (q(0)<-2.85)    { tau_joint_limit(0)=+10; }
  if (q(1)>1.7)      { tau_joint_limit(1)=-10; }
  if (q(1)<-1.7)     { tau_joint_limit(1)=+10; }
  if (q(2)>2.85)     { tau_joint_limit(2)=-10; }
  if (q(2)<-2.85)    { tau_joint_limit(2)=+10; }
  if (q(3)>-0.1)     { tau_joint_limit(3)=-10; }
  if (q(3)<-3.0)     { tau_joint_limit(3)=+10; }
  if (q(4)>2.85)     { tau_joint_limit(4)=-10; }
  if (q(4)<-2.85)    { tau_joint_limit(4)=+10; }
  if (q(5)>3.7)      { tau_joint_limit(5)=-10; }
  if (q(5)<-0.1)     { tau_joint_limit(5)=+10; }
  if (q(6)>2.8)      { tau_joint_limit(6)=-10; }
  if (q(6)<-2.8)     { tau_joint_limit(6)=+10; }
  // Desired torque
  tau_d << tau_joint + coriolis + tau_joint_limit;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
}

Eigen::Matrix<double, 7, 1> JointVariableImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void JointVariableImpedanceController::complianceJointParamCallback(
    franka_human_friendly_controllers::compliance_joint_paramConfig& config, uint32_t /*level*/) {
  joint_stiffness_target_.setIdentity();
  joint_stiffness_target_(0,0) = config.joint_1;
  joint_stiffness_target_(1,1) = config.joint_2;
  joint_stiffness_target_(2,2) = config.joint_3;
  joint_stiffness_target_(3,3) = config.joint_4;
  joint_stiffness_target_(4,4) = config.joint_5;
  joint_stiffness_target_(5,5) = config.joint_6;
  joint_stiffness_target_(6,6) = config.joint_7;
  damping_ratio=config.damping_ratio;
  joint_damping_target_(0,0)=2*damping_ratio* sqrt(joint_stiffness_target_(0,0)); 
  joint_damping_target_(1,1)=2*damping_ratio* sqrt(joint_stiffness_target_(1,1)) ;
  joint_damping_target_(2,2)=2*damping_ratio* sqrt(joint_stiffness_target_(2,2)) ;
  joint_damping_target_(3,3)=2*damping_ratio* sqrt(joint_stiffness_target_(3,3)) ;
  joint_damping_target_(4,4)=2*damping_ratio* sqrt(joint_stiffness_target_(4,4)) ;
  joint_damping_target_(5,5)=2*damping_ratio* sqrt(joint_stiffness_target_(5,5)) ;
  joint_damping_target_(6,6)=2*damping_ratio* sqrt(joint_stiffness_target_(6,6)) ;
}


void JointVariableImpedanceController::equilibriumConfigurationCallback( const sensor_msgs::JointState::ConstPtr& joint) {
  
  for(int i=0; i<7; ++i)
  {
    q_d_[i] = joint->position[i];
  }
  return;
}
}
  // namespace franka_human_friendly_controllers

PLUGINLIB_EXPORT_CLASS(franka_human_friendly_controllers::JointVariableImpedanceController,
                       controller_interface::ControllerBase)
