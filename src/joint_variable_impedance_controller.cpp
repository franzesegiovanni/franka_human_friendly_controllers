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
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;


  sub_stiffness_ = node_handle.subscribe(
    "/stiffness", 20, &JointVariableImpedanceController::equilibriumStiffnessCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());
  // sub_equilibrium_pose_ik = node_handle.subscribe(
  //       "/equilibrium_pose", 1, &JointVariableImpedanceController::equilibriumConfigurationIKCallback, this,
  //       ros::TransportHints().reliable().tcpNoDelay());
  sub_equilibrium_config_ = node_handle.subscribe(
      "/equilibrium_configuration", 20, &JointVariableImpedanceController::equilibriumConfigurationCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  pub_stiff_update_ = node_handle.advertise<dynamic_reconfigure::Config>(
    "/dynamic_reconfigure_compliance_param_node/parameter_updates", 5);


  pub_cartesian_pose_= node_handle.advertise<geometry_msgs::PoseStamped>("/cartesian_pose",1);
  pub_force_torque_= node_handle.advertise<geometry_msgs::WrenchStamped>("/force_torque_ext",1);

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

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  stiff_.setZero();

  return true;
}

void JointVariableImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  q_d_ = q_initial;
  force_torque_old.setZero();
  double time_old=ros::Time::now().toSec();
}

void JointVariableImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
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
  double time_=ros::Time::now().toSec();

  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_ext(robot_state.tau_ext_hat_filtered.data());
  std::array<double, 7> gravity = model_handle_->getGravity();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Matrix<double, 7, 1>  tau_f;
  Eigen::MatrixXd jacobian_transpose_pinv;
  Eigen::MatrixXd Null_mat;

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
  filter_step=filter_step+1;
  filter_step_=10;
  alpha=1;
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
  pub_cartesian_pose_.publish(pose_msg);
  // compute error to desired pose

  Eigen::VectorXd tau_joint(7), tau_d(7), error_vect(7), tau_joint_limit(7);


  error_vect.setZero();
  // error_vect(0)=std::max(-0.1,std::min((q_d_(0) - q(0)),0.1));
  // error_vect(1)=std::max(-0.1,std::min((q_d_(1) - q(1)),0.1));
  // error_vect(2)=std::max(-0.1,std::min((q_d_(2) - q(2)),0.1));
  // error_vect(3)=std::max(-0.1,std::min((q_d_(3) - q(3)),0.1));
  // error_vect(4)=std::max(-0.1,std::min((q_d_(4) - q(4)),0.1));
  // error_vect(5)=std::max(-0.1,std::min((q_d_(5) - q(5)),0.1));
  // error_vect(6)=std::max(-0.1,std::min((q_d_(6) - q(6)),0.1));
  error_vect(0)=(q_d_(0) - q(0));
  error_vect(1)=(q_d_(1) - q(1));
  error_vect(2)=(q_d_(2) - q(2));
  error_vect(3)=(q_d_(3) - q(3));
  error_vect(4)=(q_d_(4) - q(4));
  error_vect(5)=(q_d_(5) - q(5));
  error_vect(6)=(q_d_(6) - q(6));
  tau_joint << joint_stiffness_target_ * (error_vect) -  joint_damping_target_ * (dq); //double critic damping
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
  tau_d << tau_joint + tau_joint_limit;
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

void JointVariableImpedanceController::equilibriumStiffnessCallback(
    const std_msgs::Float32MultiArray::ConstPtr& stiffness_){

  int i = 0;
  // print all the remaining numbers
  for(std::vector<float>::const_iterator it = stiffness_->data.begin(); it != stiffness_->data.end(); ++it)
  {
    stiff_[i] = *it;
    i++;
  }
  for (int i = 0; i < 7; i++){
  for (int j = 0; j < 7; j++) {
  joint_stiffness_target_(i,j)=std::max(std::min(stiff_[i+j], float(100.0)), float(0.0));
  }
  }

  ROS_INFO_STREAM("Stiffness matrix is:" << joint_stiffness_target_);
  calculateDamping(q_d_); //check what damping ratio is actually taking
  ROS_INFO_STREAM("Damping matrix is:" << joint_damping_target_);
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
  joint_damping_target_(0,0)=2*damping_ratio*joint_stiffness_target_(0,0); 
  joint_damping_target_(1,1)=2*damping_ratio*joint_stiffness_target_(1,1) ;
  joint_damping_target_(2,2)=2*damping_ratio*joint_stiffness_target_(2,2) ;
  joint_damping_target_(3,3)=2*damping_ratio*joint_stiffness_target_(3,3) ;
  joint_damping_target_(4,4)=2*damping_ratio*joint_stiffness_target_(4,4) ;
  joint_damping_target_(5,5)=2*damping_ratio*joint_stiffness_target_(5,5) ;
  joint_damping_target_(5,5)=2*damping_ratio*joint_stiffness_target_(6,6) ;
  ROS_INFO_STREAM("Stiffness matrix is:" << joint_stiffness_target_);
  ROS_INFO_STREAM("Damping matrix is:" << joint_damping_target_);
}


void JointVariableImpedanceController::equilibriumConfigurationCallback( const sensor_msgs::JointState::ConstPtr& joint) {
  int i = 0;
  Eigen::Matrix<double, 7, 1> q_d_damp;

  for(int i=0; i<7; ++i)
  {
    q_d_damp[i] = joint->position[i];
  }

  calculateDamping(q_d_damp);
  ROS_INFO_STREAM("Stiffness matrix is:" << joint_stiffness_target_);
  ROS_INFO_STREAM("Damping matrix is:" << joint_damping_target_);

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
