// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_human_friendly_controllers/cartesian_variable_impedance_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <franka_human_friendly_controllers/franka_model.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka_human_friendly_controllers/pseudo_inversion.h>
namespace franka_human_friendly_controllers {

bool CartesianVariableImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "/equilibrium_pose", 20, &CartesianVariableImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_equilibrium_config_ = node_handle.subscribe(
      "/equilibrium_configuration", 20, &CartesianVariableImpedanceController::equilibriumConfigurationCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  // We want to add the subscriber to the note for reading the desired stiffness in the different directions
  sub_stiffness_ = node_handle.subscribe(
    "/stiffness", 20, &CartesianVariableImpedanceController::equilibriumStiffnessCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());
  sub_rot_stiffness_ = node_handle.subscribe(
    "/stiffness_rotation", 20, &CartesianVariableImpedanceController::StiffnessEllipsoidPoseCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());  
  sub_vibration_ = node_handle.subscribe(
      "/vibration", 20, &CartesianVariableImpedanceController::equilibriumVibrationCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  pub_stiff_update_ = node_handle.advertise<dynamic_reconfigure::Config>(
    "/dynamic_reconfigure_compliance_param_node/parameter_updates", 5);

  pub_cartesian_pose_= node_handle.advertise<geometry_msgs::PoseStamped>("/cartesian_pose",1);

  pub_force_torque_= node_handle.advertise<geometry_msgs::WrenchStamped>("/force_torque_ext",1);

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

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianVariableImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianVariableImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_.reset(
      new dynamic_reconfigure::Server<franka_human_friendly_controllers::compliance_paramConfig>(
          dynamic_reconfigure_compliance_param_node_));
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianVariableImpedanceController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  //position_d_target_.setZero();
  //orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  stiff_.setZero();

  return true;
}

void CartesianVariableImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  // set equilibrium point to current state
  position_d_ = initial_transform.translation(); // this allows the robot to start on the starting configuration
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear()); // this allows the robot to start on the starting configuration
  //position_d_target_ = initial_transform.translation();
  //orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
  force_torque_old.setZero();
  double time_old=ros::Time::now().toSec();
  count_vibration=1000;
}

void CartesianVariableImpedanceController::update(const ros::Time& /*time*/,
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
  // Compute the value of the friction
  tau_f(0) =  FI_11/(1+exp(-FI_21*(dq(0)+FI_31))) - TAU_F_CONST_1;
  tau_f(1) =  FI_12/(1+exp(-FI_22*(dq(1)+FI_32))) - TAU_F_CONST_2;
  tau_f(2) =  FI_13/(1+exp(-FI_23*(dq(2)+FI_33))) - TAU_F_CONST_3;
  tau_f(3) =  FI_14/(1+exp(-FI_24*(dq(3)+FI_34))) - TAU_F_CONST_4;
  tau_f(4) =  FI_15/(1+exp(-FI_25*(dq(4)+FI_35))) - TAU_F_CONST_5;
  tau_f(5) =  FI_16/(1+exp(-FI_26*(dq(5)+FI_36))) - TAU_F_CONST_6;
  tau_f(6) =  FI_17/(1+exp(-FI_27*(dq(6)+FI_37))) - TAU_F_CONST_7;

   //Low pass filter for the external force estimation
  float iCutOffFrequency=10.0;
  force_torque+=(-jacobian_transpose_pinv*(tau_ext-tau_f)-force_torque)*(1-exp(-0.001 * 2.0 * M_PI * iCutOffFrequency));
  geometry_msgs::WrenchStamped force_torque_msg;
  force_torque_msg.wrench.force.x=force_torque[0];
  force_torque_msg.wrench.force.y=force_torque[1];
  force_torque_msg.wrench.force.z=force_torque[2];
  force_torque_msg.wrench.torque.x=force_torque[3];
  force_torque_msg.wrench.torque.y=force_torque[4];
  force_torque_msg.wrench.torque.z=force_torque[5];
  pub_force_torque_.publish(force_torque_msg);
  
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose.position.x=position[0];
  pose_msg.pose.position.y=position[1];
  pose_msg.pose.position.z=position[2];
  pose_msg.pose.orientation.x=orientation.x();
  pose_msg.pose.orientation.y=orientation.y();
  pose_msg.pose.orientation.z=orientation.z();
  pose_msg.pose.orientation.w=orientation.w();
  pub_cartesian_pose_.publish(pose_msg);
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), null_vect(7), tau_joint_limit(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse

  Null_mat=(Eigen::MatrixXd::Identity(7, 7) -jacobian.transpose() * jacobian_transpose_pinv);
  null_vect.setZero();
  null_vect(0)=(q_d_nullspace_(0) - q(0));
  null_vect(1)=(q_d_nullspace_(1) - q(1));
  null_vect(2)=(q_d_nullspace_(2) - q(2));
  null_vect(3)=(q_d_nullspace_(3) - q(3));
  null_vect(4)=(q_d_nullspace_(4) - q(4));
  null_vect(5)=(q_d_nullspace_(5) - q(5));
  null_vect(6)=(q_d_nullspace_(6) - q(6));
  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error -  cartesian_damping_ * (jacobian * dq)); //double critic damping
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * null_vect -
                        1*(2.0 * sqrt(nullspace_stiffness_)) * dq); //double critic damping
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
  tau_d << tau_task + tau_nullspace + coriolis+ tau_joint_limit;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  if (count_vibration<1000.0*duration_vibration){tau_d(6)=tau_d(6)+5.0*sin(100.0/1000.0*2.0*3.14*count_vibration);
  count_vibration=count_vibration+1;
  //ROS_INFO_STREAM("count_vibration" << count_vibration << "tau" << tau_d);
}

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  cartesian_stiffness_ =cartesian_stiffness_target_;
  cartesian_damping_ = cartesian_damping_target_;
  nullspace_stiffness_ = nullspace_stiffness_target_;
  Eigen::AngleAxisd aa_orientation_d(orientation_d_);
  orientation_d_ = Eigen::Quaterniond(aa_orientation_d);
}

Eigen::Matrix<double, 7, 1> CartesianVariableImpedanceController::saturateTorqueRate(
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

void CartesianVariableImpedanceController::equilibriumStiffnessCallback(
    const std_msgs::Float32MultiArray::ConstPtr& stiffness_){

  int i = 0;
  // print all the remaining numbers
  for(std::vector<float>::const_iterator it = stiffness_->data.begin(); it != stiffness_->data.end(); ++it)
  {
    stiff_[i] = *it;
    i++;
  }

  cartesian_stiffness_target_(0,0)=std::max(std::min(stiff_[0], float(4000.0)), float(0.0));
  cartesian_stiffness_target_(1,1)=std::max(std::min(stiff_[1], float(4000.0)), float(0.0));
  cartesian_stiffness_target_(2,2)=std::max(std::min(stiff_[2], float(4000.0)), float(0.0));

  cartesian_damping_target_(0,0)=2.0 * sqrt(cartesian_stiffness_target_(0,0));
  cartesian_damping_target_(1,1)=2.0 * sqrt(cartesian_stiffness_target_(1,1));
  cartesian_damping_target_(2,2)=2.0 * sqrt(cartesian_stiffness_target_(2,2));

  cartesian_stiffness_target_(3,3)=std::max(std::min(stiff_[3], float(50.0)), float(0.0));
  cartesian_stiffness_target_(4,4)=std::max(std::min(stiff_[4], float(50.0)), float(0.0));
  cartesian_stiffness_target_(5,5)=std::max(std::min(stiff_[5], float(50.0)), float(0.0));

  cartesian_damping_target_(3,3)=2.0 * sqrt(cartesian_stiffness_target_(3,3));
  cartesian_damping_target_(4,4)=2.0 * sqrt(cartesian_stiffness_target_(4,4));
  cartesian_damping_target_(5,5)=2.0 * sqrt(cartesian_stiffness_target_(5,5));

  nullspace_stiffness_target_= std::max(std::min(stiff_[6], float(50.0)), float(0.0));


  dynamic_reconfigure::Config set_Kx;
  dynamic_reconfigure::DoubleParameter param_X_double;
  param_X_double.name = "translational_stiffness_X";
  param_X_double.value = cartesian_stiffness_target_(0,0);
  set_Kx.doubles = {param_X_double};
  pub_stiff_update_.publish(set_Kx);

  dynamic_reconfigure::Config set_Ky;
  dynamic_reconfigure::DoubleParameter param_Y_double;
  param_Y_double.name = "translational_stiffness_Y";
  param_Y_double.value = cartesian_stiffness_target_(1,1);
  set_Ky.doubles = {param_Y_double};
  pub_stiff_update_.publish(set_Ky);

  dynamic_reconfigure::Config set_Kz;
  dynamic_reconfigure::DoubleParameter param_Z_double;
  param_Z_double.name = "translational_stiffness_Z";
  param_Z_double.value = cartesian_stiffness_target_(2,2);
  set_Kz.doubles = {param_Z_double};
  pub_stiff_update_.publish(set_Kz);

  dynamic_reconfigure::Config set_K_alpha;
  dynamic_reconfigure::DoubleParameter param_alpha_double;
  param_alpha_double.name = "rotational_stiffness_X";
  param_alpha_double.value = cartesian_stiffness_target_(3,3);
  set_K_alpha.doubles = {param_alpha_double};
  pub_stiff_update_.publish(set_K_alpha);

  dynamic_reconfigure::Config set_K_beta;
  dynamic_reconfigure::DoubleParameter param_beta_double;
  param_beta_double.name = "rotational_stiffness_Y";
  param_beta_double.value = cartesian_stiffness_target_(4,4);
  set_K_beta.doubles = {param_beta_double};
  pub_stiff_update_.publish(set_K_beta);

  dynamic_reconfigure::Config set_K_gamma;
  dynamic_reconfigure::DoubleParameter param_gamma_double;
  param_gamma_double.name = "rotational_stiffness_Z";
  param_gamma_double.value = cartesian_stiffness_target_(5,5);
  set_K_gamma.doubles = {param_gamma_double};
  pub_stiff_update_.publish(set_K_gamma);

  dynamic_reconfigure::Config set_nullspace;
  dynamic_reconfigure::DoubleParameter param_nullspace_double;
  param_nullspace_double.name = "nullspace_stiffness";
  param_nullspace_double.value = nullspace_stiffness_target_;
  set_nullspace.doubles = {param_nullspace_double};
  pub_stiff_update_.publish(set_nullspace);

}

void CartesianVariableImpedanceController::complianceParamCallback(
    franka_human_friendly_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_(0,0)=config.translational_stiffness_X;
  cartesian_stiffness_target_(1,1)=config.translational_stiffness_Y;
  cartesian_stiffness_target_(2,2)=config.translational_stiffness_Z;
  cartesian_stiffness_target_(3,3)=config.rotational_stiffness_X;
  cartesian_stiffness_target_(4,4)=config.rotational_stiffness_Y;
  cartesian_stiffness_target_(5,5)=config.rotational_stiffness_Z;

  cartesian_damping_target_(0,0)=2.0 * sqrt(config.translational_stiffness_X);
  cartesian_damping_target_(1,1)=2.0 * sqrt(config.translational_stiffness_Y);
  cartesian_damping_target_(2,2)=2.0 * sqrt(config.translational_stiffness_Z);
  cartesian_damping_target_(3,3)=2.0 * sqrt(config.rotational_stiffness_X);
  cartesian_damping_target_(4,4)=2.0 * sqrt(config.rotational_stiffness_Y);
  cartesian_damping_target_(5,5)=2.0 * sqrt(config.rotational_stiffness_Z);
  nullspace_stiffness_target_ = config.nullspace_stiffness;

  // Use this if you want to use rotation matrix to change the orientation of the stiffness matrix
  // Eigen::AngleAxisd rollAngle(config.roll, Eigen::Vector3d::UnitX());
  // Eigen::AngleAxisd yawAngle(config.yaw, Eigen::Vector3d::UnitZ());
  // Eigen::AngleAxisd pitchAngle(config.pitch, Eigen::Vector3d::UnitY());
  // Eigen::Quaternion<double> q = rollAngle *  pitchAngle * yawAngle;
  // Eigen::Matrix3d StiffRotationMatrix = q.matrix();
  
  // cartesian_stiffness_target_.block(0, 0, 3, 3)=StiffRotationMatrix*cartesian_stiffness_target_.block(0, 0, 3, 3)*StiffRotationMatrix.transpose();
  // cartesian_stiffness_target_.block(3, 3, 6, 6)=StiffRotationMatrix*cartesian_stiffness_target_.block(3, 3, 6, 6)*StiffRotationMatrix.transpose();

  // cartesian_damping_target_.block(0, 0, 3, 3)=StiffRotationMatrix*cartesian_damping_target_.block(0, 0, 3, 3)*StiffRotationMatrix.transpose();
  // cartesian_damping_target_.block(3, 3, 6, 6)=StiffRotationMatrix*cartesian_damping_target_.block(3, 3, 6, 6)*StiffRotationMatrix.transpose();

}



void CartesianVariableImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_(orientation_d_);
  orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
    orientation_d_.coeffs() << -orientation_d_.coeffs();
}
}

void CartesianVariableImpedanceController::StiffnessEllipsoidPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
      // This subscriver is used to receive a pose and rotate the rotation matrix of that amount
  Eigen::Quaterniond last_orientation_d_stiff_(orientation_d_stiff_);
  orientation_d_stiff_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_stiff_.coeffs().dot(orientation_d_stiff_.coeffs()) < 0.0) {
    orientation_d_stiff_.coeffs() << -orientation_d_stiff_.coeffs();

  Eigen::Matrix3d StiffRotationMatrix = orientation_d_stiff_.toRotationMatrix(); 

  // Eigen::Vector3d euler = orientation_d_stiff_.toRotationMatrix().eulerAngles(0, 1, 2);

  cartesian_stiffness_target_.block(0, 0, 3, 3)=StiffRotationMatrix*cartesian_stiffness_target_.block(0, 0, 3, 3)*StiffRotationMatrix.transpose();
  cartesian_stiffness_target_.block(3, 3, 6, 6)=StiffRotationMatrix*cartesian_stiffness_target_.block(3, 3, 6, 6)*StiffRotationMatrix.transpose();
  // ROS_INFO_STREAM("stiffness translational matrix is:" << cartesian_stiffness_linear_target_rotated_ );

  cartesian_damping_target_.block(0, 0, 3, 3)=StiffRotationMatrix*cartesian_damping_target_.block(0, 0, 3, 3)*StiffRotationMatrix.transpose();
  cartesian_damping_target_.block(3, 3, 6, 6)=StiffRotationMatrix*cartesian_damping_target_.block(3, 3, 6, 6)*StiffRotationMatrix.transpose();
  // ROS_INFO_STREAM("damping translational matrix is:" << cartesian_damping_linear_target_rotated_ );

}
}

void CartesianVariableImpedanceController::equilibriumConfigurationCallback( const std_msgs::Float32MultiArray::ConstPtr& joint) {
  int i = 0;
  for(std::vector<float>::const_iterator it = joint->data.begin(); it != joint->data.end(); ++it)
  {
    q_d_nullspace_[i] = *it;
    i++;
  }
  return;
}
void CartesianVariableImpedanceController::equilibriumVibrationCallback( const std_msgs::Float32::ConstPtr& vibration_msg) {
  count_vibration = 0;
  duration_vibration = vibration_msg->data;

}


}  // namespace franka_human_friendly_controllers

PLUGINLIB_EXPORT_CLASS(franka_human_friendly_controllers::CartesianVariableImpedanceController,
                       controller_interface::ControllerBase)
