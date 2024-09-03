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
  sub_vibration_ = node_handle.subscribe(
      "/haptic_feedback", 20, &CartesianVariableImpedanceController::equilibriumVibrationCallback, this,
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
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  stiff_.setZero();

      // Define variables to store parameter values
    const std::string limit_types[2] = {"lower", "upper"};

  // Read parameters from the parameter server
  for (int i = 0; i < 7; ++i) {
      for (const std::string& limit_type : limit_types) {
          std::string param_name = "/joint" + std::to_string(i + 1) + "/limit/" + limit_type;
          if (!node_handle.getParam(param_name, joint_limits[i][limit_type == "lower" ? 0 : 1])) {
              ROS_ERROR("Failed to retrieve parameter: %s", param_name.c_str());
              return 1;
          }
      }
  }

    // Stream the parameter values
    ROS_INFO("Joint limits:");
    for (int i = 0; i < 7; ++i) {
        ROS_INFO("Joint %d: lower=%.4f, upper=%.4f", i + 1, joint_limits[i][0], joint_limits[i][1]);
    }

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
    std::array<double, 42> jacobian_array_adaptive =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian_adaptive(jacobian_array_adaptive.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  // set equilibrium point to current state
  position_d_ = initial_transform.translation(); // this allows the robot to start on the starting configuration
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear()); // this allows the robot to start on the
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
  std::array<double, 42> jacobian_array_adaptive =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian_adaptive(jacobian_array_adaptive.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  double time_=ros::Time::now().toSec();

  // this line allows to have the nullspace active in the degree of freedom that do not have stiffness anymore. 
  //So if you pull the cartesian stiffness down, the control goes from cartesian plus nullspace control to pure joint control
  for(int i=0; i<6; i++){
     if (cartesian_stiffness_target_(i,i)==0){
        for(int j=0; j<7; j++){ jacobian_adaptive(i, j) = 0; } }}

  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_ext(robot_state.tau_ext_hat_filtered.data());
  std::array<double, 7> gravity = model_handle_->getGravity();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  Eigen::Matrix<double, 7, 1>  tau_f;
  Eigen::MatrixXd jacobian_transpose_pinv;
  Eigen::MatrixXd jacobian_transpose_pinv_adaptive;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  pseudoInverse(jacobian_adaptive.transpose(), jacobian_transpose_pinv_adaptive);
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

  error[0]=std::max(-delta_lim_lin, std::min(error[0], delta_lim_lin));
  error[1]=std::max(-delta_lim_lin, std::min(error[1], delta_lim_lin));
  error[2]=std::max(-delta_lim_lin, std::min(error[2], delta_lim_lin));
  error[3]=std::max(-delta_lim_ori, std::min(error[3], delta_lim_ori));
  error[4]=std::max(-delta_lim_ori, std::min(error[4], delta_lim_ori));
  error[5]=std::max(-delta_lim_ori, std::min(error[5], delta_lim_ori));
  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), null_vect(7), tau_joint_limit(7), tau_joint_limit_nullspace(7), tau_joint_limit_ns(7), tau_default_joint_damping(7), tau_vibration(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse

  null_vect.setZero();
  for (int i = 0; i < 7; ++i) {
      null_vect(i) = q_d_nullspace_(i) - q(i);
  }

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error -  cartesian_damping_ * (jacobian * dq)); //double critic damping
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian_adaptive.transpose() * jacobian_transpose_pinv_adaptive) *
                       (nullspace_stiffness_ * null_vect -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq); //double critic damping
  tau_joint_limit.setZero();
  tau_joint_limit_nullspace.setZero();
  // (double q_value, double threshold, double magnitude, double upper_bound, double lower_bound) 

for (int i = 0; i < 7; ++i) {
    tau_joint_limit(i) = calculateTauJointLimit(q(i), 0.05, 4.0, joint_limits[i][1], joint_limits[i][0]);
    tau_joint_limit_nullspace(i) = calculateTauJointLimit(q(i), 0.1, 4.0, joint_limits[i][1], joint_limits[i][0]);
    tau_joint_limit(i) = std::max(std::min(tau_joint_limit(i), 5.0), -5.0);
    tau_joint_limit_nullspace(i) = std::max(std::min(tau_joint_limit_nullspace(i), 5.0), -5.0);
}

tau_joint_limit_ns << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       tau_joint_limit_nullspace;

tau_default_joint_damping.setZero();
tau_default_joint_damping << -joint_default_damping_ * dq;

tau_vibration.setZero();
if (count_vibration<1000.0*duration_vibration){
  tau_vibration(5)=2.0*sin(100.0/1000.0*2.0*3.14*count_vibration);
  tau_vibration(6)=2.0*sin(100.0/1000.0*2.0*3.14*count_vibration);
count_vibration=count_vibration+1;}
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis+ tau_joint_limit+ tau_joint_limit_ns+ tau_default_joint_damping + tau_vibration;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // cartesian_stiffness_ =cartesian_stiffness_target_;
  // cartesian_damping_ = cartesian_damping_target_;
  cartesian_stiffness_ =cartesian_stiffness_+ 0.02*(cartesian_stiffness_target_-cartesian_stiffness_);
  cartesian_damping_ =cartesian_damping_+ 0.02*(cartesian_damping_target_-cartesian_damping_);
  nullspace_stiffness_ = nullspace_stiffness_+ 0.02*(nullspace_stiffness_target_-nullspace_stiffness_);
  Eigen::AngleAxisd aa_orientation_d(orientation_d_);
  orientation_d_ = Eigen::Quaterniond(aa_orientation_d);
}


double CartesianVariableImpedanceController::calculateTauJointLimit(double q_value, double threshold, double magnitude, double upper_bound, double lower_bound) {
    double upper_limit = upper_bound - threshold;
    double lower_limit = lower_bound + threshold;
    if (q_value > (upper_limit)) {
        return -magnitude * (std::exp( std::abs( q_value - upper_limit )/threshold) - 1);
    } else if (q_value < lower_limit) {
        return +magnitude * (std::exp( std::abs( q_value - lower_limit )/threshold) - 1);
    } else {
        return 0;
    }
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
  joint_default_damping_=config.joint_default_damping;
  nullspace_stiffness_target_ = config.nullspace_stiffness;
  delta_lim_lin=config.max_delta_lin;
  delta_lim_ori=config.max_delta_ori;
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
