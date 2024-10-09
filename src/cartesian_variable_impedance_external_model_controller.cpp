#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <franka_human_friendly_controllers/cartesian_variable_impedance_external_model_controller.h>
#include <ros/package.h>


namespace franka_human_friendly_controllers {


void CartesianVariableImpedanceExternalModelController::loadModel() {
  std::string package_path = ros::package::getPath("franka_human_friendly_controllers");
  urdf_path_ = package_path + "/urdf/panda_calibrated_grounded.urdf";
  std::cout << urdf_path_ << std::endl;

  std::cout << "Loading urdf into pinocchio as we are using the urdf model" << std::endl;
  pinocchio::urdf::buildModel(urdf_path_, model_pin_);
  data_pin_ = new pinocchio::Data(model_pin_);
  std::cout << "Succesfully loaded the model and created the data pointer." << std::endl;
}

double* CartesianVariableImpedanceExternalModelController::get_fk(franka::RobotState robot_state)
{
  Eigen::Map<Eigen::Matrix<double, 9, 1>> q(robot_state.q.data());
  Eigen::VectorXd q_vector = Eigen::VectorXd::Map(q.data(), q.size());
  int frame_id = model_pin_.getFrameId("panda_hand");

  pinocchio::forwardKinematics(model_pin_, *data_pin_, q_vector);
  pinocchio::updateFramePlacement(model_pin_, *data_pin_, frame_id);
  const auto& transformation = data_pin_->oMf[frame_id];  // Get the transformation of the frame
  
  // Allocate memory for the result
  double* result = new double[16];
  std::memcpy(result, transformation.toHomogeneousMatrix().data(), 16 * sizeof(double));
  return result; // Caller is responsible for deleting the allocated memory
}

}  // namespace franka_human_friendly_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_human_friendly_controllers::CartesianVariableImpedanceExternalModelController,
    controller_interface::ControllerBase
)
