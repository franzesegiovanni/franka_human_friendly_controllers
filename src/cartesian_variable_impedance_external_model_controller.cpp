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
  pinocchio::Model model_pin_;
  pinocchio::urdf::buildModel(urdf_path_, model_pin_);
  pinocchio::Data data_pin_(model_pin_);
}


double* CartesianVariableImpedanceExternalModelController::get_fk(franka::RobotState robot_state)
{

  // update the model
  //const auto& frame_name = model.frames[frame_id].name;
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  std::cout << q << std::endl;
  pinocchio::forwardKinematics(model_pin_, data_pin_, q);
  const auto& transformation = data_pin_.oMf[5];  // Get the transformation of the frame
  std::cout << q << std::endl;
  return new double[16];
}

}  // namespace franka_human_friendly_controllers

PLUGINLIB_EXPORT_CLASS(
    franka_human_friendly_controllers::CartesianVariableImpedanceExternalModelController,
    controller_interface::ControllerBase
)
