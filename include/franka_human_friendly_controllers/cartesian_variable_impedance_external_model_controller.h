// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <franka_human_friendly_controllers/cartesian_variable_impedance_controller.h>

namespace franka_human_friendly_controllers {

class CartesianVariableImpedanceExternalModelController : public CartesianVariableImpedanceController {
  private:
    std::string urdf_path_;
    pinocchio::Model model_pin_;
    pinocchio::Data* data_pin_;
  public:
    double* get_fk(franka::RobotState robot_state) override;
    void loadModel() override;
};

}  // namespace franka_human_friendly_controllers
