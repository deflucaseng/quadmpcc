// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "Model/model.h"

#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
namespace mpcc {
Model::Model() : Ts_(1.0) {
  std::cout << "default constructor, not everything is initialized properly"
            << std::endl;
}

Model::Model(double Ts, const PathToJson &path)
    : Ts_(Ts), param_(Param(path.param_path)) {
  // Check if parameters have changed
  std::hash<std::string> hasher;

  std::string base_path =
      path.param_path.substr(0, path.param_path.find_last_of("/\\"));
  std::ifstream iConfig(base_path + "/config.json");
  if (!iConfig.is_open()) {
    throw std::runtime_error("Could not open config.json at " + base_path +
                             "/config.json");
  }
  json jsonConfig;
  iConfig >> jsonConfig;
  size_t current_config_hash = hasher(jsonConfig.dump());

  std::string integrator_name = "RK4";
  if (jsonConfig.contains("integrator")) {
    std::string int_str = jsonConfig["integrator"];
    if (int_str == "Euler" || int_str == "ForwardEuler") {
      integrator_name = "Euler";
    }
  }

  std::ifstream iModel(path.param_path);
  if (!iModel.is_open()) {
    throw std::runtime_error("Could not open model.json at " + path.param_path);
  }
  json jsonModel;
  iModel >> jsonModel;
  size_t current_model_hash = hasher(jsonModel.dump());

  std::ifstream hashFile(path.adcodegen_path + "/params_hash.txt");
  if (!hashFile.is_open()) {
    throw std::runtime_error(
        "Could not open params_hash.txt. Please run ADCodeGen.");
  }
  size_t stored_config_hash, stored_model_hash;
  hashFile >> stored_config_hash >> stored_model_hash;

  if (current_config_hash != stored_config_hash ||
      current_model_hash != stored_model_hash) {
    throw std::runtime_error(
        "Parameters have changed since last binary "
        "generation. Please re-run ADCodeGen.");
  }

  integrator_lib_ = (std::make_unique<CppAD::cg::LinuxDynamicLib<double>>(
      path.adcodegen_path + "/cppad_cg_" + integrator_name + ".so"));
  f_dyn_lib_ = (std::make_unique<CppAD::cg::LinuxDynamicLib<double>>(
      path.adcodegen_path + "/cppad_cg_f_dyn.so"));
  integrator_model_ = integrator_lib_->model(integrator_name);
  f_dyn_model_ = f_dyn_lib_->model("f_dyn");
}

StateVector Model::getF(const State &x, const Input &u) const {
  return Eigen::Map<StateVector>(
      (f_dyn_model_->ForwardZero(stateInputToVector(x, u))).data());
}

LinModelMatrix Model::discretizeModel(const State &x, const Input &u,
                                      const State &x_next) const {
  // State x_lin = x;
  // x_lin.vxNonZero(param_.vx_zero);
  // std::vector<double> x_v_lin = stateInputToVector(x_lin,u);
  std::vector<double> x_v = stateInputToVector(x, u);
  Eigen::MatrixXd jac_vec = Eigen::Map<Eigen::Matrix<double, NX *(NX + NU), 1>>(
      (integrator_model_->Jacobian(x_v)).data());
  jac_vec.resize(NX + NU, NX);
  Eigen::MatrixXd jac = jac_vec.transpose();

  const A_MPC A_d = jac.block<NX, NX>(0, 0);
  const B_MPC B_d = jac.block<NX, NU>(0, NX);

  StateVector x_integrator = Eigen::Map<Eigen::Matrix<double, NX, 1>>(
      (integrator_model_->ForwardZero(x_v)).data());
  const g_MPC g_d = -stateToVector(x_next) + x_integrator;
  return {A_d, B_d, g_d};
}

LinModelMatrix Model::getLinModel(const State &x, const Input &u,
                                  const State &x_next) const {
  // discretize the system
  return discretizeModel(x, u, x_next);
}
}  // namespace mpcc
