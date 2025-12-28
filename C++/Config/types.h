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

#ifndef MPCC_TYPES_H
#define MPCC_TYPES_H

#include <algorithm>
#include <string>
#include <vector>

#include "Config/config.h"

namespace mpcc {
struct State {
  double X;
  double Y;
  double phi;
  double vx;
  double vy;
  double r;
  double s;
  double D;
  double B;
  double delta;
  double vs;

  void setZero() {
    X = 0.0;
    Y = 0.0;
    phi = 0.0;
    vx = 0.0;
    vy = 0.0;
    r = 0.0;
    s = 0.0;
    D = 0.0;
    B = 0.0;
    delta = 0.0;
    vs = 0.0;
  }

  void unwrap(double track_length) {
    while (phi > PI) phi -= 2.0 * PI;
    while (phi < -PI) phi += 2.0 * PI;

    s = fmod(s, track_length);
    if (s < 0) s += track_length;
  }

  void vxNonZero(double vx_zero) {
    if (vx < vx_zero) {
      vx = vx_zero;
      vy = 0.0;
      r = 0.0;
      delta = 0.0;
    }
  }

  void vxNonZero1(double vx_zero) {
    if (vx < vx_zero) {
      vx = vx_zero;
    }
    vx = std::max(vx, 5.0);
  }
};

struct Input {
  double dD;
  double dB;
  double dDelta;
  double dVs;

  void setZero() {
    dD = 0.0;
    dB = 0.0;
    dDelta = 0.0;
    dVs = 0.0;
  }
};

struct PathToJson {
  const std::string param_path;
  const std::string cost_path;
  const std::string bounds_path;
  const std::string track_path;
  const std::string normalization_path;
  const std::string adcodegen_path;
};

using StateVector = Eigen::Matrix<double, NX, 1>;
using InputVector = Eigen::Matrix<double, NU, 1>;

using A_MPC = Eigen::Matrix<double, NX, NX>;
using B_MPC = Eigen::Matrix<double, NX, NU>;
using g_MPC = Eigen::Matrix<double, NX, 1>;

using Q_MPC = Eigen::Matrix<double, NX, NX>;
using R_MPC = Eigen::Matrix<double, NU, NU>;
using S_MPC = Eigen::Matrix<double, NX, NU>;

using q_MPC = Eigen::Matrix<double, NX, 1>;
using r_MPC = Eigen::Matrix<double, NU, 1>;

using C_MPC = Eigen::Matrix<double, NPC, NX>;
using C_i_MPC = Eigen::Matrix<double, 1, NX>;
using D_MPC = Eigen::Matrix<double, NPC, NU>;
using d_MPC = Eigen::Matrix<double, NPC, 1>;

using Z_MPC = Eigen::Matrix<double, NS, NS>;
using z_MPC = Eigen::Matrix<double, NS, 1>;

using TX_MPC = Eigen::Matrix<double, NX, NX>;
using TU_MPC = Eigen::Matrix<double, NU, NU>;
using TS_MPC = Eigen::Matrix<double, NS, NS>;

using Bounds_x = Eigen::Matrix<double, NX, 1>;
using Bounds_u = Eigen::Matrix<double, NU, 1>;
using Bounds_s = Eigen::Matrix<double, NS, 1>;

StateVector stateToVector(const State &x);
InputVector inputToVector(const Input &u);

State vectorToState(const StateVector &xk);
Input vectorToInput(const InputVector &uk);

State arrayToState(double *xk);
Input arrayToInput(double *uk);

std::vector<double> stateInputToVector(const State x, const Input u);
}  // namespace mpcc
#endif  // MPCC_TYPES_H
