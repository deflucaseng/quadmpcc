#pragma once

#include "mpcc/config.h"

namespace mpcc {

// ── State ─────────────────────────────────────────────────────────────────────
// [X, Y, Z, qw, qx, qy, qz, vx, vy, vz, p, q, r, s, T, vs]
struct State {
    double X  = 0.0;   // world position x
    double Y  = 0.0;   // world position y
    double Z  = 0.0;   // world position z (z-up)
    double qw = 1.0;   // quaternion scalar (unit: body-to-world)
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double vx = 0.0;   // world-frame velocity x
    double vy = 0.0;
    double vz = 0.0;
    double p  = 0.0;   // body roll rate
    double q  = 0.0;   // body pitch rate
    double r  = 0.0;   // body yaw rate
    double s  = 0.0;   // arc-length progress along path
    double T  = 0.0;   // collective thrust (N)
    double vs = 0.0;   // virtual path speed (ds/dt)

    void setZero() {
        X = Y = Z = 0.0;
        qw = 1.0; qx = qy = qz = 0.0;
        vx = vy = vz = 0.0;
        p = q = r = 0.0;
        s = 0.0;
        T = 0.0;
        vs = 0.0;
    }

    // Wrap arc-length progress to [0, track_length)
    void wrapS(double track_length) {
        while (s > track_length) s -= track_length;
        while (s < 0.0)          s += track_length;
    }
};

// ── Input ─────────────────────────────────────────────────────────────────────
// [dT, dp, dq, dr, dvs]  — rates of thrust, body rates, and virtual speed
struct Input {
    double dT  = 0.0;
    double dp  = 0.0;
    double dq  = 0.0;
    double dr  = 0.0;
    double dvs = 0.0;

    void setZero() {
        dT = dp = dq = dr = dvs = 0.0;
    }
};

// ── Path specification (file paths to JSON param files + waypoints) ───────────
struct PathToJson {
    const std::string param_path;
    const std::string cost_path;
    const std::string bounds_path;
    const std::string normalization_path;
    const std::string waypoints_path;  // YAML file for PathWeaver
};

// ── Eigen type aliases ─────────────────────────────────────────────────────────
typedef Eigen::Matrix<double, NX, 1>  StateVector;
typedef Eigen::Matrix<double, NU, 1>  InputVector;

typedef Eigen::Matrix<double, NX, NX> A_MPC;
typedef Eigen::Matrix<double, NX, NU> B_MPC;
typedef Eigen::Matrix<double, NX, 1>  g_MPC;

typedef Eigen::Matrix<double, NX, NX> Q_MPC;
typedef Eigen::Matrix<double, NU, NU> R_MPC;
typedef Eigen::Matrix<double, NX, NU> S_MPC;

typedef Eigen::Matrix<double, NX, 1>  q_MPC;
typedef Eigen::Matrix<double, NU, 1>  r_MPC;

typedef Eigen::Matrix<double, NPC, NX> C_MPC;
typedef Eigen::Matrix<double, 1,  NX>  C_i_MPC;
typedef Eigen::Matrix<double, NPC, NU> D_MPC;
typedef Eigen::Matrix<double, NPC, 1>  d_MPC;

typedef Eigen::Matrix<double, NS, NS>  Z_MPC;
typedef Eigen::Matrix<double, NS, 1>   z_MPC;

typedef Eigen::Matrix<double, NX, NX>  TX_MPC;
typedef Eigen::Matrix<double, NU, NU>  TU_MPC;
typedef Eigen::Matrix<double, NS, NS>  TS_MPC;

typedef Eigen::Matrix<double, NX, 1>  Bounds_x;
typedef Eigen::Matrix<double, NU, 1>  Bounds_u;
typedef Eigen::Matrix<double, NS, 1>  Bounds_s;

// ── Conversion helpers ─────────────────────────────────────────────────────────
StateVector stateToVector(const State &x);
InputVector inputToVector(const Input &u);

State vectorToState(const StateVector &xk);
Input vectorToInput(const InputVector &uk);

State arrayToState(double *xk);
Input arrayToInput(double *uk);

} // namespace mpcc
