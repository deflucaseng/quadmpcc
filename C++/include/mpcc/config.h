#pragma once

#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace mpcc {

// Quadrotor state: [X, Y, Z, qw, qx, qy, qz, vx, vy, vz, p, q, r, s, T, vs]
#define NX 16
// Inputs: [dT, dp, dq, dr, dvs]
#define NU 5
// Polytopic tunnel constraints (4 halfspace walls from PathWeaver)
#define NPC 4
// Soft constraint slacks (one per tunnel wall)
#define NS 4
// Max bounds (NX + NU)
#define NB 21

static constexpr int N = 60;
static constexpr double INF = 1e5;

struct StateInputIndex {
    // Position
    int X  = 0;
    int Y  = 1;
    int Z  = 2;
    // Quaternion (body-to-world, FLU body convention)
    int qw = 3;
    int qx = 4;
    int qy = 5;
    int qz = 6;
    // World-frame velocity
    int vx = 7;
    int vy = 8;
    int vz = 9;
    // Body rates
    int p  = 10;
    int q  = 11;
    int r  = 12;
    // Arc-length progress
    int s  = 13;
    // Collective thrust (N) — treated as state for rate formulation
    int T  = 14;
    // Virtual path speed
    int vs = 15;

    // Input indices: [dT, dp, dq, dr, dvs]
    int dT  = 0;
    int dp  = 1;
    int dq  = 2;
    int dr  = 3;
    int dvs = 4;

    // Tunnel constraint indices
    int con_t1 = 0;
    int con_t2 = 1;
    int con_t3 = 2;
    int con_t4 = 3;
};

static const StateInputIndex si_index;

} // namespace mpcc
