#pragma once

#include <vector>
#include <nlohmann/json.hpp>
#include "mpcc/config.h"
#include "mpcc/types.h"

namespace mpcc {

using json = nlohmann::json;

// ── Model parameters ──────────────────────────────────────────────────────────
class Param {
public:
    double m;              // mass (kg)
    double g;              // gravity (m/s²)
    double arm_length;     // rotor arm length (m)
    double k_thrust;       // thrust coefficient (not used in rigid-body model)
    double max_thrust;     // max collective thrust (N)
    double max_bodyrate;   // max body rate (rad/s)
    double initial_velocity;
    double s_trust_region;

    Param();
    explicit Param(const std::string &file);
};

// ── Cost parameters ───────────────────────────────────────────────────────────
class CostParam {
public:
    double qC;             // contouring (lateral + vertical)
    double qL;             // lag error
    double qVs;            // progress maximisation
    double qMu;            // heading alignment
    double qOmega;         // body-rate regularisation

    double rT;             // thrust state cost
    double rP, rQ, rR;    // body-rate state costs
    double rVs;            // virtual speed state cost

    double rdT;            // thrust-rate input cost
    double rdP, rdQ, rdR; // body-rate input costs
    double rdVs;           // virtual-speed input cost

    double qCNmult;        // terminal contouring multiplier
    double qOmegaNmult;    // terminal omega multiplier

    double sc_quad_tunnel; // quadratic soft penalty per tunnel wall
    double sc_lin_tunnel;  // linear  soft penalty per tunnel wall

    CostParam();
    explicit CostParam(const std::string &file);
};

// ── Bounds parameters ─────────────────────────────────────────────────────────
class BoundsParam {
public:
    struct LowerStateBounds {
        double X_l, Y_l, Z_l;
        double qw_l, qx_l, qy_l, qz_l;
        double vx_l, vy_l, vz_l;
        double p_l, q_l, r_l;
        double s_l, T_l, vs_l;
    };
    struct UpperStateBounds {
        double X_u, Y_u, Z_u;
        double qw_u, qx_u, qy_u, qz_u;
        double vx_u, vy_u, vz_u;
        double p_u, q_u, r_u;
        double s_u, T_u, vs_u;
    };
    struct LowerInputBounds {
        double dT_l, dp_l, dq_l, dr_l, dvs_l;
    };
    struct UpperInputBounds {
        double dT_u, dp_u, dq_u, dr_u, dvs_u;
    };

    LowerStateBounds lower_state_bounds;
    UpperStateBounds upper_state_bounds;
    LowerInputBounds lower_input_bounds;
    UpperInputBounds upper_input_bounds;

    BoundsParam();
    explicit BoundsParam(const std::string &file);
};

// ── Normalization parameters ──────────────────────────────────────────────────
class NormalizationParam {
public:
    TX_MPC T_x;
    TX_MPC T_x_inv;
    TU_MPC T_u;
    TU_MPC T_u_inv;
    TS_MPC T_s;
    TS_MPC T_s_inv;

    NormalizationParam();
    explicit NormalizationParam(const std::string &file);
};

} // namespace mpcc
