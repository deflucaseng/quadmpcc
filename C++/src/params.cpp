#include "mpcc/params.h"

namespace mpcc {

// ── Param ─────────────────────────────────────────────────────────────────────
Param::Param()
{
    std::cout << "Default initialization of model params" << std::endl;
    m = 1.0; g = 9.81; arm_length = 0.15; k_thrust = 1e-6;
    max_thrust = 20.0; max_bodyrate = 10.0;
    initial_velocity = 2.0; s_trust_region = 3.0;
}

Param::Param(const std::string &file)
{
    std::ifstream iModel(file);
    json j; iModel >> j;

    m               = j["m"];
    g               = j["g"];
    arm_length      = j["arm_length"];
    k_thrust        = j["k_thrust"];
    max_thrust      = j["max_thrust"];
    max_bodyrate    = j["max_bodyrate"];
    initial_velocity = j["initial_velocity"];
    s_trust_region  = j["s_trust_region"];
}

// ── CostParam ─────────────────────────────────────────────────────────────────
CostParam::CostParam()
{
    std::cout << "Default initialization of cost params" << std::endl;
    qC = 0.1; qL = 200.0; qVs = 0.02; qMu = 0.01; qOmega = 1e-3;
    rT = 1e-4; rP = 1e-4; rQ = 1e-4; rR = 1e-4; rVs = 1e-4;
    rdT = 1e-3; rdP = 1e-3; rdQ = 1e-3; rdR = 1e-3; rdVs = 1e-4;
    qCNmult = 5.0; qOmegaNmult = 5.0;
    sc_quad_tunnel = 100.0; sc_lin_tunnel = 10.0;
}

CostParam::CostParam(const std::string &file)
{
    std::ifstream iCost(file);
    json j; iCost >> j;

    qC      = j["qC"];
    qL      = j["qL"];
    qVs     = j["qVs"];
    qMu     = j["qMu"];
    qOmega  = j["qOmega"];

    rT      = j["rT"];
    rP      = j["rP"];
    rQ      = j["rQ"];
    rR      = j["rR"];
    rVs     = j["rVs"];

    rdT     = j["rdT"];
    rdP     = j["rdP"];
    rdQ     = j["rdQ"];
    rdR     = j["rdR"];
    rdVs    = j["rdVs"];

    qCNmult     = j["qCNmult"];
    qOmegaNmult = j["qOmegaNmult"];

    sc_quad_tunnel = j["sc_quad_tunnel"];
    sc_lin_tunnel  = j["sc_lin_tunnel"];
}

// ── BoundsParam ───────────────────────────────────────────────────────────────
BoundsParam::BoundsParam()
{
    std::cout << "Default initialization of bounds" << std::endl;
}

BoundsParam::BoundsParam(const std::string &file)
{
    std::ifstream iBounds(file);
    json j; iBounds >> j;

    lower_state_bounds.X_l  = j["Xl"];
    lower_state_bounds.Y_l  = j["Yl"];
    lower_state_bounds.Z_l  = j["Zl"];
    lower_state_bounds.qw_l = j["qwl"];
    lower_state_bounds.qx_l = j["qxl"];
    lower_state_bounds.qy_l = j["qyl"];
    lower_state_bounds.qz_l = j["qzl"];
    lower_state_bounds.vx_l = j["vxl"];
    lower_state_bounds.vy_l = j["vyl"];
    lower_state_bounds.vz_l = j["vzl"];
    lower_state_bounds.p_l  = j["pl"];
    lower_state_bounds.q_l  = j["ql"];
    lower_state_bounds.r_l  = j["rl"];
    lower_state_bounds.s_l  = j["sl"];
    lower_state_bounds.T_l  = j["Tl"];
    lower_state_bounds.vs_l = j["vsl"];

    upper_state_bounds.X_u  = j["Xu"];
    upper_state_bounds.Y_u  = j["Yu"];
    upper_state_bounds.Z_u  = j["Zu"];
    upper_state_bounds.qw_u = j["qwu"];
    upper_state_bounds.qx_u = j["qxu"];
    upper_state_bounds.qy_u = j["qyu"];
    upper_state_bounds.qz_u = j["qzu"];
    upper_state_bounds.vx_u = j["vxu"];
    upper_state_bounds.vy_u = j["vyu"];
    upper_state_bounds.vz_u = j["vzu"];
    upper_state_bounds.p_u  = j["pu"];
    upper_state_bounds.q_u  = j["qu"];
    upper_state_bounds.r_u  = j["ru"];
    upper_state_bounds.s_u  = j["su"];
    upper_state_bounds.T_u  = j["Tu"];
    upper_state_bounds.vs_u = j["vsu"];

    lower_input_bounds.dT_l  = j["dTl"];
    lower_input_bounds.dp_l  = j["dpl"];
    lower_input_bounds.dq_l  = j["dql"];
    lower_input_bounds.dr_l  = j["drl"];
    lower_input_bounds.dvs_l = j["dvsl"];

    upper_input_bounds.dT_u  = j["dTu"];
    upper_input_bounds.dp_u  = j["dpu"];
    upper_input_bounds.dq_u  = j["dqu"];
    upper_input_bounds.dr_u  = j["dru"];
    upper_input_bounds.dvs_u = j["dvsu"];
}

// ── NormalizationParam ────────────────────────────────────────────────────────
NormalizationParam::NormalizationParam()
{
    std::cout << "Default initialization of normalization" << std::endl;
    T_x.setIdentity();     T_x_inv.setIdentity();
    T_u.setIdentity();     T_u_inv.setIdentity();
    T_s.setIdentity();     T_s_inv.setIdentity();
}

NormalizationParam::NormalizationParam(const std::string &file)
{
    std::ifstream iNorm(file);
    json j; iNorm >> j;

    T_x.setIdentity();
    T_x(si_index.X,  si_index.X)  = j["X"];
    T_x(si_index.Y,  si_index.Y)  = j["Y"];
    T_x(si_index.Z,  si_index.Z)  = j["Z"];
    T_x(si_index.qw, si_index.qw) = j["qw"];
    T_x(si_index.qx, si_index.qx) = j["qx"];
    T_x(si_index.qy, si_index.qy) = j["qy"];
    T_x(si_index.qz, si_index.qz) = j["qz"];
    T_x(si_index.vx, si_index.vx) = j["vx"];
    T_x(si_index.vy, si_index.vy) = j["vy"];
    T_x(si_index.vz, si_index.vz) = j["vz"];
    T_x(si_index.p,  si_index.p)  = j["p"];
    T_x(si_index.q,  si_index.q)  = j["q"];
    T_x(si_index.r,  si_index.r)  = j["r"];
    T_x(si_index.s,  si_index.s)  = j["s"];
    T_x(si_index.T,  si_index.T)  = j["T"];
    T_x(si_index.vs, si_index.vs) = j["vs"];

    T_x_inv.setIdentity();
    for (int i = 0; i < NX; i++)
        T_x_inv(i, i) = 1.0 / T_x(i, i);

    T_u.setIdentity();
    T_u(si_index.dT,  si_index.dT)  = j["dT"];
    T_u(si_index.dp,  si_index.dp)  = j["dp"];
    T_u(si_index.dq,  si_index.dq)  = j["dq"];
    T_u(si_index.dr,  si_index.dr)  = j["dr"];
    T_u(si_index.dvs, si_index.dvs) = j["dvs"];

    T_u_inv.setIdentity();
    for (int i = 0; i < NU; i++)
        T_u_inv(i, i) = 1.0 / T_u(i, i);

    T_s.setIdentity();
    T_s_inv.setIdentity();
}

} // namespace mpcc
