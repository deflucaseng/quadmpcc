#include "mpcc/bounds.h"

namespace mpcc {

Bounds::Bounds()
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
    l_bounds_x_.setConstant(-INF);
    u_bounds_x_.setConstant( INF);
    l_bounds_u_.setConstant(-INF);
    u_bounds_u_.setConstant( INF);
    l_bounds_s_.setZero();
    u_bounds_s_.setZero();
}

Bounds::Bounds(const BoundsParam &bp)
{
    // State lower bounds
    l_bounds_x_(si_index.X)  = bp.lower_state_bounds.X_l;
    l_bounds_x_(si_index.Y)  = bp.lower_state_bounds.Y_l;
    l_bounds_x_(si_index.Z)  = bp.lower_state_bounds.Z_l;
    l_bounds_x_(si_index.qw) = bp.lower_state_bounds.qw_l;
    l_bounds_x_(si_index.qx) = bp.lower_state_bounds.qx_l;
    l_bounds_x_(si_index.qy) = bp.lower_state_bounds.qy_l;
    l_bounds_x_(si_index.qz) = bp.lower_state_bounds.qz_l;
    l_bounds_x_(si_index.vx) = bp.lower_state_bounds.vx_l;
    l_bounds_x_(si_index.vy) = bp.lower_state_bounds.vy_l;
    l_bounds_x_(si_index.vz) = bp.lower_state_bounds.vz_l;
    l_bounds_x_(si_index.p)  = bp.lower_state_bounds.p_l;
    l_bounds_x_(si_index.q)  = bp.lower_state_bounds.q_l;
    l_bounds_x_(si_index.r)  = bp.lower_state_bounds.r_l;
    l_bounds_x_(si_index.s)  = bp.lower_state_bounds.s_l;
    l_bounds_x_(si_index.T)  = bp.lower_state_bounds.T_l;
    l_bounds_x_(si_index.vs) = bp.lower_state_bounds.vs_l;

    // State upper bounds
    u_bounds_x_(si_index.X)  = bp.upper_state_bounds.X_u;
    u_bounds_x_(si_index.Y)  = bp.upper_state_bounds.Y_u;
    u_bounds_x_(si_index.Z)  = bp.upper_state_bounds.Z_u;
    u_bounds_x_(si_index.qw) = bp.upper_state_bounds.qw_u;
    u_bounds_x_(si_index.qx) = bp.upper_state_bounds.qx_u;
    u_bounds_x_(si_index.qy) = bp.upper_state_bounds.qy_u;
    u_bounds_x_(si_index.qz) = bp.upper_state_bounds.qz_u;
    u_bounds_x_(si_index.vx) = bp.upper_state_bounds.vx_u;
    u_bounds_x_(si_index.vy) = bp.upper_state_bounds.vy_u;
    u_bounds_x_(si_index.vz) = bp.upper_state_bounds.vz_u;
    u_bounds_x_(si_index.p)  = bp.upper_state_bounds.p_u;
    u_bounds_x_(si_index.q)  = bp.upper_state_bounds.q_u;
    u_bounds_x_(si_index.r)  = bp.upper_state_bounds.r_u;
    u_bounds_x_(si_index.s)  = bp.upper_state_bounds.s_u;
    u_bounds_x_(si_index.T)  = bp.upper_state_bounds.T_u;
    u_bounds_x_(si_index.vs) = bp.upper_state_bounds.vs_u;

    // Input lower bounds
    l_bounds_u_(si_index.dT)  = bp.lower_input_bounds.dT_l;
    l_bounds_u_(si_index.dp)  = bp.lower_input_bounds.dp_l;
    l_bounds_u_(si_index.dq)  = bp.lower_input_bounds.dq_l;
    l_bounds_u_(si_index.dr)  = bp.lower_input_bounds.dr_l;
    l_bounds_u_(si_index.dvs) = bp.lower_input_bounds.dvs_l;

    // Input upper bounds
    u_bounds_u_(si_index.dT)  = bp.upper_input_bounds.dT_u;
    u_bounds_u_(si_index.dp)  = bp.upper_input_bounds.dp_u;
    u_bounds_u_(si_index.dq)  = bp.upper_input_bounds.dq_u;
    u_bounds_u_(si_index.dr)  = bp.upper_input_bounds.dr_u;
    u_bounds_u_(si_index.dvs) = bp.upper_input_bounds.dvs_u;

    // Soft-constraint slack bounds (both zero → slack >= 0)
    l_bounds_s_.setZero();
    u_bounds_s_.setZero();
}

Bounds_x Bounds::getBoundsLX() const { return l_bounds_x_; }
Bounds_x Bounds::getBoundsUX() const { return u_bounds_x_; }
Bounds_u Bounds::getBoundsLU() const { return l_bounds_u_; }
Bounds_u Bounds::getBoundsUU() const { return u_bounds_u_; }
Bounds_s Bounds::getBoundsLS() const { return l_bounds_s_; }
Bounds_s Bounds::getBoundsUS() const { return u_bounds_s_; }

} // namespace mpcc
