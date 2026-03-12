#include "mpcc/constraints.h"

namespace mpcc {

Constraints::Constraints()
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Constraints::Constraints(double /*Ts*/, const PathToJson &path)
: param_(Param(path.param_path))
{
}

// ── Single tunnel-wall constraint (linearised) ─────────────────────────────────
//
// PathWeaver provides 4 halfspace constraints of the form c_i(pos, s) >= 0.
// Linearised around (pos0, s0):
//
//   c_i(pos0, s0) + dC_dpos_i * (pos - pos0) + dC_ds_i * (s - s0) >= 0
//
// Rearranged as a lower bound on a linear expression in [x; u]:
//
//   dC_dpos_i * [X,Y,Z]^T + dC_ds_i * s  >=  -c_i + dC_dpos_i*pos0 + dC_ds_i*s0
//                                          =    lower_bound
//
OneDConstraint Constraints::getTunnelConstraint(const pathweaver::Tunnel &tunnel,
                                                  const pathweaver::FrenetFrame &frames,
                                                  const State &x,
                                                  int wall_index) const
{
    const double s0 = x.s;
    const Eigen::Vector3d pos0(x.X, x.Y, x.Z);

    // Evaluate constraint values at current point
    pathweaver::TunnelConstraints tc = tunnel.evaluate(pos0, s0);

    // Pick the correct wall
    double c_i = 0.0;
    switch (wall_index) {
        case 0: c_i = tc.c1; break;
        case 1: c_i = tc.c2; break;
        case 2: c_i = tc.c3; break;
        case 3: c_i = tc.c4; break;
        default: c_i = tc.c1; break;
    }

    // Position Jacobian: 4×3 matrix (one row per wall)
    Eigen::Matrix<double,4,3> dC_dpos = tunnel.positionJacobian(s0);
    // Arc-length Jacobian: 4×1 vector (one entry per wall)
    Eigen::Vector4d dC_ds = tunnel.thetaJacobian(s0);

    // Extract the wall_index row
    const Eigen::RowVector3d dC_dpos_i = dC_dpos.row(wall_index);
    const double dC_ds_i = dC_ds(wall_index);

    // Build 1×NX Jacobian row
    C_i_MPC C_row = C_i_MPC::Zero();
    C_row(si_index.X) = dC_dpos_i(0);
    C_row(si_index.Y) = dC_dpos_i(1);
    C_row(si_index.Z) = dC_dpos_i(2);
    C_row(si_index.s) = dC_ds_i;

    // Lower bound:  dl = -(c_i - dC_dpos_i*pos0 - dC_ds_i*s0)
    //                  = dC_dpos_i*pos0 + dC_ds_i*s0 - c_i
    const double lower = dC_dpos_i.dot(pos0) + dC_ds_i*s0 - c_i;
    const double upper = INF;

    return {C_row, lower, upper};
}

// ── Public entry point ────────────────────────────────────────────────────────
ConstrainsMatrix Constraints::getConstraints(const pathweaver::Tunnel &tunnel,
                                              const pathweaver::FrenetFrame &frames,
                                              const State &x,
                                              const Input &/*u*/) const
{
    C_MPC C  = C_MPC::Zero();
    D_MPC D  = D_MPC::Zero();
    d_MPC dl = d_MPC::Zero();
    d_MPC du = d_MPC::Constant(INF);

    for (int i = 0; i < NPC; i++) {
        const OneDConstraint con = getTunnelConstraint(tunnel, frames, x, i);
        C.row(i)  = con.C_i;
        dl(i)     = con.dl_i;
        du(i)     = con.du_i;
    }

    return {C, D, dl, du};
}

} // namespace mpcc
