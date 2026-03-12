#pragma once

#include "mpcc/config.h"
#include "mpcc/types.h"
#include "mpcc/params.h"
#include <PathWeaver/pathweaver.hpp>

namespace mpcc {

struct ConstrainsMatrix {
    // dl <= C*xk + D*uk <= du
    C_MPC C;   // polytopic state constraints  (NPC × NX)
    D_MPC D;   // polytopic input constraints  (NPC × NU)
    d_MPC dl;  // lower bounds
    d_MPC du;  // upper bounds
};

struct OneDConstraint {
    C_i_MPC C_i;  // 1 × NX row
    double  dl_i;
    double  du_i;
};

class Constraints {
public:
    ConstrainsMatrix getConstraints(const pathweaver::Tunnel &tunnel,
                                    const pathweaver::FrenetFrame &frames,
                                    const State &x,
                                    const Input &u) const;

    Constraints();
    Constraints(double Ts, const PathToJson &path);

private:
    /// Returns one linearised halfspace row for tunnel wall i (0..3)
    OneDConstraint getTunnelConstraint(const pathweaver::Tunnel &tunnel,
                                       const pathweaver::FrenetFrame &frames,
                                       const State &x,
                                       int wall_index) const;
    Param param_;
};

} // namespace mpcc
