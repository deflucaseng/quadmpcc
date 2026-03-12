#pragma once

#include "mpcc/config.h"
#include "mpcc/types.h"
#include "mpcc/params.h"
#include <PathWeaver/pathweaver.hpp>

namespace mpcc {

struct CostMatrix {
    Q_MPC Q;
    R_MPC R;
    S_MPC S;
    q_MPC q;
    r_MPC r;
    Z_MPC Z;
    z_MPC z;
};

class Cost {
public:
    /// Compute the quadratic MPCC cost at stage k, linearised around (x, u).
    CostMatrix getCost(const pathweaver::FrenetFrame &frames,
                       const pathweaver::HermiteSpline &spline,
                       const State &x, int k) const;

    Cost();
    explicit Cost(const PathToJson &path);

private:
    // 3-D contouring + lag + heading + progress
    CostMatrix getContouringCost(const pathweaver::FrenetFrame &frames,
                                 const pathweaver::HermiteSpline &spline,
                                 const State &x, int k) const;
    // Heading-alignment cost (yaw vs. path tangent)
    CostMatrix getHeadingCost(const pathweaver::FrenetFrame &frames,
                              const State &x, int k) const;
    // Input + state-regularisation cost
    CostMatrix getInputCost() const;
    // Soft-constraint penalty (tunnel walls)
    CostMatrix getSoftConstraintCost() const;

    CostParam cost_param_;
};

} // namespace mpcc
