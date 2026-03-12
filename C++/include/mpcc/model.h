#pragma once

#include "mpcc/config.h"
#include "mpcc/types.h"
#include "mpcc/params.h"

namespace mpcc {

struct LinModelMatrix {
    A_MPC A;
    B_MPC B;
    g_MPC g;
};

class Model {
public:
    /// Evaluate continuous-time dynamics f(x, u)
    StateVector getF(const State &x, const Input &u) const;

    /// Compute linearised, discretised model (ZOH matrix exponential)
    LinModelMatrix getLinModel(const State &x, const Input &u) const;

    Model();
    Model(double Ts, const PathToJson &path);

private:
    LinModelMatrix getModelJacobian(const State &x, const Input &u) const;
    LinModelMatrix discretizeModel(const LinModelMatrix &lin_model_c) const;

    Param param_;
    double Ts_;
};

} // namespace mpcc
