#pragma once

#include "mpcc/config.h"
#include "mpcc/types.h"
#include "mpcc/model.h"

namespace mpcc {

class Integrator {
public:
    State RK4(const State &x, const Input &u, double ts) const;
    State EF(const State &x, const Input &u, double ts) const;
    State simTimeStep(const State &x, const Input &u, double ts) const;

    Integrator();
    Integrator(double Ts, const PathToJson &path);

private:
    static constexpr double fine_time_step_ = 0.001;
    Model model_;
};

} // namespace mpcc
