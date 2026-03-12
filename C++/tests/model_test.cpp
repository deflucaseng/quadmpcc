// Compilation-check stub for model.cpp
// Tests that the Model class compiles correctly and basic dynamics are sane.
#include "mpcc/model.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    using namespace mpcc;

    // Build a PathToJson pointing at the params directory
    PathToJson paths{"params/model.json", "params/cost.json",
                     "params/bounds.json", "params/normalization.json",
                     "params/waypoints.yaml"};

    Model model(0.02, paths);

    // Hovering state: identity quaternion, m*g thrust, zero velocity
    State x;
    x.setZero();
    x.qw = 1.0;
    x.T  = 9.81;   // hover thrust (m = 1.0 kg, g = 9.81)

    Input u;
    u.setZero();

    // Evaluate continuous dynamics at hover
    StateVector f = model.getF(x, u);

    // At hover:
    // - position rates should be zero (zero velocity)
    assert(std::fabs(f(si_index.X))  < 1e-9);
    assert(std::fabs(f(si_index.Y))  < 1e-9);
    assert(std::fabs(f(si_index.Z))  < 1e-9);
    // - acceleration in vz: (T/m)*R22 - g = (9.81/1.0)*1.0 - 9.81 = 0
    assert(std::fabs(f(si_index.vz)) < 1e-9);

    // Linearised model should be constructible
    LinModelMatrix lm = model.getLinModel(x, u);
    assert(lm.A.rows() == NX && lm.A.cols() == NX);
    assert(lm.B.rows() == NX && lm.B.cols() == NU);

    std::cout << "model_test PASSED\n";
    return 0;
}
