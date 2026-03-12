#include "mpcc/integrator.h"

namespace mpcc {

Integrator::Integrator()
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Integrator::Integrator(double Ts, const PathToJson &path)
: model_(Ts, path)
{
}

State Integrator::RK4(const State &x, const Input &u, const double ts) const
{
    const StateVector x_vec = stateToVector(x);
    const StateVector k1 = model_.getF(vectorToState(x_vec),          u);
    const StateVector k2 = model_.getF(vectorToState(x_vec + ts/2.*k1), u);
    const StateVector k3 = model_.getF(vectorToState(x_vec + ts/2.*k2), u);
    const StateVector k4 = model_.getF(vectorToState(x_vec + ts*k3),    u);
    const StateVector x_next = x_vec + ts*(k1/6. + k2/3. + k3/3. + k4/6.);

    // Re-normalise quaternion after integration to prevent drift
    State x_out = vectorToState(x_next);
    const double qnorm = std::sqrt(x_out.qw*x_out.qw + x_out.qx*x_out.qx
                                 + x_out.qy*x_out.qy + x_out.qz*x_out.qz);
    if (qnorm > 1e-9) {
        x_out.qw /= qnorm;
        x_out.qx /= qnorm;
        x_out.qy /= qnorm;
        x_out.qz /= qnorm;
    }
    return x_out;
}

State Integrator::EF(const State &x, const Input &u, const double ts) const
{
    const StateVector x_vec = stateToVector(x);
    const StateVector f     = model_.getF(x, u);
    return vectorToState(x_vec + ts*f);
}

State Integrator::simTimeStep(const State &x, const Input &u, const double ts) const
{
    State x_next = x;
    const int steps = static_cast<int>(ts / fine_time_step_);
    for (int i = 0; i < steps; i++)
        x_next = RK4(x_next, u, fine_time_step_);
    return x_next;
}

} // namespace mpcc
