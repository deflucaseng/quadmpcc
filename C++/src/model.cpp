#include "mpcc/model.h"

namespace mpcc {

Model::Model()
: Ts_(1.0)
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Model::Model(double Ts, const PathToJson &path)
: Ts_(Ts), param_(Param(path.param_path))
{
}

// ── Continuous-time dynamics ──────────────────────────────────────────────────
//
// State:  [X, Y, Z, qw, qx, qy, qz, vx, vy, vz, p, q, r, s, T, vs]
// Input:  [dT, dp, dq, dr, dvs]
//
// Body-to-world rotation column: R[:,2] = [2(qx*qz+qw*qy), 2(qy*qz-qw*qx), 1-2(qx²+qy²)]
//
StateVector Model::getF(const State &x, const Input &u) const
{
    const double qw = x.qw, qx = x.qx, qy = x.qy, qz = x.qz;
    const double p  = x.p,  q  = x.q,  r_ = x.r;
    const double T  = x.T;
    const double m  = param_.m;
    const double g  = param_.g;

    // z-column of rotation matrix (body z = up in FLU, maps to world frame)
    const double R20 = 2.0*(qx*qz + qw*qy);
    const double R21 = 2.0*(qy*qz - qw*qx);
    const double R22 = 1.0 - 2.0*(qx*qx + qy*qy);

    StateVector f;
    // position kinematics
    f(si_index.X)  = x.vx;
    f(si_index.Y)  = x.vy;
    f(si_index.Z)  = x.vz;
    // quaternion kinematics  (Omega body rates)
    f(si_index.qw) =  0.5*(-qx*p  - qy*q  - qz*r_);
    f(si_index.qx) =  0.5*( qw*p  + qy*r_ - qz*q);
    f(si_index.qy) =  0.5*( qw*q  - qx*r_ + qz*p);
    f(si_index.qz) =  0.5*( qw*r_ + qx*q  - qy*p);
    // velocity (thrust + gravity)
    f(si_index.vx) = (T/m)*R20;
    f(si_index.vy) = (T/m)*R21;
    f(si_index.vz) = (T/m)*R22 - g;
    // body rates driven by inputs
    f(si_index.p)  = u.dp;
    f(si_index.q)  = u.dq;
    f(si_index.r)  = u.dr;
    // path progress
    f(si_index.s)  = x.vs;
    // thrust and virtual speed are states, driven by inputs
    f(si_index.T)  = u.dT;
    f(si_index.vs) = u.dvs;

    return f;
}

// ── Continuous Jacobians ──────────────────────────────────────────────────────
LinModelMatrix Model::getModelJacobian(const State &x, const Input &u) const
{
    const double qw = x.qw, qx = x.qx, qy = x.qy, qz = x.qz;
    const double p  = x.p,  q  = x.q,  r_ = x.r;
    const double T  = x.T;
    const double m  = param_.m;

    A_MPC A = A_MPC::Zero();
    B_MPC B = B_MPC::Zero();
    g_MPC g_vec;

    // ── A matrix: df/dx ──────────────────────────────────────────────────────

    // position → velocity
    A(si_index.X,  si_index.vx) = 1.0;
    A(si_index.Y,  si_index.vy) = 1.0;
    A(si_index.Z,  si_index.vz) = 1.0;

    // quaternion kinematics — rows 3-6
    // dqw/dt = 0.5*(-qx*p - qy*q - qz*r)
    A(si_index.qw, si_index.qx) = -0.5*p;
    A(si_index.qw, si_index.qy) = -0.5*q;
    A(si_index.qw, si_index.qz) = -0.5*r_;
    A(si_index.qw, si_index.p)  = -0.5*qx;
    A(si_index.qw, si_index.q)  = -0.5*qy;
    A(si_index.qw, si_index.r)  = -0.5*qz;

    // dqx/dt = 0.5*(qw*p + qy*r - qz*q)
    A(si_index.qx, si_index.qw) = +0.5*p;
    A(si_index.qx, si_index.qy) = +0.5*r_;
    A(si_index.qx, si_index.qz) = -0.5*q;
    A(si_index.qx, si_index.p)  = +0.5*qw;
    A(si_index.qx, si_index.q)  = -0.5*qz;
    A(si_index.qx, si_index.r)  = +0.5*qy;

    // dqy/dt = 0.5*(qw*q - qx*r + qz*p)
    A(si_index.qy, si_index.qw) = +0.5*q;
    A(si_index.qy, si_index.qx) = -0.5*r_;
    A(si_index.qy, si_index.qz) = +0.5*p;
    A(si_index.qy, si_index.p)  = +0.5*qz;
    A(si_index.qy, si_index.q)  = +0.5*qw;
    A(si_index.qy, si_index.r)  = -0.5*qx;

    // dqz/dt = 0.5*(qw*r + qx*q - qy*p)
    A(si_index.qz, si_index.qw) = +0.5*r_;
    A(si_index.qz, si_index.qx) = +0.5*q;
    A(si_index.qz, si_index.qy) = -0.5*p;
    A(si_index.qz, si_index.p)  = -0.5*qy;
    A(si_index.qz, si_index.q)  = +0.5*qx;
    A(si_index.qz, si_index.r)  = +0.5*qw;

    // velocity ← quaternion and thrust
    // dvx/dt = (T/m)*2*(qx*qz + qw*qy)
    //   d/dqw:  (T/m)*2*qy
    //   d/dqx:  (T/m)*2*qz
    //   d/dqy:  (T/m)*2*qw
    //   d/dqz:  (T/m)*2*qx
    //   d/dT:   2*(qx*qz + qw*qy)/m
    A(si_index.vx, si_index.qw) = (T/m)*2.0*qy;
    A(si_index.vx, si_index.qx) = (T/m)*2.0*qz;
    A(si_index.vx, si_index.qy) = (T/m)*2.0*qw;
    A(si_index.vx, si_index.qz) = (T/m)*2.0*qx;
    A(si_index.vx, si_index.T)  = 2.0*(qx*qz + qw*qy)/m;

    // dvy/dt = (T/m)*2*(qy*qz - qw*qx)
    //   d/dqw:  (T/m)*(-2*qx)
    //   d/dqx:  (T/m)*(-2*qw)
    //   d/dqy:  (T/m)*2*qz
    //   d/dqz:  (T/m)*2*qy
    //   d/dT:   2*(qy*qz - qw*qx)/m
    A(si_index.vy, si_index.qw) = (T/m)*(-2.0*qx);
    A(si_index.vy, si_index.qx) = (T/m)*(-2.0*qw);
    A(si_index.vy, si_index.qy) = (T/m)*2.0*qz;
    A(si_index.vy, si_index.qz) = (T/m)*2.0*qy;
    A(si_index.vy, si_index.T)  = 2.0*(qy*qz - qw*qx)/m;

    // dvz/dt = (T/m)*(1 - 2*(qx² + qy²)) - g
    //   d/dqx:  (T/m)*(-4*qx)
    //   d/dqy:  (T/m)*(-4*qy)
    //   d/dT:   (1 - 2*(qx² + qy²))/m
    A(si_index.vz, si_index.qx) = (T/m)*(-4.0*qx);
    A(si_index.vz, si_index.qy) = (T/m)*(-4.0*qy);
    A(si_index.vz, si_index.T)  = (1.0 - 2.0*(qx*qx + qy*qy))/m;

    // path progress
    A(si_index.s, si_index.vs) = 1.0;

    // ── B matrix: df/du ──────────────────────────────────────────────────────
    // Body rates are inputs directly:  dp/dt = u.dp, etc.
    B(si_index.p,  si_index.dp)  = 1.0;
    B(si_index.q,  si_index.dq)  = 1.0;
    B(si_index.r,  si_index.dr)  = 1.0;
    B(si_index.T,  si_index.dT)  = 1.0;
    B(si_index.vs, si_index.dvs) = 1.0;

    // zero-order affine term
    g_vec = getF(x, u) - A*stateToVector(x) - B*inputToVector(u);

    return {A, B, g_vec};
}

// ── ZOH discretisation via matrix exponential ─────────────────────────────────
LinModelMatrix Model::discretizeModel(const LinModelMatrix &lin_model_c) const
{
    constexpr int SZ = NX + NU + 1;
    Eigen::Matrix<double, SZ, SZ> temp = Eigen::Matrix<double, SZ, SZ>::Zero();

    temp.block<NX, NX>(0, 0)      = lin_model_c.A;
    temp.block<NX, NU>(0, NX)     = lin_model_c.B;
    temp.block<NX, 1> (0, NX+NU)  = lin_model_c.g;
    temp *= Ts_;

    const auto temp_res = temp.exp();

    const A_MPC A_d = temp_res.block<NX, NX>(0, 0);
    const B_MPC B_d = temp_res.block<NX, NU>(0, NX);
    const g_MPC g_d = temp_res.block<NX, 1> (0, NX+NU);

    return {A_d, B_d, g_d};
}

LinModelMatrix Model::getLinModel(const State &x, const Input &u) const
{
    const LinModelMatrix lin_model_c = getModelJacobian(x, u);
    return discretizeModel(lin_model_c);
}

} // namespace mpcc
