#include "mpcc/cost.h"

namespace mpcc {

Cost::Cost()
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Cost::Cost(const PathToJson &path)
: cost_param_(CostParam(path.cost_path))
{
}

// ── Contouring + lag + progress cost ─────────────────────────────────────────
//
// FrenetFrame::at(s) returns Frame { .t, .n, .b, .position } (all Eigen::Vector3d)
//
// e = [X - x_ref, Y - y_ref, Z - z_ref]   (position error vector)
// e_lag  = tangent · e            (along-path error)
// e_n    = normal  · e            (lateral contouring error)
// e_b    = binorm  · e            (vertical contouring error)
//
// We linearise each scalar error around x0:
//   e_i(x) ≈ e_i(x0) + J_i*(x - x0)
// Leading to quadratic:
//   J_i^T * J_i  (contributes to Q)  and  (e_i(x0) - J_i*x0) * J_i^T (to q)
//
CostMatrix Cost::getContouringCost(const pathweaver::FrenetFrame &frames,
                                    const pathweaver::HermiteSpline &/*spline*/,
                                    const State &x, int k) const
{
    const double s = x.s;
    pathweaver::Frame frame = frames.at(s);

    const Eigen::Vector3d &tan  = frame.t;
    const Eigen::Vector3d &norm = frame.n;
    const Eigen::Vector3d &bin  = frame.b;
    const Eigen::Vector3d &pref = frame.position;

    // position error
    const Eigen::Vector3d e3d(x.X - pref(0), x.Y - pref(1), x.Z - pref(2));

    const double e_lag = tan.dot(e3d);
    const double e_n   = norm.dot(e3d);
    const double e_b   = bin.dot(e3d);

    // ── Jacobians (1 × NX) ───────────────────────────────────────────────────
    Eigen::Matrix<double,1,NX> J_lag = Eigen::Matrix<double,1,NX>::Zero();
    Eigen::Matrix<double,1,NX> J_n   = Eigen::Matrix<double,1,NX>::Zero();
    Eigen::Matrix<double,1,NX> J_b   = Eigen::Matrix<double,1,NX>::Zero();

    // de_i/d[X,Y,Z]
    J_lag(si_index.X)  = tan(0);
    J_lag(si_index.Y)  = tan(1);
    J_lag(si_index.Z)  = tan(2);
    J_lag(si_index.s)  = -1.0;   // ≈ -||tangent||² for unit tangent

    J_n(si_index.X)    = norm(0);
    J_n(si_index.Y)    = norm(1);
    J_n(si_index.Z)    = norm(2);
    J_n(si_index.s)    = 0.0;    // normal ⊥ tangent

    J_b(si_index.X)    = bin(0);
    J_b(si_index.Y)    = bin(1);
    J_b(si_index.Z)    = bin(2);
    J_b(si_index.s)    = 0.0;    // binormal ⊥ tangent

    // ── Weights ───────────────────────────────────────────────────────────────
    const bool terminal = (k >= N);
    const double w_c  = terminal ? cost_param_.qCNmult * cost_param_.qC : cost_param_.qC;
    const double w_l  = cost_param_.qL;

    // ── Q and q contributions ─────────────────────────────────────────────────
    const StateVector x_vec = stateToVector(x);

    const double e_lag0 = e_lag - J_lag*x_vec;
    const double e_n0   = e_n   - J_n  *x_vec;
    const double e_b0   = e_b   - J_b  *x_vec;

    Q_MPC Q = 2.0*(w_l * J_lag.transpose()*J_lag
                 + w_c * J_n.transpose()*J_n
                 + w_c * J_b.transpose()*J_b);
    q_MPC q = 2.0*(w_l * e_lag0 * J_lag.transpose()
                 + w_c * e_n0   * J_n.transpose()
                 + w_c * e_b0   * J_b.transpose());

    // Progress maximisation: -qVs * vs
    q(si_index.vs) -= cost_param_.qVs;

    return {Q, R_MPC::Zero(), S_MPC::Zero(), q, r_MPC::Zero(),
            Z_MPC::Zero(), z_MPC::Zero()};
}

// ── Heading cost ──────────────────────────────────────────────────────────────
//
// Yaw from quaternion:  psi = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
// Reference yaw from path tangent:  psi_ref = atan2(tan(1), tan(0))
// Cost:  qMu * (psi - psi_ref)²
// Linearised: Q += 2*qMu * (d_psi)^T * (d_psi)
//             q +=  2*qMu * (psi0 - d_psi * x_vec) * d_psi^T
//             and constant term -2*qMu*psi_ref * d_psi^T absorbed in q
//
CostMatrix Cost::getHeadingCost(const pathweaver::FrenetFrame &frames,
                                 const State &x, int /*k*/) const
{
    const pathweaver::Frame frame = frames.at(x.s);
    const Eigen::Vector3d &tan = frame.t;

    const double psi_ref = std::atan2(tan(1), tan(0));

    // Yaw from current quaternion
    const double qw = x.qw, qx = x.qx, qy = x.qy, qz = x.qz;
    const double psi = std::atan2(2.0*(qw*qz + qx*qy),
                                  1.0 - 2.0*(qy*qy + qz*qz));

    // Jacobian of psi w.r.t. quaternion components
    const double denom_sq = std::pow(2.0*(qw*qz + qx*qy), 2)
                          + std::pow(1.0 - 2.0*(qy*qy + qz*qz), 2);
    const double safe_denom = (denom_sq < 1e-9) ? 1e-9 : denom_sq;

    const double a = 2.0*(qw*qz + qx*qy);      // numerator
    const double b = 1.0 - 2.0*(qy*qy + qz*qz); // denominator

    // d(atan2(a,b))/dqw = da/dqw * b - a * db/dqw) / (a²+b²)
    const double dpsi_dqw = (2.0*qz * b - a * 0.0) / safe_denom;
    const double dpsi_dqx = (2.0*qy * b - a * 0.0) / safe_denom;
    const double dpsi_dqy = (2.0*qx * b - a * (-4.0*qy)) / safe_denom;
    const double dpsi_dqz = (2.0*qw * b - a * (-4.0*qz)) / safe_denom;

    Eigen::Matrix<double,1,NX> J_psi = Eigen::Matrix<double,1,NX>::Zero();
    J_psi(si_index.qw) = dpsi_dqw;
    J_psi(si_index.qx) = dpsi_dqx;
    J_psi(si_index.qy) = dpsi_dqy;
    J_psi(si_index.qz) = dpsi_dqz;

    const StateVector x_vec = stateToVector(x);
    // Linearised error: psi - psi_ref ≈ (psi - J_psi*x0) + J_psi*(x - x0) - psi_ref
    // Treat:  error = (psi - psi_ref - J_psi*x_vec) + J_psi * x
    //       = e0 + J_psi * x,   where e0 = psi - psi_ref - J_psi*x_vec (< this is zero-order)
    const double e0 = psi - psi_ref - J_psi*x_vec;

    Q_MPC Q = 2.0*cost_param_.qMu * J_psi.transpose()*J_psi;
    q_MPC q = 2.0*cost_param_.qMu * e0 * J_psi.transpose();

    return {Q, R_MPC::Zero(), S_MPC::Zero(), q, r_MPC::Zero(),
            Z_MPC::Zero(), z_MPC::Zero()};
}

// ── Input + state regularisation cost ────────────────────────────────────────
CostMatrix Cost::getInputCost() const
{
    Q_MPC Q = Q_MPC::Zero();
    R_MPC R = R_MPC::Zero();

    // State-embedded "real" input costs (T, p, q, r, vs are states)
    Q(si_index.T,  si_index.T)  = 2.0 * cost_param_.rT;
    Q(si_index.p,  si_index.p)  = 2.0 * cost_param_.rP;
    Q(si_index.q,  si_index.q)  = 2.0 * cost_param_.rQ;
    Q(si_index.r,  si_index.r)  = 2.0 * cost_param_.rR;
    Q(si_index.vs, si_index.vs) = 2.0 * cost_param_.rVs;

    // True input rate costs
    R(si_index.dT,  si_index.dT)  = 2.0 * cost_param_.rdT;
    R(si_index.dp,  si_index.dp)  = 2.0 * cost_param_.rdP;
    R(si_index.dq,  si_index.dq)  = 2.0 * cost_param_.rdQ;
    R(si_index.dr,  si_index.dr)  = 2.0 * cost_param_.rdR;
    R(si_index.dvs, si_index.dvs) = 2.0 * cost_param_.rdVs;

    return {Q, R, S_MPC::Zero(), q_MPC::Zero(), r_MPC::Zero(),
            Z_MPC::Zero(), z_MPC::Zero()};
}

// ── Soft-constraint penalty ───────────────────────────────────────────────────
CostMatrix Cost::getSoftConstraintCost() const
{
    Z_MPC Z = Z_MPC::Identity() * cost_param_.sc_quad_tunnel;
    z_MPC z = z_MPC::Ones()    * cost_param_.sc_lin_tunnel;
    return {Q_MPC::Zero(), R_MPC::Zero(), S_MPC::Zero(), q_MPC::Zero(), r_MPC::Zero(), Z, z};
}

// ── Public entry point ────────────────────────────────────────────────────────
CostMatrix Cost::getCost(const pathweaver::FrenetFrame &frames,
                          const pathweaver::HermiteSpline &spline,
                          const State &x, int k) const
{
    const CostMatrix contouring = getContouringCost(frames, spline, x, k);
    const CostMatrix heading    = getHeadingCost(frames, x, k);
    const CostMatrix inputs     = getInputCost();
    const CostMatrix soft       = getSoftConstraintCost();

    Q_MPC Q_raw = contouring.Q + heading.Q + inputs.Q;
    // Symmetrise to suppress numerical asymmetry from linearisation
    const Q_MPC Q = 0.5*(Q_raw + Q_raw.transpose()) + 1e-9*Q_MPC::Identity();
    const R_MPC R = inputs.R;
    const q_MPC q = contouring.q + heading.q + inputs.q;
    const r_MPC r = r_MPC::Zero();
    const Z_MPC Z = soft.Z;
    const z_MPC z = soft.z;

    return {Q, R, S_MPC::Zero(), q, r, Z, z};
}

} // namespace mpcc
