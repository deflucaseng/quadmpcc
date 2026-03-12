#include "mpcc/mpc.h"

namespace mpcc {

MPC::MPC()
: Ts_(1.0), valid_initial_guess_(false)
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

MPC::MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts, const PathToJson &path)
: Ts_(Ts),
  valid_initial_guess_(false),
  solver_interface_(new HpipmInterface()),
  param_(Param(path.param_path)),
  normalization_param_(NormalizationParam(path.normalization_path)),
  bounds_(BoundsParam(path.bounds_path)),
  constraints_(Constraints(Ts, path)),
  cost_(Cost(path)),
  integrator_(Integrator(Ts, path)),
  model_(Model(Ts, path))
{
    n_sqp_           = n_sqp;
    sqp_mixing_      = sqp_mixing;
    n_non_solves_    = 0;
    n_no_solves_sqp_ = 0;
    n_reset_         = n_reset;
}

// ── Track loading ─────────────────────────────────────────────────────────────
void MPC::setTrack(const std::string &waypoints_yaml,
                   double tunnel_width,
                   double tunnel_gate_width,
                   double tunnel_k,
                   double frame_ds)
{
    // Load spline from YAML waypoints
    spline_ = pathweaver::HermiteSpline(waypoints_yaml);
    frames_ = pathweaver::FrenetFrame(spline_, frame_ds);

    // Build tunnel (PathWeaver API)
    pathweaver::TunnelParams tp;
    tp.width      = tunnel_width;
    tp.gate_width = tunnel_gate_width;
    tp.k          = tunnel_k;

    if (tunnel_) { delete tunnel_; tunnel_ = nullptr; }
    tunnel_ = new pathweaver::Tunnel(spline_, frames_, tp);
}

// ── Stage setup ───────────────────────────────────────────────────────────────
void MPC::setMPCProblem()
{
    for (int i = 0; i <= N; i++)
        setStage(initial_guess_[i].xk, initial_guess_[i].uk, i);
}

void MPC::setStage(const State &xk, const Input &uk, int k)
{
    stages_[k].nx = NX;
    stages_[k].nu = NU;

    // No polytopic constraints at stage 0 (x0 is fixed)
    if (k == 0) {
        stages_[k].ng = 0;
        stages_[k].ns = 0;
    } else {
        stages_[k].ng = NPC;
        stages_[k].ns = NS;
    }

    stages_[k].cost_mat = normalizeCost(
        cost_.getCost(frames_, spline_, xk, k));

    stages_[k].lin_model = normalizeDynamics(
        model_.getLinModel(xk, uk));

    if (k > 0 && tunnel_ != nullptr) {
        stages_[k].constrains_mat = normalizeCon(
            constraints_.getConstraints(*tunnel_, frames_, xk, uk));
    } else {
        // Empty constraints for stage 0
        ConstrainsMatrix empty;
        empty.C  = C_MPC::Zero();
        empty.D  = D_MPC::Zero();
        empty.dl = d_MPC::Constant(-INF);
        empty.du = d_MPC::Constant(INF);
        stages_[k].constrains_mat = empty;
    }

    stages_[k].l_bounds_x = normalization_param_.T_x_inv * bounds_.getBoundsLX();
    stages_[k].u_bounds_x = normalization_param_.T_x_inv * bounds_.getBoundsUX();
    stages_[k].l_bounds_u = normalization_param_.T_u_inv * bounds_.getBoundsLU();
    stages_[k].u_bounds_u = normalization_param_.T_u_inv * bounds_.getBoundsUU();
    stages_[k].l_bounds_s = normalization_param_.T_s_inv * bounds_.getBoundsLS();
    stages_[k].u_bounds_s = normalization_param_.T_s_inv * bounds_.getBoundsUS();

    // Tighten s trust region around the initial guess arc-length
    const double inv_s = normalization_param_.T_x_inv(si_index.s, si_index.s);
    stages_[k].l_bounds_x(si_index.s) =
        inv_s * (initial_guess_[k].xk.s - param_.s_trust_region);
    stages_[k].u_bounds_x(si_index.s) =
        inv_s * (initial_guess_[k].xk.s + param_.s_trust_region);
}

// ── Normalisation helpers ──────────────────────────────────────────────────────
CostMatrix MPC::normalizeCost(const CostMatrix &cm)
{
    return {
        normalization_param_.T_x * cm.Q * normalization_param_.T_x,
        normalization_param_.T_u * cm.R * normalization_param_.T_u,
        S_MPC::Zero(),
        normalization_param_.T_x * cm.q,
        normalization_param_.T_u * cm.r,
        normalization_param_.T_s * cm.Z * normalization_param_.T_s,
        normalization_param_.T_s * cm.z
    };
}

LinModelMatrix MPC::normalizeDynamics(const LinModelMatrix &lm)
{
    return {
        normalization_param_.T_x_inv * lm.A * normalization_param_.T_x,
        normalization_param_.T_x_inv * lm.B * normalization_param_.T_u,
        normalization_param_.T_x_inv * lm.g
    };
}

ConstrainsMatrix MPC::normalizeCon(const ConstrainsMatrix &cm)
{
    return {
        cm.C * normalization_param_.T_x,
        cm.D * normalization_param_.T_u,
        cm.dl,
        cm.du
    };
}

std::array<OptVariables, N+1> MPC::deNormalizeSolution(
    const std::array<OptVariables, N+1> &sol)
{
    std::array<OptVariables, N+1> out;
    for (int i = 0; i <= N; i++) {
        out[i].xk = vectorToState(normalization_param_.T_x * stateToVector(sol[i].xk));
        out[i].uk = vectorToInput(normalization_param_.T_u * inputToVector(sol[i].uk));
    }
    return out;
}

// ── Initial-guess management ───────────────────────────────────────────────────
void MPC::updateInitialGuess(const State &x0)
{
    // Shift horizon by one step
    for (int i = 1; i < N; i++)
        initial_guess_[i-1] = initial_guess_[i];

    initial_guess_[0].xk = x0;
    initial_guess_[0].uk.setZero();

    initial_guess_[N-1].xk = initial_guess_[N-2].xk;
    initial_guess_[N-1].uk.setZero();

    initial_guess_[N].xk = integrator_.RK4(initial_guess_[N-1].xk,
                                             initial_guess_[N-1].uk, Ts_);
    initial_guess_[N].uk.setZero();
}

void MPC::generateNewInitialGuess(const State &x0)
{
    initial_guess_[0].xk = x0;
    initial_guess_[0].uk.setZero();

    for (int i = 1; i <= N; i++) {
        initial_guess_[i].xk.setZero();
        initial_guess_[i].uk.setZero();

        // Project along the path at the predicted arc-length
        const double s_i = initial_guess_[i-1].xk.s + Ts_ * param_.initial_velocity;
        initial_guess_[i].xk.s = s_i;

        // Use FrenetFrame to seed position along path
        pathweaver::Frame frame = frames_.at(s_i);
        initial_guess_[i].xk.X  = frame.position(0);
        initial_guess_[i].xk.Y  = frame.position(1);
        initial_guess_[i].xk.Z  = frame.position(2);

        // Align with path tangent (yaw only, body flat: qw = cos(yaw/2), qz = sin(yaw/2))
        const double yaw = std::atan2(frame.t(1), frame.t(0));
        initial_guess_[i].xk.qw = std::cos(yaw / 2.0);
        initial_guess_[i].xk.qx = 0.0;
        initial_guess_[i].xk.qy = 0.0;
        initial_guess_[i].xk.qz = std::sin(yaw / 2.0);

        // Hover thrust
        initial_guess_[i].xk.T  = param_.m * 9.81;
        initial_guess_[i].xk.vx = param_.initial_velocity * frame.t(0);
        initial_guess_[i].xk.vy = param_.initial_velocity * frame.t(1);
        initial_guess_[i].xk.vz = param_.initial_velocity * frame.t(2);
        initial_guess_[i].xk.vs = param_.initial_velocity;
    }

    valid_initial_guess_ = true;
}

// ── SQP solution mixing ────────────────────────────────────────────────────────
std::array<OptVariables, N+1> MPC::sqpSolutionUpdate(
    const std::array<OptVariables, N+1> &last,
    const std::array<OptVariables, N+1> &current)
{
    std::array<OptVariables, N+1> updated;
    for (int i = 0; i <= N; i++) {
        updated[i].xk = vectorToState(
            sqp_mixing_ * stateToVector(current[i].xk) +
            (1.0 - sqp_mixing_) * stateToVector(last[i].xk));
        updated[i].uk = vectorToInput(
            sqp_mixing_ * inputToVector(current[i].uk) +
            (1.0 - sqp_mixing_) * inputToVector(last[i].uk));
    }
    return updated;
}

// ── Main MPC entry point ───────────────────────────────────────────────────────
MPCReturn MPC::runMPC(State &x0)
{
    auto t1 = std::chrono::high_resolution_clock::now();

    // Project x0 onto the spline to find the closest arc-length
    // (PathWeaver: find closest point on spline)
    if (tunnel_ != nullptr) {
        // Use the FrenetFrame to project x0's arc-length to nearest valid s
        // Simple approach: keep x0.s as-is; refine via closest-point search
        // (PathWeaver may provide this directly; use the current s as seed)
        const double s_est = x0.s;
        const double track_length = spline_.totalArcLength();
        // Clamp s to valid range
        x0.s = std::fmod(s_est, track_length);
        if (x0.s < 0.0) x0.s += track_length;
    }

    if (valid_initial_guess_)
        updateInitialGuess(x0);
    else
        generateNewInitialGuess(x0);

    int solver_status = -1;
    n_no_solves_sqp_ = 0;

    for (int sqp_it = 0; sqp_it < n_sqp_; sqp_it++) {
        setMPCProblem();
        State x0_norm = vectorToState(normalization_param_.T_x_inv * stateToVector(x0));
        optimal_solution_ = solver_interface_->solveMPC(stages_, x0_norm, &solver_status);
        optimal_solution_ = deNormalizeSolution(optimal_solution_);
        if (solver_status != 0) n_no_solves_sqp_++;
        if (solver_status <= 1)
            initial_guess_ = sqpSolutionUpdate(initial_guess_, optimal_solution_);
    }

    const int max_error = std::max(n_sqp_ - 1, 1);
    if (n_no_solves_sqp_ >= max_error)
        n_non_solves_++;
    else
        n_non_solves_ = 0;

    if (n_non_solves_ >= n_reset_)
        valid_initial_guess_ = false;

    auto t2 = std::chrono::high_resolution_clock::now();
    const double time_nmpc =
        std::chrono::duration<double>(t2 - t1).count();

    return {initial_guess_[0].uk, initial_guess_, time_nmpc};
}

} // namespace mpcc
