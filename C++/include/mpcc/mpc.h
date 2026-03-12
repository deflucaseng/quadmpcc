#pragma once

#include "mpcc/config.h"
#include "mpcc/types.h"
#include "mpcc/params.h"
#include "mpcc/model.h"
#include "mpcc/integrator.h"
#include "mpcc/cost.h"
#include "mpcc/constraints.h"
#include "mpcc/bounds.h"
#include "mpcc/solver_interface.h"
#include "mpcc/hpipm_interface.h"

#include <PathWeaver/pathweaver.hpp>

#include <array>
#include <memory>
#include <ctime>
#include <chrono>
#include <list>

namespace mpcc {

struct OptVariables {
    State xk;
    Input uk;
};

struct Stage {
    LinModelMatrix    lin_model;
    CostMatrix        cost_mat;
    ConstrainsMatrix  constrains_mat;

    Bounds_x u_bounds_x;
    Bounds_x l_bounds_x;
    Bounds_u u_bounds_u;
    Bounds_u l_bounds_u;
    Bounds_s u_bounds_s;
    Bounds_s l_bounds_s;

    int nx, nu, nbx, nbu, ng, ns;
};

struct MPCReturn {
    const Input u0;
    const std::array<OptVariables, N + 1> mpc_horizon;
    const double time_total;
};

class MPC {
public:
    MPC();
    MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts, const PathToJson &path);

    /// Load PathWeaver spline + build tunnel (call once before runMPC)
    void setTrack(const std::string &waypoints_yaml,
                  double tunnel_width,
                  double tunnel_gate_width,
                  double tunnel_k,
                  double frame_ds);

    MPCReturn runMPC(State &x0);

private:
    void setMPCProblem();
    void setStage(const State &xk, const Input &uk, int time_step);

    CostMatrix       normalizeCost(const CostMatrix &cost_mat);
    LinModelMatrix   normalizeDynamics(const LinModelMatrix &lin_model);
    ConstrainsMatrix normalizeCon(const ConstrainsMatrix &con_mat);

    std::array<OptVariables, N + 1> deNormalizeSolution(
        const std::array<OptVariables, N + 1> &solution);

    void updateInitialGuess(const State &x0);
    void generateNewInitialGuess(const State &x0);

    std::array<OptVariables, N + 1> sqpSolutionUpdate(
        const std::array<OptVariables, N + 1> &last,
        const std::array<OptVariables, N + 1> &current);

    // SQP bookkeeping
    int    n_sqp_;
    double sqp_mixing_;
    int    n_non_solves_;
    int    n_no_solves_sqp_;
    int    n_reset_;
    bool   valid_initial_guess_;

    const double Ts_;

    // MPCC components
    Model       model_;
    Integrator  integrator_;
    Cost        cost_;
    Constraints constraints_;
    Bounds      bounds_;

    NormalizationParam normalization_param_;
    Param              param_;

    std::array<Stage,        N + 1> stages_;
    std::array<OptVariables, N + 1> initial_guess_;
    std::array<OptVariables, N + 1> optimal_solution_;

    std::unique_ptr<SolverInterface> solver_interface_;

    // PathWeaver track representation
    pathweaver::HermiteSpline spline_;
    pathweaver::FrenetFrame   frames_;
    pathweaver::Tunnel       *tunnel_ = nullptr;
};

} // namespace mpcc
