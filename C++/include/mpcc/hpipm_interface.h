#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include <blasfeo_d_aux_ext_dep.h>
#include "hpipm_d_ocp_qp_ipm.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "hpipm_timing.h"

#include "mpcc/config.h"
#include "mpcc/types.h"
#include "mpcc/solver_interface.h"

#include <array>
#include <vector>

namespace mpcc {

struct OptVariables;
struct Stage;

struct HpipmBound {
    std::vector<int>    idx_u;
    std::vector<int>    idx_x;
    std::vector<int>    idx_s;
    std::vector<double> lower_bounds_u;
    std::vector<double> upper_bounds_u;
    std::vector<double> lower_bounds_x;
    std::vector<double> upper_bounds_x;
};

class HpipmInterface : public SolverInterface {
public:
    std::array<OptVariables, N + 1> solveMPC(
        std::array<Stage, N + 1> &stages,
        const State &x0,
        int *status) override;

    ~HpipmInterface() override {
        std::cout << "Deleting HpipmInterface" << std::endl;
    }

private:
    // Per-stage sizes
    int nx_[N + 1];
    int nu_[N + 1];
    int nbx_[N + 1];
    int nbu_[N + 1];
    int ng_[N + 1];
    int nsbx_[N + 1];
    int nsbu_[N + 1];
    int nsg_[N + 1];

    // Dynamics:  x_{k+1} = A_k x_k + B_k u_k + b_k
    double *hA_[N];
    double *hB_[N];
    double *hb_[N];

    // Cost:  0.5*[x;u]^T*[Q,S;S^T,R]*[x;u] + [q;r]^T*[x;u]
    double *hQ_[N + 1];
    double *hS_[N + 1];
    double *hR_[N + 1];
    double *hq_[N + 1];
    double *hr_[N + 1];

    // Polytopic constraints:  g_lo <= D*x + C*u <= g_hi
    double *hlg_[N + 1];
    double *hug_[N + 1];
    double *hC_[N + 1];
    double *hD_[N + 1];

    // Box bounds
    int    *hidxbx_[N + 1];
    double *hlbx_[N + 1];
    double *hubx_[N + 1];
    int    *hidxbu_[N + 1];
    double *hlbu_[N + 1];
    double *hubu_[N + 1];

    // Soft-constraint cost
    double *hZl_[N + 1];
    double *hZu_[N + 1];
    double *hzl_[N + 1];
    double *hzu_[N + 1];

    // Slack bounds & indices
    double *hlls_[N + 1];
    double *hlus_[N + 1];
    int    *hidxs_[N + 1];

    std::array<HpipmBound, N + 1> hpipm_bounds_;
    Eigen::Matrix<double, NX, 1>  b0_;

    void setDynamics(std::array<Stage, N + 1> &stages, const State &x0);
    void setCost(std::array<Stage, N + 1> &stages);
    void setBounds(std::array<Stage, N + 1> &stages, const State &x0);
    void setPolytopicConstraints(std::array<Stage, N + 1> &stages);
    void setSoftConstraints(std::array<Stage, N + 1> &stages);

    std::array<OptVariables, N + 1> Solve(int *status);

    void print_data();
};

} // namespace mpcc
