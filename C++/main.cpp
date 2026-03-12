#include "mpcc/mpc.h"
#include "mpcc/integrator.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <list>

using json = nlohmann::json;

int main()
{
    using namespace mpcc;

    std::ifstream iConfig("params/config.json");
    if (!iConfig.is_open()) {
        std::cerr << "ERROR: could not open params/config.json\n";
        return 1;
    }
    json jc;
    iConfig >> jc;

    PathToJson paths{
        jc["model_path"].get<std::string>(),
        jc["cost_path"].get<std::string>(),
        jc["bounds_path"].get<std::string>(),
        jc["normalization_path"].get<std::string>(),
        jc["waypoints_path"].get<std::string>()
    };

    MPC mpc(jc["n_sqp"], jc["n_reset"], jc["sqp_mixing"], jc["Ts"], paths);
    mpc.setTrack(paths.waypoints_path,
                 jc["tunnel_width"],
                 jc["tunnel_gate_width"],
                 jc["tunnel_k"],
                 jc["frame_ds"]);

    Integrator integrator(jc["Ts"], paths);

    // Initial state: hovering at origin, identity quaternion (body z-up), at rest
    State x0;
    x0.setZero();
    x0.qw = 1.0;                          // identity rotation
    x0.T  = 9.81;                         // hover thrust = m*g = 1.0 * 9.81
    x0.vs = jc["v0"].get<double>();
    x0.vx = jc["v0"].get<double>();

    std::list<MPCReturn> log;
    const int    n_sim = jc["n_sim"].get<int>();
    const double Ts    = jc["Ts"].get<double>();

    for (int i = 0; i < n_sim; i++) {
        MPCReturn sol = mpc.runMPC(x0);
        x0 = integrator.simTimeStep(x0, sol.u0, Ts);
        log.push_back(sol);
    }

    double mean_t = 0.0, max_t = 0.0;
    for (auto &l : log) {
        mean_t += l.time_total;
        if (l.time_total > max_t) max_t = l.time_total;
    }
    mean_t /= static_cast<double>(log.size());

    std::cout << "mean nmpc time: " << mean_t << " s\n";
    std::cout << "max  nmpc time: " << max_t  << " s\n";
    return 0;
}
