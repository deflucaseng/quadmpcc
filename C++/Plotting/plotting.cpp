#include "Plotting/plotting.h"

#include <chrono>  // NOLINT(build/c++11)
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>  // NOLINT(build/c++11)
#include <vector>

namespace mpcc {

Plotting::Plotting(double Ts, PathToJson path, const TrackFull &track)
    : model_(Model(Ts, path)),
      param_(Param(path.param_path)),
      constraints_(Constraints(Ts, path)),
      track_(BoostSplines(path)) {
  track_.genSplines(track);
}

void Plotting::plotRun(const std::list<MPCReturn> &log,
                       const TrackPos &track_xy) const {
  // In this web-based version, we'll combine plotRun and plotSim functionality.
  // plotRun in the original code showed static plots of the whole run.
  std::cout << "Starting Web Plotting Server..." << std::endl;

  httplib::Server svr;

  // Serve static index.html
  svr.Get("/", [](const httplib::Request &, httplib::Response &res) {
    std::ifstream file("visualization/index.html");
    if (file) {
      std::stringstream buffer;
      buffer << file.rdbuf();
      res.set_content(buffer.str(), "text/html");
    } else {
      res.set_content("<h1>Error: visualization/index.html not found</h1>",
                      "text/html");
    }
  });

  // Serve data
  svr.Get("/data", [&](const httplib::Request &, httplib::Response &res) {
    nlohmann::json root;
    root["track"] = jsonTrack(track_xy);
    root["log"] = jsonLog(log);
    res.set_content(root.dump(), "application/json");
  });

  std::cout << "Open http://localhost:8080 in your browser to view the plots."
            << std::endl;
  std::cout << "Press Ctrl+C to stop the server." << std::endl;

  svr.listen("0.0.0.0", 8080);
}

void Plotting::plotSim(const std::list<MPCReturn> &log,
                       const TrackPos &track_xy) const {
  // Redirect to plotRun as the web interface handles both static and dynamic
  // visualization
  plotRun(log, track_xy);
}

void Plotting::plotBox(const State &x0) const {
  // Deprecated for internal use, logic moved to frontend or json generation if
  // needed
}

nlohmann::json Plotting::jsonTrack(const TrackPos &track_xy) const {
  nlohmann::json j;
  j["X"] = std::vector<double>(track_xy.X.data(),
                               track_xy.X.data() + track_xy.X.size());
  j["Y"] = std::vector<double>(track_xy.Y.data(),
                               track_xy.Y.data() + track_xy.Y.size());
  j["X_inner"] =
      std::vector<double>(track_xy.X_inner.data(),
                          track_xy.X_inner.data() + track_xy.X_inner.size());
  j["Y_inner"] =
      std::vector<double>(track_xy.Y_inner.data(),
                          track_xy.Y_inner.data() + track_xy.Y_inner.size());
  j["X_outer"] =
      std::vector<double>(track_xy.X_outer.data(),
                          track_xy.X_outer.data() + track_xy.X_outer.size());
  j["Y_outer"] =
      std::vector<double>(track_xy.Y_outer.data(),
                          track_xy.Y_outer.data() + track_xy.Y_outer.size());
  return j;
}

nlohmann::json Plotting::jsonLog(const std::list<MPCReturn> &log) const {
  nlohmann::json j_log = nlohmann::json::array();

  for (const auto &log_i : log) {
    nlohmann::json step;
    const auto &xk = log_i.mpc_horizon[0].xk;

    step["x"] = xk.X;
    step["y"] = xk.Y;
    step["phi"] = xk.phi;
    step["vx"] = xk.vx;
    step["vy"] = xk.vy;
    step["r"] = xk.r;
    step["s"] = xk.s;
    step["D"] = xk.D;
    step["B"] = xk.B;
    step["delta"] = xk.delta;
    step["vs"] = xk.vs;
    step["kappa"] = track_.getCurvature(xk.s);

    // Inputs
    const auto &uk = log_i.u0;
    step["dD"] = uk.dD;
    step["dB"] = uk.dB;
    step["dDelta"] = uk.dDelta;
    step["dVs"] = uk.dVs;

    // Horizon
    std::vector<double> horizon_x, horizon_y, horizon_phi, horizon_vx,
        horizon_vy, horizon_r, horizon_s, horizon_D, horizon_B, horizon_delta,
        horizon_vs, horizon_kappa;
    std::vector<double> horizon_dD, horizon_dB, horizon_dDelta, horizon_dVs;
    for (const auto &h_step : log_i.mpc_horizon) {
      horizon_x.push_back(h_step.xk.X);
      horizon_y.push_back(h_step.xk.Y);
      horizon_phi.push_back(h_step.xk.phi);
      horizon_vx.push_back(h_step.xk.vx);
      horizon_vy.push_back(h_step.xk.vy);
      horizon_r.push_back(h_step.xk.r);
      horizon_s.push_back(h_step.xk.s);
      horizon_D.push_back(h_step.xk.D);
      horizon_B.push_back(h_step.xk.B);
      horizon_delta.push_back(h_step.xk.delta);
      horizon_vs.push_back(h_step.xk.vs);
      horizon_kappa.push_back(track_.getCurvature(h_step.xk.s));

      horizon_dD.push_back(h_step.uk.dD);
      horizon_dB.push_back(h_step.uk.dB);
      horizon_dDelta.push_back(h_step.uk.dDelta);
      horizon_dVs.push_back(h_step.uk.dVs);
    }
    step["horizon_x"] = horizon_x;
    step["horizon_y"] = horizon_y;
    step["horizon_phi"] = horizon_phi;
    step["horizon_vx"] = horizon_vx;
    step["horizon_vy"] = horizon_vy;
    step["horizon_r"] = horizon_r;
    step["horizon_s"] = horizon_s;
    step["horizon_D"] = horizon_D;
    step["horizon_B"] = horizon_B;
    step["horizon_delta"] = horizon_delta;
    step["horizon_vs"] = horizon_vs;
    step["horizon_kappa"] = horizon_kappa;

    step["horizon_dD"] = horizon_dD;
    step["horizon_dB"] = horizon_dB;
    step["horizon_dDelta"] = horizon_dDelta;
    step["horizon_dVs"] = horizon_dVs;

    step["terminal_vx_constraint"] =
        track_.getVelocity(log_i.mpc_horizon[N].xk.s);

    j_log.push_back(step);
  }
  return j_log;
}

}  // namespace mpcc
