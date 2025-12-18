#include "plotting.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

namespace mpcc {

Plotting::Plotting(double Ts, PathToJson path)
    : model_(Model(Ts, path)), param_(Param(path.param_path)), constraints_(Constraints(Ts, path)) {}

void Plotting::plotRun(const std::list<MPCReturn> &log, const TrackPos &track_xy) const {
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
            res.set_content("<h1>Error: visualization/index.html not found</h1>", "text/html");
        }
    });

    // Serve data
    svr.Get("/data", [&](const httplib::Request &, httplib::Response &res) {
        nlohmann::json root;
        root["track"] = jsonTrack(track_xy);
        root["log"] = jsonLog(log);
        res.set_content(root.dump(), "application/json");
    });

    std::cout << "Open http://localhost:8080 in your browser to view the plots." << std::endl;
    std::cout << "Press Ctrl+C to stop the server." << std::endl;

    svr.listen("0.0.0.0", 8080);
}

void Plotting::plotSim(const std::list<MPCReturn> &log, const TrackPos &track_xy) const {
    // Redirect to plotRun as the web interface handles both static and dynamic visualization
    plotRun(log, track_xy);
}

void Plotting::plotBox(const State &x0) const {
    // Deprecated for internal use, logic moved to frontend or json generation if needed
}

nlohmann::json Plotting::jsonTrack(const TrackPos &track_xy) const {
    nlohmann::json j;
    j["X"] = std::vector<double>(track_xy.X.data(), track_xy.X.data() + track_xy.X.size());
    j["Y"] = std::vector<double>(track_xy.Y.data(), track_xy.Y.data() + track_xy.Y.size());
    j["X_inner"] = std::vector<double>(track_xy.X_inner.data(), track_xy.X_inner.data() + track_xy.X_inner.size());
    j["Y_inner"] = std::vector<double>(track_xy.Y_inner.data(), track_xy.Y_inner.data() + track_xy.Y_inner.size());
    j["X_outer"] = std::vector<double>(track_xy.X_outer.data(), track_xy.X_outer.data() + track_xy.X_outer.size());
    j["Y_outer"] = std::vector<double>(track_xy.Y_outer.data(), track_xy.Y_outer.data() + track_xy.Y_outer.size());
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
        step["delta"] = xk.delta;
        step["vs"] = xk.vs;

        // Horizon
        std::vector<double> horizon_x, horizon_y;
        for (const auto &h_step : log_i.mpc_horizon) {
            horizon_x.push_back(h_step.xk.X);
            horizon_y.push_back(h_step.xk.Y);
        }
        step["horizon_x"] = horizon_x;
        step["horizon_y"] = horizon_y;

        j_log.push_back(step);
    }
    return j_log;
}

}  // namespace mpcc