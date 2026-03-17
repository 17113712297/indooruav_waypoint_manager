#pragma once

#include <ctime>
#include <string>
#include <iomanip>
#include <sstream>

namespace indooruav_waypoint_manager
{

struct Waypoint
{
    double x_m      = 0.0;
    double y_m      = 0.0;
    double z_m      = 0.0;
    double q_x = 0.0;
    double q_y = 0.0;
    double q_z = 0.0;
    double q_w = 1.0;
    std::string time;          // ISO-8601 timestamp, e.g. "2025-03-11T08:30:00.123"

    Waypoint() = default;

    Waypoint(double x, double y, double z,
             double q_x, double q_y, double q_z, double q_w,
             const std::string& timestamp)
        : x_m(x), y_m(y), z_m(z)
        , q_x(q_x), q_y(q_y), q_z(q_z), q_w(q_w)
        , time(timestamp)
    {}
};

} // namespace indooruav_waypoint_manager
