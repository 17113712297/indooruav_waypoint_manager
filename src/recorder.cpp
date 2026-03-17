#include "indooruav_waypoint_manager/recorder.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include <ros/ros.h>

namespace indooruav_waypoint_manager
{

// ─── 辅助：将 double 写成固定小数位 JSON 数值 ──────────────────────────────
static std::string dbl(double v, int prec = 6)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(prec) << v;
    return oss.str();
}

// ─── 辅助：JSON 字符串转义（仅处理时间戳内可能出现的字符） ──────────────────
static std::string jsonStr(const std::string& s)
{
    std::string out;
    out.reserve(s.size() + 2);
    out += '"';
    for (char c : s) {
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else out += c;
    }
    out += '"';
    return out;
}

// ─── Constructor ─────────────────────────────────────────────────────────────
Recorder::Recorder(const std::string& output_path)
    : output_path_(output_path)
{}

// ─── Destructor：自动保存 ─────────────────────────────────────────────────
Recorder::~Recorder()
{
    if (!waypoints_.empty()) {
        save();
    }
}

// ─── record ──────────────────────────────────────────────────────────────────
void Recorder::record(const Waypoint& wp)
{
    waypoints_.push_back(wp);
    ROS_INFO_STREAM("[Recorder] 已记录第 " << waypoints_.size()
                    << " 个航点  time=" << wp.time
                    << "  x=" << wp.x_m << "  y=" << wp.y_m);
}

// ─── save ─────────────────────────────────────────────────────────────────────
bool Recorder::save() const
{
    std::ofstream ofs(output_path_);
    if (!ofs.is_open()) {
        ROS_ERROR_STREAM("[Recorder] 无法打开输出文件：" << output_path_);
        return false;
    }

    ofs << "{\n";
    ofs << "  \"waypoints\": [\n";

    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
        const auto& wp = waypoints_[i];
        ofs << "    {\n";
        ofs << "      \"x_m\":       " << dbl(wp.x_m)       << ",\n";
        ofs << "      \"y_m\":       " << dbl(wp.y_m)       << ",\n";
        ofs << "      \"z_m\":       " << dbl(wp.z_m)       << ",\n";
        ofs << "      \"q_x\":       " << dbl(wp.q_x)       << ",\n";
        ofs << "      \"q_y\":       " << dbl(wp.q_y)       << ",\n";
        ofs << "      \"q_z\":       " << dbl(wp.q_z)       << ",\n";
        ofs << "      \"q_w\":       " << dbl(wp.q_w)       << ",\n";
        ofs << "      \"time\":      " << jsonStr(wp.time)  << "\n";
        ofs << "    }";
        if (i + 1 < waypoints_.size()) ofs << ",";
        ofs << "\n";
    }

    ofs << "  ]\n";
    ofs << "}\n";

    ROS_INFO_STREAM("[Recorder] 已将 " << waypoints_.size()
                    << " 个航点写入文件：" << output_path_);
    return true;
}

} // namespace indooruav_waypoint_manager
