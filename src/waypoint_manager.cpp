#include "indooruav_waypoint_manager/waypoint_manager.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

#include <tf/transform_datatypes.h>

namespace indooruav_waypoint_manager
{

// ─── Constructor ─────────────────────────────────────────────────────────────
WaypointManager::WaypointManager(ros::NodeHandle& nh,
                                 double           delta_L_m,
                                 double           delta_T_s,
                                 const std::string& odom_topic,
                                 const std::string& output_path)
    : recorder_(output_path)
    , delta_L_m_(delta_L_m)
    , delta_T_s_(delta_T_s)
{
    ROS_INFO_STREAM("[WaypointManager] 初始化完成");
    ROS_INFO_STREAM("  ΔL = " << delta_L_m_ << " m");
    ROS_INFO_STREAM("  ΔT = " << delta_T_s_ << " s");
    ROS_INFO_STREAM("  odom_topic  = " << odom_topic);
    ROS_INFO_STREAM("  output_path = " << output_path);

    odom_sub_ = nh.subscribe(odom_topic, 10,
                             &WaypointManager::odomCallback, this);
}

// ─── odomCallback ────────────────────────────────────────────────────────────
void WaypointManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    const double cx = msg->pose.pose.position.x;
    const double cy = msg->pose.pose.position.y;
    const double cz = msg->pose.pose.position.z;
    const ros::Time now = msg->header.stamp;

    // ── 首条消息：直接记录并初始化状态 ─────────────────────────────────────
    if (first_msg_) {
        first_msg_       = false;
        last_x_          = cx;
        last_y_          = cy;
        last_z_          = cz;
        last_record_time_ = now;
        recorder_.record(makeWaypoint(msg));
        return;
    }

    // ── 计算当前步长并累加位移 ───────────────────────────────────────────────
    const double step = distance3D(cx, cy, cz, last_x_, last_y_, last_z_);
    accum_dist_ += step;

    // 更新"上一帧位置"（每帧都更新，保证累计位移连续）
    last_x_ = cx;
    last_y_ = cy;
    last_z_ = cz;

    const double elapsed = (now - last_record_time_).toSec();

    // ── 判断 ΔL / ΔT 触发条件 ────────────────────────────────────────────────
    const bool trigger_L = (accum_dist_ >= delta_L_m_);
    const bool trigger_T = (elapsed     >= delta_T_s_);

    if (trigger_L || trigger_T) {
        recorder_.record(makeWaypoint(msg));

        // 重置累计量
        accum_dist_       = 0.0;
        last_record_time_ = now;

        if (trigger_L && trigger_T) {
            ROS_DEBUG("[WaypointManager] 触发：ΔL + ΔT");
        } else if (trigger_L) {
            ROS_DEBUG_STREAM("[WaypointManager] 触发：ΔL（accum=" << accum_dist_ << " m）");
        } else {
            ROS_DEBUG_STREAM("[WaypointManager] 触发：ΔT（elapsed=" << elapsed << " s）");
        }
    }
}

// ─── makeWaypoint ────────────────────────────────────────────────────────────
Waypoint WaypointManager::makeWaypoint(const nav_msgs::Odometry::ConstPtr& msg) const
{
    Waypoint wp;
    wp.x_m = msg->pose.pose.position.x;
    wp.y_m = msg->pose.pose.position.y;
    wp.z_m = msg->pose.pose.position.z;

    const auto& q = msg->pose.pose.orientation;
    wp.q_x = q.x;
    wp.q_y = q.y;
    wp.q_z = q.z;
    wp.q_w = q.w;
    wp.time      = formatTime(msg->header.stamp);

    return wp;
}

// ─── distance3D ──────────────────────────────────────────────────────────────
double WaypointManager::distance3D(double x1, double y1, double z1,
                                   double x2, double y2, double z2)
{
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    const double dz = z1 - z2;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// ─── formatTime ──────────────────────────────────────────────────────────────
std::string WaypointManager::formatTime(const ros::Time& t)
{
    const std::time_t secs = static_cast<std::time_t>(t.sec);
    const uint32_t    nsecs = t.nsec;

    std::tm* tm_info = std::gmtime(&secs);
    std::ostringstream oss;
    oss << std::put_time(tm_info, "%Y-%m-%dT%H:%M:%S");
    // 追加毫秒
    oss << '.' << std::setw(3) << std::setfill('0') << (nsecs / 1000000u);
    return oss.str();
}

} // namespace indooruav_waypoint_manager
