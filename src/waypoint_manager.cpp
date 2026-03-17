#include "indooruav_waypoint_manager/waypoint_manager.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

namespace indooruav_waypoint_manager
{

// ─── Constructor ─────────────────────────────────────────────────────────────
WaypointManager::WaypointManager(ros::NodeHandle&   nh,
                                 double             delta_L_m,
                                 double             delta_T_s,
                                 double             delta_A_deg,
                                 const std::string& odom_topic,
                                 const std::string& output_path)
    : recorder_(output_path)
    , delta_L_m_(delta_L_m)
    , delta_T_s_(delta_T_s)
    , delta_A_rad_(delta_A_deg * M_PI / 180.0)
{
    ROS_INFO_STREAM("[WaypointManager] 初始化完成");
    ROS_INFO_STREAM("  ΔL（XYZ）= " << delta_L_m_  << " m");
    ROS_INFO_STREAM("  ΔT       = " << delta_T_s_  << " s");
    ROS_INFO_STREAM("  ΔA       = " << delta_A_deg << " °  ("
                    << delta_A_rad_ << " rad)");
    ROS_INFO_STREAM("  odom_topic  = " << odom_topic);
    ROS_INFO_STREAM("  output_path = " << output_path);

    odom_sub_ = nh.subscribe(odom_topic, 10,
                             &WaypointManager::odomCallback, this);
}

// ─── odomCallback ────────────────────────────────────────────────────────────
void WaypointManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    const double cx  = msg->pose.pose.position.x;
    const double cy  = msg->pose.pose.position.y;
    const double cz  = msg->pose.pose.position.z;
    const ros::Time now = msg->header.stamp;

    const auto& q = msg->pose.pose.orientation;
    const double current_yaw = quaternionToYaw(q.x, q.y, q.z, q.w);

    // ── 首条消息：直接记录并初始化状态 ─────────────────────────────────────
    if (first_msg_) {
        first_msg_        = false;
        last_x_           = cx;
        last_y_           = cy;
        last_z_           = cz;
        last_record_time_ = now;
        last_yaw_         = current_yaw;
        recorder_.record(makeWaypoint(msg));
        return;
    }

    // ── 计算当前步长并累加 XYZ 位移 ───────────────────────────────────────────
    const double step = distance3D(cx, cy, cz, last_x_, last_y_, last_z_);
    accum_dist_ += step;

    // 每帧更新位置（保证累计位移连续）
    last_x_ = cx;
    last_y_ = cy;
    last_z_ = cz;

    const double elapsed  = (now - last_record_time_).toSec();
    const double yaw_diff = yawDiff(current_yaw, last_yaw_);

    // ── 判断 ΔL / ΔT / ΔA 触发条件 ──────────────────────────────────────────
    const bool trigger_L = (accum_dist_  >= delta_L_m_);
    const bool trigger_T = (elapsed      >= delta_T_s_);
    const bool trigger_A = (delta_A_rad_ > 0.0) && (yaw_diff >= delta_A_rad_);

    if (trigger_L || trigger_T || trigger_A) {
        recorder_.record(makeWaypoint(msg));

        // 重置累计量
        accum_dist_       = 0.0;
        last_record_time_ = now;
        last_yaw_         = current_yaw;

        // 调试日志（仅在 DEBUG 级别输出，不影响正常运行性能）
        std::string reasons;
        if (trigger_L) reasons += "ΔL ";
        if (trigger_T) reasons += "ΔT ";
        if (trigger_A) reasons += "ΔA ";
        ROS_DEBUG_STREAM("[WaypointManager] 触发：" << reasons
                         << "| accum=" << accum_dist_ << " m"
                         << "  elapsed=" << elapsed << " s"
                         << "  yaw_diff=" << (yaw_diff * 180.0 / M_PI) << " °");
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
    wp.time = formatTime(msg->header.stamp);

    return wp;
}

// ─── distance3D ──────────────────────────────────────────────────────────────
double WaypointManager::distance3D(double x1, double y1, double z1,
                                   double x2, double y2, double z2)
{
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    const double dz = z1 - z2;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ─── quaternionToYaw ─────────────────────────────────────────────────────────
double WaypointManager::quaternionToYaw(double qx, double qy,
                                        double qz, double qw)
{
    // 标准 ZYX 欧拉角中的偏航角（yaw）提取公式
    return std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));
}

// ─── yawDiff ─────────────────────────────────────────────────────────────────
double WaypointManager::yawDiff(double yaw1, double yaw2)
{
    double diff = yaw1 - yaw2;
    // 归一化到 (-π, π]
    while (diff >  M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return std::fabs(diff);
}

// ─── formatTime ──────────────────────────────────────────────────────────────
std::string WaypointManager::formatTime(const ros::Time& t)
{
    const std::time_t secs  = static_cast<std::time_t>(t.sec);
    const uint32_t    nsecs = t.nsec;

    std::tm* tm_info = std::gmtime(&secs);
    std::ostringstream oss;
    oss << std::put_time(tm_info, "%Y-%m-%dT%H:%M:%S");
    oss << '.' << std::setw(3) << std::setfill('0') << (nsecs / 1000000u);
    return oss.str();
}

} // namespace indooruav_waypoint_manager
