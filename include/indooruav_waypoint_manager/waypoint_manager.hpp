#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "indooruav_waypoint_manager/waypoint.hpp"
#include "indooruav_waypoint_manager/recorder.hpp"

namespace indooruav_waypoint_manager
{

/**
 * @brief WaypointManager 订阅 /odom（或用户指定话题），
 *        满足以下任一条件时触发一次 Recorder::record()：
 *          - 累计行驶距离 >= delta_L_m_（ΔL 条件）
 *          - 距上次记录时间 >= delta_T_s_（ΔT 条件）
 */
class WaypointManager
{
public:
    /**
     * @param nh          ROS 节点句柄（用于订阅话题、读取参数）。
     * @param delta_L_m   触发记录的最小位移阈值（米）。
     * @param delta_T_s   触发记录的最大时间间隔（秒）。
     * @param odom_topic  里程计话题名称。
     * @param output_path JSON 输出文件路径。
     */
    WaypointManager(ros::NodeHandle& nh,
                    double           delta_L_m,
                    double           delta_T_s,
                    const std::string& odom_topic,
                    const std::string& output_path);

    ~WaypointManager() = default;

    /// 显式保存（可在 shutdown 钩子中调用）。
    void save() const { recorder_.save(); }

private:
    // ── 里程计回调 ──────────────────────────────────────────────────────────
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    // ── 辅助函数 ────────────────────────────────────────────────────────────
    /// 从当前消息构建 Waypoint（含时间戳）。
    Waypoint makeWaypoint(const nav_msgs::Odometry::ConstPtr& msg) const;

    /// 计算两点间欧氏距离（3D）。
    static double distance3D(double x1, double y1, double z1,
                             double x2, double y2, double z2);

    /// 格式化 ROS 时间为 ISO-8601 字符串。
    static std::string formatTime(const ros::Time& t);

    // ── 成员 ────────────────────────────────────────────────────────────────
    ros::Subscriber odom_sub_;

    Recorder        recorder_;          ///< 组合的记录器

    double          delta_L_m_;         ///< ΔL 阈值（米）
    double          delta_T_s_;         ///< ΔT 阈值（秒）

    // 上次记录时的状态
    bool            first_msg_         = true;
    double          last_x_            = 0.0;
    double          last_y_            = 0.0;
    double          last_z_            = 0.0;
    ros::Time       last_record_time_;

    // 累计未记录的位移
    double          accum_dist_        = 0.0;
};

} // namespace indooruav_waypoint_manager
