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
 *          - XY 平面累计行驶距离 >= delta_L_m_（ΔL 条件，仅 XY）
 *          - 距上次记录时间 >= delta_T_s_（ΔT 条件）
 *          - 偏航角累计变化 >= delta_A_deg_（ΔA 条件）
 */
class WaypointManager
{
public:
    /**
     * @param nh          ROS 节点句柄。
     * @param delta_L_m   触发记录的最小 XY 位移阈值（米）。
     * @param delta_T_s   触发记录的最大时间间隔（秒）。
     * @param delta_A_deg 触发记录的最小偏航角变化阈值（度，0 表示禁用）。
     * @param odom_topic  里程计话题名称。
     * @param output_path JSON 输出文件路径。
     */
    WaypointManager(ros::NodeHandle&   nh,
                    double             delta_L_m,
                    double             delta_T_s,
                    double             delta_A_deg,
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

    /// 计算两点间 XY 平面欧氏距离（忽略 Z 轴）。
    static double distance2D(double x1, double y1,
                             double x2, double y2);

    /// 从四元数提取偏航角（弧度，范围 [-π, π]）。
    static double quaternionToYaw(double qx, double qy,
                                  double qz, double qw);

    /// 计算两偏航角之差的绝对值，结果归一化到 [0, π]。
    static double yawDiff(double yaw1, double yaw2);

    /// 格式化 ROS 时间为 ISO-8601 字符串。
    static std::string formatTime(const ros::Time& t);

    // ── 成员 ────────────────────────────────────────────────────────────────
    ros::Subscriber odom_sub_;

    Recorder        recorder_;          ///< 组合的记录器

    double          delta_L_m_;         ///< ΔL 阈值（米，仅 XY）
    double          delta_T_s_;         ///< ΔT 阈值（秒）
    double          delta_A_rad_;       ///< ΔA 阈值（弧度，内部存储）

    // 上次记录时的状态
    bool            first_msg_         = true;
    double          last_x_            = 0.0;
    double          last_y_            = 0.0;
    ros::Time       last_record_time_;
    double          last_yaw_          = 0.0;   ///< 上次记录时的偏航角（弧度）

    // 累计未记录的位移
    double          accum_dist_        = 0.0;
};

} // namespace indooruav_waypoint_manager
