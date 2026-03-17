#include <ros/ros.h>
#include <csignal>
#include <memory>

#include "indooruav_waypoint_manager/waypoint_manager.hpp"

// 全局指针，供信号处理函数调用 save()
static std::shared_ptr<indooruav_waypoint_manager::WaypointManager> g_manager;

void sigintHandler(int /*sig*/)
{
    if (g_manager) {
        ROS_INFO("[indooruav_waypoint_manager] 收到 SIGINT，正在保存数据…");
        g_manager->save();
    }
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "indooruav_waypoint_manager",
              ros::init_options::NoSigintHandler);
    setlocale(LC_ALL, "");  
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");   // 私有参数命名空间

    // ── 读取参数 ────────────────────────────────────────────────────────────
    double      delta_L_m   = 1.0;
    double      delta_T_s   = 5.0;
    std::string odom_topic  = "/odom";
    std::string output_path = "/tmp/waypoints.json";

    pnh.param("delta_L_m",   delta_L_m,   delta_L_m);
    pnh.param("delta_T_s",   delta_T_s,   delta_T_s);
    pnh.param("odom_topic",  odom_topic,  odom_topic);
    pnh.param("output_path", output_path, output_path);

    // ── 参数校验 ────────────────────────────────────────────────────────────
    if (delta_L_m <= 0.0) {
        ROS_WARN("[indooruav_waypoint_manager] delta_L_m <= 0，已重置为 1.0 m");
        delta_L_m = 1.0;
    }
    if (delta_T_s <= 0.0) {
        ROS_WARN("[indooruav_waypoint_manager] delta_T_s <= 0，已重置为 5.0 s");
        delta_T_s = 5.0;
    }

    // ── 创建管理器 ───────────────────────────────────────────────────────────
    g_manager = std::make_shared<indooruav_waypoint_manager::WaypointManager>(
        nh, delta_L_m, delta_T_s, odom_topic, output_path);

    // 注册信号处理（确保 Ctrl+C 时保存文件）
    std::signal(SIGINT, sigintHandler);

    ROS_INFO("[indooruav_waypoint_manager] 节点启动，等待里程计数据…");
    ros::spin();

    // ros::shutdown() 已在 sigintHandler 中调用，此处仅作保底
    if (g_manager) g_manager->save();

    return 0;
}
