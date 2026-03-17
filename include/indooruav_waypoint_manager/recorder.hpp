#pragma once

#include <string>
#include <vector>

#include "indooruav_waypoint_manager/waypoint.hpp"

namespace indooruav_waypoint_manager
{

/**
 * @brief  Recorder 负责收集 Waypoint 并在析构或显式调用时将其写入 JSON 文件。
 *
 * 线程安全说明：record() 仅在 ROS 回调线程中被调用，无需额外加锁。
 */
class Recorder
{
public:
    /**
     * @param output_path  JSON 文件的完整输出路径（含文件名）。
     */
    explicit Recorder(const std::string& output_path);

    /// 析构时自动将 waypoints_ 落盘。
    ~Recorder();

    /**
     * @brief 追加一条 Waypoint 记录。
     * @param wp  要记录的航点。
     */
    void record(const Waypoint& wp);

    /**
     * @brief 将当前 waypoints_ 写入 JSON 文件。
     *        可被外部显式调用（例如收到 SIGINT 时）。
     * @return true 表示写入成功。
     */
    bool save() const;

    /// 返回当前已记录的航点数量。
    std::size_t size() const { return waypoints_.size(); }

    /// 返回只读的航点列表引用。
    const std::vector<Waypoint>& waypoints() const { return waypoints_; }

private:
    std::string           output_path_;
    std::vector<Waypoint> waypoints_;
};

} // namespace indooruav_waypoint_manager
