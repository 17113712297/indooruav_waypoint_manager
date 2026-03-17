# indooruav_waypoint_manager

> **ROS 功能包** · 室内无人机航点自动记录器

订阅全局里程计话题，当满足**累计位移阈值（ΔL）**或**时间间隔阈值（ΔT）**任一条件时，自动采集机器人当前位姿并追加写入 JSON 航点文件。

---

## 目录

- [功能概述](#功能概述)
- [工作原理](#工作原理)
- [目录结构](#目录结构)
- [依赖环境](#依赖环境)
- [编译](#编译)
- [配置参数](#配置参数)
- [启动](#启动)
- [输出格式](#输出格式)
- [模块说明](#模块说明)
- [注意事项](#注意事项)

---

## 功能概述

| 特性 | 说明 |
|------|------|
| 双触发条件 | 累计位移 \(\geq \Delta L\) **或** 经过时间 \(\geq \Delta T\) 时记录一个航点 |
| 三维位姿 | 记录 \((x,\,y,\,z)\) 位置与四元数 \((q_x,\,q_y,\,q_z,\,q_w)\) 姿态 |
| ISO-8601 时间戳 | 每条航点附带毫秒精度时间戳，例如 `2025-03-11T08:30:00.123` |
| JSON 输出 | 所有航点序列化为结构化 JSON 文件，便于后续解析与回放 |
| 安全落盘 | 节点收到 `SIGINT`（Ctrl+C）时自动保存，析构时也会兜底写盘 |

---

## 工作原理

节点在每一帧里程计消息回调中执行如下逻辑：

\[
\text{记录} \iff \underbrace{\sum_{k} \|p_k - p_{k-1}\|_2 \;\geq\; \Delta L}_{\text{位移触发}} \;\;\mathbf{OR}\;\; \underbrace{t_{\text{now}} - t_{\text{last}} \;\geq\; \Delta T}_{\text{时间触发}}
\]

其中累计位移 \(\sum_{k} \|p_k - p_{k-1}\|_2\) 在每次记录后清零，\(t_{\text{last}}\) 同步更新为当前帧时间戳。

**首帧消息**无条件记录，作为航点序列的起点。

---

## 目录结构

```
indooruav_waypoint_manager/
├── CMakeLists.txt
├── package.xml
├── README.md
├── .gitignore
├── config/
│   └── config.yaml                  # 可配置参数
├── data/
│   └── waypoints.json               # 运行后生成的航点文件
├── doc/
│   └── doc.drawio                   # 架构示意图
├── include/
│   └── indooruav_waypoint_manager/
│       ├── waypoint.hpp             # Waypoint 数据结构
│       ├── recorder.hpp             # Recorder 类声明
│       └── waypoint_manager.hpp     # WaypointManager 类声明
├── launch/
│   └── bringup_indooruav_waypoint_manager.launch
└── src/
    ├── main.cpp                     # 节点入口
    ├── waypoint_manager.cpp         # 触发逻辑实现
    └── recorder.cpp                 # JSON 序列化实现
```

---

## 依赖环境

| 依赖 | 版本要求 |
|------|----------|
| ROS | Noetic（或 Melodic） |
| roscpp | 随 ROS 安装 |
| nav_msgs | 随 ROS 安装 |
| C++ 标准 | C++14 |

---

## 编译

将功能包放置于 catkin 工作空间的 `src/` 目录下，然后执行：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 配置参数

所有参数集中在 `config/config.yaml`，也可在 launch 文件或命令行中通过私有命名空间 `~` 覆盖。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `delta_L_m` | `double` | `1.0` | 触发记录的最小累计位移阈值（米），须 \(> 0\) |
| `delta_T_s` | `double` | `5.0` | 触发记录的最大时间间隔阈值（秒），须 \(> 0\) |
| `odom_topic` | `string` | `"/Odometry_global"` | 订阅的里程计话题名称 |
| `output_path` | `string` | `"/tmp/waypoints.json"` | JSON 输出文件的完整路径（含文件名） |

> **提示：** 若 `delta_L_m` 或 `delta_T_s` 传入非正值，节点会打印警告并自动重置为默认值。

---

## 启动

### 方式一：使用 launch 文件（推荐）

```bash
roslaunch indooruav_waypoint_manager bringup_indooruav_waypoint_manager.launch
```

### 方式二：直接运行节点并覆盖参数

```bash
rosrun indooruav_waypoint_manager indooruav_waypoint_manager_node \
    _delta_L_m:=0.5 \
    _delta_T_s:=3.0 \
    _odom_topic:=/Odometry_global \
    _output_path:=/home/user/data/waypoints.json
```

### 停止节点

按 **Ctrl+C** 即可触发 `SIGINT`，节点会在退出前自动将已采集的全部航点写入文件。

---

## 输出格式

航点以 JSON 数组形式存储，每条记录包含位置、姿态和时间戳：

```json
{
  "waypoints": [
    {
      "x_m":       1.234567,
      "y_m":       0.987654,
      "z_m":       0.500000,
      "q_x":       0.000000,
      "q_y":       0.000000,
      "q_z":       0.707107,
      "q_w":       0.707107,
      "time":      "2025-03-11T08:30:00.123"
    },
    ...
  ]
}
```

| 字段 | 单位 | 说明 |
|------|------|------|
| `x_m` / `y_m` / `z_m` | 米 | 机器人在全局坐标系下的位置 |
| `q_x` / `q_y` / `q_z` / `q_w` | 无量纲 | 姿态四元数（单位四元数） |
| `time` | — | ISO-8601 格式时间戳，精度到毫秒（UTC） |

---

## 模块说明

### `WaypointManager`

核心控制器，负责订阅里程计话题并执行触发判断。

- **构造函数**：接收参数并向 ROS 订阅 `odom_topic`。
- **`odomCallback`**：每帧更新累计位移与经过时间，满足条件则调用 `Recorder::record()`。
- **`makeWaypoint`**：从里程计消息中提取位姿与时间戳，封装为 `Waypoint` 结构体。
- **`save()`**：代理调用 `Recorder::save()`，供外部（`SIGINT` 钩子）显式触发。

### `Recorder`

专职 I/O 组件，负责航点的内存缓存与 JSON 序列化落盘。

- **`record(wp)`**：将 `Waypoint` 追加至内部 `std::vector`，并打印日志。
- **`save()`**：将缓存中的全部航点以格式化 JSON 写入 `output_path_`。
- **析构函数**：若缓存非空则自动调用 `save()`，防止异常退出时数据丢失。

### `Waypoint`

轻量级数据结构，聚合单条航点的所有字段：位置、四元数姿态、ISO-8601 时间戳。

---

## 注意事项

1. **输出目录须提前存在**：节点不会自动创建父目录，请确保 `output_path` 所在目录已存在，否则文件写入会失败。
2. **时间戳基准为 UTC**：`formatTime` 使用 `std::gmtime`，输出时间为 UTC 而非本地时间。
3. **话题类型**：`odom_topic` 须发布 `nav_msgs/Odometry` 类型消息，否则订阅无效。
4. **多次 `save()` 调用**：`Recorder::save()` 是幂等的（每次调用都会覆盖写入整个文件），`SIGINT` 与析构的双重保护不会造成数据损坏，但会写入两次磁盘。
