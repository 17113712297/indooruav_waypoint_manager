# indooruav_waypoint_manager

> ROS 航点自动记录器：订阅里程计话题，按**位移（XYZ）**、**时间**、**偏航角**三重触发条件自动采集机器人位姿，并将结果序列化为 JSON 文件。

---

## 功能概述

`indooruav_waypoint_manager` 是一个轻量级 ROS 节点，负责：

- 订阅 `nav_msgs/Odometry` 消息（话题可配置）；
- 根据三种可独立配置的触发条件，自动判断何时记录一个**航点（Waypoint）**；
- 将所有航点（含位置、姿态四元数、时间戳）写入指定路径的 **JSON 文件**；
- 支持通过 `Ctrl+C`（SIGINT）安全退出并自动落盘，析构时也会兜底保存。

---

## 触发条件说明

三种条件满足**任意一种**即立即记录当前位姿，随后重置所有累计量。

### ΔL — XYZ 三维位移触发

当机器人相对上次记录点的 **XYZ 三维欧氏距离**达到阈值时触发：

$$
d_{XYZ} = \sqrt{(x - x_{\text{last}})^2 + (y - y_{\text{last}})^2 + (z - z_{\text{last}})^2} \geq \Delta L
$$



对应参数：`delta_L_m`（单位：米）

---

### ΔT — 时间间隔触发

距上次记录时刻已经过的秒数：

$$
t_{\text{elapsed}} = t_{\text{now}} - t_{\text{last}} \geq \Delta T
$$

可防止机器人静止时长时间无航点输出。

对应参数：`delta_T_s`（单位：秒）

---

### ΔA — 偏航角触发

提取里程计四元数的偏航角（Yaw），计算与上次记录时偏航角的**最短角度差**：

$$
\Delta\psi = \bigl|\operatorname{atan2}(2(q_w q_z + q_x q_y),\ 1 - 2(q_y^2 + q_z^2)) - \psi_{\text{last}}\bigr|_{\bmod 2\pi} \geq \Delta A
$$

角度差归一化到 $[0,\,\pi]$，避免跨越 ±180° 时的误触发。

对应参数：`delta_A_deg`（单位：**度**，内部自动转换为弧度）。设为 `0` 可禁用此触发。

---


## 参数配置

所有参数均在 `config/config.yaml` 中定义，由 launch 文件通过 `<rosparam>` 加载到节点私有命名空间（`~/`）。

| 参数名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `delta_L_m` | double | `1.0` | XYZ 三维位移触发阈值（米）。必须 > 0。 |
| `delta_T_s` | double | `5.0` | 时间触发阈值（秒）。必须 > 0。 |
| `delta_A_deg` | double | `30.0` | 偏航角触发阈值（度）。设为 `0` 禁用。 |
| `odom_topic` | string | `"/Odometry_global"` | 订阅的里程计话题名称。 |
| `output_path` | string | `"/tmp/waypoints.json"` | 输出 JSON 文件的完整路径。 |

**配置示例**（`config/config.yaml`）：

```yaml
delta_L_m:   1.0
delta_T_s:   5.0
delta_A_deg: 30.0
odom_topic:  "/Odometry_global"
output_path: "/home/user/data/waypoints.json"
```

---

## 输出格式

记录完成后生成标准 JSON 文件，每个航点包含位置、姿态四元数与 UTC 时间戳（ISO-8601，精度至毫秒）。

```json
{
  "waypoints": [
    {
      "x_m":   1.234567,
      "y_m":   2.345678,
      "z_m":   0.050000,
      "q_x":   0.000000,
      "q_y":   0.000000,
      "q_z":   0.382683,
      "q_w":   0.923880,
      "time":  "2025-03-11T08:30:00.123"
    },
    {
      "x_m":   2.100000,
      ...
    }
  ]
}
```

> 姿态以 `(q_x, q_y, q_z, q_w)` 四元数表示，遵循 ROS 约定（`geometry_msgs/Quaternion`）。

---

## 包结构

```
indooruav_waypoint_manager/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── config.yaml               # 可配置参数
├── launch/
│   └── bringup_indooruav_waypoint_manager.launch
├── include/
│   └── indooruav_waypoint_manager/
│       ├── waypoint.hpp          # Waypoint 数据结构
│       ├── recorder.hpp          # 航点记录器接口
│       └── waypoint_manager.hpp  # 主逻辑管理器接口
└── src/
    ├── main.cpp                  # 节点入口、参数读取、SIGINT 处理
    ├── waypoint_manager.cpp      # 里程计回调、三重触发逻辑
    └── recorder.cpp              # JSON 序列化与文件写入
```

---

## 依赖项

| 依赖 | 版本要求 | 说明 |
|---|---|---|
| ROS (roscpp) | Melodic / Noetic | 核心通信框架 |
| nav_msgs | 随 ROS 发行版 | 提供 `nav_msgs/Odometry` |
| C++14 | — | 标准库特性（`std::make_shared` 等） |

> 原先依赖 `tf` 包用于四元数转换，本版本已改为内联数学公式实现，**无需额外依赖 `tf`**。

---

## 构建与使用

```bash
# 进入 catkin 工作空间
cd ~/catkin_ws

# 编译
catkin_make --only-pkg-with-deps indooruav_waypoint_manager

# source 环境
source devel/setup.bash

# 启动
roslaunch indooruav_waypoint_manager bringup_indooruav_waypoint_manager.launch
```

