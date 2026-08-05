## 概述

`wander_avoid_wide` 是一个基于 ROS 的 **反射式避障漫游** 节点，面向开鸿 4.1 底盘。它使用 RPLIDAR 激光雷达实时检测周围障碍物，通过一个有限状态机驱动机器人在未知环境中自由漫游，遇到障碍物时执行大幅度的避障转弯。

与普通版 `wander_avoid.py` 最大的区别在于：**本节点使用里程计偏航角（yaw）来约束转弯幅度**，保证机器人至少转过一个可配置的最小角度（默认 55°）后才允许恢复前进，从而避免在同一个障碍物周围来回振荡，产生更果断、更宽的避障弧线。

## 文件结构

| 文件 | 说明 |
|------|------|
| `wander_avoid_wide.py` | 核心 ROS 节点，由 Python 3 运行 |
| `wander_avoid_wide` | Shell 启动脚本（自动 source 环境后执行节点） |

## 依赖

| 组件 | 说明 |
|------|------|
| RPLIDAR 驱动节点 | 发布 `/scan` 话题（`sensor_msgs/LaserScan`） |
| 底盘控制器栈 | 接收 `/cmd_vel`（`geometry_msgs/Twist`），发布 `/odom`（`nav_msgs/Odometry`） |
| roscore | 主节点管理器 |

## ROS 接口

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 订阅 | `/scan` | `sensor_msgs/LaserScan` | 激光雷达点云，用于障碍物检测 |
| 订阅 | `/odom` | `nav_msgs/Odometry` | 里程计，用于测量转弯角度 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 速度指令，驱动机器人运动 |

## 工作原理

### 扇形区域划分

节点将激光雷达的 360° 视野划分为 6 个检测扇区：

| 扇区名称 | 角度范围 | 用途 |
|----------|----------|------|
| `front` | -30° ~ 30° | 正前方检测，触发避障 |
| `front_left` | 30° ~ 90° | 左前方，评估转弯方向 |
| `front_right` | -90° ~ -30° | 右前方，评估转弯方向 |
| `left` | 90° ~ 150° | 左侧，后退脱困后选方向 |
| `right` | -150° ~ -90° | 右侧，后退脱困后选方向 |
| `back` | +/-165° | 后方综合 |

每个扇区取该范围内所有激光点的 **最小距离** 作为该扇区的障碍物距离，忽略无效值（inf、NaN、超出量程）。

### 有限状态机

```
         +---------+
         | FORWARD |
         +----+----+
              | (front < clearance)
              v
     +--------------------+
     | 哪一侧空间更大？    |
     +--+---------+-------+
        | 左边宽   | 右边宽        均太窄
        v         v                v
  +----------+ +-----------+  +--------+
  |TURN_LEFT | |TURN_RIGHT |  |BACK_UP |
  +----+-----+ +-----+-----+  +---+----+
       |               |           | 后退完成
       |  转弯角度 >=              |
       |  turn_angle               v
       |  且前方清空        +------------+
       +------------------->| 选择转弯方向 |
                            +------------+
                                   |
                         +---------+---------+
                         v                   v
                   +----------+       +-----------+
                   |TURN_LEFT |       |TURN_RIGHT |
                   +----------+       +-----------+
```

**FORWARD（前进）**：以 `linear_speed` 的速度直线前进。持续检测前方扇区距离，一旦低于 `clearance` 阈值即进入避障决策。

**TURN_LEFT / TURN_RIGHT（左/右转）**：以 `angular_speed` 原地旋转。核心约束是 **必须转过至少 `turn_angle_deg`（默认 55°）** 且前方无障碍后才恢复 FORWARD。里程计偏航角用于精确测量已转过的角度，不受打滑影响。同时设有 `turn_timeout`（默认 4 秒）作为安全兜底。

**BACK_UP（后退）**：当前方阻塞且左右空间均不足时启动，以 `backup_speed` 倒车。持续 `backup_duration` 秒后，比较左右侧扇区空间，选择更开阔的一侧转弯脱困。

### 安全机制

- **数据超时检测**：若 `/scan` 或 `/odom` 数据超过 `stale_timeout`（默认 1 秒）未更新，立即停车
- **速度硬限幅**：线性速度不超过 0.18 m/s，角速度不超过 0.70 rad/s，倒车速度不低于 -0.18 m/s
- **转弯角度范围**：`turn_angle_deg` 被钳制在 15° ~ 170°
- **信号处理**：接收 SIGINT / SIGTERM 后优雅停车

## 可调参数

所有参数均支持 ROS 参数服务器动态配置，带 `~` 前缀（节点私有命名空间）。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `linear_speed` | float | 0.10 | 前进线速度（m/s），上限 0.18 |
| `angular_speed` | float | 0.55 | 转弯角速度（rad/s），上限 0.70 |
| `backup_speed` | float | -0.06 | 后退线速度（m/s），下限 -0.18 |
| `clearance` | float | 0.45 | 前方避障触发距离（m） |
| `side_clearance` | float | 0.30 | 侧方空间最小要求（m） |
| `backup_clearance` | float | 0.25 | 后退触发距离（m） |
| `turn_angle_deg` | float | 55.0 | 最小避障转弯角度（度），范围 15 ~ 170 |
| `sector_half_width` | float | 15.0 | 扇区采样的半角宽度（度） |
| `rate` | float | 10.0 | 控制循环频率（Hz） |
| `stale_timeout` | float | 1.0 | 传感器数据超时阈值（秒） |
| `turn_timeout` | float | 4.0 | 转弯安全超时（秒） |
| `backup_duration` | float | 1.0 | 后退持续时间（秒） |

## 使用方式

### 方法一：使用 Shell 启动脚本（推荐）

```bash
cd /data/robot-host

# 默认参数运行
./player/wander_avoid_wide

# 自定义参数
./player/wander_avoid_wide --turn-angle-deg 70 --angular-speed 0.60
```

启动脚本会自动加载 `robot-env.sh` 环境，无需手动 source。

### 方法二：直接运行 Python 节点

```bash
cd /data/robot-host && . ./robot-env.sh
./bin/python3 player/wander_avoid_wide.py
```

### 通过 ROS 参数服务器传参

```bash
cd /data/robot-host && . ./robot-env.sh
./bin/python3 player/wander_avoid_wide.py \
    _linear_speed:=0.12 \
    _angular_speed:=0.60 \
    _clearance:=0.50 \
    _turn_angle_deg:=70
```

## 适用场景

- 开鸿 4.1 底盘在室内平面环境中的自主漫游
- 需要比默认行为更大角度避障转弯的场景（如家具密集的房间）
- 作为更高层导航算法的底层安全反射层

## 与 wander_avoid.py 的比较

| 特性 | wander_avoid.py | wander_avoid_wide.py |
|------|-----------------|----------------------|
| 转弯策略 | 前方清空即恢复前进 | 至少转过最小角度 + 前方清空才能恢复 |
| 转弯角度测量 | 无 | 基于里程计偏航角 |
| 抗振荡能力 | 一般 | 更强（大角度转弯打断振荡循环） |
| 转弯幅度 | 较小 | 更宽、更果断 |
