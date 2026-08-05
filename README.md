# Kaihong Robot Lidar SLAM Mapping

深开鸿（Kaihong）赛道 AIY 黑客松 —— 基于激光雷达的场景建图（SLAM）模块。
服务于**避障 / 导航**场景：让小车在自己的工作环境中自动建立二维栅格地图，供后续导航与避障使用。

## 功能

- **激光雷达驱动**：启动 RPLIDAR A1（CH341 串口适配器），发布 360° `/scan`
- **SLAM 建图**：gmapping 融合 `/scan` + `/odom` 实时构建 `/map` 二维占据栅格地图
- **键盘遥操作**：`w/a/s/d` 遥控小车在场景内行驶建图（发布到 `/cmd_vel`）
- **地图保存**：`map_server map_saver` 导出 `.pgm` + `.yaml`
- **RGB 视觉检测**（辅助/预留）：HSV 多段阈值检测彩色物体，反投影估计 3D 位置

## 硬件 / 软件

| 项 | 说明 |
| --- | --- |
| 小车 | M-Robots Board-3588S（OpenHarmony 4.1, M-Robots OS 4.1） |
| 激光雷达 | RPLIDAR A1（USB 串口 CH341, `1a86:7523`, `/dev/ttyUSB0`） |
| 底盘 | STM32 底盘控制器 `/dev/ttyCH343USB0`，接收 `/cmd_vel` |
| ROS | ROS1 noetic，master 在板端 `http://<wlan0-ip>:11311` |
| SLAM | `gmapping` + `map_server`（已在板端 `/data/local/release` 安装） |
| 其他 | 板端 Python 3.12（`/data/robot-host/bin/python3`，无 OpenCV，视觉需在容器内跑） |

## 目录结构

```
.
├── README.md                    本说明
├── navigation/
│   ├── teleop_keyboard.py       键盘遥操作（发布 /cmd_vel）
│   ├── final_map.png            最终建图结果（PNG 预览）
│   └── partial_run_map.png      半程建图（过程参考）
├── vision/
│   ├── detect_objects.py        RGB-D 物体检测节点（容器内运行）
│   └── color_ranges.json        HSV 颜色阈值配置
└── pick_place_stack.py          （master 分支的机械臂抓取堆叠，与本分支无关）
```

## 建图流程

SSH 连接小车（端口 2223，账号 root，密钥登录）：

```sh
ssh root@<小车IP> -p 2223
cd /data/robot-host
```

### 1. 启动底盘栈（roscore + chassis + odom）

```sh
sh /data/robot-host/start-host-chassis.sh
# 期望输出: host chassis started / ROS_MASTER_URI=http://<ip>:11311
```

### 2. 启动激光雷达

```sh
sh /data/robot-host/start-lidar-4.1.sh
# 期望输出: lidar=RUNNING SCAN=READY
sh /data/robot-host/healthcheck-lidar-4.1.sh   # 全 PASS 即正常
```

### 3. 启动 SLAM

```sh
sh /data/robot-host/start-slam-4.1.sh
# 期望输出: slam=RUNNING mode=host
```

SLAM launch（`/data/robot-host/navigation_runtime/launch/slam.launch`）要点：

- 静态变换 `base_to_laser`：`x=0 y=0 z=0.20`（默认值，实际安装高度需标定）
- gmapping：`map_update_interval=2.0`、`particles=30`、`delta=0.05`（5cm/像素）
- 地图范围默认 ±10m，建图会随小车移动自动扩展

### 4. 遥控小车建图

在**电脑**上新开终端（需要 TTY，交互式按键）：

```sh
ssh aiy-car -t "cd /data/robot-host && . ./robot-env.sh && bin/python3 teleop_keyboard.py"
```

| 按键 | 动作 |
| --- | --- |
| `w` | 前进 |
| `s` | 后退 |
| `a` | 左转 |
| `d` | 右转 |
| 其他 | 停止 |
| `Ctrl-C` | 退出（自动停车） |

### 5. 保存地图

```sh
# 注意: 此板端环境 rosrun 找不到 map_saver，需直接调用实际路径
. /data/robot-host/robot-env.sh
/data/local/release/usr/lib/map_server/map_saver -f /data/robot/maps/final_map
# 生成 /data/robot/maps/final_map.pgm + .yaml
```

## 建图质量要点（实测经验）

底盘里程计为**开环**（`cmd_vel_odom_node.py` 积分指令，无编码器反馈），转弯会累积漂移。
为保证地图质量：

- **慢速平稳**行驶（遥控已限速 0.15 m/s）
- **少转弯、缓转弯**：每次转角 ≤45°，转完稍停让 gmapping 激光匹配稳定
- **多往返覆盖**：直线来回多走几趟，让扫描匹配反复校正
- **控制在场景范围内**：雷达量程 12m，会扫到周边房间的墙，不要离场景太远

## 已知问题 / 排查

| 现象 | 原因 / 处理 |
| --- | --- |
| 雷达 `SCAN=NO_DATA`，日志报 `no valid RPLIDAR measurement nodes` | 电机未转动。**重新插拔雷达 USB 线**（硬复位），再 `start-lidar-4.1.sh` |
| 板端 `import cv2` 段错误 | 板端宿主 Python 无 OpenCV，视觉检测需在 `rk3588s-vision` 容器内运行 |
| `rosrun map_server map_saver` 找不到 | 发布版布局，直接调 `/data/local/release/usr/lib/map_server/map_saver` |
| 换热点后 SSH 连不上 | 小车 IP 变化，重新扫描网段 2223 端口并更新 SSH config |
| 地图漂移/噪点 | 开环 odom 所致，见「建图质量要点」；必要时降低 `angularUpdate` |

## 视觉检测（可选，容器内）

`vision/detect_objects.py` 在 `rk3588s-vision` 容器内运行，检测彩色物体（球/正方体/长方体），
输出颜色、形状、相机系 3D 坐标与物理尺寸，发布到 `/student/grasp/objects`。

```sh
# 板端持久目录
scp -P 2223 vision/detect_objects.py vision/color_ranges.json root@<小车IP>:/data/robot-host/vision/
# 拷入容器（容器重启会丢，需重拷）
docker cp /data/robot-host/vision/detect_objects.py rk3588s-vision:/data/vision/
docker cp /data/robot-host/vision/color_ranges.json rk3588s-vision:/data/vision/
# 容器内单帧测试
docker exec rk3588s-vision bash -lc \
  'source /opt/ros/noetic/setup.bash; source /vision_ws/devel/setup.bash; \
   python3 /data/vision/detect_objects.py --config /data/vision/color_ranges.json --once --output-dir /tmp'
```

## License

MIT
