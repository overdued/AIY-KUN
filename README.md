# Kaihong Robot Pick-and-Stack

Kaihong 4.1 小车的机械臂叠块与放置任务。机器人通过机械臂上的 RGB 相机(icspring)识别彩色方块,自动完成**抓取 → 叠放 → 按原位放回**的完整流程。

## 功能

- **颜色识别**:通过 HSV 阈值实时检测蓝、绿、粉红三种颜色的方块
- **动态对准**:机械臂 ID1 匀速转动扫描 + 小步闭环,把方块精确对准到画面中心
- **抓取**:下爪 → 夹爪闭合 → 抬升,支持地面高度和叠放高度两种下爪深度
- **叠放**:按标定高度把方块叠到目标方块上方(蓝叠绿、绿放粉红底座)
- **放回**:按叠放时的高度抓取,放回各自的初始位置

## 任务流程

```
[1] 抓绿色        -> 放到粉红底座上
[2] 抓蓝色        -> 叠到绿色上(更高)
[3] 抓蓝色(叠放处) -> 放回蓝色原位
[4] 抓绿色(叠放处) -> 放回绿色原位
```

## 硬件依赖

- Kaihong 4.1 小车(M-Robots OS, OpenHarmony 4.1)
- 机械臂(舵机 ID1-5 + 夹爪 ID10)
- 机械臂 RGB 相机(icspring, UVC `/dev/video20`)
- ROS 1 noetic,机械臂栈 host 模式

## 安装

把 `pick_place_stack.py` 放到小车板端:

```sh
scp -P 2223 pick_place_stack.py root@<小车IP>:/data/robot-host/student/mission/
```

## 使用

SSH 连接小车后:

```sh
ssh root@<小车IP> -p 2223
cd /data/robot-host

# 1. 确认机械臂栈就绪
./status-host-arm-4.1.sh          # 应显示 host_arm=READY

# 2. 启动机械臂相机节点(如未运行)
docker exec -d rk3588s-vision bash -lc \
  'source /opt/ros/noetic/setup.bash; exec python3 /tmp/icspring_node.py >/tmp/icspring.log 2>&1'

# 3. 摆好方块: 粉红底座、绿色、蓝色 各就各位

# 4. 运行叠块任务
cd /data/robot-host
. ./robot-env.sh
/bin/run python3 student/mission/pick_place_stack.py
```

## 关键标定参数

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `GRASP` | (140, 325, 365) | 地面抓取下爪高度 (ID2,ID3,ID4) |
| `PLACE` | (180, 300, 330) | 放置高度(比抓取高约 5-7cm) |
| `STACK` | (250, 275, 305) | 叠放高度(比 PLACE 再高约 5cm) |
| `GRIPPER_OPEN` | 200 | 夹爪全张 |
| `GRIPPER_CLOSE` | 620 | 夹爪全闭 |

> 这些值因小车机械臂安装而异,首次使用请现场校准。

## 颜色 HSV 阈值

| 颜色 | HSV 范围 |
| --- | --- |
| 绿 | (35,40,40)-(90,255,255) |
| 蓝 | (85,40,40)-(140,255,255) |
| 粉红 | (140,60,40)-(180,255,255) |

> 粉色阈值 S>60 用于排除白色背景干扰。

## 许可证

MIT
