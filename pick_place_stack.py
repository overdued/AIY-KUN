#!/usr/bin/env python3
"""叠放任务:抓取绿/红方块,叠放到粉色上,最后依次放回原位。

流程:
  1. 抓绿色 -> 放到粉色上 (放置高度 = 抓取高度 - 放置补偿)
  2. 抓红色 -> 叠到绿色上
  3. 抓红色 -> 放回红色原位
  4. 抓绿色 -> 放回绿色原位

依赖:
  - icspring 相机节点在运行(容器内),发布 /arm_camera/image_raw
  - 机械臂栈 host_arm=READY
  - 电机/底盘栈运行

运行(板端):
  /bin/run python3 /data/robot-host/student/mission/pick_place_stack.py
"""

from __future__ import annotations

import json
import math
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur


# ---------------------------------------------------------------------------
# 标定参数(2026-08-05 实测)
# ---------------------------------------------------------------------------
# 抓取/放置姿态(ID2, ID3, ID4)
GRASP_ID2, GRASP_ID3, GRASP_ID4 = 140, 325, 365   # 抓取: 夹爪到地面方块
PLACE_ID2, PLACE_ID3, PLACE_ID4 = 180, 300, 330  # 放置: 比抓取高约5-7cm(现场实测校准)
# 蓝色叠到绿色上: 比 PLACE 再高约5cm(原8cm实测偏高3cm, 下调3cm≈ID2-25)
STACK_ID2, STACK_ID3, STACK_ID4 = PLACE_ID2 + 70, PLACE_ID3 - 25, PLACE_ID4 - 25
HOME_ID2, HOME_ID3, HOME_ID4 = 720, 100, 120     # 采姿/回收

GRIPPER_OPEN = 200     # 全张
GRIPPER_CLOSE = 620    # 全闭(夹住方块)

# 颜色 HSV(在机械臂相机下实测)
# 2026-08-05 三色布局: 蓝、绿、粉红(底座)
# 粉红底座用低饱和宽范围(大块), 蓝绿分别检测
COLORS = {
    "green": ((35, 40, 40), (90, 255, 255)),
    "blue":  ((85, 40, 40), (140, 255, 255)),
    "pink":  ((140, 60, 40), (180, 255, 255)),   # 粉红底座: S>60 排除白色背景
}

BLOCK_MIN_AREA = 300
BLOCK_MAX_AREA = 30000
GROUND_Y_MIN, GROUND_Y_MAX = 200, 470   # 地面方块所在的画面 y 范围
CENTER_X = 320

# ID1 对准: 像素偏差 -> 脉冲增量 (约 1.7 px/脉冲)
ID1_PX_PER_PULSE = 1.7


class PickPlaceStack:
    def __init__(self):
        rospy.init_node("pick_place_stack", anonymous=True)
        self.pub = rospy.Publisher(
            "/servo_controllers/port_id_1/multi_id_pos_dur",
            MultiRawIdPosDur, queue_size=1,
        )
        self._last_frame = None
        self._sub = rospy.Subscriber("/arm_camera/image_raw", Image, self._cb, queue_size=2)
        # 记录初始位置
        self.init_pos = {}   # color -> (id1_pulse, block_center)

    # ---- 相机 ------------------------------------------------------------
    def _cb(self, msg: Image) -> None:
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            self._last_frame = img.copy()
        except Exception:
            pass

    def _wait_frame(self, timeout: float = 8.0) -> np.ndarray:
        end = time.time() + timeout
        while time.time() < end:
            if self._last_frame is not None:
                return self._last_frame
            time.sleep(0.1)
        raise RuntimeError("no camera frame")

    def detect_block(self, color: str):
        """返回最大候选块 (cx, cy, w, h) 或 None。"""
        img = self._wait_frame()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lo, hi = COLORS[color]
        m = cv2.inRange(hsv, np.array(lo), np.array(hi))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cands = []
        for c in cnts:
            area = cv2.contourArea(c)
            x, y, w, h = cv2.boundingRect(c)
            if BLOCK_MIN_AREA < area < BLOCK_MAX_AREA and GROUND_Y_MIN <= y + h and y <= GROUND_Y_MAX:
                cands.append((x + w / 2.0, y + h / 2.0, w, h))
        if not cands:
            return None
        # 最大的候选(排除粉色这类大底座,用面积范围控制)
        cands.sort(key=lambda t: t[2] * t[3], reverse=True)
        return cands[0]

    # ---- 机械臂执行 ------------------------------------------------------
    def _send(self, pulses: list, dur: float) -> None:
        msg = MultiRawIdPosDur(id_pos_dur_list=[
            RawIdPosDur(id=i, position=p, duration=dur) for i, p in pulses
        ])
        self.pub.publish(msg)
        time.sleep(dur + 0.3)

    def set_pose(self, id1: int, id2: int, id3: int, id4: int,
                 gripper: int, dur: float = 1.5) -> None:
        self._send([(1, id1), (2, id2), (3, id3), (4, id4), (5, 500), (10, gripper)], dur)

    def go_home(self) -> None:
        self.set_pose(500, HOME_ID2, HOME_ID3, HOME_ID4, GRIPPER_OPEN, 2.0)

    def align_id1(self, color: str, start_id1: int | None = None,
                  max_scan: int = 20, px_per_pulse: float = 1.45) -> int:
        """转 ID1 找方块并精确对准到画面中心。

        逻辑:
          1) 若当前视野能看到方块 -> 直接小步闭环对准(优先, 不扫描)。
          2) 若看不到 -> 双向扫描: 先朝方块最可能方向(-ID1, 偏右方块的
             对侧)匀速小幅转, 扫一段再反向, 找到后停下。
          3) 找到后小步闭环对准中心。
        关键: 扫描方向覆盖两个方向, 不默认只扫一个方向。
        """
        id1 = start_id1 if start_id1 is not None else 500

        # 先探测当前视野
        block = self.detect_block(color)
        if block is None:
            # ---- 扫描: 两个方向都试 ----
            rospy.loginfo(f"align: 扫描找 {color} (从 ID1={id1})")
            found = False
            for scan_dir in (-1, 1):   # 先 -ID1, 再 +ID1
                scanned = 0
                while scanned < max_scan // 2:
                    id1 = max(0, min(1000, id1 + 5 * scan_dir))
                    self.set_pose(id1, HOME_ID2, HOME_ID3, HOME_ID4,
                                  GRIPPER_OPEN, 0.35)
                    time.sleep(0.25)
                    block = self.detect_block(color)
                    scanned += 1
                    if block is not None:
                        found = True
                        break
                if found:
                    break
            if not found:
                rospy.logwarn(f"align: 双向扫描未找到 {color}")
                return id1

        # ---- 精确对准: 小步闭环 ----
        for _ in range(15):
            block = self.detect_block(color)
            if block is None:
                break
            cx, cy, w, h = block
            dx = cx - CENTER_X
            if abs(dx) <= 10:
                break
            # 方块偏右(dx>0)需要减 ID1 让方块左移到中心(实测: ID1减->方块左移)
            delta = -int(round(dx / px_per_pulse))
            delta = max(-15, min(15, delta))
            id1 = max(0, min(1000, id1 + delta))
            self.set_pose(id1, HOME_ID2, HOME_ID3, HOME_ID4, GRIPPER_OPEN, 0.5)
            time.sleep(0.4)
        return id1

    def record_initial(self, color: str) -> None:
        """记录某颜色方块的初始 ID1 位置。"""
        self.go_home()
        time.sleep(1)
        block = self.detect_block(color)
        id1 = 500
        if block is not None:
            id1 = self.align_id1(color)
        self.init_pos[color] = id1
        print(f"[init] {color} at id1={id1}", flush=True)

    def grasp(self, color: str, id1: int,
              grasp_depth: tuple | None = None) -> bool:
        """抓取指定颜色的方块,抬升到采姿。

        grasp_depth: 下爪高度 (id2,id3,id4)。默认地面抓取高度 GRASP;
        叠放/堆叠位置的方块需传对应高度(放上去的高度)。
        """
        g2, g3, g4 = grasp_depth if grasp_depth else (GRASP_ID2, GRASP_ID3, GRASP_ID4)
        # 下爪到抓取高度
        self.set_pose(id1, g2, g3, g4, GRIPPER_OPEN, 2.0)
        time.sleep(0.5)
        # 夹爪闭合
        self.set_pose(id1, g2, g3, g4, GRIPPER_CLOSE, 1.5)
        time.sleep(1.0)
        # 抬升
        self.set_pose(id1, HOME_ID2, HOME_ID3, HOME_ID4, GRIPPER_CLOSE, 2.0)
        return True

    def place_on(self, target_color: str, id1: int, place_depth: tuple) -> bool:
        """在目标方块上方放下。place_depth=(id2,id3,id4) 为放置高度。"""
        pid2, pid3, pid4 = place_depth
        # 下降放置高度
        self.set_pose(id1, pid2, pid3, pid4, GRIPPER_CLOSE, 2.0)
        time.sleep(0.5)
        # 张开夹爪放下
        self.set_pose(id1, pid2, pid3, pid4, GRIPPER_OPEN, 1.5)
        time.sleep(1.0)
        # 抬升回收
        self.go_home()
        return True


def main() -> int:
    app = PickPlaceStack()
    time.sleep(2)  # 等相机帧

    print("=== 叠放任务开始 ===", flush=True)

    # ---- 记录初始位置 ----
    print("记录蓝色/绿色初始位置...", flush=True)
    app.record_initial("blue")
    app.record_initial("green")

    # ---- 1. 抓绿色 -> 放粉红底座上 ----
    print("\n[1] 抓绿色 -> 放粉红底座上", flush=True)
    app.go_home(); time.sleep(1)
    pink = app.detect_block("pink")
    if pink is None:
        print("ERROR: 找不到粉红方块", flush=True)
        return 1
    id1_pink = app.align_id1("pink")
    app.go_home()

    green = app.detect_block("green")
    id1_green = app.align_id1("green") if green else app.init_pos["green"]
    app.grasp("green", id1_green)
    app.place_on("pink", id1_pink, (PLACE_ID2, PLACE_ID3, PLACE_ID4))
    print("[1] 绿色已放到粉红底座上", flush=True)

    # ---- 2. 抓蓝色 -> 叠到绿色上(更高, +8cm) ----
    print("\n[2] 抓蓝色 -> 叠到绿色上", flush=True)
    app.go_home(); time.sleep(1)
    blue = app.detect_block("blue")
    id1_blue = app.align_id1("blue") if blue else app.init_pos["blue"]
    app.grasp("blue", id1_blue)
    # 叠放: 蓝色叠到绿色上, 用 STACK 高度(比 PLACE 再高约8cm)
    app.place_on("green", id1_pink, (STACK_ID2, STACK_ID3, STACK_ID4))
    print("[2] 蓝色已叠到绿色上", flush=True)

    # ---- 3. 抓蓝色(叠放处, 用叠放高度) -> 放回蓝色原位 ----
    print("\n[3] 抓蓝色 -> 放回原位", flush=True)
    app.go_home(); time.sleep(1)
    id1_blue = app.align_id1("blue")  # 动态对准叠放位置的蓝色
    app.grasp("blue", id1_blue, (STACK_ID2, STACK_ID3, STACK_ID4))  # 按叠放高度抓
    id1_blue_init = app.init_pos["blue"]
    app.place_on("blue", id1_blue_init, (GRASP_ID2, GRASP_ID3, GRASP_ID4))
    print("[3] 蓝色已放回原位", flush=True)

    # ---- 4. 抓绿色(叠放处, 用放置高度) -> 放回绿色原位 ----
    print("\n[4] 抓绿色 -> 放回原位", flush=True)
    app.go_home(); time.sleep(1)
    id1_green = app.align_id1("green")  # 动态对准叠放位置的绿色
    app.grasp("green", id1_green, (PLACE_ID2, PLACE_ID3, PLACE_ID4))  # 按放置高度抓
    id1_green_init = app.init_pos["green"]
    app.place_on("green", id1_green_init, (GRASP_ID2, GRASP_ID3, GRASP_ID4))
    print("[4] 绿色已放回原位", flush=True)

    app.go_home()
    print("\n=== 叠放任务完成 ===", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", flush=True)
        raise
