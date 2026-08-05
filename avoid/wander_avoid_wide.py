#!/usr/bin/env python3
"""Lidar-based free wander with WIDE obstacle avoidance turns for the Kaihong 4.1 chassis.

Compared to wander_avoid.py, this variant turns more aggressively when an obstacle
is detected. It uses odometry yaw to ensure the robot rotates by at least a
configurable minimum angle (default 55 degrees) before it is allowed to resume
forward motion. This produces wider, more decisive avoidance arcs and reduces the
chance of getting stuck oscillating around the same obstacle.

Publishes:
    /cmd_vel  (geometry_msgs/Twist)

Subscribes:
    /scan     (sensor_msgs/LaserScan)
    /odom     (nav_msgs/Odometry)   -- used to measure how far we have turned

Requires:
    - RPLIDAR node publishing /scan
    - Host chassis stack (roscore, chassis_controller, cmd_vel_odom)

Run directly:
    cd /data/robot-host && . ./robot-env.sh
    ./bin/python3 player/wander_avoid_wide.py

Or use the wrapper:
    cd /data/robot-host && player/wander_avoid_wide
"""

from __future__ import annotations

import math
import signal
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quaternion(q):
    """Extract yaw ( radians, [-pi, pi] ) from a geometry_msgs/Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_diff(a, b):
    """Return smallest signed difference a - b in radians, wrapped to [-pi, pi]."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class WanderAvoidWide:
    """Reflex obstacle avoider with wide, odometry-gated turns."""

    # Sector angles in degrees, centered on the robot heading (0 = forward)
    SECTORS = {
        "front": (-30.0, 30.0),
        "front_left": (30.0, 90.0),
        "front_right": (-90.0, -30.0),
        "left": (90.0, 150.0),
        "right": (-150.0, -90.0),
        "back": (-180.0, 180.0),  # handled specially
    }

    def __init__(self):
        rospy.init_node("wander_avoid_wide", anonymous=True)

        # Tunable parameters
        self.linear_speed = rospy.get_param("~linear_speed", 0.10)          # m/s forward
        self.angular_speed = rospy.get_param("~angular_speed", 0.55)        # rad/s turn (wide default)
        self.backup_speed = rospy.get_param("~backup_speed", -0.06)         # m/s reverse
        self.clearance = rospy.get_param("~clearance", 0.45)                # m, min front distance
        self.side_clearance = rospy.get_param("~side_clearance", 0.30)      # m, min side distance
        self.backup_clearance = rospy.get_param("~backup_clearance", 0.25)  # m, trigger backup
        self.turn_angle_deg = rospy.get_param("~turn_angle_deg", 55.0)      # minimum avoidance turn
        self.sector_half_width = rospy.get_param("~sector_half_width", 15.0)  # degrees
        self.rate_hz = rospy.get_param("~rate", 10.0)
        self.stale_timeout = rospy.get_param("~stale_timeout", 1.0)         # seconds
        self.turn_timeout = rospy.get_param("~turn_timeout", 4.0)           # seconds safety cap
        self.backup_duration = rospy.get_param("~backup_duration", 1.0)     # seconds
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")

        # Safety clamp against host stack limits
        self.linear_speed = max(0.0, min(0.18, self.linear_speed))
        self.angular_speed = max(0.0, min(0.70, self.angular_speed))
        self.backup_speed = max(-0.18, min(0.0, self.backup_speed))
        self.turn_angle = math.radians(max(15.0, min(170.0, self.turn_angle_deg)))

        self.latest_scan: LaserScan | None = None
        self.scan_time = 0.0
        self.latest_yaw: float | None = None
        self.odom_time = 0.0

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)

        self._stop = False
        signal.signal(signal.SIGINT, self._sigint)
        signal.signal(signal.SIGTERM, self._sigint)

    def _sigint(self, signum, frame):
        rospy.loginfo("wander_avoid_wide: stopping on signal %d", signum)
        self._stop = True

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg
        self.scan_time = time.time()

    def _odom_cb(self, msg: Odometry):
        self.latest_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_time = time.time()

    def _sector_distance(self, scan: LaserScan, center_deg: float) -> float | None:
        """Return the minimum valid distance in a sector centered at center_deg."""
        center = math.radians(center_deg)
        half = math.radians(self.sector_half_width)
        values = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            delta = (angle - center + math.pi) % (2.0 * math.pi) - math.pi
            if abs(delta) <= half:
                values.append(distance)
        if not values:
            return None
        return float(np.min(values))

    def _evaluate_sectors(self) -> dict[str, float | None]:
        scan = self.latest_scan
        if scan is None:
            return {name: None for name in self.SECTORS}
        result = {}
        for name, (left, right) in self.SECTORS.items():
            if name == "back":
                left_val = self._sector_distance(scan, 165.0)
                right_val = self._sector_distance(scan, -165.0)
                if left_val is not None and right_val is not None:
                    result[name] = min(left_val, right_val)
                else:
                    result[name] = left_val if left_val is not None else right_val
            else:
                center = (left + right) / 2.0
                result[name] = self._sector_distance(scan, center)
        return result

    def _make_cmd(self, linear: float, angular: float) -> Twist:
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        return cmd

    def _stop_cmd(self) -> Twist:
        return self._make_cmd(0.0, 0.0)

    def run(self):
        rospy.loginfo(
            "wander_avoid_wide started: linear=%.2f angular=%.2f clearance=%.2f turn_angle=%.0fdeg",
            self.linear_speed,
            self.angular_speed,
            self.clearance,
            math.degrees(self.turn_angle),
        )

        rate = rospy.Rate(self.rate_hz)
        state = "FORWARD"
        state_deadline = 0.0
        turn_start_yaw: float | None = None

        # Wait for first scan and first odometry
        rospy.loginfo("waiting for /scan and /odom...")
        deadline = time.time() + 10.0
        while (
            (self.latest_scan is None or self.latest_yaw is None)
            and time.time() < deadline
            and not self._stop
        ):
            rate.sleep()
        if self.latest_scan is None:
            rospy.logerr("no /scan data received")
            return 1
        if self.latest_yaw is None:
            rospy.logerr("no /odom data received")
            return 1

        rospy.loginfo("sensors ready, starting to wander")

        while not rospy.is_shutdown() and not self._stop:
            now = time.time()

            # Emergency stop if sensor data is stale
            stale = (now - self.scan_time > self.stale_timeout) or (
                self.latest_yaw is not None and now - self.odom_time > self.stale_timeout
            )
            if stale:
                rospy.logwarn_throttle(2.0, "stale sensor data; stopping")
                self.cmd_pub.publish(self._stop_cmd())
                rate.sleep()
                continue

            sectors = self._evaluate_sectors()
            front = sectors.get("front")
            front_left = sectors.get("front_left")
            front_right = sectors.get("front_right")
            left = sectors.get("left")
            right = sectors.get("right")

            if state == "FORWARD":
                if front is not None and front < self.clearance:
                    rospy.loginfo("obstacle ahead at %.2f m; choosing wide turn", front)
                    left_space = front_left if front_left is not None else 0.0
                    right_space = front_right if front_right is not None else 0.0

                    if left_space >= right_space and left_space > self.side_clearance:
                        state = "TURN_LEFT"
                    elif right_space > self.side_clearance:
                        state = "TURN_RIGHT"
                    else:
                        state = "BACK_UP"

                    state_deadline = now + self.turn_timeout
                    turn_start_yaw = self.latest_yaw
                else:
                    self.cmd_pub.publish(self._make_cmd(self.linear_speed, 0.0))

            elif state in ("TURN_LEFT", "TURN_RIGHT"):
                sign = 1.0 if state == "TURN_LEFT" else -1.0
                self.cmd_pub.publish(self._make_cmd(0.0, sign * self.angular_speed))

                # Wide-turn gate: must turn at least turn_angle before we are allowed
                # to resume forward motion, even if front is already clear.
                turned_enough = False
                if turn_start_yaw is not None and self.latest_yaw is not None:
                    turned = abs(angle_diff(self.latest_yaw, turn_start_yaw))
                    turned_enough = turned >= self.turn_angle

                front_clear = front is not None and front >= self.clearance
                timed_out = now >= state_deadline

                if (front_clear and turned_enough) or timed_out:
                    rospy.loginfo(
                        "wide turn complete: turned %.0f/%.0f deg, front=%s",
                        math.degrees(turned) if turn_start_yaw and self.latest_yaw else 0.0,
                        math.degrees(self.turn_angle),
                        "%.2f" % front if front is not None else "inf",
                    )
                    state = "FORWARD"
                    turn_start_yaw = None

            elif state == "BACK_UP":
                self.cmd_pub.publish(self._make_cmd(self.backup_speed, 0.0))
                if now >= state_deadline:
                    left_space = left if left is not None else 0.0
                    right_space = right if right is not None else 0.0
                    state = "TURN_LEFT" if left_space >= right_space else "TURN_RIGHT"
                    state_deadline = now + self.turn_timeout
                    turn_start_yaw = self.latest_yaw

            rospy.loginfo_throttle(
                2.0,
                "state=%s front=%s left=%s right=%s",
                state,
                "%.2f" % front if front is not None else "inf",
                "%.2f" % front_left if front_left is not None else "inf",
                "%.2f" % front_right if front_right is not None else "inf",
            )

            rate.sleep()

        # Ensure the robot stops when node exits
        self.cmd_pub.publish(self._stop_cmd())
        rospy.loginfo("wander_avoid_wide stopped")
        return 0


def main():
    node = WanderAvoidWide()
    return node.run()


if __name__ == "__main__":
    raise SystemExit(main())
