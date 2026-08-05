#!/usr/bin/env python3
# encoding: utf-8
"""Keyboard teleop for SLAM mapping. Publishes to /cmd_vel (w/a/s/d).

Run on the car (foreground, needs a TTY):
    cd /data/robot-host && . ./robot-env.sh && bin/python3 teleop_keyboard.py
"""

import sys
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

HELP = """
Keyboard teleop — 建图遥控
---------------------------
  w    前进
  s    后退
  a    左转
  d    右转
  <其他>  停止
  Ctrl-C 退出（自动停车）
速度: linear %.2f m/s, angular %.2f rad/s
"""


def main():
    rospy.init_node("teleop_keyboard")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    lin = rospy.get_param("~max_linear", 0.15)
    ang = rospy.get_param("~max_angular", 0.35)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        print(HELP % (lin, ang), flush=True)
        while not rospy.is_shutdown():
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ""

            twist = Twist()
            if key == "w":
                twist.linear.x = lin
            elif key == "s":
                twist.linear.x = -lin
            elif key == "a":
                twist.angular.z = ang
            elif key == "d":
                twist.angular.z = -ang
            elif key == "\x03":
                break
            pub.publish(twist)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        pub.publish(Twist())  # ensure zero velocity on exit


if __name__ == "__main__":
    main()
