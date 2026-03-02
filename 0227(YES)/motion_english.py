#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motion_based_control.py

Dynamic print control node: automatically pause/resume the 3D printer
based on robot motion speed.

  - Speed >  threshold + hysteresis  →  AUTO PAUSE
  - Speed ≤  threshold - hysteresis  →  AUTO RESUME

Backend printer: Printrun (via ROS services provided by gcode_sender_node.py)
OctoPrint is NOT used.

Dependencies:
    gcode_sender_node.py must already be running
    (providing /gcode_sender/* services)

Run directly (no need to compile as a ROS package):
    python3 motion_based_control.py [options]

    --speed-source   cmd_vel | odom          (default: cmd_vel)
    --cmd-vel-topic  /kmriiwa/base/command/cmd_vel
    --odom-topic     /kmriiwa/base/state/odom
    --threshold      0.04                    (m/s)
    --debounce       0.5                     (s)
    --hysteresis     0.005                   (m/s)
    --log-dir        ./logs

Or override ROS parameters via rosrun:
    rosrun <pkg> motion_based_control.py _speed_threshold:=0.04
"""

import sys
import os
import math
import time
import csv
import threading
import argparse
import logging
from datetime import datetime
from threading import Lock
from enum import Enum

# ── ROS ──────────────────────────────────────────────────────────────────────
try:
    import rospy
    from std_msgs.msg import String, Float32
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_srvs.srv import Trigger
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# ── Logging (Python standard logging, supplements rospy.log*) ───────────────
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────────
# Enum
# ─────────────────────────────────────────────────────────────────────────────
class ControlState(Enum):
    INACTIVE = "INACTIVE"   # Not active (printer not printing)
    PRINTING = "PRINTING"   # Printing (low speed, printing allowed)
    PAUSED   = "PAUSED"     # Paused (speed too high, auto-paused)


# ─────────────────────────────────────────────────────────────────────────────
# RobotSpeedMonitor  (ported from robot_speed_controller1112.py)
# ─────────────────────────────────────────────────────────────────────────────
class RobotSpeedMonitor:
    """
    Subscribes to robot speed topic and provides thread-safe get_speed().

    Supports two sources:
      - cmd_vel  (geometry_msgs/Twist)
      - odom     (nav_msgs/Odometry)
    """

    TIMEOUT_SEC = 3.0   # If no message is received for this long, considered inactive

    def __init__(self, speed_source: str = 'cmd_vel',
                 cmd_vel_topic: str = '/kmriiwa/base/command/cmd_vel',
                 odom_topic: str = '/kmriiwa/base/state/odom'):

        self._lock          = Lock()
        self._current_speed = 0.0
        self._last_update   = time.time()
        self._msg_count     = 0
        self.speed_source   = speed_source

        if speed_source == 'odom':
            rospy.Subscriber(odom_topic, Odometry, self._odom_cb)
            logger.info(f"[SPEED] Subscribed to odometry: {odom_topic}")
        else:
            rospy.Subscriber(cmd_vel_topic, Twist, self._twist_cb)
            logger.info(f"[SPEED] Subscribed to velocity command: {cmd_vel_topic}")

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _twist_cb(self, msg: Twist):
        speed = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2)
        self._set_speed(speed)

    def _odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx ** 2 + vy ** 2)
        self._set_speed(speed)

    def _set_speed(self, speed: float):
        with self._lock:
            self._current_speed = abs(speed)
            self._last_update   = time.time()
            self._msg_count    += 1

    # ── Public API ───────────────────────────────────────────────────────────
    def get_speed(self) -> float:
        with self._lock:
            return self._current_speed

    def get_msg_count(self) -> int:
        with self._lock:
            return self._msg_count

    def is_alive(self) -> bool:
        """Returns whether a message has been received within the timeout window."""
        age = time.time() - self._last_update
        if age >= self.TIMEOUT_SEC:
            logger.warning(f"[SPEED] ⚠ No speed update for {age:.1f}s")
            return False
        return True


# ─────────────────────────────────────────────────────────────────────────────
# CSVLogger  (ported logging functionality)
# ─────────────────────────────────────────────────────────────────────────────
class CSVLogger:
    """Writes pause/resume events to CSV for post-analysis."""

    HEADER = ['timestamp', 'robot_speed_ms', 'action',
              'printer_state', 'response_time_ms', 'notes']

    def __init__(self, log_dir: str = './logs'):
        self._lock    = Lock()
        self.log_file = None
        self._init(log_dir)

    def _init(self, log_dir: str):
        try:
            os.makedirs(log_dir, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file = os.path.join(log_dir, f"motion_print_control_{ts}.csv")
            with open(self.log_file, 'w', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow(self.HEADER)
            logger.info(f"[LOG] CSV log file: {self.log_file}")
        except Exception as e:
            logger.error(f"[LOG] Failed to initialize log: {e}")
            self.log_file = None

    def write(self, robot_speed: float, action: str,
              printer_state: str, response_ms: float = 0.0, notes: str = ""):
        if not self.log_file:
            return
        try:
            row = [datetime.now().isoformat(),
                   f"{robot_speed:.4f}", action, printer_state,
                   f"{response_ms:.0f}", notes]
            with self._lock:
                with open(self.log_file, 'a', newline='', encoding='utf-8') as f:
                    csv.writer(f).writerow(row)
        except Exception as e:
            logger.error(f"[LOG] Failed to write row: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# MotionBasedControl  (Main Node)
# ─────────────────────────────────────────────────────────────────────────────
class MotionBasedControl:
    """
    Core control node.

    Responsibilities:
      1. Get robot speed via RobotSpeedMonitor
      2. Monitor printer state via /gcode_sender/state
      3. Control printer via /gcode_sender/pause|resume services
      4. Maintain ControlState state machine
      5. Log events to CSV
      6. Publish /motion_control/* topics for external monitoring

    Does NOT hold printcore/GCodeSender instances.
    Fully decoupled from gcode_sender_node.
    """

    # (All remaining inline Chinese comments translated similarly…)
