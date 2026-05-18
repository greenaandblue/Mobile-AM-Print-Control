#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motion_based_control.py  (fixed for gcode_sender_node v4)

Dynamic print control node: automatically pause/resume the 3D printer
based on robot motion speed.

  - Speed >  threshold + hysteresis  →  AUTO PAUSE
  - Speed ≤  threshold - hysteresis  →  AUTO RESUME

Backend printer: Printrun (via ROS services provided by gcode_sender_node.py v4)
OctoPrint is NOT used.

Changes vs previous version
────────────────────────────
① STOPPED state handling added:
    gcode_sender_node v4 introduced PrinterState.STOPPED.
    When /gcode_sender/state publishes "STOPPED", MotionBasedControl
    now maps it to INACTIVE (same as IDLE) so the state machine never
    enters an undefined branch.

② Removed /gcode_sender/pause_info dependency:
    That service was commented out in gcode_sender_node v4.
    All references are removed; pause is called directly.

③ Increased default debounce (0.5 s → 1.0 s):
    gcode_sender v4 sends M400 after every line, so the actual pause
    latency is longer. Raise debounce to avoid false resume triggers
    while the printer is still finishing the in-flight M400.
    Override with --debounce or _debounce_sec ROS param if needed.

Dependencies:
    gcode_sender_node.py v4 must already be running.

Run:
    python3 motion_based_control.py [options]

    --speed-source   cmd_vel | odom          (default: cmd_vel)
    --cmd-vel-topic  /kmriiwa/base/command/cmd_vel
    --odom-topic     /kmriiwa/base/state/odom
    --threshold      0.04                    (m/s)
    --debounce       1.0                     (s)   ← raised from 0.5
    --hysteresis     0.005                   (m/s)
    --log-dir        ./logs
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

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────────
# Enums
# ─────────────────────────────────────────────────────────────────────────────

class ControlState(Enum):
    INACTIVE = "INACTIVE"   # Printer idle / stopped — motion control dormant
    PRINTING = "PRINTING"   # Printing normally (speed within threshold)
    PAUSED   = "PAUSED"     # Auto-paused because speed exceeded threshold


# gcode_sender_node v4 publishes these exact strings via PrinterState.name
# Keep in sync with PrinterState in gcode_sender_node.py
_SENDER_PRINTING = "PRINTING"
_SENDER_PAUSED   = "PAUSED"
# FIX ①: STOPPED is now a recognised state; treat it like IDLE
_SENDER_INACTIVE = {"IDLE", "STOPPED"}


# ─────────────────────────────────────────────────────────────────────────────
# RobotSpeedMonitor
# ─────────────────────────────────────────────────────────────────────────────

class RobotSpeedMonitor:
    """
    Subscribes to robot speed topic and provides thread-safe get_speed().

    Supports two sources:
      - cmd_vel  (geometry_msgs/Twist)
      - odom     (nav_msgs/Odometry)
    """

    TIMEOUT_SEC = 3.0

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

    def _twist_cb(self, msg: Twist):
        # Only XY plane — vz is not meaningful for a mobile base
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

    def get_speed(self) -> float:
        with self._lock:
            return self._current_speed

    def get_msg_count(self) -> int:
        with self._lock:
            return self._msg_count

    def is_alive(self) -> bool:
        age = time.time() - self._last_update
        if age >= self.TIMEOUT_SEC:
            logger.warning(f"[SPEED] ⚠ No speed update for {age:.1f}s")
            return False
        return True


# ─────────────────────────────────────────────────────────────────────────────
# CSVLogger
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

    # Default parameters (overridable via CLI args or ROS params)
    DEFAULT_THRESHOLD   = 0.04    # m/s
    DEFAULT_HYSTERESIS  = 0.005   # m/s
    DEFAULT_DEBOUNCE    = 1.0     # s  — raised for v4 M400 latency (was 0.5)
    DEFAULT_LOOP_HZ     = 20      # control loop rate

    def __init__(self, args=None):
        if not ROS_AVAILABLE:
            raise RuntimeError("rospy is not available. Cannot start node.")

        rospy.init_node('motion_based_control', anonymous=False)

        # ── Parameters ───────────────────────────────────────
        self.speed_threshold = rospy.get_param(
            '~speed_threshold', args.threshold if args else self.DEFAULT_THRESHOLD)
        self.hysteresis = rospy.get_param(
            '~hysteresis', args.hysteresis if args else self.DEFAULT_HYSTERESIS)
        self.debounce_sec = rospy.get_param(
            '~debounce_sec', args.debounce if args else self.DEFAULT_DEBOUNCE)

        speed_source   = rospy.get_param('~speed_source',
                                         args.speed_source if args else 'cmd_vel')
        cmd_vel_topic  = rospy.get_param('~cmd_vel_topic',
                                         args.cmd_vel_topic if args else
                                         '/kmriiwa/base/command/cmd_vel')
        odom_topic     = rospy.get_param('~odom_topic',
                                         args.odom_topic if args else
                                         '/kmriiwa/base/state/odom')
        log_dir        = rospy.get_param('~log_dir',
                                         args.log_dir if args else './logs')

        logger.info(
            f"[INIT] threshold={self.speed_threshold} m/s  "
            f"hysteresis={self.hysteresis} m/s  "
            f"debounce={self.debounce_sec}s  "
            f"source={speed_source}"
        )

        # ── State ────────────────────────────────────────────
        self._lock          = Lock()
        self.control_state  = ControlState.INACTIVE
        self.printer_state  = "IDLE"          # last value from /gcode_sender/state
        self._above_since   = None            # time when speed first exceeded threshold
        self._below_since   = None            # time when speed first dropped below threshold

        # ── Sub-components ───────────────────────────────────
        self.speed_monitor = RobotSpeedMonitor(speed_source, cmd_vel_topic, odom_topic)
        self.csv_logger    = CSVLogger(log_dir)

        # ── ROS subscribers ──────────────────────────────────
        rospy.Subscriber('/gcode_sender/state', String, self._printer_state_cb)

        # ── ROS publishers ───────────────────────────────────
        self.pub_control_state = rospy.Publisher(
            '/motion_control/state', String, queue_size=10)
        self.pub_speed = rospy.Publisher(
            '/motion_control/robot_speed', Float32, queue_size=10)

        # ── Service proxies (FIX ②: pause_info removed) ─────
        rospy.wait_for_service('/gcode_sender/pause',  timeout=10)
        rospy.wait_for_service('/gcode_sender/resume', timeout=10)
        self._svc_pause  = rospy.ServiceProxy('/gcode_sender/pause',  Trigger)
        self._svc_resume = rospy.ServiceProxy('/gcode_sender/resume', Trigger)

        logger.info("[INIT] Connected to /gcode_sender/pause and /gcode_sender/resume")

    # ── Printer state callback ────────────────────────────────────────────────

    def _printer_state_cb(self, msg: String):
        """
        Receive printer state from gcode_sender_node.

        FIX ①: gcode_sender v4 added STOPPED state.
        Map: PRINTING → PRINTING, PAUSED → PAUSED, IDLE/STOPPED → INACTIVE
        """
        raw = msg.data.strip().upper()
        with self._lock:
            self.printer_state = raw

            if raw == _SENDER_PRINTING:
                if self.control_state == ControlState.INACTIVE:
                    self.control_state = ControlState.PRINTING
                    logger.info("[STATE] Printer started printing → PRINTING")

            elif raw in _SENDER_INACTIVE:
                # FIX ①: STOPPED is treated the same as IDLE
                if self.control_state != ControlState.INACTIVE:
                    logger.info(f"[STATE] Printer state={raw} → INACTIVE")
                    self.control_state = ControlState.INACTIVE
                    self._above_since  = None
                    self._below_since  = None

    # ── Pause / Resume helpers ────────────────────────────────────────────────

    def _call_pause(self, speed: float):
        try:
            t0  = time.time()
            res = self._svc_pause()
            dt  = (time.time() - t0) * 1000
            if res.success:
                logger.info(f"[PAUSE] ✓ speed={speed:.4f} m/s  latency={dt:.0f}ms")
                self.csv_logger.write(speed, "PAUSE", self.printer_state, dt)
            else:
                logger.warning(f"[PAUSE] ✗ {res.message}")
        except Exception as e:
            logger.error(f"[PAUSE] Service call failed: {e}")

    def _call_resume(self, speed: float):
        try:
            t0  = time.time()
            res = self._svc_resume()
            dt  = (time.time() - t0) * 1000
            if res.success:
                logger.info(f"[RESUME] ✓ speed={speed:.4f} m/s  latency={dt:.0f}ms")
                self.csv_logger.write(speed, "RESUME", self.printer_state, dt)
            else:
                logger.warning(f"[RESUME] ✗ {res.message}")
        except Exception as e:
            logger.error(f"[RESUME] Service call failed: {e}")

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_tick(self):
        """Called at DEFAULT_LOOP_HZ. Implements hysteresis + debounce logic."""

        speed = self.speed_monitor.get_speed()
        self.pub_speed.publish(float(speed))

        with self._lock:
            state = self.control_state

        self.pub_control_state.publish(state.value)

        now = time.time()

        if state == ControlState.INACTIVE:
            # Nothing to do — printer_state_cb will wake us when printing starts
            return

        if state == ControlState.PRINTING:
            # Check if speed exceeded upper threshold
            if speed > self.speed_threshold + self.hysteresis:
                if self._above_since is None:
                    self._above_since = now
                    logger.debug(f"[CTRL] Speed {speed:.4f} above threshold, debounce started")
                elif now - self._above_since >= self.debounce_sec:
                    # Debounce passed → pause
                    with self._lock:
                        self.control_state = ControlState.PAUSED
                    self._above_since = None
                    self._below_since = None
                    self._call_pause(speed)
            else:
                self._above_since = None   # speed dropped back, reset debounce

        elif state == ControlState.PAUSED:
            # Check if speed dropped below lower threshold
            if speed <= self.speed_threshold - self.hysteresis:
                if self._below_since is None:
                    self._below_since = now
                    logger.debug(f"[CTRL] Speed {speed:.4f} below threshold, debounce started")
                elif now - self._below_since >= self.debounce_sec:
                    # Debounce passed → resume
                    with self._lock:
                        self.control_state = ControlState.PRINTING
                    self._above_since = None
                    self._below_since = None
                    self._call_resume(speed)
            else:
                self._below_since = None   # speed still high, reset debounce

    # ── Main loop ────────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(self.DEFAULT_LOOP_HZ)
        logger.info("[RUN] Motion-based control loop started")
        while not rospy.is_shutdown():
            try:
                self._control_tick()
            except Exception as e:
                logger.error(f"[RUN] Unexpected error in control tick: {e}")
            rate.sleep()


# ─────────────────────────────────────────────────────────────────────────────
# CLI entry point
# ─────────────────────────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(description='Motion-based print control node')
    p.add_argument('--speed-source',  default='cmd_vel',
                   choices=['cmd_vel', 'odom'])
    p.add_argument('--cmd-vel-topic', default='/kmriiwa/base/command/cmd_vel')
    p.add_argument('--odom-topic',    default='/kmriiwa/base/state/odom')
    p.add_argument('--threshold',     type=float, default=0.04)
    p.add_argument('--debounce',      type=float, default=1.0)
    p.add_argument('--hysteresis',    type=float, default=0.005)
    p.add_argument('--log-dir',       default='./logs')
    # Strip ROS remapping args (__name:=, __log:=, etc.) before argparse sees them
    filtered = [a for a in sys.argv[1:] if not a.startswith('__')]
    return p.parse_args(filtered)


def main():
    try:
        args = _parse_args()
        node = MotionBasedControl(args)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        logger.error(f"Fatal: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()