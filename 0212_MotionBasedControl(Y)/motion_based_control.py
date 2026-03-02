#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motion_based_control.py

动态打印控制节点：根据机器人移动速度自动暂停/恢复 3D 打印机
  - 速度 >  threshold + hysteresis  →  AUTO PAUSE
  - 速度 ≤  threshold - hysteresis  →  AUTO RESUME

后端打印机：Printrun（通过 gcode_sender_node.py 提供的 ROS Service）
不使用 OctoPrint。

依赖：
    gcode_sender_node.py 必须已在运行（提供 /gcode_sender/* 服务）

直接运行（无需编译成 ROS package）：
    python3 motion_based_control.py [选项]

    --speed-source   cmd_vel | odom          (默认 cmd_vel)
    --cmd-vel-topic  /kmriiwa/base/command/cmd_vel
    --odom-topic     /kmriiwa/base/state/odom
    --threshold      0.04                    (m/s)
    --debounce       0.5                     (s)
    --hysteresis     0.005                   (m/s)
    --log-dir        ./logs

或通过 rosrun 覆盖 ROS 参数：
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

# ── Logging（Python 标准，补充 rospy.log*）───────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────────
# 枚举
# ─────────────────────────────────────────────────────────────────────────────
class ControlState(Enum):
    INACTIVE = "INACTIVE"   # 未激活（打印机未在打印）
    PRINTING = "PRINTING"   # 正在打印（速度低，允许打印）
    PAUSED   = "PAUSED"     # 已暂停（速度过快，自动暂停中）


# ─────────────────────────────────────────────────────────────────────────────
# RobotSpeedMonitor  （移植自 robot_speed_controller1112.py）
# ─────────────────────────────────────────────────────────────────────────────
class RobotSpeedMonitor:
    """
    订阅机器人速度 topic，提供线程安全的 get_speed()。
    支持两种来源：
      - cmd_vel  (geometry_msgs/Twist)
      - odom     (nav_msgs/Odometry)
    """

    TIMEOUT_SEC = 3.0   # 超过此时间未收到消息视为失活

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
            logger.info(f"[SPEED] 订阅里程计: {odom_topic}")
        else:
            rospy.Subscriber(cmd_vel_topic, Twist, self._twist_cb)
            logger.info(f"[SPEED] 订阅速度命令: {cmd_vel_topic}")

    # ── 回调 ──────────────────────────────────────────────────────────────────
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

    # ── 公开接口 ──────────────────────────────────────────────────────────────
    def get_speed(self) -> float:
        with self._lock:
            return self._current_speed

    def get_msg_count(self) -> int:
        with self._lock:
            return self._msg_count

    def is_alive(self) -> bool:
        """是否在超时时间内收到过消息"""
        age = time.time() - self._last_update
        if age >= self.TIMEOUT_SEC:
            logger.warning(f"[SPEED] ⚠ 速度话题已 {age:.1f}s 无更新")
            return False
        return True


# ─────────────────────────────────────────────────────────────────────────────
# CSVLogger  （移植自 robot_speed_controller1112.py 的日志功能）
# ─────────────────────────────────────────────────────────────────────────────
class CSVLogger:
    """将暂停/恢复事件写入 CSV 文件，便于事后分析。"""

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
            logger.info(f"[LOG] CSV 日志: {self.log_file}")
        except Exception as e:
            logger.error(f"[LOG] 日志初始化失败: {e}")
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
            logger.error(f"[LOG] 写入失败: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# MotionBasedControl  （主节点）
# ─────────────────────────────────────────────────────────────────────────────
class MotionBasedControl:
    """
    核心控制节点。

    职责：
      1. 通过 RobotSpeedMonitor 获取机器人速度
      2. 通过 /gcode_sender/state topic 感知打印机状态
      3. 通过 /gcode_sender/pause|resume ROS Service 控制打印机
      4. 维护 ControlState 状态机
      5. 将事件写入 CSV 日志
      6. 发布 /motion_control/* topics 供外部监控

    不持有 printcore / GCodeSender 实例，与 gcode_sender_node 完全解耦。
    """

    def __init__(self,
                 speed_threshold: float = 0.04,
                 speed_source:    str   = 'cmd_vel',
                 cmd_vel_topic:   str   = '/kmriiwa/base/command/cmd_vel',
                 odom_topic:      str   = '/kmriiwa/base/state/odom',
                 debounce_time:   float = 0.5,
                 hysteresis:      float = 0.005,
                 speed_window:    int   = 5,
                 log_dir:         str   = './logs'):

        # ── ROS 参数（rosrun 时可覆盖，否则使用传入值）─────────────────────
        if ROS_AVAILABLE and rospy.core.is_initialized():
            speed_threshold = rospy.get_param('~speed_threshold', speed_threshold)
            speed_source    = rospy.get_param('~speed_source',    speed_source)
            cmd_vel_topic   = rospy.get_param('~cmd_vel_topic',   cmd_vel_topic)
            odom_topic      = rospy.get_param('~odom_topic',      odom_topic)
            debounce_time   = rospy.get_param('~debounce_time',   debounce_time)
            hysteresis      = rospy.get_param('~hysteresis',      hysteresis)
            speed_window    = rospy.get_param('~speed_window',    speed_window)
            log_dir         = rospy.get_param('~log_dir',         log_dir)

        # ── 配置保存 ──────────────────────────────────────────────────────────
        self.speed_threshold = speed_threshold
        self.debounce_time   = debounce_time
        self.hysteresis      = hysteresis
        self.speed_window    = speed_window

        # 暂停阈值 = threshold + hysteresis（防止频繁抖动）
        # 恢复阈值 = threshold - hysteresis
        self._pause_thr  = speed_threshold + hysteresis
        self._resume_thr = speed_threshold - hysteresis

        # ── 状态 ──────────────────────────────────────────────────────────────
        self._lock               = Lock()
        self.control_state       = ControlState.INACTIVE
        self.printer_state       = "IDLE"       # 来自 /gcode_sender/state
        self._smoothed_speed     = 0.0
        self._speed_history      = []
        self.control_enabled     = False        # 标志：是否正在监控

        # 防抖计时（移植 robot_speed_controller1112 的 stable_counter 逻辑，改为时间制）
        self._above_since        = None         # 速度首次超阈值的时刻
        self._below_since        = None         # 速度首次低于恢复阈值的时刻

        # 统计
        self.pause_count         = 0
        self.resume_count        = 0
        self._session_start      = time.time()

        # ── 子模块 ────────────────────────────────────────────────────────────
        self._speed_monitor = RobotSpeedMonitor(
            speed_source  = speed_source,
            cmd_vel_topic = cmd_vel_topic,
            odom_topic    = odom_topic
        )
        self._csv = CSVLogger(log_dir)

        # ── ROS Publishers ────────────────────────────────────────────────────
        self._pub_state        = rospy.Publisher('/motion_control/state',          String,  queue_size=10)
        self._pub_raw_speed    = rospy.Publisher('/motion_control/current_speed',  Float32, queue_size=10)
        self._pub_smooth_speed = rospy.Publisher('/motion_control/smoothed_speed', Float32, queue_size=10)

        # ── ROS Subscribers ───────────────────────────────────────────────────
        rospy.Subscriber('/gcode_sender/state', String, self._printer_state_cb)

        # ── 等待 gcode_sender 服务 ────────────────────────────────────────────
        self._wait_for_services()

        # ── 启动日志 ──────────────────────────────────────────────────────────
        self._log_startup(speed_source, cmd_vel_topic, odom_topic)

    # ─────────────────────────────────────────────────────────────────────────
    # 初始化辅助
    # ─────────────────────────────────────────────────────────────────────────
    def _wait_for_services(self):
        for srv_name in ('/gcode_sender/pause', '/gcode_sender/resume'):
            try:
                rospy.wait_for_service(srv_name, timeout=10.0)
                logger.info(f"[INIT] ✓ {srv_name}")
            except rospy.ROSException:
                logger.warning(f"[INIT] ✗ {srv_name} 暂不可用（将在调用时重试）")

    def _log_startup(self, speed_source, cmd_vel_topic, odom_topic):
        sep = "=" * 64
        logger.info(sep)
        logger.info("  Motion-Based Print Control  (Printrun / ROS backend)")
        logger.info(sep)
        logger.info(f"  速度来源      : {speed_source}")
        logger.info(f"  速度 topic    : {cmd_vel_topic if speed_source == 'cmd_vel' else odom_topic}")
        logger.info(f"  暂停阈值      : {self._pause_thr:.4f} m/s  (threshold + hysteresis)")
        logger.info(f"  恢复阈值      : {self._resume_thr:.4f} m/s  (threshold - hysteresis)")
        logger.info(f"  防抖时间      : {self.debounce_time:.2f} s")
        logger.info(f"  滑动平均窗口  : {self.speed_window} 帧")
        logger.info(f"  CSV 日志      : {self._csv.log_file}")
        logger.info(sep)

    # ─────────────────────────────────────────────────────────────────────────
    # 速度更新（主循环轮询）
    # ─────────────────────────────────────────────────────────────────────────
    def _get_smoothed_speed(self) -> float:
        """从 monitor 取原始速度，计算滑动平均，发布，返回平滑值。"""
        raw = self._speed_monitor.get_speed()

        with self._lock:
            self._speed_history.append(raw)
            if len(self._speed_history) > self.speed_window:
                self._speed_history.pop(0)
            self._smoothed_speed = sum(self._speed_history) / len(self._speed_history)
            smooth = self._smoothed_speed

        self._pub_raw_speed.publish(raw)
        self._pub_smooth_speed.publish(smooth)
        return smooth

    # ─────────────────────────────────────────────────────────────────────────
    # 打印机状态同步
    # ─────────────────────────────────────────────────────────────────────────
    def _printer_state_cb(self, msg: String):
        """
        订阅 /gcode_sender/state，同步打印机状态。
        - 打印机进入 PRINTING  → 激活速度控制
        - 打印机回到 IDLE      → 停用速度控制
        """
        with self._lock:
            prev               = self.printer_state
            self.printer_state = msg.data

            if prev != "IDLE" and self.printer_state == "IDLE":
                self.control_state   = ControlState.INACTIVE
                self.control_enabled = False
                logger.info("[CTRL] 打印机进入 IDLE，速度控制停用")

            if self.printer_state == "PRINTING" and self.control_state == ControlState.INACTIVE:
                self.control_state   = ControlState.PRINTING
                self.control_enabled = True
                logger.info("[CTRL] 打印机开始打印，速度控制激活 ✓")

    # ─────────────────────────────────────────────────────────────────────────
    # 核心控制决策（主循环调用）
    # ─────────────────────────────────────────────────────────────────────────
    def _evaluate(self):
        """
        基于平滑速度 + 迟滞带 + 时间防抖，决定是否 pause / resume。

        防抖逻辑（移植自 robot_speed_controller1112 的 stable_counter，改为时间制）：
          - 速度连续超过 pause_thr  达 debounce_time 秒 → 暂停
          - 速度连续低于 resume_thr 达 debounce_time 秒 → 恢复
        """
        if not self.control_enabled:
            return

        speed = self._get_smoothed_speed()
        now   = time.time()

        with self._lock:
            state = self.control_state

        # ── 暂停判断 ──────────────────────────────────────────────────────────
        if state == ControlState.PRINTING:
            if speed > self._pause_thr:
                if self._above_since is None:
                    self._above_since = now
                    logger.debug(f"[EVAL] 速度 {speed:.4f} > {self._pause_thr:.4f}，开始防抖计时")
                elif now - self._above_since >= self.debounce_time:
                    self._do_pause(speed)
                    self._above_since = None
                self._below_since = None
            else:
                self._above_since = None  # 速度回落，重置计时

        # ── 恢复判断 ──────────────────────────────────────────────────────────
        elif state == ControlState.PAUSED:
            if speed <= self._resume_thr:
                if self._below_since is None:
                    self._below_since = now
                    logger.debug(f"[EVAL] 速度 {speed:.4f} ≤ {self._resume_thr:.4f}，开始防抖计时")
                elif now - self._below_since >= self.debounce_time:
                    self._do_resume(speed)
                    self._below_since = None
                self._above_since = None
            else:
                self._below_since = None  # 速度仍高，重置计时

    # ─────────────────────────────────────────────────────────────────────────
    # 执行暂停 / 恢复
    # ─────────────────────────────────────────────────────────────────────────
    def _do_pause(self, speed: float):
        """调用 /gcode_sender/pause，更新状态，写日志。"""
        t0 = time.perf_counter()
        try:
            resp  = rospy.ServiceProxy('/gcode_sender/pause', Trigger)()
            rt_ms = (time.perf_counter() - t0) * 1000

            if resp.success:
                with self._lock:
                    self.control_state = ControlState.PAUSED
                    self.pause_count  += 1
                    cnt  = self.pause_count
                    p_st = self.printer_state

                logger.warning(
                    f"[ACTION] ⚡ AUTO-PAUSE #{cnt}  "
                    f"speed={speed:.4f} m/s > {self._pause_thr:.4f} m/s  "
                    f"(响应 {rt_ms:.0f}ms)"
                )
                self._pub_state.publish(ControlState.PAUSED.value)
                self._csv.write(speed, 'PAUSE', p_st, rt_ms,
                                f"高速: {speed:.4f} m/s")
            else:
                logger.warning(f"[ACTION] 暂停失败: {resp.message}")
                self._csv.write(speed, 'PAUSE_FAILED', self.printer_state,
                                rt_ms, resp.message)

        except rospy.ServiceException as e:
            logger.error(f"[ACTION] pause 服务异常: {e}")

    def _do_resume(self, speed: float):
        """调用 /gcode_sender/resume，更新状态，写日志。"""
        t0 = time.perf_counter()
        try:
            resp  = rospy.ServiceProxy('/gcode_sender/resume', Trigger)()
            rt_ms = (time.perf_counter() - t0) * 1000

            if resp.success:
                with self._lock:
                    self.control_state  = ControlState.PRINTING
                    self.resume_count  += 1
                    cnt  = self.resume_count
                    p_st = self.printer_state

                logger.info(
                    f"[ACTION] ✅ AUTO-RESUME #{cnt}  "
                    f"speed={speed:.4f} m/s ≤ {self._resume_thr:.4f} m/s  "
                    f"(响应 {rt_ms:.0f}ms)"
                )
                self._pub_state.publish(ControlState.PRINTING.value)
                self._csv.write(speed, 'RESUME', p_st, rt_ms,
                                f"低速: {speed:.4f} m/s")
            else:
                logger.warning(f"[ACTION] 恢复失败: {resp.message}")
                self._csv.write(speed, 'RESUME_FAILED', self.printer_state,
                                rt_ms, resp.message)

        except rospy.ServiceException as e:
            logger.error(f"[ACTION] resume 服务异常: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # 公开控制接口（供 interactive_shell 调用）
    # ─────────────────────────────────────────────────────────────────────────
    def set_speed_threshold(self, threshold: float):
        if threshold <= 0:
            logger.error("[CONFIG] 速度阈值必须为正数")
            return
        self.speed_threshold = threshold
        self._pause_thr      = threshold + self.hysteresis
        self._resume_thr     = threshold - self.hysteresis
        logger.info(f"[CONFIG] 速度阈值更新: {threshold:.4f} m/s "
                    f"(pause>{self._pause_thr:.4f}, resume≤{self._resume_thr:.4f})")

    def set_debounce_time(self, t: float):
        if t < 0:
            logger.error("[CONFIG] 防抖时间不能为负数")
            return
        self.debounce_time = t
        logger.info(f"[CONFIG] 防抖时间更新: {t:.2f} s")

    def get_status(self) -> dict:
        with self._lock:
            return {
                'timestamp'          : datetime.now().isoformat(),
                'control_state'      : self.control_state.value,
                'printer_state'      : self.printer_state,
                'current_speed_ms'   : f"{self._speed_monitor.get_speed():.4f}",
                'smoothed_speed_ms'  : f"{self._smoothed_speed:.4f}",
                'speed_threshold'    : f"{self.speed_threshold:.4f}",
                'pause_threshold'    : f"{self._pause_thr:.4f}",
                'resume_threshold'   : f"{self._resume_thr:.4f}",
                'debounce_time'      : f"{self.debounce_time:.2f} s",
                'speed_monitor_ok'   : self._speed_monitor.is_alive(),
                'msgs_received'      : self._speed_monitor.get_msg_count(),
                'auto_pause_count'   : self.pause_count,
                'auto_resume_count'  : self.resume_count,
                'csv_log'            : self._csv.log_file,
                'runtime_s'          : f"{time.time() - self._session_start:.1f}",
            }

    # ─────────────────────────────────────────────────────────────────────────
    # 主循环
    # ─────────────────────────────────────────────────────────────────────────
    def run(self):
        """
        主循环：以 10 Hz 轮询速度 + 执行控制决策。
        同时保持 ROS topic 发布和状态日志。
        """
        rate = rospy.Rate(10)   # 10 Hz
        logger.info("[RUN] 进入主控制循环（10 Hz）...")

        loop_count = 0
        while not rospy.is_shutdown():
            self._evaluate()

            with self._lock:
                state = self.control_state
            self._pub_state.publish(state.value)

            # 每 5s 打印一次摘要（DEBUG 级别，生产时不刷屏）
            loop_count += 1
            if loop_count % 50 == 0:
                s = self.get_status()
                logger.debug(
                    f"[LOOP] state={s['control_state']:<10} "
                    f"printer={s['printer_state']:<10} "
                    f"speed={s['smoothed_speed_ms']} m/s | "
                    f"pause={s['auto_pause_count']} resume={s['auto_resume_count']}"
                )

            rate.sleep()

    # ─────────────────────────────────────────────────────────────────────────
    # 关闭
    # ─────────────────────────────────────────────────────────────────────────
    def shutdown(self):
        elapsed = time.time() - self._session_start
        sep = "=" * 64
        logger.info(sep)
        logger.info("  Motion-Based Print Control — 会话摘要")
        logger.info(sep)
        logger.info(f"  运行时间        : {elapsed:.1f} s")
        logger.info(f"  自动暂停次数    : {self.pause_count}")
        logger.info(f"  自动恢复次数    : {self.resume_count}")
        logger.info(f"  速度阈值        : {self.speed_threshold:.4f} m/s")
        logger.info(f"  CSV 日志        : {self._csv.log_file}")
        logger.info(sep)


# ─────────────────────────────────────────────────────────────────────────────
# 交互式命令行  （移植自 script1112.py 的 interactive_shell）
# ─────────────────────────────────────────────────────────────────────────────
def interactive_shell(node: MotionBasedControl):
    """
    提供与 script1112.py 风格一致的交互式命令行。
    在独立线程中运行，不阻塞 ROS 主循环。

    命令：
      status              - 显示系统状态
      threshold <value>   - 设置速度阈值 (m/s)
      debounce <time>     - 设置防抖时间 (s)
      logs                - 显示 CSV 日志路径
      help                - 显示帮助
      quit                - 退出节点
    """
    sep = "=" * 64
    print(f"\n{sep}")
    print("  Motion-Based Print Control — 交互式控制台")
    print(f"{sep}")
    print("  命令:")
    print("    status              - 显示系统状态")
    print("    threshold <value>   - 设置速度阈值 (m/s)")
    print("    debounce <time>     - 设置防抖时间 (s)")
    print("    logs                - 显示 CSV 日志路径")
    print("    help                - 显示帮助")
    print("    quit                - 退出")
    print(f"{sep}")
    print()
    print("  注意: 打印的 start / stop / connect 请使用 test_gcode_sender.py")
    print("        本节点在打印机进入 PRINTING 状态后自动激活速度控制")
    print()

    try:
        while not rospy.is_shutdown():
            try:
                raw = input(">>> ").strip()
            except EOFError:
                break

            if not raw:
                continue

            parts = raw.split(' ', 1)
            cmd   = parts[0].lower()
            arg   = parts[1] if len(parts) > 1 else None

            if cmd == 'quit':
                logger.info("[CMD] 收到 quit，正在关闭节点...")
                rospy.signal_shutdown("User quit")
                break

            elif cmd == 'status':
                st = node.get_status()
                print("\n  系统状态:")
                for k, v in sorted(st.items()):
                    print(f"    {k:<25}: {v}")
                print()

            elif cmd == 'threshold':
                if arg:
                    try:
                        node.set_speed_threshold(float(arg))
                        print(f"  ✓ 速度阈值 → {float(arg):.4f} m/s")
                    except ValueError:
                        print("  ✗ 请输入有效数值")
                else:
                    print("  用法: threshold <m/s>")

            elif cmd == 'debounce':
                if arg:
                    try:
                        node.set_debounce_time(float(arg))
                        print(f"  ✓ 防抖时间 → {float(arg):.2f} s")
                    except ValueError:
                        print("  ✗ 请输入有效数值")
                else:
                    print("  用法: debounce <seconds>")

            elif cmd == 'logs':
                print(f"  CSV 日志: {node._csv.log_file}")

            elif cmd == 'help':
                print()
                print("  status              - 显示系统状态")
                print("  threshold <value>   - 调整速度阈值（默认 0.04 m/s）")
                print("  debounce <time>     - 调整防抖时间（默认 0.5 s）")
                print("  logs                - 查看 CSV 日志路径")
                print()

            else:
                print(f"  ✗ 未知命令: {cmd}，输入 help 查看帮助")

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")


# ─────────────────────────────────────────────────────────────────────────────
# 入口
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    """argparse — 兼容直接 python3 运行；rosrun 时参数通过 ROS ~param 覆盖。"""
    parser = argparse.ArgumentParser(
        description='Motion-Based 3D Print Control (Printrun + ROS backend)'
    )
    parser.add_argument('--speed-source',  default='cmd_vel',
                        choices=['cmd_vel', 'odom'],
                        help='速度来源 (default: cmd_vel)')
    parser.add_argument('--cmd-vel-topic', default='/kmriiwa/base/command/cmd_vel',
                        help='cmd_vel topic')
    parser.add_argument('--odom-topic',    default='/kmriiwa/base/state/odom',
                        help='odom topic')
    parser.add_argument('--threshold',     type=float, default=0.04,
                        help='速度阈值 m/s (default: 0.04)')
    parser.add_argument('--debounce',      type=float, default=0.5,
                        help='防抖时间 s (default: 0.5)')
    parser.add_argument('--hysteresis',    type=float, default=0.005,
                        help='迟滞带 m/s (default: 0.005)')
    parser.add_argument('--log-dir',       default='./logs',
                        help='CSV 日志目录 (default: ./logs)')
    # 过滤 ROS 注入的 __name:= __log:= 等参数
    args, _ = parser.parse_known_args()
    return args


def main():
    if not ROS_AVAILABLE:
        print("✗ 错误: 需要 ROS 环境")
        print("  请执行: source /opt/ros/*/setup.bash")
        sys.exit(1)

    args = parse_args()

    rospy.init_node('motion_based_control', anonymous=False)

    node = MotionBasedControl(
        speed_threshold = args.threshold,
        speed_source    = args.speed_source,
        cmd_vel_topic   = args.cmd_vel_topic,
        odom_topic      = args.odom_topic,
        debounce_time   = args.debounce,
        hysteresis      = args.hysteresis,
        log_dir         = args.log_dir
    )

    rospy.on_shutdown(node.shutdown)

    # 交互式命令行在独立线程中运行（不阻塞 ROS spin）
    shell_thread = threading.Thread(
        target=interactive_shell,
        args=(node,),
        daemon=True,
        name='InteractiveShell'
    )
    shell_thread.start()

    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()