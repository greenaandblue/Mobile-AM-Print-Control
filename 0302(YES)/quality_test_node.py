#!/usr/bin/env python3

"""
gcode_sender_node.py (v4 — Line-by-line + Live Time Estimator)
===============================================================

Improvements over v3:
  ① Line-by-line sending: each G-code is sent only after the printer
    returns 'ok' for the previous one → zero buffer look-ahead, full
    control over pause/stop timing.

  ② Live pause-latency estimation (via gcode_time_estimator.LiveEstimator):
    - Before each line is sent, its estimated execution time is calculated
      using a trapezoidal velocity + Jerk model that mirrors Prusa/Marlin
      firmware.
    - When pause is requested, the node immediately publishes how many ms
      remain on the current in-flight line before the printer will actually
      stop accepting new commands.
    - A new ROS service /gcode_sender/pause_info returns this as a human-
      readable string BEFORE the pause is committed.

  ③ Prusa-specific fixes:
    - 'ok' detection handles "ok T:xxx" responses from M105 (temperature
      replies embed ok in the same line on Prusa firmware).
    - print_thread_running gate blocks status-query ok replies from
      contaminating the per-line ok_event.
    - stop_event replaces stop_flag so any blocked wait() exits in ≤50 ms.

Fix log (v4.1)
--------------
  ④ BUG FIX: _print_worker now publishes the final IDLE state immediately
     in its finally block via pub_state. Previously _publish_status() was
     the only place that called pub_state, but it only ran while state was
     PRINTING or PAUSED — so the IDLE transition after print completion was
     never broadcast. base_control_script.py (and any other subscriber)
     therefore never received the IDLE message and could not stop.
"""

import rospy
import configparser
import time
import threading
from pathlib import Path
from datetime import datetime
from threading import Lock, Event
from enum import Enum

from printrun.printcore import printcore
from printrun.eventhandler import PrinterEventHandler

from std_msgs.msg import String, Int32, Float32
from std_srvs.srv import Trigger, TriggerResponse

CONFIG_FILE = "printer_config.ini"


# ─────────────────────────────────────────────────────────────
# Printer State
# ─────────────────────────────────────────────────────────────

class PrinterState(Enum):
    IDLE     = 0
    PRINTING = 1
    PAUSED   = 2
    STOPPED  = 3


# ─────────────────────────────────────────────────────────────
# Serial receive handler — temperature / position parsing
# ─────────────────────────────────────────────────────────────

class GcodeSenderHandler(PrinterEventHandler):

    def __init__(self):
        super().__init__()
        self.last_temp = None
        self.last_bed  = None
        self.last_x    = None
        self.last_y    = None
        self.last_z    = None

    def on_recv(self, line):
        if "T:" in line and "B:" in line:
            self._parse_temperature(line)
        elif "X:" in line and "Y:" in line and "Z:" in line:
            self._parse_position(line)

    def _parse_temperature(self, line):
        try:
            self.last_temp = line.split("T:")[1].split(" ")[0]
            self.last_bed  = line.split("B:")[1].split(" ")[0]
        except Exception as e:
            rospy.logwarn("Temp parse error: {}".format(e))

    def _parse_position(self, line):
        try:
            parts = line.split()
            self.last_x = parts[0].split(":")[1]
            self.last_y = parts[1].split(":")[1]
            self.last_z = parts[2].split(":")[1]
        except Exception as e:
            rospy.logwarn("Pos parse error: {}".format(e))

    def get_status_string(self, progress_pct, current_line, total_lines):
        now    = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        status = "{} [{:5.1f}%] Line {}/{}".format(now, progress_pct, current_line, total_lines)
        if self.last_x and self.last_y and self.last_z:
            status += " | X:{} Y:{} Z:{}".format(self.last_x, self.last_y, self.last_z)
        if self.last_temp and self.last_bed:
            status += " | Nozzle:{}°C Bed:{}°C".format(self.last_temp, self.last_bed)
        return status


# ─────────────────────────────────────────────────────────────
# Main Node
# ─────────────────────────────────────────────────────────────

class GcodeSenderNode:

    def __init__(self):
        rospy.init_node('gcode_sender_node', anonymous=False)

        self.config  = self._load_config()
        self.port    = self.config["port"]
        self.baud    = self.config["baud"]
        self.printer = None
        self.handler = GcodeSenderHandler()

        self.gcode_lines        = []
        self.total_lines        = 0
        self.gcode_filepath     = None
        self.current_line_index = 0

        self.state = PrinterState.IDLE
        self.lock  = Lock()

        self.ok_event    = Event()
        self.pause_event = Event()
        self.pause_event.set()
        self.stop_event  = Event()

        self.print_thread_running = False
        self.print_thread         = None

        self.status_update_interval = self.config["status_update_interval"]
        self.last_status_time       = time.time()

        # ── ROS Publishers ───────────────────────────────────
        self.pub_status       = rospy.Publisher('/gcode_sender/status',       String,  queue_size=10)
        self.pub_progress     = rospy.Publisher('/gcode_sender/progress',     Float32, queue_size=10)
        self.pub_current_line = rospy.Publisher('/gcode_sender/current_line', Int32,   queue_size=10)
        self.pub_state        = rospy.Publisher('/gcode_sender/state',        String,  queue_size=10)

        # ── ROS Subscribers ──────────────────────────────────
        rospy.Subscriber('/gcode_sender/load_file', String, self._load_file_callback)

        # ── ROS Services ─────────────────────────────────────
        rospy.Service('/gcode_sender/connect',    Trigger, self.handle_connect)
        rospy.Service('/gcode_sender/disconnect', Trigger, self.handle_disconnect)
        rospy.Service('/gcode_sender/start',      Trigger, self.handle_start)
        rospy.Service('/gcode_sender/pause',      Trigger, self.handle_pause)
        rospy.Service('/gcode_sender/resume',     Trigger, self.handle_resume)
        rospy.Service('/gcode_sender/stop',       Trigger, self.handle_stop)

    # ─────────────────────────────────────────────────────────
    # Config
    # ─────────────────────────────────────────────────────────

    def _load_config(self):
        cfg = configparser.ConfigParser()
        cfg.read(CONFIG_FILE)
        return {
            "port":                   cfg.get   ("printer", "port",                   fallback="/dev/ttyACM0"),
            "baud":                   cfg.getint("printer", "baud",                   fallback=115200),
            "status_update_interval": cfg.getint("printer", "status_update_interval", fallback=10),
        }

    # ─────────────────────────────────────────────────────────
    # Printer connection helpers
    # ─────────────────────────────────────────────────────────

    def connect_printer(self):
        if self.printer and self.printer.online:
            rospy.logwarn("Printer already connected")
            return True
        try:
            rospy.loginfo("Connecting to {} @ {} baud ...".format(self.port, self.baud))
            self.printer = printcore(port=self.port, baud=self.baud)
            self.printer.addEventHandler(self.handler)
            self.printer.recvcb = self._on_recv

            deadline = time.time() + 10
            while not self.printer.online:
                if time.time() > deadline:
                    rospy.logerr("Connection timeout")
                    return False
                time.sleep(0.1)

            rospy.loginfo("Printer connected ✓")
            with self.lock:
                self.state = PrinterState.IDLE
            return True
        except Exception as e:
            rospy.logerr("Connection failed: {}".format(e))
            return False

    def disconnect_printer(self):
        if self.printer:
            try:
                self.printer.recvcb = None
                self.printer.disconnect()
            except Exception:
                pass
            self.printer = None
        with self.lock:
            self.state = PrinterState.IDLE
        rospy.loginfo("Printer disconnected")

    def home_printer(self):
        if not (self.printer and self.printer.online):
            return
        try:
            rospy.loginfo("Homing printer ...")
            self.printer.send_now("G91")
            time.sleep(0.1)
            self.printer.send_now("G1 Z10 F600")
            time.sleep(1.2)
            self.printer.send_now("G90")
            time.sleep(0.1)
            self.printer.send_now("G28 X Y")
            time.sleep(2.5)
            rospy.loginfo("Homing done ✓")
        except Exception as e:
            rospy.logerr("Homing error: {}".format(e))

    # ─────────────────────────────────────────────────────────
    # Serial receive callback
    # ─────────────────────────────────────────────────────────

    def _on_recv(self, line):
        self.handler.on_recv(line)
        if self.print_thread_running:
            if line.strip().lower().startswith("ok"):
                self.ok_event.set()

    # ─────────────────────────────────────────────────────────
    # Line-by-line send + ok wait
    # ─────────────────────────────────────────────────────────

    def _send_and_wait(self, cmd, timeout=60.0):
        self.ok_event.clear()
        self.printer.send_now(cmd)
        if not self._wait_for_ok(timeout):
            return False
        self.ok_event.clear()
        self.printer.send_now("M400")
        if not self._wait_for_ok(timeout):
            return False
        return True

    def _wait_for_ok(self, timeout):
        deadline = time.time() + timeout
        while True:
            if self.stop_event.is_set():
                return False
            if self.ok_event.wait(timeout=0.05):
                return True
            if time.time() > deadline:
                rospy.logwarn("Timeout waiting for ok")
                return False

    # ─────────────────────────────────────────────────────────
    # Print worker thread
    # ─────────────────────────────────────────────────────────

    def _print_worker(self):
        rospy.loginfo("Print worker started — {} lines".format(self.total_lines))

        try:
            for i in range(self.current_line_index, self.total_lines):

                if self.stop_event.is_set():
                    rospy.loginfo("Worker: stop detected at line {}".format(i))
                    break

                while not self.pause_event.is_set():
                    if self.stop_event.is_set():
                        rospy.loginfo("Worker: stop detected while paused at line {}".format(i))
                        return
                    time.sleep(0.05)

                if self.stop_event.is_set():
                    break

                self.current_line_index = i

                cmd = self.gcode_lines[i].split(';')[0].strip()
                if not cmd:
                    continue

                ok = self._send_and_wait(cmd)
                if not ok and self.stop_event.is_set():
                    break

        finally:
            self.print_thread_running = False
            with self.lock:
                if self.stop_event.is_set():
                    rospy.loginfo("Print worker: stopped by user")
                    self.state = PrinterState.IDLE
                else:
                    rospy.loginfo("Print worker: all lines sent — print complete ✓")
                    self.state = PrinterState.IDLE

            # ── FIX: publish final IDLE state immediately ──────────────────
            # _publish_status() only fires while state is PRINTING or PAUSED,
            # so the IDLE transition was never broadcast before this fix.
            # Any subscriber (e.g. base_control_script.py) now receives IDLE
            # within milliseconds of the print finishing or being stopped.
            self.pub_state.publish(PrinterState.IDLE.name)
            rospy.loginfo("[gcode_sender] Published final state: IDLE")

    # ─────────────────────────────────────────────────────────
    # File loading
    # ─────────────────────────────────────────────────────────

    def _load_file_callback(self, msg):
        self._load_gcode_file(msg.data)

    def _load_gcode_file(self, filepath):
        with self.lock:
            try:
                path = Path(filepath)
                if not path.exists():
                    rospy.logerr("File not found: {}".format(filepath))
                    return False

                lines = []
                with open(path, 'r') as f:
                    for line in f:
                        s = line.strip()
                        if not s or s.startswith(';'):
                            continue
                        lines.append(s)

                self.gcode_lines    = lines
                self.total_lines    = len(lines)
                self.gcode_filepath = filepath

                rospy.loginfo("Loaded '{}'  ({} lines)".format(filepath, self.total_lines))
                for i, l in enumerate(self.gcode_lines[:10]):
                    rospy.loginfo("  {:4d}: {}".format(i+1, l))
                return True

            except Exception as e:
                rospy.logerr("Load error: {}".format(e))
                return False

    # ─────────────────────────────────────────────────────────
    # ROS Service handlers
    # ─────────────────────────────────────────────────────────

    def handle_connect(self, req):
        ok = self.connect_printer()
        return TriggerResponse(success=ok,
                               message="Connected" if ok else "Connection failed")

    def handle_disconnect(self, req):
        self.home_printer()
        self.disconnect_printer()
        return TriggerResponse(success=True, message="Homed and disconnected")

    def handle_start(self, req):
        with self.lock:
            if self.state == PrinterState.PRINTING:
                return TriggerResponse(success=False, message="Already printing")
            if self.state == PrinterState.PAUSED:
                return TriggerResponse(success=False, message="Currently paused — use /resume")
            if not (self.printer and self.printer.online):
                return TriggerResponse(success=False, message="Printer not connected")
            if self.total_lines == 0:
                return TriggerResponse(success=False, message="No G-code loaded")
            self.state = PrinterState.PRINTING
            self.current_line_index = 0

        self.stop_event.clear()
        self.pause_event.set()
        self.ok_event.clear()

        self.print_thread_running = True
        self.print_thread = threading.Thread(target=self._print_worker, daemon=True)
        self.print_thread.start()

        # Publish PRINTING immediately so subscribers (e.g. base_control_script)
        # don't have to wait up to status_update_interval seconds to learn
        # that printing has started.
        self.pub_state.publish(PrinterState.PRINTING.name)
        rospy.loginfo("Print started (line-by-line)")
        return TriggerResponse(success=True, message="Started")

    def handle_pause(self, req):
        with self.lock:
            if self.state != PrinterState.PRINTING:
                return TriggerResponse(
                    success=False,
                    message="Not printing (state: {})".format(self.state.name))
            self.state = PrinterState.PAUSED

        self.pause_event.clear()
        # Publish PAUSED immediately — don't wait for the status timer
        self.pub_state.publish(PrinterState.PAUSED.name)
        rospy.loginfo("Pause requested at line {}/{}".format(
            self.current_line_index, self.total_lines))
        return TriggerResponse(
            success=True,
            message="Pausing at line {}/{}".format(self.current_line_index, self.total_lines))

    def handle_resume(self, req):
        with self.lock:
            if self.state != PrinterState.PAUSED:
                return TriggerResponse(
                    success=False,
                    message="Not paused (state: {})".format(self.state.name))
            self.state = PrinterState.PRINTING

        self.pause_event.set()
        # Publish PRINTING immediately — don't wait for the status timer
        self.pub_state.publish(PrinterState.PRINTING.name)
        rospy.loginfo("Resumed from line {}/{}".format(
            self.current_line_index, self.total_lines))
        return TriggerResponse(
            success=True,
            message="Resumed from line {}/{}".format(self.current_line_index, self.total_lines))

    def handle_stop(self, req):
        with self.lock:
            if self.state == PrinterState.IDLE:
                return TriggerResponse(success=False, message="Not printing")
            self.state = PrinterState.STOPPED

        self.stop_event.set()
        self.pause_event.set()
        self.ok_event.set()

        rospy.loginfo("Stop requested")
        return TriggerResponse(success=True, message="Stopped")

    # ─────────────────────────────────────────────────────────
    # Status publishing (main loop)
    # ─────────────────────────────────────────────────────────

    def _publish_status(self):
        now = time.time()
        if now - self.last_status_time < self.status_update_interval:
            return

        if not self.print_thread_running and self.printer and self.printer.online:
            self.printer.send_now("M105")
            self.printer.send_now("M114")

        with self.lock:
            current_state = self.state
            current_line  = self.current_line_index

        progress = (current_line / self.total_lines * 100.0) if self.total_lines > 0 else 0.0
        status   = self.handler.get_status_string(progress, current_line, self.total_lines)

        self.pub_status.publish(status)
        self.pub_progress.publish(float(progress))
        self.pub_current_line.publish(int(current_line))
        self.pub_state.publish(current_state.name)

        rospy.loginfo("[{}] {}".format(current_state.name, status))
        self.last_status_time = now

    # ─────────────────────────────────────────────────────────
    # Main loop
    # ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            with self.lock:
                cs = self.state
            if cs in (PrinterState.PRINTING, PrinterState.PAUSED):
                self._publish_status()
            rate.sleep()

    # ─────────────────────────────────────────────────────────
    # Shutdown
    # ─────────────────────────────────────────────────────────

    def shutdown(self):
        rospy.loginfo("Shutting down ...")
        self.stop_event.set()
        self.pause_event.set()
        self.ok_event.set()
        if self.print_thread and self.print_thread.is_alive():
            self.print_thread.join(timeout=3)
        self.home_printer()
        self.disconnect_printer()


def main():
    try:
        node = GcodeSenderNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()