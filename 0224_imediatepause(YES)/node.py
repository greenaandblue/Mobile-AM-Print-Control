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

New ROS topics published:
  /gcode_sender/line_eta_ms    (Float32)  — estimated ms for current line
  /gcode_sender/pause_eta_ms   (Float32)  — estimated ms until pause takes effect

New ROS services:
  /gcode_sender/pause_info     (Trigger)  — returns human-readable pause ETA string
                                            without actually pausing

Place gcode_time_estimator.py in the same directory (or on PYTHONPATH).
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

# ── Estimator import ────────────────────────────────────────
try:
    from estimator import LiveEstimator
    ESTIMATOR_AVAILABLE = True
except ImportError:
    ESTIMATOR_AVAILABLE = False
    rospy.logwarn_once("gcode_time_estimator.py not found — pause ETA disabled. "
                       "Place it in the same directory as this node.")

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
            rospy.logwarn(f"Temp parse error: {e}")

    def _parse_position(self, line):
        try:
            parts = line.split()
            self.last_x = parts[0].split(":")[1]
            self.last_y = parts[1].split(":")[1]
            self.last_z = parts[2].split(":")[1]
        except Exception as e:
            rospy.logwarn(f"Pos parse error: {e}")

    def get_status_string(self, progress_pct, current_line, total_lines):
        now    = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        status = f"{now} [{progress_pct:5.1f}%] Line {current_line}/{total_lines}"
        if self.last_x and self.last_y and self.last_z:
            status += f" | X:{self.last_x} Y:{self.last_y} Z:{self.last_z}"
        if self.last_temp and self.last_bed:
            status += f" | Nozzle:{self.last_temp}°C Bed:{self.last_bed}°C"
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

        # ── G-code data ──────────────────────────────────────
        self.gcode_lines        = []
        self.total_lines        = 0
        self.gcode_filepath     = None
        self.current_line_index = 0   # index of line currently being sent

        # ── Printer state ────────────────────────────────────
        self.state = PrinterState.IDLE
        self.lock  = Lock()

        # ── Thread control events ────────────────────────────
        # ok_event     : set when printer sends 'ok' for current line
        # pause_event  : clear=paused, set=running
        # stop_event   : set → worker exits within 50 ms
        self.ok_event    = Event()
        self.pause_event = Event()
        self.pause_event.set()
        self.stop_event  = Event()

        # Flag: True while print worker thread is alive.
        # Prevents status-query 'ok' replies from triggering ok_event.
        self.print_thread_running = False
        self.print_thread         = None

        # ── Live estimator ───────────────────────────────────
        self.estimator: LiveEstimator = LiveEstimator() if ESTIMATOR_AVAILABLE else None

        # ── Status publishing ────────────────────────────────
        self.status_update_interval = self.config["status_update_interval"]
        self.last_status_time       = time.time()

        # ── ROS Publishers ───────────────────────────────────
        self.pub_status       = rospy.Publisher('/gcode_sender/status',       String,  queue_size=10)
        self.pub_progress     = rospy.Publisher('/gcode_sender/progress',     Float32, queue_size=10)
        self.pub_current_line = rospy.Publisher('/gcode_sender/current_line', Int32,   queue_size=10)
        self.pub_state        = rospy.Publisher('/gcode_sender/state',        String,  queue_size=10)
        # Estimator topics
        self.pub_line_eta_ms  = rospy.Publisher('/gcode_sender/line_eta_ms',  Float32, queue_size=10)
        self.pub_pause_eta_ms = rospy.Publisher('/gcode_sender/pause_eta_ms', Float32, queue_size=10)

        # ── ROS Subscribers ──────────────────────────────────
        rospy.Subscriber('/gcode_sender/load_file', String, self._load_file_callback)

        # ── ROS Services ─────────────────────────────────────
        rospy.Service('/gcode_sender/connect',    Trigger, self.handle_connect)
        rospy.Service('/gcode_sender/disconnect', Trigger, self.handle_disconnect)
        rospy.Service('/gcode_sender/start',      Trigger, self.handle_start)
        rospy.Service('/gcode_sender/pause',      Trigger, self.handle_pause)
        rospy.Service('/gcode_sender/resume',     Trigger, self.handle_resume)
        rospy.Service('/gcode_sender/stop',       Trigger, self.handle_stop)
        # New: query pause ETA without actually pausing
        rospy.Service('/gcode_sender/pause_info', Trigger, self.handle_pause_info)

        est_status = "enabled" if ESTIMATOR_AVAILABLE else "DISABLED (estimator not found)"
        rospy.loginfo(f"G-code Sender Node v4 — port={self.port} baud={self.baud} "
                      f"estimator={est_status}")

    # ─────────────────────────────────────────────────────────
    # Config
    # ─────────────────────────────────────────────────────────

    def _load_config(self):
        cfg = configparser.ConfigParser()
        cfg.read(CONFIG_FILE)
        return {
            "port":                   cfg.get   ("printer", "port",                   fallback="/dev/ttyACM1"),
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
            rospy.loginfo(f"Connecting to {self.port} @ {self.baud} baud …")
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
            rospy.logerr(f"Connection failed: {e}")
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
        """Lift nozzle 10 mm then home X Y — called before disconnect/stop."""
        if not (self.printer and self.printer.online):
            return
        try:
            rospy.loginfo("Homing printer …")
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
            rospy.logerr(f"Homing error: {e}")

    # ─────────────────────────────────────────────────────────
    # Serial receive callback
    # ─────────────────────────────────────────────────────────

    def _on_recv(self, line):
        """
        Called by printcore every time a line arrives from the printer.

        Prusa firmware notes:
          • Normal move reply:  "ok"
          • Temperature reply:  "ok T:200.0 /200.0 B:60.0 /60.0 …"
          • Both start with "ok", so startswith("ok") catches both.

        Gate: ok_event is only triggered while print_thread_running is True.
        This prevents M105 / M114 status-query replies (issued from the main
        loop when the thread is NOT running) from waking the send loop.
        """
        # Always parse temperature / position data
        self.handler.on_recv(line)

        # Only signal 'ok' while the print worker is active
        if self.print_thread_running:
            if line.strip().lower().startswith("ok"):
                self.ok_event.set()

    # ─────────────────────────────────────────────────────────
    # Line-by-line send + ok wait
    # ─────────────────────────────────────────────────────────

    def _send_and_wait(self, cmd: str, timeout: float = 60.0) -> bool:
        """
        Send a command and block until the motion is physically completed.
        Uses M400 to flush planner queue.
        """

        # 1 Send motion command
        self.ok_event.clear()
        self.printer.send_now(cmd)

        if not self._wait_for_ok(timeout):
            return False

        # 2 Force motion completion
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
        """
        Runs in a daemon thread while printing.

        For each G-code line:
          1. Check stop_event → exit if set.
          2. Wait for pause_event (blocks here while paused, checks stop every 50 ms).
          3. Estimate line execution time via LiveEstimator.
          4. Send the line and wait for 'ok'.
          5. Publish ETA data.
        """
        rospy.loginfo(f"Print worker started — {self.total_lines} lines")

        try:
            for i in range(self.current_line_index, self.total_lines):

                # ① Stop check
                if self.stop_event.is_set():
                    rospy.loginfo(f"Worker: stop detected at line {i}")
                    break

                # ② Pause wait — poll every 50 ms so stop is still responsive
                while not self.pause_event.is_set():
                    if self.stop_event.is_set():
                        rospy.loginfo(f"Worker: stop detected while paused at line {i}")
                        return
                    time.sleep(0.05)

                if self.stop_event.is_set():
                    break

                self.current_line_index = i

                # Strip inline comment → pure G-code command
                cmd = self.gcode_lines[i].split(';')[0].strip()
                if not cmd:
                    continue

                # ③ Estimate this line's execution time and advance estimator state
                if self.estimator:
                    eta_ms = self.estimator.estimate_and_commit(cmd)
                    self.pub_line_eta_ms.publish(float(eta_ms))
                    rospy.logdebug(f"[{i+1}/{self.total_lines}] {cmd}  ETA={eta_ms:.1f}ms")
                else:
                    rospy.logdebug(f"[{i+1}/{self.total_lines}] {cmd}")

                # ④ Send + wait ok
                ok = self._send_and_wait(cmd)
                if not ok and self.stop_event.is_set():
                    break

        finally:
            self.print_thread_running = False
            with self.lock:
                if self.stop_event.is_set():
                    rospy.loginfo("Print worker: stopped by user")
                else:
                    rospy.loginfo("Print worker: all lines sent — print complete ✓")
                self.state = PrinterState.IDLE

    # ─────────────────────────────────────────────────────────
    # File loading
    # ─────────────────────────────────────────────────────────

    def _load_file_callback(self, msg):
        self._load_gcode_file(msg.data)

    def _load_gcode_file(self, filepath: str) -> bool:
        """
        Load a .gcode file.
        Filters out blank lines and pure-comment lines (starting with ';').
        Inline comments are preserved here and stripped per-line at send time.
        """
        with self.lock:
            try:
                path = Path(filepath)
                if not path.exists():
                    rospy.logerr(f"File not found: {filepath}")
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

                rospy.loginfo(f"Loaded '{filepath}'  ({self.total_lines} lines)")
                for i, l in enumerate(self.gcode_lines[:10]):
                    rospy.loginfo(f"  {i+1:4d}: {l}")
                return True

            except Exception as e:
                rospy.logerr(f"Load error: {e}")
                return False

    # ─────────────────────────────────────────────────────────
    # Pause ETA helper
    # ─────────────────────────────────────────────────────────

    def _get_pause_eta_ms(self) -> float:
        """
        Returns estimated milliseconds until the current in-flight line
        finishes and the pause will actually take effect.

        In line-by-line mode this equals the remaining execution time
        of the single line currently being sent to the printer.
        """
        if self.estimator and self.print_thread_running:
            return self.estimator.remaining_ms()
        return 0.0

    def _get_pause_info_string(self) -> str:
        """Human-readable pause ETA string for service responses and logging."""
        if not ESTIMATOR_AVAILABLE or not self.estimator:
            return "Pause ETA unavailable (estimator not loaded)"
        if not self.print_thread_running:
            return "Not printing"
        eta_ms  = self._get_pause_eta_ms()
        cmd     = self.estimator.current_line_cmd
        total   = self.estimator.current_line_ms
        elapsed = self.estimator.elapsed_ms()
        return (f"Pause in ~{eta_ms:.0f}ms  "
                f"(cmd={cmd}  total={total:.0f}ms  elapsed={elapsed:.0f}ms)")

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

        # Reset all control events and estimator
        self.stop_event.clear()
        self.pause_event.set()
        self.ok_event.clear()
        if self.estimator:
            self.estimator.reset()

        self.print_thread_running = True
        self.print_thread = threading.Thread(target=self._print_worker, daemon=True)
        self.print_thread.start()

        rospy.loginfo("Print started (line-by-line + estimator)")
        return TriggerResponse(success=True, message="Started")

    def handle_pause(self, req):
        """
        Pause printing.

        The pause takes effect after the current line's 'ok' is received.
        The estimated wait time is reported in the response message and
        published to /gcode_sender/pause_eta_ms.
        """
        with self.lock:
            if self.state != PrinterState.PRINTING:
                return TriggerResponse(
                    success=False,
                    message=f"Not printing (state: {self.state.name})")
            self.state = PrinterState.PAUSED

        # Publish pause ETA before clearing the event
        eta_ms = self._get_pause_eta_ms()
        self.pub_pause_eta_ms.publish(float(eta_ms))
        info = self._get_pause_info_string()
        rospy.loginfo(f"Pause requested — {info}")

        # Clear pause_event: worker blocks after current line's ok arrives
        self.pause_event.clear()

        return TriggerResponse(
            success=True,
            message=f"Pausing at line {self.current_line_index}/{self.total_lines} — {info}")

    def handle_resume(self, req):
        with self.lock:
            if self.state != PrinterState.PAUSED:
                return TriggerResponse(
                    success=False,
                    message=f"Not paused (state: {self.state.name})")
            self.state = PrinterState.PRINTING

        self.pause_event.set()
        rospy.loginfo(f"Resumed from line {self.current_line_index}/{self.total_lines}")
        return TriggerResponse(
            success=True,
            message=f"Resumed from line {self.current_line_index}/{self.total_lines}")

    def handle_stop(self, req):
        """
        Stop printing immediately.
        stop_event breaks the _send_and_wait poll loop within 50 ms.
        pause_event is also set so the worker isn't stuck in pause-wait.
        """
        with self.lock:
            if self.state == PrinterState.IDLE:
                return TriggerResponse(success=False, message="Not printing")
            self.state = PrinterState.STOPPED

        self.stop_event.set()
        self.pause_event.set()   # unblock pause-wait so worker can reach stop check
        self.ok_event.set()      # unblock ok-wait immediately

        rospy.loginfo("Stop requested")
        return TriggerResponse(success=True, message="Stopped")

    def handle_pause_info(self, req):
        """
        Query pause ETA without actually pausing.
        Useful for the test script to display how long the current line will take.
        """
        with self.lock:
            state = self.state
        if state != PrinterState.PRINTING:
            return TriggerResponse(
                success=False,
                message=f"Not printing (state: {state.name})")
        info = self._get_pause_info_string()
        eta_ms = self._get_pause_eta_ms()
        self.pub_pause_eta_ms.publish(float(eta_ms))
        return TriggerResponse(success=True, message=info)

    # ─────────────────────────────────────────────────────────
    # Status publishing (main loop — NOT during print thread)
    # ─────────────────────────────────────────────────────────

    def _publish_status(self):
        """
        Publish progress/temperature/position at the configured interval.

        ★ M105 and M114 are ONLY sent when the print worker is NOT running.
          This is critical: their 'ok' replies must not trigger ok_event and
          cause the worker to skip a line.
        """
        now = time.time()
        if now - self.last_status_time < self.status_update_interval:
            return

        if not self.print_thread_running and self.printer and self.printer.online:
            self.printer.send_now("M105")   # temperature
            self.printer.send_now("M114")   # position

        with self.lock:
            current_state = self.state
            current_line  = self.current_line_index

        progress = (current_line / self.total_lines * 100.0) if self.total_lines > 0 else 0.0
        status   = self.handler.get_status_string(progress, current_line, self.total_lines)

        self.pub_status.publish(status)
        self.pub_progress.publish(float(progress))
        self.pub_current_line.publish(int(current_line))
        self.pub_state.publish(current_state.name)

        rospy.loginfo(f"[{current_state.name}] {status}")
        self.last_status_time = now

    # ─────────────────────────────────────────────────────────
    # Main loop
    # ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(1)   # 1 Hz — status publishing only
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
        rospy.loginfo("Shutting down …")
        self.stop_event.set()
        self.pause_event.set()
        self.ok_event.set()
        if self.print_thread and self.print_thread.is_alive():
            self.print_thread.join(timeout=3)
        self.home_printer()
        self.disconnect_printer()


# ─────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────

def main():
    try:
        node = GcodeSenderNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()