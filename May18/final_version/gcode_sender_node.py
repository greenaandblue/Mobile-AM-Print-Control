#!/usr/bin/env python3

"""
gcode_sender_node.py (v5 — Override + Injector)
================================================

New in v5:
  ⑤ Override function:
     - When paused, call /gcode_sender/preview_upcoming to see the next N
       lines (default 10) from the current position.
     - Call /gcode_sender/override_line with JSON payload to replace a
       specific upcoming line's G-code with a custom command.
     - Call /gcode_sender/clear_overrides to wipe all pending overrides.
     - Overrides are stored in a runtime dict keyed by absolute line index.
       The original gcode_lines list is NEVER modified.

  ⑥ Injector function:
     - When paused, call /gcode_sender/inject_before with JSON payload to
       insert one or more G-code lines immediately before a specified
       upcoming line index.
     - Call /gcode_sender/clear_injections to wipe all pending injections.
     - Injections are stored in a runtime dict keyed by absolute line index.
       The original gcode_lines list is NEVER modified.

  Both features are cleared automatically when a new print starts (handle_start)
  or when a file is reloaded, keeping state predictable.

Payload formats (passed as JSON strings via service `request` field — 
use Trigger-compatible String services, see handle_override_line /
handle_inject_before for details):

  override_line:
    {"index": 42, "replacement": "G1 X100 Y100 F3000"}
    - index     : absolute line index (as shown by preview_upcoming)
    - replacement: full G-code string to use instead

  inject_before:
    {"index": 42, "lines": ["G1 Z5 F600", "G4 P500"]}
    - index : inject the listed lines BEFORE this absolute line index
    - lines : list of G-code strings to inject

Note: Because Trigger has no request body, override/inject use
      std_msgs/String subscribers instead (simpler for ROS1 interop):
        /gcode_sender/override_line   (String JSON)
        /gcode_sender/inject_before   (String JSON)
      And plain Trigger services for preview/clear operations.

Fix log (v4.1 retained)
-----------------------
  ④ BUG FIX: _print_worker publishes final IDLE state immediately in its
     finally block via pub_state.
"""

import json
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

        # ── Override / Injection runtime tables ──────────────
        # Neither table ever touches self.gcode_lines.
        #
        # overrides  : {abs_line_index: "replacement G-code string"}
        #              When the worker reaches line i, if i is in overrides,
        #              the replacement is sent instead of gcode_lines[i].
        #
        # injections : {abs_line_index: ["gcode1", "gcode2", ...]}
        #              Before the worker sends line i, any lines listed under
        #              key i are sent first (in order), then consumed so they
        #              fire only once.
        self.overrides  = {}   # {int: str}
        self.injections = {}   # {int: list[str]}
        self.oi_lock    = Lock()  # separate lock so worker isn't blocked by UI

        # ── ROS Publishers ───────────────────────────────────
        self.pub_status        = rospy.Publisher('/gcode_sender/status',         String,  queue_size=10)
        self.pub_progress      = rospy.Publisher('/gcode_sender/progress',       Float32, queue_size=10)
        self.pub_current_line  = rospy.Publisher('/gcode_sender/current_line',   Int32,   queue_size=10)
        self.pub_state         = rospy.Publisher('/gcode_sender/state',          String,  queue_size=10)
        self.pub_upcoming      = rospy.Publisher('/gcode_sender/upcoming_lines', String,  queue_size=10)

        # ── ROS Subscribers ──────────────────────────────────
        rospy.Subscriber('/gcode_sender/load_file',     String, self._load_file_callback)
        rospy.Subscriber('/gcode_sender/override_line', String, self._override_line_callback)
        rospy.Subscriber('/gcode_sender/inject_before', String, self._inject_before_callback)

        # ── ROS Services ─────────────────────────────────────
        rospy.Service('/gcode_sender/connect',           Trigger, self.handle_connect)
        rospy.Service('/gcode_sender/disconnect',        Trigger, self.handle_disconnect)
        rospy.Service('/gcode_sender/start',             Trigger, self.handle_start)
        rospy.Service('/gcode_sender/pause',             Trigger, self.handle_pause)
        rospy.Service('/gcode_sender/resume',            Trigger, self.handle_resume)
        rospy.Service('/gcode_sender/stop',              Trigger, self.handle_stop)
        rospy.Service('/gcode_sender/preview_upcoming',  Trigger, self.handle_preview_upcoming)
        rospy.Service('/gcode_sender/clear_overrides',   Trigger, self.handle_clear_overrides)
        rospy.Service('/gcode_sender/clear_injections',  Trigger, self.handle_clear_injections)

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
            "preview_lines":          cfg.getint("printer", "preview_lines",          fallback=10),
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
    # Override helpers
    # ─────────────────────────────────────────────────────────

    def _get_effective_line(self, index):
        """
        Return the G-code that will actually be sent for gcode_lines[index].

        Priority:
          1. An override registered for this exact index  → use override
          2. Otherwise                                    → use original line

        The override is NOT consumed here — it stays until explicitly cleared
        via /gcode_sender/clear_overrides or a new print starts.  This lets
        you inspect what you registered without it disappearing on first use.
        (If you want one-shot semantics, set consume=True below.)
        """
        with self.oi_lock:
            if index in self.overrides:
                return self.overrides[index], True   # (cmd, was_overridden)
        raw = self.gcode_lines[index]
        return raw.split(';')[0].strip(), False

    def _pop_injections(self, index):
        """
        Return and CONSUME any injected lines registered to fire before
        gcode_lines[index].  Returns [] if nothing registered.
        Injections are one-shot: they are removed after being returned.
        """
        with self.oi_lock:
            return self.injections.pop(index, [])

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

                # ── INJECTIONS: fire any lines queued before index i ──────
                injected = self._pop_injections(i)
                for inj_cmd in injected:
                    inj_cmd = inj_cmd.split(';')[0].strip()
                    if not inj_cmd:
                        continue
                    rospy.loginfo("  [INJECT] {}".format(inj_cmd))
                    ok = self._send_and_wait(inj_cmd)
                    if not ok and self.stop_event.is_set():
                        return

                # ── OVERRIDE or original line ──────────────────────────────
                cmd, was_overridden = self._get_effective_line(i)
                if not cmd:
                    continue
                if was_overridden:
                    rospy.loginfo("  [OVERRIDE line {}] {}".format(i, cmd))

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

            # Publish final IDLE state immediately (fix from v4.1)
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

                # Clear runtime tables whenever a new file is loaded
                with self.oi_lock:
                    self.overrides.clear()
                    self.injections.clear()

                rospy.loginfo("Loaded '{}'  ({} lines)".format(filepath, self.total_lines))
                for i, l in enumerate(self.gcode_lines[:10]):
                    rospy.loginfo("  {:4d}: {}".format(i+1, l))
                return True

            except Exception as e:
                rospy.logerr("Load error: {}".format(e))
                return False

    # ─────────────────────────────────────────────────────────
    # Override / Injection subscriber callbacks
    # ─────────────────────────────────────────────────────────

    def _override_line_callback(self, msg):
        """
        Topic: /gcode_sender/override_line
        Payload (JSON string):
          {"index": <int>, "replacement": "<gcode>"}

        Registers a one-off replacement for the given absolute line index.
        The original gcode_lines list is NOT modified.

        Example (rostopic pub):
          rostopic pub /gcode_sender/override_line std_msgs/String \
            '{"index": 42, "replacement": "G1 X100 Y100 F3000"}'
        """
        try:
            data = json.loads(msg.data)
            idx  = int(data["index"])
            repl = str(data["replacement"]).strip()

            if idx < 0 or idx >= self.total_lines:
                rospy.logwarn("override_line: index {} out of range (0–{})".format(
                    idx, self.total_lines - 1))
                return

            with self.oi_lock:
                self.overrides[idx] = repl

            original = self.gcode_lines[idx].split(';')[0].strip()
            rospy.loginfo("Override registered — line {}: [{}] → [{}]".format(
                idx, original, repl))

        except (KeyError, ValueError, json.JSONDecodeError) as e:
            rospy.logerr("override_line: bad payload — {} | data={}".format(e, msg.data))

    def _inject_before_callback(self, msg):
        """
        Topic: /gcode_sender/inject_before
        Payload (JSON string):
          {"index": <int>, "lines": ["<gcode1>", "<gcode2>", ...]}

        Registers one or more G-code lines to be sent immediately BEFORE
        gcode_lines[index] executes.  Injections are one-shot (consumed
        once the worker reaches that index).

        Multiple inject_before calls for the same index are APPENDED in
        arrival order.

        Example:
          rostopic pub /gcode_sender/inject_before std_msgs/String \
            '{"index": 42, "lines": ["G1 Z5 F600", "G4 P500"]}'
        """
        try:
            data  = json.loads(msg.data)
            idx   = int(data["index"])
            lines = [str(l).strip() for l in data["lines"]]

            if idx < 0 or idx >= self.total_lines:
                rospy.logwarn("inject_before: index {} out of range (0–{})".format(
                    idx, self.total_lines - 1))
                return

            with self.oi_lock:
                existing = self.injections.setdefault(idx, [])
                existing.extend(lines)

            rospy.loginfo("Injection registered before line {} — {} cmd(s): {}".format(
                idx, len(lines), lines))

        except (KeyError, ValueError, json.JSONDecodeError) as e:
            rospy.logerr("inject_before: bad payload — {} | data={}".format(e, msg.data))

    # ─────────────────────────────────────────────────────────
    # Preview / Clear services
    # ─────────────────────────────────────────────────────────

    def handle_preview_upcoming(self, req):
        """
        Service: /gcode_sender/preview_upcoming  (Trigger)

        Returns a formatted string listing the next N lines starting from
        current_line_index, showing:
          • The original G-code from the file
          • [OVERRIDE] marker + replacement if an override is registered
          • [INJECT BEFORE] marker + lines if injections are queued

        N is read from printer_config.ini → preview_lines (default 10).
        """
        with self.lock:
            start = self.current_line_index
        n = self.config["preview_lines"]

        if self.total_lines == 0:
            return TriggerResponse(success=False, message="No G-code loaded")

        end = min(start + n, self.total_lines)
        lines_out = ["Upcoming lines {}-{} (of {}):".format(start, end - 1, self.total_lines)]

        with self.oi_lock:
            overrides_snap  = dict(self.overrides)
            injections_snap = {k: list(v) for k, v in self.injections.items()}

        for i in range(start, end):
            original = self.gcode_lines[i].split(';')[0].strip() or "<empty/comment>"
            row = "  [{:5d}] {}".format(i, original)

            if i in overrides_snap:
                row += "\n           ↳ [OVERRIDE] → {}".format(overrides_snap[i])

            if i in injections_snap:
                for inj in injections_snap[i]:
                    row += "\n           ↑ [INJECT BEFORE] {}".format(inj)

            lines_out.append(row)

        # Also note overrides/injections outside the preview window (if any)
        extra_ov  = {k: v for k, v in overrides_snap.items()  if k >= end}
        extra_inj = {k: v for k, v in injections_snap.items() if k >= end}
        if extra_ov or extra_inj:
            lines_out.append(
                "\n  (+ {} override(s) and {} injection(s) beyond preview window)".format(
                    len(extra_ov), len(extra_inj)))

        # Publish on topic too so subscribers can react
        preview_str = "\n".join(lines_out)
        self.pub_upcoming.publish(preview_str)

        return TriggerResponse(success=True, message=preview_str)

    def handle_clear_overrides(self, req):
        with self.oi_lock:
            count = len(self.overrides)
            self.overrides.clear()
        msg = "Cleared {} override(s)".format(count)
        rospy.loginfo(msg)
        return TriggerResponse(success=True, message=msg)

    def handle_clear_injections(self, req):
        with self.oi_lock:
            count = sum(len(v) for v in self.injections.values())
            self.injections.clear()
        msg = "Cleared injections ({} cmd(s) across {} index(es))".format(
            count, len(self.injections))
        rospy.loginfo(msg)
        return TriggerResponse(success=True, message=msg)

    # ─────────────────────────────────────────────────────────
    # ROS Service handlers (original)
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

        # Clear any leftover overrides/injections from a previous run
        with self.oi_lock:
            self.overrides.clear()
            self.injections.clear()

        self.stop_event.clear()
        self.pause_event.set()
        self.ok_event.clear()

        self.print_thread_running = True
        self.print_thread = threading.Thread(target=self._print_worker, daemon=True)
        self.print_thread.start()

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