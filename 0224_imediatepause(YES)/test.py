#!/usr/bin/env python3

"""
test_gcode_sender.py (v4)

Test script for gcode_sender_node v4.
New in v4:
  - 'e' command: query pause ETA without pausing (/gcode_sender/pause_info)
  - Shows /gcode_sender/line_eta_ms and /gcode_sender/pause_eta_ms live
  - Pause response now includes estimated latency in the message
"""

import rospy
import sys
import time
from std_msgs.msg import String, Float32, Int32
from std_srvs.srv import Trigger


class GcodeSenderTest:

    def __init__(self):
        rospy.init_node('gcode_sender_test', anonymous=True)

        # State
        self.current_status   = "Idle"
        self.current_progress = 0.0
        self.current_state    = "IDLE"
        self.current_line     = 0
        self.line_eta_ms      = 0.0
        self.pause_eta_ms     = 0.0

        # Subscribers
        rospy.Subscriber('/gcode_sender/status',       String,  self._cb_status)
        rospy.Subscriber('/gcode_sender/progress',     Float32, self._cb_progress)
        rospy.Subscriber('/gcode_sender/state',        String,  self._cb_state)
        rospy.Subscriber('/gcode_sender/current_line', Int32,   self._cb_line)
        rospy.Subscriber('/gcode_sender/line_eta_ms',  Float32, self._cb_line_eta)
        rospy.Subscriber('/gcode_sender/pause_eta_ms', Float32, self._cb_pause_eta)

        # Publisher
        self.pub_file = rospy.Publisher('/gcode_sender/load_file', String, queue_size=10)

    # ── Callbacks ────────────────────────────────────────────
    def _cb_status(self, msg):    self.current_status   = msg.data
    def _cb_progress(self, msg):  self.current_progress = msg.data
    def _cb_state(self, msg):     self.current_state    = msg.data
    def _cb_line(self, msg):      self.current_line     = msg.data
    def _cb_line_eta(self, msg):  self.line_eta_ms      = msg.data
    def _cb_pause_eta(self, msg): self.pause_eta_ms     = msg.data

    # ── Service helpers ───────────────────────────────────────
    def _wait_for_service(self, name, timeout=5.0):
        try:
            rospy.wait_for_service(name, timeout=timeout)
            return True
        except rospy.ROSException:
            print(f"  ✗ Service not available: {name}")
            return False

    def _call(self, name):
        try:
            proxy = rospy.ServiceProxy(name, Trigger)
            r = proxy()
            return r.success, r.message
        except rospy.ServiceException as e:
            return False, str(e)

    # ── Display helpers ───────────────────────────────────────
    def _hdr(self, text):
        print("\n" + "=" * 72)
        print(f"  {text}")
        print("=" * 72)

    def _sec(self, text):
        print(f"\n[*] {text}")

    def _print_status(self):
        print(f"\n  {'─'*68}")
        print(f"  State      : {self.current_state}")
        print(f"  Progress   : {self.current_progress:.1f}%  (line {self.current_line})")
        print(f"  Line ETA   : {self.line_eta_ms:.1f} ms  (current line estimated duration)")
        print(f"  Pause ETA  : {self.pause_eta_ms:.1f} ms  (time until pause takes effect)")
        if self.current_status and self.current_status != "Idle":
            print(f"  Detail     : {self.current_status}")
        print(f"  {'─'*68}")

    # ── Main ─────────────────────────────────────────────────
    def run(self):
        self._hdr("G-code Sender Node v4 — Test Script")

        # Wait for all services
        self._sec("Checking ROS services …")
        services = [
            '/gcode_sender/connect',
            '/gcode_sender/start',
            '/gcode_sender/pause',
            '/gcode_sender/resume',
            '/gcode_sender/stop',
            '/gcode_sender/disconnect',
            '/gcode_sender/pause_info',
        ]
        all_ok = all(self._wait_for_service(s) for s in services)
        if not all_ok:
            print("\n  ✗ Some services missing. Is gcode_sender_node running?")
            return
        print("\n  ✓ All services ready")
        time.sleep(0.3)

        # Load G-code file
        self._sec("Load G-code file")
        gcode_file = sys.argv[1] if len(sys.argv) > 1 else input("  Enter path: ").strip()
        if not gcode_file:
            print("  ✗ No path given"); return
        self.pub_file.publish(gcode_file)
        print(f"  Published: {gcode_file}")
        time.sleep(1.0)

        # Connect
        self._sec("Connect")
        ok, msg = self._call('/gcode_sender/connect')
        print(f"  {'✓' if ok else '✗'} {msg}")
        if not ok: return
        time.sleep(0.5)

        # Start
        self._sec("Start printing")
        ok, msg = self._call('/gcode_sender/start')
        print(f"  {'✓' if ok else '✗'} {msg}")
        if not ok: return

        print("""
  ℹ  Line-by-line mode active:
       • Each G-code sent only after printer confirms the previous.
       • Pause takes effect after the current line finishes.
       • /gcode_sender/line_eta_ms shows how long each line takes.
       • /gcode_sender/pause_eta_ms shows remaining time when pause is issued.
        """)
        time.sleep(0.5)

        # Interactive control
        self._hdr("Interactive Control")
        print("""
  Commands:
    p  — Pause    (shows estimated ms until pause takes effect)
    r  — Resume
    s  — Stop     (cancel print)
    e  — ETA info (query pause ETA without pausing)
    i  — Status   (progress, line ETA, pause ETA)
    q  — Quit     (stop + disconnect)
        """)

        try:
            while not rospy.is_shutdown():
                cmd = input("\n  > ").strip().lower()

                if cmd == 'p':
                    # Query ETA first so we can show it before the actual pause
                    ok_info, info_msg = self._call('/gcode_sender/pause_info')
                    if ok_info:
                        print(f"  ℹ  {info_msg}")
                    ok, msg = self._call('/gcode_sender/pause')
                    print(f"  {'✓ Paused:' if ok else '✗ Failed:'} {msg}")

                elif cmd == 'r':
                    ok, msg = self._call('/gcode_sender/resume')
                    print(f"  {'✓ Resumed:' if ok else '✗ Failed:'} {msg}")

                elif cmd == 's':
                    ok, msg = self._call('/gcode_sender/stop')
                    print(f"  {'✓ Stopped:' if ok else '✗ Failed:'} {msg}")
                    time.sleep(0.5)

                elif cmd == 'e':
                    ok, msg = self._call('/gcode_sender/pause_info')
                    if ok:
                        print(f"\n  Pause ETA info:")
                        print(f"    {msg}")
                        print(f"    line_eta_ms  = {self.line_eta_ms:.1f} ms  "
                              f"(full estimated duration of current line)")
                        print(f"    pause_eta_ms = {self.pause_eta_ms:.1f} ms  "
                              f"(time remaining until pause would take effect)")
                    else:
                        print(f"  ✗ {msg}")

                elif cmd == 'i':
                    self._print_status()

                elif cmd == 'q':
                    self._safe_exit()
                    break

                elif cmd == '':
                    self._print_status()

                else:
                    print("  ✗ Unknown command — use: p  r  s  e  i  q")

        except KeyboardInterrupt:
            print("\n  Interrupted.")
            self._safe_exit()
        except Exception as e:
            print(f"\n  Error: {e}")
            self._safe_exit()

    def _safe_exit(self):
        print("  Stopping and disconnecting …")
        if self.current_state in ('PRINTING', 'PAUSED'):
            self._call('/gcode_sender/stop')
            time.sleep(1.0)
        self._call('/gcode_sender/disconnect')
        print("  ✓ Done.\n")


def main():
    try:
        GcodeSenderTest().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()