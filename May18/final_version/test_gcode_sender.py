#!/usr/bin/env python3

"""
test_gcode_sender.py (v6 — Override + Injector)

Test script for gcode_sender_node (line-by-line mode).

New commands:
  v  — Preview upcoming N lines (shows overrides/injections too)
  o  — Register an override for a specific line
  j  — Inject G-code before a specific line
  co — Clear all overrides
  cj — Clear all injections
"""

import json
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

        # Subscribers
        rospy.Subscriber('/gcode_sender/status',         String,  self._cb_status)
        rospy.Subscriber('/gcode_sender/progress',       Float32, self._cb_progress)
        rospy.Subscriber('/gcode_sender/state',          String,  self._cb_state)
        rospy.Subscriber('/gcode_sender/current_line',   Int32,   self._cb_line)
        rospy.Subscriber('/gcode_sender/upcoming_lines', String,  self._cb_upcoming)

        # Publishers
        self.pub_file     = rospy.Publisher('/gcode_sender/load_file',     String, queue_size=10)
        self.pub_override = rospy.Publisher('/gcode_sender/override_line', String, queue_size=10)
        self.pub_inject   = rospy.Publisher('/gcode_sender/inject_before', String, queue_size=10)

    # ── Callbacks ────────────────────────────────────────────
    def _cb_status(self,   msg): self.current_status   = msg.data
    def _cb_progress(self, msg): self.current_progress = msg.data
    def _cb_state(self,    msg): self.current_state    = msg.data
    def _cb_line(self,     msg): self.current_line     = msg.data
    def _cb_upcoming(self, msg): pass  # service response already prints it

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
        if self.current_status and self.current_status != "Idle":
            print(f"  Detail     : {self.current_status}")
        print(f"  {'─'*68}")

    # ── Override / Inject helpers ─────────────────────────────

    def _do_override(self):
        """Interactively register a line override."""
        print(f"\n  Current line index: {self.current_line}")
        raw_idx = input("  Line index to override: ").strip()
        if not raw_idx:
            print("  Cancelled."); return
        raw_cmd = input("  Replacement G-code: ").strip()
        if not raw_cmd:
            print("  Cancelled."); return
        try:
            payload = json.dumps({"index": int(raw_idx), "replacement": raw_cmd})
            self.pub_override.publish(payload)
            print(f"  ✓ Override published: line {raw_idx} → [{raw_cmd}]")
        except ValueError:
            print("  ✗ Invalid index.")

    def _do_inject(self):
        """Interactively register injected lines before a given index."""
        print(f"\n  Current line index: {self.current_line}")
        raw_idx = input("  Inject BEFORE line index: ").strip()
        if not raw_idx:
            print("  Cancelled."); return

        gcodes = []
        print("  Enter G-code lines to inject (empty line to finish):")
        while True:
            line = input("    > ").strip()
            if not line:
                break
            gcodes.append(line)

        if not gcodes:
            print("  No lines entered — cancelled."); return
        try:
            payload = json.dumps({"index": int(raw_idx), "lines": gcodes})
            self.pub_inject.publish(payload)
            print(f"  ✓ Injection published: {len(gcodes)} line(s) before index {raw_idx}")
        except ValueError:
            print("  ✗ Invalid index.")

    # ── Main ─────────────────────────────────────────────────
    def run(self):
        self._hdr("G-code Sender Node — Test Script (v6 Override+Inject)")

        # Wait for all services
        self._sec("Checking ROS services …")
        services = [
            '/gcode_sender/connect',
            '/gcode_sender/start',
            '/gcode_sender/pause',
            '/gcode_sender/resume',
            '/gcode_sender/stop',
            '/gcode_sender/disconnect',
            '/gcode_sender/preview_upcoming',
            '/gcode_sender/clear_overrides',
            '/gcode_sender/clear_injections',
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
        time.sleep(0.5)

        # Interactive control
        self._hdr("Interactive Control")
        print("""
  Print commands:
    p   — Pause
    r   — Resume
    s   — Stop
    i   — Status
    q   — Quit (stop + disconnect)

  Override/Inject commands (best used while PAUSED):
    v   — Preview upcoming N lines  (shows registered overrides/injections)
    o   — Override a specific line with custom G-code
    j   — Inject G-code lines before a specific line index
    co  — Clear all overrides
    cj  — Clear all injections
        """)

        try:
            while not rospy.is_shutdown():
                cmd = input("\n  > ").strip().lower()

                if cmd == 'p':
                    ok, msg = self._call('/gcode_sender/pause')
                    print(f"  {'✓ Paused:' if ok else '✗ Failed:'} {msg}")

                elif cmd == 'r':
                    ok, msg = self._call('/gcode_sender/resume')
                    print(f"  {'✓ Resumed:' if ok else '✗ Failed:'} {msg}")

                elif cmd == 's':
                    ok, msg = self._call('/gcode_sender/stop')
                    print(f"  {'✓ Stopped:' if ok else '✗ Failed:'} {msg}")
                    time.sleep(0.5)

                elif cmd == 'i' or cmd == '':
                    self._print_status()

                elif cmd == 'q':
                    self._safe_exit()
                    break

                elif cmd == 'v':
                    ok, msg = self._call('/gcode_sender/preview_upcoming')
                    if ok:
                        print("\n" + msg)
                    else:
                        print(f"  ✗ {msg}")

                elif cmd == 'o':
                    if self.current_state != 'PAUSED':
                        print("  ⚠  Tip: pause first so you can see the upcoming line numbers.")
                    self._do_override()

                elif cmd == 'j':
                    if self.current_state != 'PAUSED':
                        print("  ⚠  Tip: pause first so you can see the upcoming line numbers.")
                    self._do_inject()

                elif cmd == 'co':
                    ok, msg = self._call('/gcode_sender/clear_overrides')
                    print(f"  {'✓' if ok else '✗'} {msg}")

                elif cmd == 'cj':
                    ok, msg = self._call('/gcode_sender/clear_injections')
                    print(f"  {'✓' if ok else '✗'} {msg}")

                else:
                    print("  ✗ Unknown command — use: p  r  s  i  v  o  j  co  cj  q")

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