#!/usr/bin/env python3
"""
base_control_script.py
======================

Moves the KMR iiwa base back and forth, reversing direction every 3 metres,
and automatically stops as soon as the 3-D printer finishes (i.e.
/gcode_sender/state transitions to IDLE after having been PRINTING).

Fix log (v2)
------------
* Bug 1 fixed: _stop_requested is now a threading.Event so it is properly
  visible across the main thread and the ROS spinner/callback thread in ROS 1.
  Plain bool assignment is not guaranteed to be seen immediately by another
  thread in CPython when no lock is used; Event.set() + Event.is_set() are
  thread-safe by design.

* Bug 2 fixed (companion change in gcode_sender_node.py): gcode_sender_node
  only published status while state was PRINTING or PAUSED, so the IDLE
  transition after print completion was never broadcast. The node now
  publishes one final IDLE message from inside _print_worker's finally block.
  This script therefore now reliably receives the IDLE message.

* rospy.sleep() calls between legs are replaced with a polling loop so a stop
  arriving mid-sleep is caught within 50 ms instead of being delayed.
"""

import threading
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ─────────────────────────────────────────────────────────────────────────────
# Configuration  (override via ROS private params: _linear_speed:=0.4)
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_LINEAR_SPEED = 0.4   # m/s
DEFAULT_DISTANCE     = 2.0   # metres per leg before direction reversal
CONTROL_RATE_HZ      = 10    # Hz — cmd_vel publish rate
CMD_VEL_TOPIC        = '/kmriiwa/base/command/cmd_vel'
PRINTER_STATE_TOPIC  = '/gcode_sender/state'


class BaseControlNode:

    def __init__(self):
        rospy.init_node('base_control_script', anonymous=False)

        self.linear_speed = rospy.get_param('~linear_speed', DEFAULT_LINEAR_SPEED)
        self.distance     = rospy.get_param('~distance',     DEFAULT_DISTANCE)
        self.leg_duration = self.distance / self.linear_speed  # seconds

        self.pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)

        # ── Thread-safe state flags ──────────────────────────────────────────
        # Using threading.Event instead of plain bool so that the ROS callback
        # thread's writes are immediately visible to the main motion thread.
        self._print_was_active = threading.Event()  # set once PRINTING seen
        self._stop_requested   = threading.Event()  # set when IDLE after print

        rospy.Subscriber(PRINTER_STATE_TOPIC, String, self._printer_state_cb)

        rospy.loginfo(
            "[BaseControl] speed={} m/s  distance={} m  leg_duration={:.1f} s".format(
                self.linear_speed, self.distance, self.leg_duration)
        )
        rospy.loginfo(
            "[BaseControl] Waiting for printer to start printing ... "
            "(listening on {})".format(PRINTER_STATE_TOPIC)
        )

    # ── Printer state callback (runs in ROS spinner thread) ──────────────────

    def _printer_state_cb(self, msg):
        state = msg.data.strip().upper()
        rospy.loginfo("[BaseControl] Printer state -> {}".format(state))

        if state in ("PRINTING", "PAUSED"):
            self._print_was_active.set()

        # Only trigger stop once a real print job has been seen finishing
        if self._print_was_active.is_set() and state == "IDLE":
            rospy.loginfo("[BaseControl] Print finished — requesting robot stop.")
            self._stop_requested.set()

    # ── Motion helpers ────────────────────────────────────────────────────────

    def _publish_velocity(self, linear_x):
        twist = Twist()
        twist.linear.x = linear_x
        self.pub.publish(twist)

    def _stop_robot(self):
        # Send zero-velocity several times to make sure it is received
        for _ in range(5):
            self._publish_velocity(0.0)
            rospy.sleep(0.05)
        rospy.loginfo("[BaseControl] Robot stopped.")

    def _interruptible_sleep(self, duration):
        """Sleep for duration seconds but wake immediately if stop is requested."""
        deadline = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < deadline:
            if self._stop_requested.is_set() or rospy.is_shutdown():
                return
            rospy.sleep(0.05)

    def _drive_leg(self, direction):
        """
        Drive one leg in direction (+1 forward, -1 backward).
        Returns True if completed normally, False if interrupted.
        """
        rate       = rospy.Rate(CONTROL_RATE_HZ)
        start_time = rospy.Time.now()
        speed      = direction * self.linear_speed

        label = "-> Forward" if direction > 0 else "<- Backward"
        rospy.loginfo("[BaseControl] {}  {} m @ {:.2f} m/s".format(
            label, self.distance, abs(speed)))

        while not rospy.is_shutdown():
            if self._stop_requested.is_set():
                return False

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= self.leg_duration:
                return True   # leg completed normally

            self._publish_velocity(speed)
            rate.sleep()

        return False  # ROS shutdown

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        rospy.sleep(1.0)   # allow publisher/subscriber to register

        rospy.loginfo("[BaseControl] Waiting for PRINTING state ...")
        while not rospy.is_shutdown() and not self._print_was_active.is_set():
            rospy.sleep(0.2)

        if rospy.is_shutdown():
            return

        rospy.loginfo("[BaseControl] Printing detected — starting motion.")

        direction = 1
        leg_count = 0

        try:
            while not rospy.is_shutdown() and not self._stop_requested.is_set():
                leg_count += 1
                rospy.loginfo("[BaseControl] Leg {} ({})".format(
                    leg_count, 'FWD' if direction > 0 else 'BWD'))

                completed = self._drive_leg(direction)

                # Always stop the robot between legs (and on interruption)
                self._stop_robot()

                if not completed or self._stop_requested.is_set():
                    break

                # Interruptible pause between legs
                self._interruptible_sleep(0.5)

                if self._stop_requested.is_set():
                    break

                direction *= -1

        except rospy.ROSInterruptException:
            rospy.loginfo("[BaseControl] ROS interrupted.")
        finally:
            self._stop_robot()
            rospy.loginfo("[BaseControl] Node exiting.")


def main():
    try:
        node = BaseControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()