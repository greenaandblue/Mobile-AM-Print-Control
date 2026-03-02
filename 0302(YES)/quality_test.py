#!/usr/bin/env python3
"""
base_control_script.py
======================

Moves the KMR iiwa base back and forth, reversing direction every 3 metres,
and automatically stops as soon as the 3-D printer finishes (i.e. the
/gcode_sender/state topic transitions to IDLE).

Design notes
------------
* Speed, distance-per-leg, and update rate are all configurable at the top.
* Direction is reversed by negating linear_x — no angular component needed.
* The printer-state subscriber is the ONLY stop condition coming from outside;
  the motion loop also respects rospy.is_shutdown() for clean Ctrl-C exits.
* No changes are required in gcode_sender_node.py, motion_based_control.py,
  or test_gcode_sender.py.

Usage
-----
  # Make sure gcode_sender_node is already running and printing has started,
  # then in a separate terminal:
  python3 base_control_script.py

  # Or via rosrun:
  rosrun <your_pkg> base_control_script.py _linear_speed:=0.2 _distance:=3.0
"""



# TORUN
# Default (0.2 m/s, 3 m legs)
# python3 <filename>.py

# Custom speed via ROS param
# rosrun your_pkg base_control_script.py _linear_speed:=0.4 _distance:=3.0


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ─────────────────────────────────────────────────────────────────────────────
# Configuration  (can also be overridden via ROS private params)
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_LINEAR_SPEED = 0.2   # m/s  — choose from 0.2 / 0.4 / 0.6 / 0.8 / 1.0
DEFAULT_DISTANCE     = 3.0   # metres per leg before direction reversal
CONTROL_RATE_HZ      = 10    # Hz  — how often cmd_vel is published
CMD_VEL_TOPIC        = '/kmriiwa/base/command/cmd_vel'
PRINTER_STATE_TOPIC  = '/gcode_sender/state'


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class BaseControlNode:
    """
    Publishes cmd_vel commands that make the base travel 3 m forward,
    then 3 m backward, indefinitely — until the printer reports IDLE
    (print finished) or the node is shut down.
    """

    def __init__(self):
        rospy.init_node('base_control_script', anonymous=False)

        # Read configurable params (rosrun … _linear_speed:=0.4)
        self.linear_speed = rospy.get_param('~linear_speed', DEFAULT_LINEAR_SPEED)
        self.distance     = rospy.get_param('~distance',     DEFAULT_DISTANCE)

        # Derived: how long to drive at constant speed to cover self.distance
        self.leg_duration = self.distance / self.linear_speed   # seconds

        # Publisher
        self.pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)

        # Printer state (updated by subscriber callback)
        self._printer_state     = "UNKNOWN"
        self._print_was_active  = False   # latches True once we see PRINTING
        self._stop_requested    = False   # set when printer goes IDLE after print

        # Subscribe to printer state
        rospy.Subscriber(PRINTER_STATE_TOPIC, String, self._printer_state_cb)

        rospy.loginfo(
            f"[BaseControl] speed={self.linear_speed} m/s  "
            f"leg={self.distance} m  ({self.leg_duration:.1f} s/leg)"
        )
        rospy.loginfo(
            f"[BaseControl] Waiting for printer to start printing …  "
            f"(topic: {PRINTER_STATE_TOPIC})"
        )

    # ── Printer state callback ────────────────────────────────────────────────

    def _printer_state_cb(self, msg: String):
        state = msg.data.strip().upper()

        if state != self._printer_state:
            rospy.loginfo(f"[BaseControl] Printer state → {state}")
            self._printer_state = state

        # Latch: once we see PRINTING (or PAUSED), the job is considered active
        if state in ("PRINTING", "PAUSED"):
            self._print_was_active = True

        # Stop the robot only when the print job that was active is now done
        if self._print_was_active and state == "IDLE":
            rospy.loginfo("[BaseControl] Print finished — stopping robot.")
            self._stop_requested = True

    # ── Motion helpers ────────────────────────────────────────────────────────

    def _publish_velocity(self, linear_x: float):
        twist = Twist()
        twist.linear.x = linear_x
        self.pub.publish(twist)

    def _stop_robot(self):
        self._publish_velocity(0.0)
        rospy.loginfo("[BaseControl] Robot stopped.")

    def _drive_leg(self, direction: int) -> bool:
        """
        Drive one leg (self.distance metres) in the given direction (+1 or -1).
        Returns True if the leg completed normally, False if interrupted
        (stop requested or ROS shutdown).
        """
        rate       = rospy.Rate(CONTROL_RATE_HZ)
        start_time = rospy.Time.now()
        speed      = direction * self.linear_speed

        rospy.loginfo(
            f"[BaseControl] {'→ Forward' if direction > 0 else '← Backward'}  "
            f"{self.distance} m at {abs(speed):.2f} m/s …"
        )

        while not rospy.is_shutdown():
            # Check stop conditions every cycle
            if self._stop_requested:
                return False

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= self.leg_duration:
                return True   # leg finished normally

            self._publish_velocity(speed)
            rate.sleep()

        return False   # ROS shutdown

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        # Brief warm-up so subscriber and publisher fully register
        rospy.sleep(1.0)

        # Wait until the printer actually starts printing before moving
        wait_rate = rospy.Rate(2)
        while not rospy.is_shutdown() and not self._print_was_active:
            wait_rate.sleep()

        if rospy.is_shutdown():
            return

        rospy.loginfo("[BaseControl] Print detected — starting motion.")

        direction  = 1          # +1 = forward, −1 = backward
        leg_count  = 0

        try:
            while not rospy.is_shutdown() and not self._stop_requested:
                leg_count += 1
                rospy.loginfo(f"[BaseControl] Leg {leg_count}  (direction={'FWD' if direction>0 else 'BWD'})")

                completed = self._drive_leg(direction)

                # Brief pause between legs (robot comes to a clean stop)
                self._stop_robot()
                rospy.sleep(0.5)

                if not completed:
                    break   # stop requested or shutdown during this leg

                # Reverse direction for next leg
                direction *= -1

        except rospy.ROSInterruptException:
            rospy.loginfo("[BaseControl] ROS interrupted.")
        finally:
            self._stop_robot()
            rospy.loginfo("[BaseControl] Node exiting.")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    try:
        node = BaseControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()