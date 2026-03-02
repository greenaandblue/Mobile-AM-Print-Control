#!/usr/bin/env python3
"""
test_gcode_sender.py

Simple test client for gcode_sender_node
"""

import rospy
import sys
from std_msgs.msg import String
from std_srvs.srv import Trigger


def call_srv(name):
    rospy.wait_for_service(name)
    srv = rospy.ServiceProxy(name, Trigger)
    res = srv()
    print(f"{name}: {res.message}")
    return res.success


def main():
    rospy.init_node("gcode_sender_test")

    pub = rospy.Publisher("/gcode_sender/load_file", String, queue_size=1)
    rospy.sleep(1)

    if len(sys.argv) < 2:
        print("Usage: rosrun pkg test_gcode_sender.py file.gcode")
        return

    gcode_file = sys.argv[1]
    print(f"Loading G-code: {gcode_file}")
    pub.publish(gcode_file)
    rospy.sleep(1)

    if not call_srv("/gcode_sender/connect"):
        return

    if not call_srv("/gcode_sender/start"):
        return

    print("\nPrinting started")
    print("Press Ctrl+C to stop")

    rospy.spin()


if __name__ == "__main__":
    main()
