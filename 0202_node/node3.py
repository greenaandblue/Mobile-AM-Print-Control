#!/usr/bin/env python3
"""
gcode_sender_node.py

ROS node:
- Connects to 3D printer via Printrun
- Heats bed & nozzle first (blocking)
- Sends G-code line by line AFTER heating
"""

import rospy
import time
import configparser
from pathlib import Path
from threading import Lock, Event
from datetime import datetime

from printrun.printcore import printcore
from printrun.eventhandler import PrinterEventHandler

from std_msgs.msg import String, Bool, Int32, Float32
from std_srvs.srv import Trigger, TriggerResponse

CONFIG_FILE = "printer_config.ini"


# =========================================================
# Printer Event Handler
# =========================================================
class GcodeSenderHandler(PrinterEventHandler):
    def __init__(self):
        super().__init__()
        self.nozzle_temp = None
        self.nozzle_target = None
        self.bed_temp = None
        self.bed_target = None

    def on_recv(self, line):
        if "T:" in line and "B:" in line:
            self.parse_temperature(line)

    def parse_temperature(self, line):
        """
        Example:
        ok T:200.1 /200.0 B:60.0 /60.0
        """
        try:
            if "T:" in line:
                t_part = line.split("T:")[1]
                curr, target = t_part.split(" ")[0].split("/")
                self.nozzle_temp = float(curr)
                self.nozzle_target = float(target)

            if "B:" in line:
                b_part = line.split("B:")[1]
                curr, target = b_part.split(" ")[0].split("/")
                self.bed_temp = float(curr)
                self.bed_target = float(target)
        except Exception as e:
            rospy.logwarn(f"Temperature parse error: {e}")


# =========================================================
# G-code Sender Node
# =========================================================
class GcodeSenderNode:
    def __init__(self):
        rospy.init_node("gcode_sender_node")

        self.config = self.load_config()

        self.port = self.config["port"]
        self.baud = self.config["baud"]

        self.printer = None
        self.handler = GcodeSenderHandler()

        self.gcode_lines = []
        self.current_line = 0

        self.is_running = False
        self.heating_done = False

        self.lock = Lock()
        self.pause_event = Event()
        self.pause_event.set()

        # ---------------- Publishers ----------------
        self.pub_status = rospy.Publisher("/gcode_sender/status", String, queue_size=10)
        self.pub_progress = rospy.Publisher("/gcode_sender/progress", Float32, queue_size=10)
        self.pub_heating_done = rospy.Publisher("/gcode_sender/heating_done", Bool, queue_size=1)

        # ---------------- Subscribers ----------------
        rospy.Subscriber("/gcode_sender/load_file", String, self.load_file_cb)

        # ---------------- Services ----------------
        rospy.Service("/gcode_sender/connect", Trigger, self.srv_connect)
        rospy.Service("/gcode_sender/start", Trigger, self.srv_start)
        rospy.Service("/gcode_sender/stop", Trigger, self.srv_stop)
        rospy.Service("/gcode_sender/disconnect", Trigger, self.srv_disconnect)

        rospy.loginfo("G-code Sender Node ready")

    # =====================================================
    def load_config(self):
        cfg = configparser.ConfigParser()
        cfg.read(CONFIG_FILE)

        return {
            "port": cfg.get("printer", "port", fallback="/dev/ttyACM0"),
            "baud": cfg.getint("printer", "baud", fallback=115200),
        }

    # =====================================================
    def load_file_cb(self, msg):
        path = Path(msg.data)
        if not path.exists():
            rospy.logerr("G-code file not found")
            return

        with open(path, "r") as f:
            self.gcode_lines = [
                l.strip() for l in f
                if l.strip() and not l.startswith(";")
            ]

        self.current_line = 0
        rospy.loginfo(f"G-code loaded: {len(self.gcode_lines)} lines")

    # =====================================================
    def srv_connect(self, req):
        if self.printer and self.printer.online:
            return TriggerResponse(True, "Already connected")

        self.printer = printcore(self.port, self.baud)
        self.printer.addEventHandler(self.handler)

        timeout = time.time() + 10
        while not self.printer.online:
            if time.time() > timeout:
                return TriggerResponse(False, "Connection timeout")
            time.sleep(0.1)

        rospy.loginfo("Printer connected")
        return TriggerResponse(True, "Connected")

    # =====================================================
    def heat_and_wait(self, bed_temp=60, nozzle_temp=200):
        rospy.loginfo(f"Heating bed to {bed_temp}C")
        self.printer.send_now(f"M140 S{bed_temp}")
        self.printer.send_now(f"M190 S{bed_temp}")  # BLOCK

        rospy.loginfo("Bed heating done")

        rospy.loginfo(f"Heating nozzle to {nozzle_temp}C")
        self.printer.send_now(f"M104 S{nozzle_temp}")
        self.printer.send_now(f"M109 S{nozzle_temp}")  # BLOCK

        rospy.loginfo("Nozzle heating done")

        self.heating_done = True
        self.pub_heating_done.publish(True)
        self.pub_status.publish("Heating done")

    # =====================================================
    def srv_start(self, req):
        if not self.printer or not self.printer.online:
            return TriggerResponse(False, "Printer not connected")

        if not self.gcode_lines:
            return TriggerResponse(False, "No G-code loaded")

        self.heating_done = False
        self.is_running = True

        self.heat_and_wait()
        rospy.loginfo("Start printing")

        return TriggerResponse(True, "Heating done, printing started")

    # =====================================================
    def srv_stop(self, req):
        self.is_running = False
        return TriggerResponse(True, "Stopped")

    # =====================================================
    def srv_disconnect(self, req):
        if self.printer:
            self.printer.disconnect()
            self.printer = None
        return TriggerResponse(True, "Disconnected")

    # =====================================================
    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.is_running and self.heating_done:
                if self.current_line < len(self.gcode_lines):
                    line = self.gcode_lines[self.current_line]
                    self.printer.send_now(line)
                    self.current_line += 1

                    progress = 100.0 * self.current_line / len(self.gcode_lines)
                    self.pub_progress.publish(progress)

                else:
                    rospy.loginfo("Print finished")
                    self.is_running = False

            rate.sleep()


# =========================================================
def main():
    node = GcodeSenderNode()
    rospy.on_shutdown(lambda: rospy.loginfo("Node shutdown"))
    node.run()


if __name__ == "__main__":
    main()
