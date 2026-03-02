#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
gcode_sender_printrun.py

PRINTRUN-based G-code sender (script-style).
Run directly from terminal, NOT a ROS node.
"""

import time
import argparse
import logging
from enum import Enum
from typing import Dict

from printrun.printcore import printcore
from printrun import gcoder


# -------------------- Logger --------------------
logger = logging.getLogger(__name__)
if not logger.handlers:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s"
    )


# -------------------- States --------------------
class PrinterState(Enum):
    UNKNOWN  = 0
    READY    = 1
    IDLE     = 1
    PRINTING = 2
    PAUSED   = 3
    ERROR    = 4


# -------------------- Sender --------------------
class GCodeSender:
    def __init__(self,
                 port: str,
                 baudrate: int,
                 firmware_mode: str = "prusa",
                 temp_tolerance_c: float = 3.0) -> None:

        self.port = port
        self.baudrate = baudrate
        self.firmware_mode = firmware_mode.lower()
        self.temp_tol = temp_tolerance_c

        self.printer = printcore(self.port, self.baudrate)
        self.logger = logger
        self._cached_state = PrinterState.UNKNOWN

        self._wait_for_connection()

    # ---------- Connection ----------
    def _wait_for_connection(self) -> None:
        self.logger.info("Connecting to printer...")
        while not self.printer.online:
            time.sleep(0.1)
        self.logger.info("Printer connected.")
        self._cached_state = PrinterState.READY

    # ---------- State ----------
    def get_state(self) -> PrinterState:
        if self.printer.printing:
            self._cached_state = PrinterState.PRINTING
        else:
            self._cached_state = PrinterState.READY
        return self._cached_state

    # ---------- Immediate G-code ----------
    def send_gcode_now(self, lines) -> bool:
        try:
            cmds = [lines] if isinstance(lines, str) else list(lines)
            for cmd in cmds:
                self.printer.send(cmd.strip())
            return True
        except Exception as e:
            self.logger.error(f"[GCODE] send_gcode_now failed: {e}")
            return False

    # ---------- Firmware pause / resume ----------
    def pause(self) -> bool:
        if self.firmware_mode == "prusa":
            return self.send_gcode_now("M601")
        return True

    def resume(self) -> bool:
        if self.firmware_mode == "prusa":
            return self.send_gcode_now("M602")
        return True

    # ---------- Temperature ----------
    def _temps(self) -> Dict[str, Dict[str, float]]:
        t = self.printer.tempcb or {}
        return {
            "bed": t.get("bed", {"current": 0.0, "target": 0.0}),
            "tool0": t.get("tool0", {"current": 0.0, "target": 0.0})
        }

    def _bed_ready(self) -> bool:
        t = self._temps()["bed"]
        if t["target"] <= 0:
            return True
        return abs(t["current"] - t["target"]) <= self.temp_tol

    def _nozzle_ready(self) -> bool:
        t = self._temps()["tool0"]
        if t["target"] <= 0:
            return True
        return abs(t["current"] - t["target"]) <= self.temp_tol

    def wait_for_heating(self) -> None:
        self.logger.info("Waiting for bed and nozzle heating...")
        while True:
            temps = self._temps()
            self.logger.info(
                f"Bed {temps['bed']['current']:.1f}/{temps['bed']['target']:.1f} | "
                f"Nozzle {temps['tool0']['current']:.1f}/{temps['tool0']['target']:.1f}"
            )

            if self._bed_ready() and self._nozzle_ready():
                self.logger.info("Heating complete.")
                return

            time.sleep(0.5)

    # ---------- Print ----------
    def start_print(self, gcode_path: str) -> None:
        with open(gcode_path, "r") as f:
            gcode = gcoder.GCode(f.readlines())

        self.wait_for_heating()
        self.logger.info("Starting G-code streaming...")

        for line in gcode.lines:
            if not line or not line.raw:
                continue

            while self.printer.printing:
                time.sleep(0.05)

            self.printer.send(line.raw.strip())
            time.sleep(0.01)

        while self.printer.printing:
            time.sleep(0.1)

        self.logger.info("G-code transmission completed.")


# -------------------- main() --------------------
def main():
    parser = argparse.ArgumentParser(
        description="PRINTRUN-based G-code sender (script mode)"
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--gcode", required=True, help="Path to G-code file")
    parser.add_argument("--firmware", default="prusa",
                        choices=["prusa", "marlin", "none"],
                        help="Firmware type")

    args = parser.parse_args()

    sender = GCodeSender(
        port=args.port,
        baudrate=args.baudrate,
        firmware_mode=args.firmware
    )

    sender.start_print(args.gcode)


if __name__ == "__main__":
    main()
