#!/usr/bin/env python3
"""

G-code sender script using Printrun (printcore).

This script:
1. Reads a G-code file line by line
2. Sends G-code commands sequentially
3. Waits when the printer is busy
4. Waits for bed and nozzle heating to finish before printing
5. Prints status feedback to the terminal

This is a SCRIPT, not a ROS node.
"""

import time
import sys
from printrun.printcore import printcore
from printrun import gcoder


# =========================
# Configuration
# =========================

SERIAL_PORT = "/dev/ttyACM0"     # Update based on your printer # /dev/ttyACM0
BAUDRATE = 115200
GCODE_FILE = "/path/to/your/file.gcode" ####

CHECK_INTERVAL = 0.5             # Seconds between status checks
TEMP_TOLERANCE = 2.0             # Allowed temperature error (°C)


# =========================
# Helper functions
# =========================

def wait_for_connection(p):
    """Wait until printer connection is established."""
    print("Connecting to printer...")
    while not p.online:
        time.sleep(0.1)
    print("Printer connected.")


def wait_for_heating(p):
    """
    Wait until bed and nozzle reach target temperatures.
    Assumes temperature commands are already in the G-code.
    """
    print("Waiting for bed and nozzle to heat up...")

    while True:
        temps = p.tempcb
        if not temps:
            time.sleep(CHECK_INTERVAL)
            continue

        try:
            bed_target = temps["bed"]["target"]
            bed_current = temps["bed"]["current"]
            hotend_target = temps["tool0"]["target"]
            hotend_current = temps["tool0"]["current"]
        except KeyError:
            time.sleep(CHECK_INTERVAL)
            continue

        bed_ready = abs(bed_current - bed_target) <= TEMP_TOLERANCE
        hotend_ready = abs(hotend_current - hotend_target) <= TEMP_TOLERANCE

        print(
            f"Bed: {bed_current:.1f}/{bed_target:.1f} °C | "
            f"Nozzle: {hotend_current:.1f}/{hotend_target:.1f} °C",
            end="\r"
        )

        if bed_ready and hotend_ready:
            print("\nHeating complete. Starting G-code transmission.")
            return

        time.sleep(CHECK_INTERVAL)


def wait_until_idle(p):
    """Block until the printer is idle."""
    while p.printing:
        time.sleep(CHECK_INTERVAL)


# =========================
# Main logic
# =========================

def main():
    # Initialize printcore
    printer = printcore(SERIAL_PORT, BAUDRATE)

    # Wait for serial connection
    wait_for_connection(printer)

    # Load G-code file
    try:
        with open(GCODE_FILE, "r") as f:
            raw_gcode = f.readlines()
    except IOError:
        print("Failed to open G-code file.")
        sys.exit(1)

    gcode = gcoder.GCode(raw_gcode)

    print(f"G-code file loaded: {GCODE_FILE}")
    print(f"Total lines: {len(gcode.lines)}")

    # Start temperature monitoring
    wait_for_heating(printer)

    # Send G-code line by line
    print("Sending G-code...")

    for line in gcode.lines:
        if not line:
            continue

        wait_until_idle(printer)

        printer.send(line.raw)
        print(f"Sent: {line.raw.strip()}")

        time.sleep(0.01)  # Small delay to avoid flooding

    # Final wait until printer finishes last command
    wait_until_idle(printer)

    print("G-code transmission completed successfully.")


if __name__ == "__main__":
    main()
