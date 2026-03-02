#!/usr/bin/env python3

"""
printer_stl.py

"""

import argparse
import configparser
import subprocess
import sys
import time
from pathlib import Path
from datetime import datetime

from printrun.printcore import printcore
from printrun.gcoder import LightGCode
from printrun.eventhandler import PrinterEventHandler

CONFIG_FILE = "printer_config.ini"

# --------------------------------
# Custom Status Event Handler
# --------------------------------
class StatusPrinterHandler(PrinterEventHandler):
    def __init__(self):
        super().__init__()
        self.last_temp = None
        self.last_bed = None
        self.last_x = None
        self.last_y = None
        self.last_z = None

    def on_recv(self, line):
        if "T:" in line and "B:" in line:
            self.parse_temperature(line)
        elif "X:" in line and "Y:" in line and "Z:" in line:
            self.parse_position(line)

    def parse_temperature(self, line):
        try:
            nozzle_part = line.split("T:")[1]
            nozzle_temp = nozzle_part.split(" ")[0]
            bed_part = line.split("B:")[1]
            bed_temp = bed_part.split(" ")[0]
            self.last_temp = nozzle_temp
            self.last_bed = bed_temp
        except Exception as e:
            print(f"Error parsing temperature: {e}")

    def parse_position(self, line):
        try:
            parts = line.split()
            self.last_x = parts[0].split(":")[1]
            self.last_y = parts[1].split(":")[1]
            self.last_z = parts[2].split(":")[1]
        except Exception as e:
            print(f"Error parsing position: {e}")

    def get_status_string(self, progress_percent):
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        status = f"{now} [{progress_percent:5.1f}%] "
        if self.last_x and self.last_y and self.last_z:
            status += f"X:{self.last_x} Y:{self.last_y} Z:{self.last_z} | "
        if self.last_temp and self.last_bed:
            status += f"Nozzle:{self.last_temp}°C Bed:{self.last_bed}°C"
        return status

# --------------------------------
# Load Config
# --------------------------------
def load_config():
    config = configparser.ConfigParser()
    config.read(CONFIG_FILE)

    stl_files = []
    if config.has_section("input_print_files"):
        for key in sorted(config["input_print_files"].keys()):
            stl_files.append(config["input_print_files"][key])

    return {
        "port": config.get("printer", "port", fallback="/dev/ttyACM0"),
        "baud": config.getint("printer", "baud", fallback=115200),
        "wait": config.getboolean("printer", "wait", fallback=True),
        "slicer": config.get("paths", "slicer", fallback="prusa-slicer-console"),
        "working_dir": config.get("paths", "working_dir", fallback="."),
        "profile": config.get("paths", "profile", fallback=None),
        "input_stls": stl_files,
        "output_file": config.get("output_print_file", "output_file", fallback="print_file.gcode"),
        "status_update_interval": config.getint("printer", "status_update_interval", fallback=10),
        "auto_arrange": config.getboolean("printer", "auto_arrange", fallback=True),
        "center_positions_raw": dict(config["center_positions"]) if config.has_section("center_positions") else {},
        "rotations": dict(config["rotations"]) if config.has_section("rotations") else {},
    }

# --------------------------------
# Slice STL(s) into G-code
# --------------------------------
# --------------------------------
def slice_stl(config, output_filename: str) -> str:
    working_dir = Path(config["working_dir"]).resolve()
    output_gcode = working_dir / output_filename

    profile_path = None
    if "profile" in config:
        profile_path = str(working_dir / config["profile"])

    temp_3mf_files = []

    # Step 1: Export each STL individually with center and rotation
    for idx, stl in enumerate(config["input_stls"]):
        stl_path = working_dir / stl
        temp_3mf = working_dir / f"temp_{idx+1}.3mf"
        temp_3mf_files.append(str(temp_3mf))

        cmd = [config["slicer"]]
        if profile_path:
            cmd += ["--load", profile_path]

        # Centering if provided
        center = config["center_positions_raw"].get(f"stl_file{idx+1}", None)
        if center:
            x, y = center.split(",")
            cmd += ["--center", f"{x},{y}"]

        # Rotation if provided
        rotation = config.get("rotations", {}).get(f"stl_file{idx+1}", None)
        if rotation:
            rx, ry, rz = rotation.split(",")
            if rx.strip() != "0":
                cmd += ["--rotate-x", rx.strip()]
            if ry.strip() != "0":
                cmd += ["--rotate-y", ry.strip()]
            if rz.strip() != "0":
                cmd += ["--rotate", rz.strip()]  # rotation around Z

        cmd += ["--export-3mf", "-o", str(temp_3mf), str(stl_path)]

        print(f"Centering and rotating export: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
        print(f"Temp 3MF created: {temp_3mf}")

    # Step 2: Merge and slice
    cmd_final = [config["slicer"]]
    if profile_path:
        cmd_final += ["--load", profile_path]

    cmd_final += ["--merge", "--dont-arrange", "-g"] + temp_3mf_files
    cmd_final += ["--output", str(output_gcode)]

    print(f"Merging and slicing directly: {' '.join(cmd_final)}")
    subprocess.run(cmd_final, check=True)
    print(f"G-code slicing complete: {output_gcode}")

    return str(output_gcode)

# --------------------------------
# Connect and Send G-code to Printer
# --------------------------------
def send_gcode_to_printer(gcode_path: str, port: str, baud: int, wait_for_finish: bool, update_interval: int):
    print(f"Connecting to printer on {port} @ {baud} baud…")
    p = printcore(port=port, baud=baud)
    handler = StatusPrinterHandler()
    p.addEventHandler(handler)

    while not p.online:
        time.sleep(0.1)
    print("Printer online")

    print(f"Starting print job from: {gcode_path}")
    lines = [line.strip() for line in open(gcode_path) if line.strip()]
    total_lines = len(lines)
    p.startprint(LightGCode(lines))

    if wait_for_finish:
        print(f"Waiting for print to finish… Status updates every {update_interval} seconds.")
        last_query_time = time.time()

        while p.printing:
            now = time.time()

            if now - last_query_time >= update_interval:
                p.send_now("M105")
                p.send_now("M114")

                try:
                    lines_sent = p.queueindex
                    progress = (lines_sent / total_lines) * 100
                    status_string = handler.get_status_string(progress)
                    print(status_string)
                except Exception as e:
                    print(f"Error calculating progress: {e}")

                last_query_time = now

            time.sleep(0.5)

        print("Print complete!")
    else:
        print("Print started — not waiting.")

# --------------------------------
# Main Entry Point
# --------------------------------
def main():
    config = load_config()

    parser = argparse.ArgumentParser(description="Slice multiple STLs and send to your 3D printer.")
    parser.add_argument("--no-wait", action="store_true", help="Don't wait for print to finish")

    args = parser.parse_args()

    port = config["port"]
    baud = config["baud"]
    wait = not args.no_wait if args.no_wait else config["wait"]
    update_interval = config["status_update_interval"]
    output_filename = config["output_file"]

    gcode_path = slice_stl(config, output_filename)
    send_gcode_to_printer(gcode_path, port, baud, wait, update_interval)

if __name__ == "__main__":
    #main
    
    config = load_config()

    port = config["port"]
    baud = config["baud"]
    wait = config["wait"]
    update_interval = config["status_update_interval"]

    send_gcode_to_printer('testing/cutbox_10mm.gcode', port, baud, wait, update_interval)
