#!/usr/bin/env python3

"""
### FAIL DO NOT USE
printer_gcsender0120.py

Use file "Printer Control Test Commands" for terminal command reference

Slice multiple STLs and send to 3D printer using Printrun.
Preserves all original slicing and configuration functionality.
"""

import argparse
import configparser
import subprocess
import sys
import time
import os
from pathlib import Path
from datetime import datetime

from typing import Dict, List, Optional

from printrun_gcode_sender0120 import GCodeSender, PrinterState


CONFIG_FILE = "printer_config.ini"


# --------------------------------
# Configuration Management
# --------------------------------
def load_config() -> Dict:
    """Load configuration from printer_config.ini"""
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
# Slicing Functions (Preserved)
# --------------------------------
def slice_stl(config: Dict, output_filename: str) -> str:
    """
    Slice STL file(s) into G-code using configured slicer.
    Supports multiple STL files with individual centering and rotation.
    
    Args:
        config: Configuration dictionary
        output_filename: Output G-code filename
    
    Returns:
        Path to generated G-code file
    """
    working_dir = Path(config["working_dir"]).resolve()
    output_gcode = working_dir / output_filename

    profile_path = None
    if config.get("profile"):
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
        center_key = f"stl_file{idx+1}"
        if center_key in config["center_positions_raw"]:
            center = config["center_positions_raw"][center_key]
            x, y = center.split(",")
            cmd += ["--center", f"{x},{y}"]

        # Rotation if provided
        rotation_key = f"stl_file{idx+1}"
        if rotation_key in config["rotations"]:
            rotation = config["rotations"][rotation_key]
            rx, ry, rz = rotation.split(",")
            if rx.strip() != "0":
                cmd += ["--rotate-x", rx.strip()]
            if ry.strip() != "0":
                cmd += ["--rotate-y", ry.strip()]
            if rz.strip() != "0":
                cmd += ["--rotate", rz.strip()]

        cmd += ["--export-3mf", "-o", str(temp_3mf), str(stl_path)]

        print(f"[SLICE] Centering and rotating: {' '.join(cmd)}")
        try:
            subprocess.run(cmd, check=True)
            print(f"[SLICE] Temp 3MF created: {temp_3mf}")
        except subprocess.CalledProcessError as e:
            print(f"[ERROR] Slicing failed: {e}")
            return None

    # Step 2: Merge and slice to G-code
    cmd_final = [config["slicer"]]
    if profile_path:
        cmd_final += ["--load", profile_path]

    cmd_final += ["--merge", "--dont-arrange", "-g"] + temp_3mf_files
    cmd_final += ["--output", str(output_gcode)]

    print(f"[SLICE] Merging and slicing: {' '.join(cmd_final)}")
    try:
        subprocess.run(cmd_final, check=True)
        print(f"[SLICE] G-code complete: {output_gcode}")
        return str(output_gcode)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Final slicing failed: {e}")
        return None


# --------------------------------
# Printer Control Functions
# --------------------------------
class PrinterController:
    """
    Unified printer controller managing both slicing and printing.
    """
    
    def __init__(self, config: Dict):
        self.config = config
        self.sender = None
        self.current_gcode = None
        self.print_start_time = None
        
    def initialize(self) -> bool:
        """Initialize connection to printer."""
        self.sender = GCodeSender(
            port=self.config["port"],
            baud=self.config["baud"]
        )
        
        if self.sender.connect():
            print(f"[PRINTER] Connected to {self.config['port']} @ {self.config['baud']} baud")
            return True
        else:
            print("[ERROR] Failed to connect to printer")
            return False
    
    def close(self):
        """Close printer connection."""
        if self.sender:
            self.sender.disconnect()
            print("[PRINTER] Disconnected")
    
    def slice_and_print(self, output_filename: str = None, wait: bool = None) -> bool:
        """
        Slice STL files and immediately start printing.
        
        Args:
            output_filename: Custom output filename (uses config default if None)
            wait: Whether to wait for print completion (uses config default if None)
        
        Returns:
            True if successful
        """
        if not self.sender or not self.sender.is_online():
            print("[ERROR] Printer not connected")
            return False
        
        output_file = output_filename or self.config["output_file"]
        
        print("[PRINTER] Starting slicing process...")
        gcode_path = slice_stl(self.config, output_file)
        
        if not gcode_path:
            print("[ERROR] Slicing failed")
            return False
        
        self.current_gcode = gcode_path
        wait_for_finish = wait if wait is not None else self.config["wait"]
        
        return self.start_print(gcode_path, wait_for_finish)
    
    def start_print(self, gcode_file: str, wait: bool = None) -> bool:
        """
        Start printing a G-code file.
        
        Args:
            gcode_file: Path to G-code file
            wait: Whether to wait for completion (uses config default if None)
        
        Returns:
            True if successful
        """
        if not self.sender or not self.sender.is_online():
            print("[ERROR] Printer not connected")
            return False
        
        if not os.path.isfile(gcode_file):
            print(f"[ERROR] File not found: {gcode_file}")
            return False
        
        wait_for_finish = wait if wait is not None else self.config["wait"]
        
        print(f"[PRINT] Starting print: {gcode_file}")
        if not self.sender.start_print(gcode_file):
            print("[ERROR] Failed to start print")
            return False
        
        self.current_gcode = gcode_file
        self.print_start_time = time.time()
        
        if wait_for_finish:
            return self._monitor_print()
        else:
            print("[PRINT] Print started (not waiting for completion)")
            return True
    
    def _monitor_print(self) -> bool:
        """Monitor print progress and display status."""
        update_interval = self.config["status_update_interval"]
        last_update = time.time()
        
        print(f"[MONITOR] Waiting for print (status updates every {update_interval}s)...")
        
        try:
            while self.sender.printcore and self.sender.printcore.printing:
                now = time.time()
                
                if now - last_update >= update_interval:
                    progress = self.sender.get_progress()
                    
                    # Query temperatures for status
                    self.sender.query_temperatures()
                    time.sleep(0.1)
                    
                    status = (
                        f"[{progress['completion']:.1f}%] "
                        f"State: {progress['state']} | "
                    )
                    
                    if progress['bed_actual'] is not None:
                        status += f"Bed: {progress['bed_actual']:.1f}°C | "
                    if progress['nozzle_actual'] is not None:
                        status += f"Nozzle: {progress['nozzle_actual']:.1f}°C"
                    
                    print(status)
                    last_update = now
                
                time.sleep(0.5)
            
            elapsed = time.time() - self.print_start_time
            print(f"[PRINT] Complete! Elapsed time: {elapsed/60:.1f} minutes")
            return True
            
        except KeyboardInterrupt:
            print("\n[MONITOR] Interrupted by user")
            return False
        except Exception as e:
            print(f"[ERROR] Monitoring failed: {e}")
            return False
    
    def pause_print(self) -> bool:
        """Pause current print."""
        if not self.sender:
            print("[ERROR] Printer not connected")
            return False
        
        if self.sender.pause():
            print("[PRINT] Paused")
            return True
        else:
            print("[ERROR] Failed to pause")
            return False
    
    def resume_print(self) -> bool:
        """Resume paused print."""
        if not self.sender:
            print("[ERROR] Printer not connected")
            return False
        
        if self.sender.resume():
            print("[PRINT] Resumed")
            return True
        else:
            print("[ERROR] Failed to resume")
            return False
    
    def stop_print(self) -> bool:
        """Stop current print."""
        if not self.sender:
            print("[ERROR] Printer not connected")
            return False
        
        if self.sender.stop():
            print("[PRINT] Stopped")
            return True
        else:
            print("[ERROR] Failed to stop")
            return False
    
    def get_status(self) -> Dict:
        """Get current printer status."""
        if not self.sender:
            return {"status": "not_connected"}
        
        return {
            "connected": self.sender.is_online(),
            "state": self.sender.state.name,
            "progress": self.sender.get_progress()
        }
    
    def send_gcode(self, gcode: str) -> bool:
        """Send raw G-code command."""
        if not self.sender or not self.sender.is_online():
            print("[ERROR] Printer not connected")
            return False
        
        return self.sender.send_gcode_now(gcode)


# --------------------------------
# Command-line Interface
# --------------------------------
def create_parser() -> argparse.ArgumentParser:
    """Create command-line argument parser."""
    parser = argparse.ArgumentParser(
        description="Slice STL files and control 3D printer via Printrun",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EXAMPLES:
  python printer_stl.py slice-and-print
  python printer_stl.py print /path/to/file.gcode
  python printer_stl.py pause
  python printer_stl.py resume
  python printer_stl.py stop
  python printer_stl.py status
  python printer_stl.py gcode "M104 S200"
        """
    )
    
    subparsers = parser.add_subparsers(dest="command", help="Command to execute")
    
    # Slice and print
    subparsers.add_parser(
        "slice-and-print",
        help="Slice STL files and start printing immediately"
    ).add_argument("--no-wait", action="store_true", help="Don't wait for print to finish")
    
    # Print existing G-code
    print_parser = subparsers.add_parser("print", help="Print a G-code file")
    print_parser.add_argument("gcode_file", help="Path to G-code file")
    print_parser.add_argument("--no-wait", action="store_true", help="Don't wait for print to finish")
    
    # Pause
    subparsers.add_parser("pause", help="Pause current print")
    
    # Resume
    subparsers.add_parser("resume", help="Resume paused print")
    
    # Stop
    subparsers.add_parser("stop", help="Stop current print")
    
    # Status
    subparsers.add_parser("status", help="Get printer status")
    
    # Send raw G-code
    gcode_parser = subparsers.add_parser("gcode", help="Send raw G-code command")
    gcode_parser.add_argument("command", help="G-code command (e.g., 'M105', 'G28')")
    
    # Connect
    subparsers.add_parser("connect", help="Connect to printer")
    
    # Disconnect
    subparsers.add_parser("disconnect", help="Disconnect from printer")
    
    return parser


def main():
    """Main entry point."""
    config = load_config()
    controller = PrinterController(config)
    
    parser = create_parser()
    args = parser.parse_args()
    
    # Auto-connect unless disconnect command
    if args.command != "disconnect" and args.command is not None:
        if not controller.initialize():
            sys.exit(1)
    
    try:
        if args.command == "slice-and-print":
            wait = not args.no_wait
            controller.slice_and_print(wait=wait)
        
        elif args.command == "print":
            wait = not args.no_wait
            controller.start_print(args.gcode_file, wait=wait)
        
        elif args.command == "pause":
            controller.pause_print()
        
        elif args.command == "resume":
            controller.resume_print()
        
        elif args.command == "stop":
            controller.stop_print()
        
        elif args.command == "status":
            status = controller.get_status()
            print("[STATUS]")
            for key, value in status.items():
                print(f"  {key}: {value}")
        
        elif args.command == "gcode":
            if controller.send_gcode(args.command):
                print(f"[GCODE] Sent: {args.command}")
            else:
                print(f"[ERROR] Failed to send: {args.command}")
        
        elif args.command == "connect":
            print("[INFO] Already connected" if controller.sender and controller.sender.is_online() else "[INFO] Connected")
        
        elif args.command == "disconnect":
            controller.close()
        
        else:
            parser.print_help()
    
    finally:
        controller.close()


if __name__ == "__main__":
    main()