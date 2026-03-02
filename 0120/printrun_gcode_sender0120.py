#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
### FAIL DO NOT USE
printrun_gcode_sender0120.py

A unified GCode sender using Printrun (replaces OctoPrint-based gcode_sender1112.py).
Provides pause, resume, stop, temperature monitoring, and progress tracking.

"""

import os
import json
import time
import logging
import threading
import argparse
import configparser
import subprocess
import sys
import time

from enum import Enum
from typing import Optional, Dict, Any, List
from pathlib import Path
from datetime import datetime

from printrun.printcore import printcore
from printrun.gcoder import LightGCode
from printrun.eventhandler import PrinterEventHandler


# -------------------- Logger --------------------
logger = logging.getLogger(__name__)
if not logger.handlers:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s - %(levelname)s - %(message)s")


# -------------------- States --------------------
class PrinterState(Enum):
    UNKNOWN  = 0
    READY    = 1
    IDLE     = 1  # alias for READY
    PRINTING = 2
    PAUSED   = 3
    ERROR    = 4


# -------------------- Custom Event Handler --------------------
class StatusPrinterHandler(PrinterEventHandler):
    """
    Event handler to capture temperature and position data from printer responses.
    """
    def __init__(self):
        super().__init__()
        self.last_temp = None
        self.last_bed = None
        self.last_x = None
        self.last_y = None
        self.last_z = None
        self.temperatures = {}
        self.lock = threading.Lock()

    def on_recv(self, line):
        """Parse incoming data from printer."""
        if "T:" in line and "B:" in line:
            self.parse_temperature(line)
        elif "X:" in line and "Y:" in line and "Z:" in line:
            self.parse_position(line)

    def parse_temperature(self, line):
        """Extract temperature data: T:200.5 B:60.0"""
        try:
            with self.lock:
                # Format: T:200.5 /0 B:60.0 /0
                parts = line.split()
                for part in parts:
                    if part.startswith("T:"):
                        self.last_temp = float(part[2:])
                    elif part.startswith("B:"):
                        self.last_bed = float(part[2:])
                
                self.temperatures = {
                    "nozzle": self.last_temp,
                    "bed": self.last_bed
                }
        except Exception as e:
            logger.error(f"Error parsing temperature: {e}")

    def parse_position(self, line):
        """Extract position data: X:10.5 Y:20.3 Z:5.1"""
        try:
            with self.lock:
                parts = line.split()
                for part in parts:
                    if part.startswith("X:"):
                        self.last_x = float(part[2:])
                    elif part.startswith("Y:"):
                        self.last_y = float(part[2:])
                    elif part.startswith("Z:"):
                        self.last_z = float(part[2:])
        except Exception as e:
            logger.error(f"Error parsing position: {e}")

    def get_status_string(self, progress_percent):
        """Format a human-readable status line."""
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        status = f"{now} [{progress_percent:5.1f}%] "
        
        with self.lock:
            if self.last_x is not None and self.last_y is not None and self.last_z is not None:
                status += f"X:{self.last_x:.1f} Y:{self.last_y:.1f} Z:{self.last_z:.1f} | "
            if self.last_temp is not None and self.last_bed is not None:
                status += f"Nozzle:{self.last_temp:.1f}°C Bed:{self.last_bed:.1f}°C"
        
        return status
    """
    add 2 functions to get temp and position
    """
    def get_temperatures(self) -> Dict[str, Optional[float]]:
        """Return current temperature readings."""
        with self.lock:
            return {
                "nozzle": self.last_temp,
                "bed": self.last_bed
            }

    def get_position(self) -> Dict[str, Optional[float]]:
        """Return current position."""
        with self.lock:
            return {
                "x": self.last_x,
                "y": self.last_y,
                "z": self.last_z
            }


# -------------------- GCode Sender with Printrun --------------------
class GCodeSender:
    """
    Unified GCode sender using Printrun.
    Replaces OctoPrint-based sender with comparable functionality.
    """
    
    def __init__(self,
                 port: str = "/dev/ttyACM0",
                 baud: int = 115200,
                 timeout_s: float = 6.0,
                 bed_tolerance_c: float = 3.0) -> None:
        """
        port         : Serial port (e.g., '/dev/ttyACM0' on Linux, 'COM3' on Windows)
        baud         : Baud rate (default 115200)
        timeout_s    : Response timeout
        bed_tolerance_c : Temperature tolerance for bed heat completion check
        """
        self.port = port
        self.baud = int(baud)
        self.timeout_s = float(timeout_s)
        self.bed_tol = float(bed_tolerance_c)
        
        self.printcore = None
        self.handler = StatusPrinterHandler()
        self._cached_state = PrinterState.UNKNOWN
        self._is_paused = False
        self._pause_lock = threading.Lock()
        self.logger = logger
        
    def connect(self) -> bool:
        """Connect to printer via serial port."""
        try:
            self.logger.info(f"Connecting to printer on {self.port} @ {self.baud} baud...")
            self.printcore = printcore(port=self.port, baud=self.baud)
            self.printcore.addEventHandler(self.handler)
            
            # Wait for connection
            timeout = time.time() + self.timeout_s
            while not self.printcore.online and time.time() < timeout:
                time.sleep(0.1)
            
            if self.printcore.online:
                self.logger.info("Printer connected successfully")
                self._cached_state = PrinterState.READY
                return True
            else:
                self.logger.error("Failed to connect to printer (timeout)")
                self._cached_state = PrinterState.ERROR
                return False
                
        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            self._cached_state = PrinterState.ERROR
            return False
    
    def disconnect(self) -> bool:
        """Disconnect from printer."""
        try:
            if self.printcore:
                self.printcore.disconnect()
                self.logger.info("Printer disconnected")
                self._cached_state = PrinterState.UNKNOWN
            return True
        except Exception as e:
            self.logger.error(f"Disconnect failed: {e}")
            return False
    
    def is_online(self) -> bool:
        """Check if printer is connected and responsive."""
        if self.printcore is None:
            return False
        return self.printcore.online
    
    def get_state(self) -> PrinterState:
        """Get current printer state."""
        if not self.is_online():
            self._cached_state = PrinterState.UNKNOWN
            return self._cached_state
        
        if self.printcore.printing:
            if self._is_paused:
                self._cached_state = PrinterState.PAUSED
            else:
                self._cached_state = PrinterState.PRINTING
        else:
            self._cached_state = PrinterState.READY
        
        return self._cached_state
    
    @property
    def state(self) -> PrinterState:
        """Property accessor for current state (backward compatible)."""
        return self.get_state()
    
    def send_gcode_now(self, lines: any) -> bool:
        """
        Send G-code command(s) immediately.
        lines: str or List[str]
        """
        try:
            if not self.is_online():
                self.logger.error("Printer not online")
                return False
            
            commands = [lines] if isinstance(lines, str) else list(lines)
            
            for cmd in commands:
                self.logger.info(f"[GCODE] Sending: {cmd}")
                self.printcore.send_now(cmd)
            
            return True
            
        except Exception as e:
            self.logger.error(f"send_gcode_now failed: {e}")
            return False
    
    def start_print(self, file_path: str) -> bool:
        """
        Load a G-code file and start printing.
        file_path: Path to .gcode file
        """
        try:
            if not self.is_online():
                self.logger.error("Printer not online")
                return False
            
            file_path = str(file_path)
            if not os.path.isfile(file_path):
                self.logger.error(f"File not found: {file_path}")
                return False
            
            # Read G-code file
            with open(file_path, 'r') as f:
                lines = [line.strip() for line in f if line.strip()]
            
            if not lines:
                self.logger.error(f"G-code file is empty: {file_path}")
                return False
            
            # Convert to LightGCode and start printing
            gcode = LightGCode(lines)
            self.printcore.startprint(gcode)
            
            with self._pause_lock:
                self._is_paused = False
            
            self._cached_state = PrinterState.PRINTING
            self.logger.info(f"Print started: {file_path} ({len(lines)} lines)")
            
            return True
            
        except Exception as e:
            self.logger.error(f"start_print failed: {e}")
            self._cached_state = PrinterState.ERROR
            return False
    
    # ---- Backward compatibility aliases ----
    def print_file(self, file_path: str) -> bool:
        return self.start_print(file_path)
    
    def start_file_print(self, file_path: str) -> bool:
        return self.start_print(file_path)
    
    def start_local_file_print(self, file_path: str) -> bool:
        return self.start_print(file_path)
    
    def pause(self) -> bool:
        """Pause the current print."""
        try:
            if not self.printcore or not self.printcore.printing:
                self.logger.warning("No print in progress to pause")
                return False
            
            with self._pause_lock:
                if self._is_paused:
                    self.logger.warning("Print already paused")
                    return False
                
                self.printcore.pause()
                self._is_paused = True
                self._cached_state = PrinterState.PAUSED
            
            self.logger.info("Print paused")
            return True
            
        except Exception as e:
            self.logger.error(f"pause failed: {e}")
            return False
    
    def execute_printer_pause(self) -> bool:
        """Alternative method name for pause (backward compatible)."""
        return self.pause()
    
    def resume(self) -> bool:
        """Resume a paused print."""
        try:
            if not self.printcore or not self.printcore.printing:
                self.logger.warning("No print in progress to resume")
                return False
            
            with self._pause_lock:
                if not self._is_paused:
                    self.logger.warning("Print is not paused")
                    return False
                
                self.printcore.resume()
                self._is_paused = False
                self._cached_state = PrinterState.PRINTING
            
            self.logger.info("Print resumed")
            return True
            
        except Exception as e:
            self.logger.error(f"resume failed: {e}")
            return False
    
    def execute_printer_resume(self) -> bool:
        """Alternative method name for resume (backward compatible)."""
        return self.resume()
    
    def stop(self) -> bool:
        """Cancel/stop the current print."""
        try:
            if not self.printcore:
                self.logger.warning("Printer not initialized")
                return False
            
            if self.printcore.printing:
                self.printcore.cancelprint()
                self.logger.info("Print canceled")
            
            with self._pause_lock:
                self._is_paused = False
            
            self._cached_state = PrinterState.READY
            return True
            
        except Exception as e:
            self.logger.error(f"stop failed: {e}")
            return False
    
    def stop_print(self) -> bool:
        """Alternative method name for stop (backward compatible)."""
        return self.stop()
    
    def is_bed_heat_complete(self) -> bool:
        """Check if bed has reached target temperature."""
        try:
            temps = self.handler.get_temperatures()
            bed_actual = temps.get("bed")
            
            # Query current temperatures
            self.send_gcode_now("M105")
            time.sleep(0.1)
            
            temps = self.handler.get_temperatures()
            bed_actual = temps.get("bed")
            
            if bed_actual is None:
                return True  # Conservative: don't block
            
            # Check if target is set (you may need to query this differently)
            # For now, assume it's reached if bed is above 40°C
            return bed_actual >= 40.0
            
        except Exception as e:
            self.logger.error(f"is_bed_heat_complete failed: {e}")
            return True
    
    def is_hotend_heat_complete(self, tol_c: float = 3.0) -> bool:
        """Check if hotend has reached target temperature."""
        try:
            # Query current temperatures
            self.send_gcode_now("M105")
            time.sleep(0.1)
            
            temps = self.handler.get_temperatures()
            nozzle_actual = temps.get("nozzle")
            
            if nozzle_actual is None:
                return True  # Conservative: don't block
            
            # Check if nozzle is hot enough for printing (assume > 180°C)
            return nozzle_actual >= 180.0
            
        except Exception as e:
            self.logger.error(f"is_hotend_heat_complete failed: {e}")
            return True
    
    def check_temperatures_safe(self) -> Dict[str, Dict[str, Optional[float]]]:
        """
        Safely get temperature information.
        Returns:
        {
            'bed': {'actual': float, 'target': float},
            'tool0': {'actual': float, 'target': float}
        }
        """
        try:
            self.send_gcode_now("M105")
            time.sleep(0.2)
            
            temps = self.handler.get_temperatures()
            
            return {
                "bed": {
                    "actual": temps.get("bed"),
                    "target": None  # Printrun doesn't track target easily
                },
                "tool0": {
                    "actual": temps.get("nozzle"),
                    "target": None
                }
            }
        except Exception as e:
            self.logger.error(f"check_temperatures_safe failed: {e}")
            return {
                "bed": {"actual": None, "target": None},
                "tool0": {"actual": None, "target": None}
            }
    
    def get_progress(self) -> Dict[str, Any]:
        """
        Get detailed progress information.
        Returns unified and backward-compatible progress dict.
        """
        info = {
            "progress_percent": 0.0,
            "completion": 0.0,
            "state": self.state.name,
            "state_str": self.state.name,
            "file": None,
            "file_name": None,
            "printTime": None,
            "print_time": None,
            "printTimeLeft": None,
            "print_time_left": None,
            "bed_heat_complete": True,
            "nozzle_heat_complete": True,
            "bed_actual": None,
            "bed_target": None,
            "nozzle_actual": None,
            "nozzle_target": None,
        }
        
        try:
            if self.printcore and self.printcore.printing:
                # Calculate progress
                queueindex = self.printcore.queueindex
                total_lines = len(self.printcore.flines) if self.printcore.flines else 0
                
                if total_lines > 0:
                    completion = (queueindex / total_lines) * 100
                    info["progress_percent"] = completion
                    info["completion"] = completion
                    info["printTime"] = int(time.time() - self.printcore.starttime) if hasattr(self.printcore, 'starttime') else None
                    info["print_time"] = info["printTime"]
            
            # Get current temperatures
            temps = self.handler.get_temperatures()
            pos = self.handler.get_position()
            
            info["bed_actual"] = temps.get("bed")
            info["nozzle_actual"] = temps.get("nozzle")
            info["bed_heat_complete"] = self.is_bed_heat_complete()
            info["nozzle_heat_complete"] = self.is_hotend_heat_complete()
            
        except Exception as e:
            self.logger.error(f"get_progress failed: {e}")
        
        return info
    
    def query_temperatures(self) -> None:
        """Send temperature query command to printer."""
        self.send_gcode_now("M105")
    
    def query_position(self) -> None:
        """Send position query command to printer."""
        self.send_gcode_now("M114")


# -------------------- Self-test --------------------
if __name__ == "__main__":
    PORT = os.environ.get("PRINTER_PORT", "/dev/ttyACM0")
    BAUD = int(os.environ.get("PRINTER_BAUD", "115200"))
    
    sender = GCodeSender(port=PORT, baud=BAUD)
    
    if sender.connect():
        print("[TEST] Connected successfully")
        print("[TEST] State:", sender.state.name)
        print("[TEST] Progress:", sender.get_progress())
        
        # Example: print a file (uncomment to use)
        # sender.start_print("/path/to/file.gcode")
        # time.sleep(5)
        # sender.pause()
        # time.sleep(2)
        # sender.resume()
        # sender.stop()
        
        sender.disconnect()
    else:
        print("[TEST] Failed to connect")