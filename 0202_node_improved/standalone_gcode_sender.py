#!/usr/bin/env python3

"""
standalone_gcode_sender.py

独立的 G-code 发送器，不需要 ROS
支持暂停、恢复、停止功能
可以独立运行或集成到其他项目中
"""

import sys
import time
import configparser
from pathlib import Path
from datetime import datetime
from threading import Thread, Lock, Event
import signal

from printrun.printcore import printcore
from printrun.gcoder import LightGCode
from printrun.eventhandler import PrinterEventHandler


# --------------------------------
# Status Handler
# --------------------------------
class PrinterStatusHandler(PrinterEventHandler):
    """处理打印机状态"""
    
    def __init__(self):
        super().__init__()
        self.last_temp = None
        self.last_bed = None
        self.last_x = None
        self.last_y = None
        self.last_z = None

    def on_recv(self, line):
        """接收打印机返回数据"""
        if "T:" in line and "B:" in line:
            self.parse_temperature(line)
        elif "X:" in line and "Y:" in line and "Z:" in line:
            self.parse_position(line)

    def parse_temperature(self, line):
        """解析温度"""
        try:
            nozzle_part = line.split("T:")[1]
            nozzle_temp = nozzle_part.split(" ")[0]
            bed_part = line.split("B:")[1]
            bed_temp = bed_part.split(" ")[0]
            self.last_temp = nozzle_temp
            self.last_bed = bed_temp
        except:
            pass

    def parse_position(self, line):
        """解析位置"""
        try:
            parts = line.split()
            self.last_x = parts[0].split(":")[1]
            self.last_y = parts[1].split(":")[1]
            self.last_z = parts[2].split(":")[1]
        except:
            pass

    def get_status(self, current_line, total_lines):
        """获取状态字符串"""
        progress = (current_line / total_lines * 100) if total_lines > 0 else 0
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        status = f"{now} [{progress:5.1f}%] Line {current_line}/{total_lines}"
        
        if self.last_x and self.last_y and self.last_z:
            status += f" | X:{self.last_x} Y:{self.last_y} Z:{self.last_z}"
        
        if self.last_temp and self.last_bed:
            status += f" | Nozzle:{self.last_temp}°C Bed:{self.last_bed}°C"
        
        return status


# --------------------------------
# Standalone G-code Sender
# --------------------------------
class StandaloneGcodeSender:
    """独立的 G-code 发送器"""
    
    def __init__(self, port, baud=115200, update_interval=5):
        self.port = port
        self.baud = baud
        self.update_interval = update_interval
        
        self.printer = None
        self.handler = PrinterStatusHandler()
        
        self.gcode_lines = []
        self.total_lines = 0
        
        self.is_running = False
        self.is_paused = False
        self.should_stop = False
        self.lock = Lock()
        
        self.last_status_time = time.time()
        
        print(f"\n{'=' * 70}")
        print("  Standalone G-code Sender (No ROS)")
        print(f"{'=' * 70}")
        print(f"Port: {self.port}, Baud: {self.baud}")

    def connect(self):
        """连接打印机"""
        try:
            print(f"\n[*] Connecting to printer on {self.port} @ {self.baud} baud...")
            self.printer = printcore(port=self.port, baud=self.baud)
            self.printer.addEventHandler(self.handler)
            
            # 等待连接
            timeout = 10
            start_time = time.time()
            while not self.printer.online:
                if time.time() - start_time > timeout:
                    print("❌ Connection timeout")
                    return False
                time.sleep(0.1)
            
            print("✓ Printer connected successfully")
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        if self.printer:
            try:
                self.printer.disconnect()
            except:
                pass
            self.printer = None
            print("✓ Disconnected")

    def load_file(self, filepath):
        """加载 G-code 文件"""
        try:
            path = Path(filepath)
            if not path.exists():
                print(f"❌ File not found: {filepath}")
                return False
            
            with open(path, 'r') as f:
                self.gcode_lines = [line.strip() for line in f 
                                   if line.strip() and not line.strip().startswith(';')]
            
            self.total_lines = len(self.gcode_lines)
            print(f"\n[*] Loaded G-code file: {filepath}")
            print(f"    Total lines: {self.total_lines}")
            return True
            
        except Exception as e:
            print(f"❌ Error loading file: {e}")
            return False

    def start(self):
        """开始打印"""
        with self.lock:
            if self.is_running:
                print("❌ Already running")
                return False
            
            if not self.printer or not self.printer.online:
                print("❌ Printer not connected")
                return False
            
            if self.total_lines == 0:
                print("❌ No G-code loaded")
                return False
            
            self.is_running = True
            self.is_paused = False
            self.should_stop = False
            
            print(f"\n[*] Starting print with {self.total_lines} lines...")
            
            try:
                self.printer.startprint(LightGCode(self.gcode_lines))
                print("✓ Print started")
                return True
            except Exception as e:
                print(f"❌ Failed to start: {e}")
                self.is_running = False
                return False

    def pause(self):
        """暂停打印"""
        with self.lock:
            if not self.is_running:
                print("❌ Not running")
                return False
            
            if self.is_paused:
                print("❌ Already paused")
                return False
            
            self.is_paused = True
            
            if self.printer and self.printer.printing:
                self.printer.pause()
            
            current_line = self.printer.queueindex if self.printer else 0
            print(f"✓ Paused at line {current_line}/{self.total_lines}")
            return True

    def resume(self):
        """恢复打印"""
        with self.lock:
            if not self.is_running:
                print("❌ Not running")
                return False
            
            if not self.is_paused:
                print("❌ Not paused")
                return False
            
            self.is_paused = False
            
            if self.printer and self.printer.printing:
                self.printer.resume()
            
            current_line = self.printer.queueindex if self.printer else 0
            print(f"✓ Resumed from line {current_line}/{self.total_lines}")
            return True

    def stop(self):
        """停止打印"""
        with self.lock:
            if not self.is_running:
                print("❌ Not running")
                return False
            
            self.should_stop = True
            
            if self.printer:
                self.printer.cancelprint()
            
            print("✓ Stop requested")
            return True

    def update_status(self):
        """更新状态"""
        now = time.time()
        if now - self.last_status_time >= self.update_interval and self.is_running:
            if self.printer and self.printer.online:
                self.printer.send_now("M105")
                self.printer.send_now("M114")
            
            current_line = self.printer.queueindex if self.printer else 0
            status = self.handler.get_status(current_line, self.total_lines)
            print(f"[Status] {status}")
            
            self.last_status_time = now

    def run_interactive(self):
        """交互式运行"""
        print(f"\n{'=' * 70}")
        print("  Interactive Mode")
        print(f"{'=' * 70}")
        print("""
Commands:
    p - Pause
    r - Resume
    s - Stop
    q - Quit

Printer is running. Enter command:
        """)
        
        # 启动监控线程
        monitor_thread = Thread(target=self._monitor, daemon=True)
        monitor_thread.start()
        
        try:
            while True:
                cmd = input("> ").strip().lower()
                
                if cmd == 'p':
                    self.pause()
                elif cmd == 'r':
                    self.resume()
                elif cmd == 's':
                    self.stop()
                elif cmd == 'q':
                    print("\nStopping and disconnecting...")
                    self.stop()
                    time.sleep(1)
                    self.disconnect()
                    print("✓ Exiting\n")
                    break
                elif cmd == '':
                    pass
                else:
                    print("❌ Invalid command")
                
                # 检查打印是否完成
                if self.is_running and self.printer and not self.printer.printing:
                    with self.lock:
                        if self.should_stop:
                            print("Print stopped")
                        else:
                            print("Print completed!")
                        self.is_running = False
                        self.should_stop = False
                        break
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
            self.stop()
            time.sleep(1)
            self.disconnect()

    def _monitor(self):
        """监控线程"""
        while self.is_running:
            self.update_status()
            time.sleep(1)

    def run_auto(self):
        """自动运行（不交互）"""
        print("\n[*] Running in auto mode...")
        print("    Waiting for print to complete...")
        
        while self.is_running:
            self.update_status()
            
            if self.printer and not self.printer.printing:
                with self.lock:
                    self.is_running = False
                    if self.should_stop:
                        print("Print stopped")
                    else:
                        print("Print completed!")
            
            time.sleep(0.5)


# --------------------------------
# Command Line Interface
# --------------------------------
def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Standalone G-code sender for 3D printers"
    )
    parser.add_argument("gcode_file", help="Path to G-code file")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--auto", action="store_true", help="Auto mode (non-interactive)")
    parser.add_argument("--update-interval", type=int, default=5, help="Status update interval in seconds")
    
    args = parser.parse_args()
    
    # 创建发送器
    sender = StandaloneGcodeSender(args.port, args.baud, args.update_interval)
    
    # 信号处理
    def signal_handler(sig, frame):
        print("\n\nInterrupted by user")
        sender.stop()
        time.sleep(1)
        sender.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # 连接
        if not sender.connect():
            return 1
        
        # 加载文件
        if not sender.load_file(args.gcode_file):
            sender.disconnect()
            return 1
        
        # 开始打印
        if not sender.start():
            sender.disconnect()
            return 1
        
        # 运行
        if args.auto:
            sender.run_auto()
        else:
            sender.run_interactive()
        
        return 0
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sender.disconnect()
        return 1


if __name__ == "__main__":
    sys.exit(main())
