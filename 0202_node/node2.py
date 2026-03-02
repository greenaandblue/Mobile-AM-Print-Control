#!/usr/bin/env python3

"""
gcode_sender_node.py (增强版 v2)

ROS节点：智能G-code发送
新功能：
1. 等待热床和喷嘴加热完成后再开始打印
2. 检测打印机空闲状态
3. 完整的错误处理机制
4. 详细的日志输出
5. 从暂停点恢复
"""

import rospy
import configparser
import time
from pathlib import Path
from datetime import datetime
from threading import Lock, Thread, Event
from enum import Enum

from printrun.printcore import printcore
from printrun.gcoder import LightGCode

from std_msgs.msg import String, Bool, Int32, Float32
from std_srvs.srv import Trigger, TriggerResponse

CONFIG_FILE = "printer_config.ini"


class PrinterState(Enum):
    """打印机状态枚举"""
    DISCONNECTED = "disconnected"
    IDLE = "idle"
    HEATING = "heating"
    PRINTING = "printing"
    PAUSED = "paused"
    ERROR = "error"


# --------------------------------
# G-code Sender Node (增强版)
# --------------------------------
class GcodeSenderNode:
    """ROS节点：智能G-code发送"""
    
    def __init__(self):
        rospy.init_node('gcode_sender_node', anonymous=False)
        
        # 加载配置
        self.config = self.load_config()
        
        # 打印机连接参数
        self.port = self.config["port"]
        self.baud = self.config["baud"]
        self.printer = None
        
        # 状态数据
        self.nozzle_temp = 0.0
        self.nozzle_target = 0.0
        self.bed_temp = 0.0
        self.bed_target = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        
        # 打印机状态
        self.printer_state = PrinterState.DISCONNECTED
        self.error_message = ""
        
        # G-code文件相关
        self.gcode_lines = []
        self.total_lines = 0
        self.heating_lines = []  # 存储加热指令
        self.print_lines = []    # 存储打印指令
        
        # 状态控制
        self.is_running = False
        self.is_paused = False
        self.stop_requested = False
        self.lock = Lock()
        self.heating_done_event = Event()
        
        # 加热阈值（温度在目标温度的这个范围内认为加热完成）
        self.temp_tolerance = self.config.get("temp_tolerance", 2.0)
        
        # 状态更新间隔
        self.status_update_interval = self.config.get("status_update_interval", 3)
        self.last_status_time = time.time()
        
        # 监控线程
        self.monitor_thread = None
        self.monitor_running = False
        
        # 错误计数
        self.communication_errors = 0
        self.max_communication_errors = 5
        
        # ROS Publishers
        self.pub_status = rospy.Publisher('/gcode_sender/status', String, queue_size=10)
        self.pub_progress = rospy.Publisher('/gcode_sender/progress', Float32, queue_size=10)
        self.pub_current_line = rospy.Publisher('/gcode_sender/current_line', Int32, queue_size=10)
        self.pub_running = rospy.Publisher('/gcode_sender/is_running', Bool, queue_size=10)
        self.pub_paused = rospy.Publisher('/gcode_sender/is_paused', Bool, queue_size=10)
        self.pub_nozzle_temp = rospy.Publisher('/gcode_sender/nozzle_temp', Float32, queue_size=10)
        self.pub_bed_temp = rospy.Publisher('/gcode_sender/bed_temp', Float32, queue_size=10)
        self.pub_printer_state = rospy.Publisher('/gcode_sender/printer_state', String, queue_size=10)
        
        # ROS Subscribers
        rospy.Subscriber('/gcode_sender/load_file', String, self.load_file_callback)
        
        # ROS Services
        rospy.Service('/gcode_sender/start', Trigger, self.handle_start)
        rospy.Service('/gcode_sender/pause', Trigger, self.handle_pause)
        rospy.Service('/gcode_sender/resume', Trigger, self.handle_resume)
        rospy.Service('/gcode_sender/stop', Trigger, self.handle_stop)
        rospy.Service('/gcode_sender/connect', Trigger, self.handle_connect)
        rospy.Service('/gcode_sender/disconnect', Trigger, self.handle_disconnect)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("G-code Sender Node (Enhanced v2) initialized")
        rospy.loginfo(f"Port: {self.port}, Baud: {self.baud}")
        rospy.loginfo(f"Temperature tolerance: ±{self.temp_tolerance}°C")
        rospy.loginfo("=" * 60)

    def load_config(self):
        """加载配置文件"""
        config = configparser.ConfigParser()
        
        # 尝试在多个位置查找配置文件
        config_paths = [
            CONFIG_FILE,
            Path(__file__).parent / CONFIG_FILE,
            Path.cwd() / CONFIG_FILE
        ]
        
        config_found = False
        for path in config_paths:
            if Path(path).exists():
                config.read(path)
                rospy.loginfo(f"✓ Config loaded from: {path}")
                config_found = True
                break
        
        if not config_found:
            rospy.logwarn(f"⚠ Config file not found, using defaults")
        
        return {
            "port": config.get("printer", "port", fallback="/dev/ttyACM0"),
            "baud": config.getint("printer", "baud", fallback=115200),
            "status_update_interval": config.getint("printer", "status_update_interval", fallback=3),
            "temp_tolerance": config.getfloat("printer", "temp_tolerance", fallback=2.0),
        }

    def log_info(self, message):
        """统一的信息日志"""
        rospy.loginfo(f"[INFO] {message}")
        
    def log_warn(self, message):
        """统一的警告日志"""
        rospy.logwarn(f"[WARN] {message}")
        
    def log_error(self, message):
        """统一的错误日志"""
        rospy.logerr(f"[ERROR] {message}")

    def parse_temperature_response(self, line):
        """解析温度响应"""
        try:
            if "T:" in line:
                t_part = line.split("T:")[1].split()[0]
                if "/" in t_part:
                    current, target = t_part.split("/")
                    self.nozzle_temp = float(current)
                    self.nozzle_target = float(target)
                else:
                    self.nozzle_temp = float(t_part)
                    
            if "B:" in line:
                b_part = line.split("B:")[1].split()[0]
                if "/" in b_part:
                    current, target = b_part.split("/")
                    self.bed_temp = float(current)
                    self.bed_target = float(target)
                else:
                    self.bed_temp = float(b_part)
                    
            # 重置通信错误计数
            self.communication_errors = 0
            
        except Exception as e:
            self.communication_errors += 1
            rospy.logdebug(f"Error parsing temperature: {e}")

    def parse_position_response(self, line):
        """解析位置响应"""
        try:
            if "X:" in line:
                parts = line.split()
                for part in parts:
                    if part.startswith("X:"):
                        self.pos_x = float(part.split(":")[1])
                    elif part.startswith("Y:"):
                        self.pos_y = float(part.split(":")[1])
                    elif part.startswith("Z:"):
                        self.pos_z = float(part.split(":")[1])
                        
            self.communication_errors = 0
            
        except Exception as e:
            self.communication_errors += 1
            rospy.logdebug(f"Error parsing position: {e}")

    def check_heating_complete(self):
        """检查加热是否完成"""
        nozzle_ready = True
        bed_ready = True
        
        # 检查喷嘴温度
        if self.nozzle_target > 0:
            nozzle_ready = abs(self.nozzle_temp - self.nozzle_target) <= self.temp_tolerance
            
        # 检查热床温度
        if self.bed_target > 0:
            bed_ready = abs(self.bed_temp - self.bed_target) <= self.temp_tolerance
            
        return nozzle_ready and bed_ready

    def wait_for_heating(self):
        """等待加热完成"""
        if self.nozzle_target <= 0 and self.bed_target <= 0:
            self.log_info("No heating required, skipping...")
            return True
        
        self.log_info("=" * 60)
        self.log_info("HEATING STARTED")
        self.log_info(f"Target - Nozzle: {self.nozzle_target}°C, Bed: {self.bed_target}°C")
        self.log_info("Waiting for temperatures to reach target...")
        self.log_info("=" * 60)
        
        self.printer_state = PrinterState.HEATING
        self.heating_done_event.clear()
        
        start_time = time.time()
        last_log_time = time.time()
        
        while not rospy.is_shutdown():
            # 检查是否被停止
            if self.stop_requested:
                self.log_warn("Heating cancelled by user")
                return False
            
            # 检查通信错误
            if self.communication_errors >= self.max_communication_errors:
                self.log_error("Too many communication errors during heating")
                self.printer_state = PrinterState.ERROR
                self.error_message = "Communication error"
                return False
            
            # 查询温度
            if self.printer and self.printer.online:
                try:
                    self.printer.send_now("M105")
                    time.sleep(0.5)
                    
                    # 读取响应
                    response = self.printer.readline()
                    if response:
                        self.parse_temperature_response(response.strip())
                    
                except Exception as e:
                    self.communication_errors += 1
                    self.log_error(f"Error querying temperature: {e}")
            
            # 每3秒输出一次当前温度
            if time.time() - last_log_time >= 3:
                elapsed = int(time.time() - start_time)
                self.log_info(
                    f"[{elapsed}s] Heating... "
                    f"Nozzle: {self.nozzle_temp:.1f}/{self.nozzle_target:.1f}°C, "
                    f"Bed: {self.bed_temp:.1f}/{self.bed_target:.1f}°C"
                )
                last_log_time = time.time()
            
            # 检查是否加热完成
            if self.check_heating_complete():
                elapsed = int(time.time() - start_time)
                self.log_info("=" * 60)
                self.log_info("🔥 HEATING DONE! 🔥")
                self.log_info(f"Total heating time: {elapsed} seconds")
                self.log_info(f"Final - Nozzle: {self.nozzle_temp:.1f}°C, Bed: {self.bed_temp:.1f}°C")
                self.log_info("=" * 60)
                self.heating_done_event.set()
                return True
            
            time.sleep(1)
        
        return False

    def check_printer_idle(self):
        """检查打印机是否空闲"""
        if not self.printer or not self.printer.online:
            return False
        
        # printrun的打印机状态
        if self.printer.printing or self.printer.paused:
            return False
        
        return True

    def separate_heating_and_print_commands(self, lines):
        """分离加热指令和打印指令"""
        self.heating_lines = []
        self.print_lines = []
        
        heating_commands = ['M104', 'M109', 'M140', 'M190']  # 加热相关指令
        
        heating_phase = True
        
        for line in lines:
            # 检查是否是加热指令
            is_heating = any(line.startswith(cmd) for cmd in heating_commands)
            
            if heating_phase:
                self.heating_lines.append(line)
                # 遇到非加热指令，切换到打印阶段
                if not is_heating and line.startswith('G'):
                    heating_phase = False
            else:
                self.print_lines.append(line)
        
        self.log_info(f"Separated: {len(self.heating_lines)} heating commands, {len(self.print_lines)} print commands")

    def send_heating_commands(self):
        """发送加热指令"""
        if not self.heating_lines:
            self.log_info("No heating commands to send")
            return True
        
        self.log_info(f"Sending {len(self.heating_lines)} heating commands...")
        
        for line in self.heating_lines:
            if self.stop_requested:
                return False
            
            try:
                if self.printer and self.printer.online:
                    self.printer.send_now(line)
                    self.log_info(f"Sent: {line}")
                    
                    # 提取目标温度
                    if line.startswith('M104') or line.startswith('M109'):
                        if 'S' in line:
                            self.nozzle_target = float(line.split('S')[1].split()[0])
                    elif line.startswith('M140') or line.startswith('M190'):
                        if 'S' in line:
                            self.bed_target = float(line.split('S')[1].split()[0])
                    
                    time.sleep(0.1)
                else:
                    self.log_error("Printer not connected")
                    return False
                    
            except Exception as e:
                self.log_error(f"Error sending heating command '{line}': {e}")
                self.communication_errors += 1
                
                if self.communication_errors >= self.max_communication_errors:
                    self.log_error("Too many communication errors")
                    return False
        
        return True

    def temperature_monitor(self):
        """温度监控线程"""
        self.log_info("Temperature monitor thread started")
        
        while self.monitor_running and not rospy.is_shutdown():
            if self.printer and self.printer.online:
                try:
                    # 查询温度
                    self.printer.send_now("M105")
                    time.sleep(0.3)
                    
                    # 查询位置（仅在打印时）
                    if self.printer.printing:
                        self.printer.send_now("M114")
                        time.sleep(0.3)
                    
                    # 读取响应
                    while True:
                        response = self.printer.readline()
                        if not response:
                            break
                        line = response.strip()
                        if line:
                            self.parse_temperature_response(line)
                            self.parse_position_response(line)
                    
                except Exception as e:
                    self.communication_errors += 1
                    rospy.logdebug(f"Monitor error: {e}")
                
            time.sleep(self.status_update_interval)
        
        self.log_info("Temperature monitor thread stopped")

    def connect_printer(self):
        """连接到打印机"""
        if self.printer and self.printer.online:
            self.log_warn("Printer already connected")
            return True
            
        try:
            self.log_info(f"Connecting to printer on {self.port} @ {self.baud} baud...")
            self.printer = printcore(port=self.port, baud=self.baud)
            
            # 等待连接
            timeout = 10
            start_time = time.time()
            while not self.printer.online:
                if time.time() - start_time > timeout:
                    self.log_error("Printer connection timeout")
                    self.printer_state = PrinterState.ERROR
                    self.error_message = "Connection timeout"
                    return False
                time.sleep(0.1)
            
            self.log_info("✓ Printer connected successfully")
            self.printer_state = PrinterState.IDLE
            self.communication_errors = 0
            
            # 启动温度监控线程
            self.monitor_running = True
            self.monitor_thread = Thread(target=self.temperature_monitor, daemon=True)
            self.monitor_thread.start()
            
            return True
            
        except Exception as e:
            self.log_error(f"Failed to connect to printer: {e}")
            self.printer_state = PrinterState.ERROR
            self.error_message = str(e)
            return False

    def disconnect_printer(self):
        """断开打印机连接"""
        self.monitor_running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2)
        
        if self.printer:
            self.printer.disconnect()
            self.printer = None
            self.printer_state = PrinterState.DISCONNECTED
            self.log_info("Printer disconnected")

    def load_file_callback(self, msg):
        """加载G-code文件"""
        filepath = msg.data
        self.load_gcode_file(filepath)

    def load_gcode_file(self, filepath):
        """加载G-code文件"""
        with self.lock:
            try:
                path = Path(filepath)
                if not path.exists():
                    self.log_error(f"G-code file not found: {filepath}")
                    return False
                
                self.log_info(f"Loading G-code file: {filepath}")
                
                # 读取G-code文件
                with open(path, 'r') as f:
                    self.gcode_lines = [
                        line.strip() 
                        for line in f 
                        if line.strip() and not line.strip().startswith(';')
                    ]
                
                if not self.gcode_lines:
                    self.log_error("G-code file is empty or contains only comments")
                    return False
                
                self.total_lines = len(self.gcode_lines)
                
                # 分离加热和打印指令
                self.separate_heating_and_print_commands(self.gcode_lines)
                
                self.log_info(f"✓ Loaded {self.total_lines} lines from G-code file")
                return True
                
            except Exception as e:
                self.log_error(f"Error loading G-code file: {e}")
                return False

    def handle_connect(self, req):
        """服务：连接打印机"""
        success = self.connect_printer()
        return TriggerResponse(
            success=success,
            message="Connected" if success else f"Connection failed: {self.error_message}"
        )

    def handle_disconnect(self, req):
        """服务：断开打印机"""
        self.disconnect_printer()
        return TriggerResponse(success=True, message="Disconnected")

    def handle_start(self, req):
        """服务：开始发送G-code"""
        with self.lock:
            # 检查是否已在运行
            if self.is_running:
                return TriggerResponse(success=False, message="Already running")
            
            # 检查打印机连接
            if not self.printer or not self.printer.online:
                return TriggerResponse(success=False, message="Printer not connected")
            
            # 检查是否加载了G-code
            if self.total_lines == 0:
                return TriggerResponse(success=False, message="No G-code loaded")
            
            # 检查打印机是否空闲
            if not self.check_printer_idle():
                return TriggerResponse(success=False, message="Printer is busy")
            
            self.is_running = True
            self.stop_requested = False
            self.communication_errors = 0
            
            # 在后台线程中执行加热和打印流程
            Thread(target=self.print_workflow, daemon=True).start()
            
            return TriggerResponse(success=True, message="Print started")

    def print_workflow(self):
        """打印工作流程：加热 -> 等待 -> 打印"""
        try:
            # 步骤1：发送加热指令
            self.log_info("Step 1: Sending heating commands...")
            if not self.send_heating_commands():
                self.log_error("Failed to send heating commands")
                self.is_running = False
                return
            
            # 步骤2：等待加热完成
            self.log_info("Step 2: Waiting for heating to complete...")
            if not self.wait_for_heating():
                self.log_error("Heating failed or cancelled")
                self.is_running = False
                return
            
            # 步骤3：开始打印
            self.log_info("Step 3: Starting print...")
            self.printer_state = PrinterState.PRINTING
            
            if self.print_lines:
                gcode = LightGCode(self.print_lines)
                self.printer.startprint(gcode)
                self.log_info(f"✓ Print started with {len(self.print_lines)} commands")
            else:
                self.log_warn("No print commands to send")
                self.is_running = False
                
        except Exception as e:
            self.log_error(f"Error in print workflow: {e}")
            self.printer_state = PrinterState.ERROR
            self.error_message = str(e)
            self.is_running = False

    def handle_pause(self, req):
        """服务：暂停"""
        with self.lock:
            if not self.is_running:
                return TriggerResponse(success=False, message="Not running")
            
            if self.is_paused:
                return TriggerResponse(success=False, message="Already paused")
            
            if self.printer and self.printer.printing:
                self.printer.pause()
                self.is_paused = True
                self.printer_state = PrinterState.PAUSED
                current = self.printer.queueindex if hasattr(self.printer, 'queueindex') else 0
                self.log_info(f"⏸ Paused at line {current}/{len(self.print_lines)}")
                return TriggerResponse(success=True, message=f"Paused at line {current}")
            
            return TriggerResponse(success=False, message="Printer not printing")

    def handle_resume(self, req):
        """服务：恢复"""
        with self.lock:
            if not self.is_running:
                return TriggerResponse(success=False, message="Not running")
            
            if not self.is_paused:
                return TriggerResponse(success=False, message="Not paused")
            
            if self.printer and self.printer.paused:
                self.printer.resume()
                self.is_paused = False
                self.printer_state = PrinterState.PRINTING
                current = self.printer.queueindex if hasattr(self.printer, 'queueindex') else 0
                self.log_info(f"▶ Resumed from line {current}/{len(self.print_lines)}")
                return TriggerResponse(success=True, message=f"Resumed from line {current}")
            
            return TriggerResponse(success=False, message="Printer not paused")

    def handle_stop(self, req):
        """服务：停止"""
        with self.lock:
            if not self.is_running:
                return TriggerResponse(success=False, message="Not running")
            
            self.stop_requested = True
            
            if self.printer and self.printer.printing:
                self.printer.cancelprint()
                self.log_info("⏹ Print cancelled")
            
            self.is_running = False
            self.is_paused = False
            self.printer_state = PrinterState.IDLE
            
            return TriggerResponse(success=True, message="Stopped")

    def publish_status(self):
        """发布状态信息"""
        # 计算进度
        progress = 0.0
        current_line = 0
        
        if self.printer and hasattr(self.printer, 'queueindex') and len(self.print_lines) > 0:
            current_line = self.printer.queueindex
            progress = (current_line / len(self.print_lines)) * 100.0
        
        # 构建状态字符串
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        status = f"{timestamp} | State: {self.printer_state.value.upper()} | [{progress:5.1f}%] Line {current_line}/{len(self.print_lines)}"
        status += f" | Pos: X:{self.pos_x:.2f} Y:{self.pos_y:.2f} Z:{self.pos_z:.2f}"
        status += f" | Temp: N:{self.nozzle_temp:.1f}/{self.nozzle_target:.1f}°C B:{self.bed_temp:.1f}/{self.bed_target:.1f}°C"
        
        if self.communication_errors > 0:
            status += f" | Errors: {self.communication_errors}"
        
        # 发布
        self.pub_status.publish(status)
        self.pub_progress.publish(progress)
        self.pub_current_line.publish(current_line)
        self.pub_nozzle_temp.publish(self.nozzle_temp)
        self.pub_bed_temp.publish(self.bed_temp)
        self.pub_printer_state.publish(self.printer_state.value)
        
        # 更新实际状态
        if self.printer:
            actual_running = self.printer.printing
            actual_paused = self.printer.paused
            
            self.pub_running.publish(actual_running)
            self.pub_paused.publish(actual_paused)
            
            # 检测打印完成
            if self.is_running and not actual_running and not actual_paused and self.printer_state == PrinterState.PRINTING:
                self.log_info("=" * 60)
                self.log_info("✅ PRINT COMPLETED! ✅")
                self.log_info("=" * 60)
                self.is_running = False
                self.printer_state = PrinterState.IDLE
        
        # 检测通信错误
        if self.communication_errors >= self.max_communication_errors:
            self.log_error(f"Too many communication errors ({self.communication_errors})")
            self.printer_state = PrinterState.ERROR
        
        rospy.loginfo(status)

    def run(self):
        """主循环"""
        rate = rospy.Rate(1)  # 1 Hz
        
        self.log_info("G-code Sender Node is running...")
        
        while not rospy.is_shutdown():
            self.publish_status()
            rate.sleep()

    def shutdown(self):
        """关闭节点"""
        self.log_info("Shutting down G-code Sender Node...")
        self.stop_requested = True
        self.disconnect_printer()


# --------------------------------
# Main Entry Point
# --------------------------------
def main():
    try:
        node = GcodeSenderNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()