#!/usr/bin/env python3

"""
gcode_sender_node.py (Improved)

ROS节点：正确发送G-code到3D打印机
支持暂停、恢复、停止功能
基于 printer_stl.py 的成功打印方式
"""

import rospy
import configparser
import time
from pathlib import Path
from datetime import datetime
from threading import Lock, Event
import signal

from printrun.printcore import printcore
from printrun.gcoder import LightGCode
from printrun.eventhandler import PrinterEventHandler

from std_msgs.msg import String, Bool, Int32, Float32
from std_srvs.srv import Trigger, TriggerResponse

CONFIG_FILE = "printer_config.ini"


# --------------------------------
# Custom Status Event Handler
# --------------------------------
class GcodeSenderHandler(PrinterEventHandler):
    """处理打印机返回的状态信息"""
    
    def __init__(self):
        super().__init__()
        self.last_temp = None
        self.last_bed = None
        self.last_x = None
        self.last_y = None
        self.last_z = None

    def on_recv(self, line):
        """接收打印机返回的数据"""
        if "T:" in line and "B:" in line:
            self.parse_temperature(line)
        elif "X:" in line and "Y:" in line and "Z:" in line:
            self.parse_position(line)

    def parse_temperature(self, line):
        """解析温度信息"""
        try:
            nozzle_part = line.split("T:")[1]
            nozzle_temp = nozzle_part.split(" ")[0]
            bed_part = line.split("B:")[1]
            bed_temp = bed_part.split(" ")[0]
            self.last_temp = nozzle_temp
            self.last_bed = bed_temp
        except Exception as e:
            rospy.logwarn(f"Error parsing temperature: {e}")

    def parse_position(self, line):
        """解析位置信息"""
        try:
            parts = line.split()
            self.last_x = parts[0].split(":")[1]
            self.last_y = parts[1].split(":")[1]
            self.last_z = parts[2].split(":")[1]
        except Exception as e:
            rospy.logwarn(f"Error parsing position: {e}")

    def get_status_string(self, progress_percent, current_line, total_lines):
        """获取状态字符串"""
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        status = f"{now} [{progress_percent:5.1f}%] Line {current_line}/{total_lines}"
        if self.last_x and self.last_y and self.last_z:
            status += f" | X:{self.last_x} Y:{self.last_y} Z:{self.last_z}"
        if self.last_temp and self.last_bed:
            status += f" | Nozzle:{self.last_temp}°C Bed:{self.last_bed}°C"
        return status


# --------------------------------
# G-code Sender Node
# --------------------------------
class GcodeSenderNode:
    """ROS节点：使用 printcore.startprint() 正确发送G-code"""
    
    def __init__(self):
        rospy.init_node('gcode_sender_node', anonymous=False)
        
        # 加载配置
        self.config = self.load_config()
        
        # 打印机连接参数
        self.port = self.config["port"]
        self.baud = self.config["baud"]
        self.printer = None
        self.handler = GcodeSenderHandler()
        
        # G-code文件相关
        self.gcode_lines = []
        self.total_lines = 0
        
        # 状态控制
        self.is_running = False
        self.is_paused = False
        self.stop_requested = False
        self.lock = Lock()
        
        # 状态更新间隔
        self.status_update_interval = self.config["status_update_interval"]
        self.last_status_time = time.time()
        
        # ROS Publishers
        self.pub_status = rospy.Publisher('/gcode_sender/status', String, queue_size=10)
        self.pub_progress = rospy.Publisher('/gcode_sender/progress', Float32, queue_size=10)
        self.pub_current_line = rospy.Publisher('/gcode_sender/current_line', Int32, queue_size=10)
        self.pub_running = rospy.Publisher('/gcode_sender/is_running', Bool, queue_size=10)
        self.pub_paused = rospy.Publisher('/gcode_sender/is_paused', Bool, queue_size=10)
        
        # ROS Subscribers
        rospy.Subscriber('/gcode_sender/load_file', String, self.load_file_callback)
        
        # ROS Services
        rospy.Service('/gcode_sender/start', Trigger, self.handle_start)
        rospy.Service('/gcode_sender/pause', Trigger, self.handle_pause)
        rospy.Service('/gcode_sender/resume', Trigger, self.handle_resume)
        rospy.Service('/gcode_sender/stop', Trigger, self.handle_stop)
        rospy.Service('/gcode_sender/connect', Trigger, self.handle_connect)
        rospy.Service('/gcode_sender/disconnect', Trigger, self.handle_disconnect)
        
        rospy.loginfo("G-code Sender Node initialized")
        rospy.loginfo(f"Printer port: {self.port}, Baud: {self.baud}")

    def load_config(self):
        """加载配置文件"""
        config = configparser.ConfigParser()
        config.read(CONFIG_FILE)
        
        return {
            "port": config.get("printer", "port", fallback="/dev/ttyACM1"),
            "baud": config.getint("printer", "baud", fallback=115200),
            "status_update_interval": config.getint("printer", "status_update_interval", fallback=10),
        }

    def connect_printer(self):
        """连接到打印机"""
        if self.printer and self.printer.online:
            rospy.logwarn("Printer already connected")
            return True
            
        try:
            rospy.loginfo(f"Connecting to printer on {self.port} @ {self.baud} baud...")
            self.printer = printcore(port=self.port, baud=self.baud)
            self.printer.addEventHandler(self.handler)
            
            # 等待连接
            timeout = 10
            start_time = time.time()
            while not self.printer.online:
                if time.time() - start_time > timeout:
                    rospy.logerr("Printer connection timeout")
                    return False
                time.sleep(0.1)
            
            rospy.loginfo("Printer connected successfully")
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to connect to printer: {e}")
            return False

    def disconnect_printer(self):
        """断开打印机连接"""
        if self.printer:
            try:
                self.printer.disconnect()
            except:
                pass
            self.printer = None
            rospy.loginfo("Printer disconnected")

    def load_file_callback(self, msg):
        """加载G-code文件 (通过ROS topic)"""
        filepath = msg.data
        self.load_gcode_file(filepath)

    def load_gcode_file(self, filepath):
        """加载G-code文件"""
        with self.lock:
            try:
                path = Path(filepath)
                if not path.exists():
                    rospy.logerr(f"G-code file not found: {filepath}")
                    return False
                
                # 读取G-code文件，过滤掉注释和空行
                with open(path, 'r') as f:
                    self.gcode_lines = [line.strip() for line in f 
                                       if line.strip() and not line.strip().startswith(';')]
                
                self.total_lines = len(self.gcode_lines)
                
                rospy.loginfo(f"Loaded G-code file: {filepath}")
                rospy.loginfo(f"Total lines: {self.total_lines}")
                return True
                
            except Exception as e:
                rospy.logerr(f"Error loading G-code file: {e}")
                return False

    def handle_connect(self, req):
        """服务：连接打印机"""
        success = self.connect_printer()
        return TriggerResponse(
            success=success,
            message="Connected" if success else "Connection failed"
        )

    def handle_disconnect(self, req):
        """服务：断开打印机"""
        self.disconnect_printer()
        return TriggerResponse(success=True, message="Disconnected")

    def handle_start(self, req):
        """服务：开始发送G-code"""
        with self.lock:
            if self.is_running:
                return TriggerResponse(success=False, message="Already running")
            
            if not self.printer or not self.printer.online:
                return TriggerResponse(success=False, message="Printer not connected")
            
            if self.total_lines == 0:
                return TriggerResponse(success=False, message="No G-code loaded")
            
            self.is_running = True
            self.is_paused = False
            self.stop_requested = False
            
            rospy.loginfo("Starting G-code transmission...")
            return TriggerResponse(success=True, message="Started")

    def handle_pause(self, req):
        """服务：暂停发送"""
        with self.lock:
            if not self.is_running:
                return TriggerResponse(success=False, message="Not running")
            
            if self.is_paused:
                return TriggerResponse(success=False, message="Already paused")
            
            self.is_paused = True
            
            # 暂停打印机
            if self.printer and self.printer.printing:
                self.printer.pause()
            
            current_line = self.printer.queueindex if self.printer else 0
            rospy.loginfo(f"Paused at line {current_line}/{self.total_lines}")
            return TriggerResponse(success=True, message=f"Paused at line {current_line}")

    def handle_resume(self, req):
        """服务：恢复发送"""
        with self.lock:
            if not self.is_running:
                return TriggerResponse(success=False, message="Not running")
            
            if not self.is_paused:
                return TriggerResponse(success=False, message="Not paused")
            
            self.is_paused = False
            
            # 恢复打印机
            if self.printer and self.printer.printing:
                self.printer.resume()
            
            current_line = self.printer.queueindex if self.printer else 0
            rospy.loginfo(f"Resumed from line {current_line}/{self.total_lines}")
            return TriggerResponse(success=True, message=f"Resumed from line {current_line}")

    def handle_stop(self, req):
        """服务：停止发送"""
        with self.lock:
            if not self.is_running:
                return TriggerResponse(success=False, message="Not running")
            
            self.stop_requested = True
            
            # 停止打印机
            if self.printer:
                self.printer.cancelprint()
            
            rospy.loginfo("Stop requested")
            return TriggerResponse(success=True, message="Stopping...")

    def publish_status(self):
        """发布状态信息"""
        now = time.time()
        if now - self.last_status_time >= self.status_update_interval:
            # 查询温度和位置
            if self.printer and self.printer.online:
                self.printer.send_now("M105")  # 温度
                self.printer.send_now("M114")  # 位置
            
            # 计算进度
            current_line = 0
            if self.printer:
                current_line = self.printer.queueindex
            
            progress = 0.0
            if self.total_lines > 0:
                progress = (current_line / self.total_lines) * 100.0
            
            # 构建状态字符串
            status = self.handler.get_status_string(progress, current_line, self.total_lines)
            
            # 发布状态
            self.pub_status.publish(status)
            self.pub_progress.publish(progress)
            self.pub_current_line.publish(current_line)
            self.pub_running.publish(self.is_running)
            self.pub_paused.publish(self.is_paused)
            
            rospy.loginfo(status)
            self.last_status_time = now

    def run(self):
        """主循环：使用 startprint() 发送G-code"""
        rate = rospy.Rate(1)  # 1 Hz - 更新状态
        
        while not rospy.is_shutdown():
            # 检查是否需要启动打印
            if self.is_running and not self.printer.printing:
                with self.lock:
                    # 开始打印
                    try:
                        rospy.loginfo(f"Starting print with {self.total_lines} lines...")
                        self.printer.startprint(LightGCode(self.gcode_lines))
                        rospy.loginfo("Print started successfully")
                    except Exception as e:
                        rospy.logerr(f"Error starting print: {e}")
                        self.is_running = False
            
            # 检查打印是否完成
            if self.is_running and self.printer and not self.printer.printing:
                with self.lock:
                    if self.stop_requested:
                        self.stop_requested = False
                        rospy.loginfo("Print stopped")
                    else:
                        rospy.loginfo("Print completed!")
                    self.is_running = False
            
            # 发布状态
            if self.is_running and self.printer and self.printer.printing:
                self.publish_status()
            
            rate.sleep()

    def shutdown(self):
        """关闭节点"""
        rospy.loginfo("Shutting down G-code Sender Node...")
        
        # 停止打印
        if self.printer and self.printer.printing:
            try:
                self.printer.cancelprint()
            except:
                pass
        
        # 断开连接
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


if __name__ == "__main__":
    main()
