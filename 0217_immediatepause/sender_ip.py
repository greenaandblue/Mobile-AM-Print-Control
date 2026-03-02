#!/usr/bin/env python3

"""
gcode_sender_node.py (Improved - Based on OctoPrint Logic)

ROS节点：正确发送G-code到3D打印机
- 支持暂停、恢复、停止功能
- 参考OctoPrint的状态管理逻辑
- 基于 printcore 的正确打印方式
- 暂停时发送M0立即清空打印机缓冲区
"""

import rospy
import configparser
import time
from pathlib import Path
from datetime import datetime
from threading import Lock
from enum import Enum

from printrun.printcore import printcore
from printrun.gcoder import LightGCode
from printrun.eventhandler import PrinterEventHandler

from std_msgs.msg import String, Bool, Int32, Float32
from std_srvs.srv import Trigger, TriggerResponse

CONFIG_FILE = "printer_config.ini"


# --------------------------------
# Printer State Enum
# --------------------------------
class PrinterState(Enum):
    """打印机状态枚举"""
    IDLE = 0      # 空闲，未连接或未开始打印
    PRINTING = 1  # 正在打印
    PAUSED = 2    # 暂停中
    STOPPED = 3   # 已停止


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
        self.gcode_filepath = None
        
        # 状态控制 - 参考OctoPrint的设计
        self.state = PrinterState.IDLE
        self.lock = Lock()
        
        # 状态更新间隔
        self.status_update_interval = self.config["status_update_interval"]
        self.last_status_time = time.time()
        
        # ROS Publishers
        self.pub_status = rospy.Publisher('/gcode_sender/status', String, queue_size=10)
        self.pub_progress = rospy.Publisher('/gcode_sender/progress', Float32, queue_size=10)
        self.pub_current_line = rospy.Publisher('/gcode_sender/current_line', Int32, queue_size=10)
        self.pub_state = rospy.Publisher('/gcode_sender/state', String, queue_size=10)
        
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

    def get_state(self) -> PrinterState:
        """
        获取当前打印机状态（综合判断）
        参考OctoPrint的get_state逻辑
        """
        with self.lock:
            # 如果没有打印机连接
            if not self.printer or not self.printer.online:
                return PrinterState.IDLE
            
            # 检查printcore的状态
            if self.printer.paused:
                return PrinterState.PAUSED
            elif self.printer.printing:
                return PrinterState.PRINTING
            else:
                return self.state  # 保持当前状态

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
            with self.lock:
                self.state = PrinterState.IDLE
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
            with self.lock:
                self.state = PrinterState.IDLE
            rospy.loginfo("Printer disconnected")
    
    def home_printer(self):
        """提升喷头并回到原点"""
        if self.printer and self.printer.online:
            try:
                rospy.loginfo("Lifting nozzle and homing printer...")
                # 相对定位
                self.printer.send_now("G91")
                time.sleep(0.1)
                # 提升喷头10mm
                self.printer.send_now("G1 Z10 F600")
                time.sleep(1)
                # 绝对定位
                self.printer.send_now("G90")
                time.sleep(0.1)
                # 回到原点
                self.printer.send_now("G28 X Y")
                time.sleep(2)
                rospy.loginfo("Printer homed successfully")
            except Exception as e:
                rospy.logerr(f"Error homing printer: {e}")

    def load_file_callback(self, msg):
        """加载G-code文件 (通过ROS topic)"""
        filepath = msg.data
        self.load_gcode_file(filepath)

    def load_gcode_file(self, filepath):
        """
        加载G-code文件
        重要：保留所有有效的G-code命令，包括加热、挤出等
        """
        with self.lock:
            try:
                path = Path(filepath)
                if not path.exists():
                    rospy.logerr(f"G-code file not found: {filepath}")
                    return False
                
                # 读取G-code文件
                # 只过滤掉：空行和纯注释行（以 ; 开头）
                # 保留：所有G-code命令，包括行尾注释
                self.gcode_lines = []
                with open(path, 'r') as f:
                    for line in f:
                        stripped = line.strip()
                        # 跳过空行
                        if not stripped:
                            continue
                        # 跳过纯注释行
                        if stripped.startswith(';'):
                            continue
                        # 保留所有其他行（包括带行尾注释的命令）
                        self.gcode_lines.append(stripped)
                
                self.total_lines = len(self.gcode_lines)
                self.gcode_filepath = filepath
                
                rospy.loginfo(f"Loaded G-code file: {filepath}")
                rospy.loginfo(f"Total lines: {self.total_lines}")
                
                # 显示前10行用于调试
                rospy.loginfo("First 10 lines of G-code:")
                for i, line in enumerate(self.gcode_lines[:10]):
                    rospy.loginfo(f"  {i+1}: {line}")
                
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
        self.home_printer()
        self.disconnect_printer()
        return TriggerResponse(success=True, message="Homed and disconnected")

    def handle_start(self, req):
        """
        服务：开始发送G-code
        参考OctoPrint的start_print逻辑
        """
        with self.lock:
            # 检查状态
            if self.state == PrinterState.PRINTING:
                return TriggerResponse(success=False, message="Already printing")
            
            if self.state == PrinterState.PAUSED:
                return TriggerResponse(success=False, message="Currently paused, use resume instead")
            
            if not self.printer or not self.printer.online:
                return TriggerResponse(success=False, message="Printer not connected")
            
            if self.total_lines == 0:
                return TriggerResponse(success=False, message="No G-code loaded")
            
            # 更新状态
            self.state = PrinterState.PRINTING
            
            rospy.loginfo("Starting G-code transmission...")
            return TriggerResponse(success=True, message="Started")

    def handle_pause(self, req):
        """
        服务：暂停发送
        两步操作：
          1. printer.pause()  —— 让 printcore 停止向发送队列追加新行
          2. send_now("M0")   —— 绕过队列直接发给打印机，立即清空固件缓冲区
        Resume 时 printer.resume() 会从 queueindex 断点继续，无需额外处理。
        """
        with self.lock:
            # 检查当前状态
            if self.state != PrinterState.PRINTING:
                return TriggerResponse(success=False, message=f"Not printing (current state: {self.state.name})")
            
            if self.printer and self.printer.printing:
                rospy.loginfo("Calling printer.pause() to stop sending new lines...")
                self.printer.pause()

                # 用 send_now 绕过打印队列，直接发送 M0 让打印机立即停止
                # M0: Unconditional Stop —— 清空固件缓冲区并暂停运动
                rospy.loginfo("Sending M0 via send_now() to immediately halt printer buffer...")
                self.printer.send_now("M0")

                # 更新状态
                self.state = PrinterState.PAUSED
                
                current_line = self.printer.queueindex if self.printer else 0
                rospy.loginfo(f"Paused at line {current_line}/{self.total_lines}")
                return TriggerResponse(success=True, message=f"Paused at line {current_line}")
            else:
                return TriggerResponse(success=False, message="Printer not in printing state")

    def handle_resume(self, req):
        """
        服务：恢复发送
        参考OctoPrint的execute_printer_resume逻辑
        """
        with self.lock:
            # 检查当前状态
            if self.state != PrinterState.PAUSED:
                return TriggerResponse(success=False, message=f"Not paused (current state: {self.state.name})")
            
            # 调用printcore的resume方法
            if self.printer and self.printer.paused:
                rospy.loginfo("Calling printer.resume()...")
                self.printer.resume()
                
                # 更新状态
                self.state = PrinterState.PRINTING
                
                current_line = self.printer.queueindex if self.printer else 0
                rospy.loginfo(f"Resumed from line {current_line}/{self.total_lines}")
                return TriggerResponse(success=True, message=f"Resumed from line {current_line}")
            else:
                return TriggerResponse(success=False, message="Printer not in paused state")

    def handle_stop(self, req):
        """
        服务：停止发送
        参考OctoPrint的stop_print逻辑
        """
        with self.lock:
            # 检查状态
            if self.state == PrinterState.IDLE:
                return TriggerResponse(success=False, message="Not printing")
            
            # 停止打印机
            if self.printer and (self.printer.printing or self.printer.paused):
                rospy.loginfo("Calling printer.cancelprint()...")
                self.printer.cancelprint()
                time.sleep(0.5)  # 给打印机时间响应
            
            # 更新状态
            self.state = PrinterState.STOPPED
            
            rospy.loginfo("Print stopped")
            return TriggerResponse(success=True, message="Stopped")

    def publish_status(self):
        """发布状态信息"""
        now = time.time()
        if now - self.last_status_time >= self.status_update_interval:
            # 查询温度和位置
            if self.printer and self.printer.online:
                self.printer.send_now("M105")  # 温度
                self.printer.send_now("M114")  # 位置
            
            # 获取当前状态
            current_state = self.get_state()
            
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
            self.pub_state.publish(current_state.name)
            
            rospy.loginfo(f"[{current_state.name}] {status}")
            self.last_status_time = now

    def run(self):
        """主循环：使用 startprint() 发送G-code"""
        rate = rospy.Rate(1)  # 1 Hz - 更新状态
        
        while not rospy.is_shutdown():
            current_state = self.get_state()
            
            # 检查是否需要启动打印
            with self.lock:
                if self.state == PrinterState.PRINTING and self.printer and not self.printer.printing and not self.printer.paused:
                    # 开始打印
                    try:
                        rospy.loginfo(f"Starting print: {self.gcode_filepath}")
                        rospy.loginfo(f"Total lines to send: {self.total_lines}")
                        self.printer.startprint(LightGCode(self.gcode_lines))
                        rospy.loginfo("Print started successfully")
                    except Exception as e:
                        rospy.logerr(f"Error starting print: {e}")
                        self.state = PrinterState.IDLE
            
            # 检查打印是否完成
            with self.lock:
                if self.state == PrinterState.PRINTING and self.printer and not self.printer.printing and not self.printer.paused:
                    # 打印完成
                    if self.printer.queueindex > 0:
                        rospy.loginfo("Print completed!")
                        self.state = PrinterState.IDLE
                
                # 如果是停止状态，也转回IDLE
                if self.state == PrinterState.STOPPED and self.printer and not self.printer.printing:
                    self.state = PrinterState.IDLE
            
            # 发布状态
            if self.state in [PrinterState.PRINTING, PrinterState.PAUSED]:
                self.publish_status()
            
            rate.sleep()

    def shutdown(self):
        """关闭节点"""
        rospy.loginfo("Shutting down G-code Sender Node...")
        
        # 停止打印
        if self.printer and (self.printer.printing or self.printer.paused):
            try:
                self.printer.cancelprint()
                time.sleep(1)
            except:
                pass
        
        # 回到原点
        self.home_printer()
        
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