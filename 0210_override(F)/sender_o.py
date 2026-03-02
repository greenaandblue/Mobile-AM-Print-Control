#!/usr/bin/env python3

"""
gcode_sender_node.py (With Speed Override)

ROS节点：正确发送G-code到3D打印机
- 支持暂停、恢复、停止功能
- 新增速度覆写（Speed Override）功能
- 参考OctoPrint的状态管理逻辑
- 基于 printcore 的正确打印方式
"""

import rospy
import configparser
import time
import re
from pathlib import Path
from datetime import datetime
from threading import Lock
from enum import Enum

from printrun.printcore import printcore
from printrun.gcoder import LightGCode
from printrun.eventhandler import PrinterEventHandler

from std_msgs.msg import String, Bool, Int32, Float32
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from gcode_sender.srv import SetSpeedOverride, SetSpeedOverrideResponse

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
# Speed Override Mode Enum
# --------------------------------
class SpeedOverrideMode(Enum):
    """速度覆写模式"""
    DISABLED = 0      # 禁用，使用原始速度
    PERCENTAGE = 1    # 百分比模式（如50%）
    FIXED_VALUE = 2   # 固定值模式（如F1500）


# --------------------------------
# G-code Speed Modifier
# --------------------------------
class GcodeSpeedModifier:
    """G-code速度修改器"""
    
    # 匹配移动指令中的F参数的正则表达式
    # 例如: G1 X10 Y20 F3000 或 G0 F5000
    F_PATTERN = re.compile(r'\bF(\d+\.?\d*)\b', re.IGNORECASE)
    
    # 匹配移动指令（G0/G1）
    MOVE_PATTERN = re.compile(r'\b(G0|G1)\b', re.IGNORECASE)
    
    def __init__(self):
        self.mode = SpeedOverrideMode.DISABLED
        self.percentage = 100.0  # 百分比（100 = 原速度）
        self.fixed_value = 3000.0  # 固定速度值
        self.last_speed = None  # 记录最后一次的速度（用于模态）
    
    def set_mode(self, mode: SpeedOverrideMode, value: float = None):
        """
        设置速度覆写模式
        mode: SpeedOverrideMode枚举
        value: 百分比值（0-200）或固定速度值（>0）
        """
        self.mode = mode
        if mode == SpeedOverrideMode.PERCENTAGE and value is not None:
            self.percentage = max(1.0, min(200.0, value))  # 限制1%-200%
        elif mode == SpeedOverrideMode.FIXED_VALUE and value is not None:
            self.fixed_value = max(1.0, value)  # 最小1mm/min
        
        rospy.loginfo(f"Speed override mode set to: {mode.name}")
        if mode == SpeedOverrideMode.PERCENTAGE:
            rospy.loginfo(f"  Percentage: {self.percentage}%")
        elif mode == SpeedOverrideMode.FIXED_VALUE:
            rospy.loginfo(f"  Fixed value: F{self.fixed_value}")
    
    def disable(self):
        """禁用速度覆写"""
        self.mode = SpeedOverrideMode.DISABLED
        self.last_speed = None
        rospy.loginfo("Speed override disabled")
    
    def modify_line(self, line: str) -> str:
        """
        修改G-code行中的速度参数
        
        逻辑：
        1. 如果禁用，直接返回原始行
        2. 检查是否是移动指令（G0/G1）
        3. 如果有F参数，提取并修改
        4. 如果没有F参数但是移动指令，注入上次使用的修改后速度（模态）
        
        返回：修改后的G-code行
        """
        # 如果禁用覆写，直接返回原行
        if self.mode == SpeedOverrideMode.DISABLED:
            # 仍然需要跟踪原始速度（用于后续可能启用的覆写）
            match = self.F_PATTERN.search(line)
            if match:
                self.last_speed = float(match.group(1))
            return line
        
        # 检查是否是移动指令
        is_move = self.MOVE_PATTERN.search(line) is not None
        
        # 查找F参数
        f_match = self.F_PATTERN.search(line)
        
        if f_match:
            # 提取原始速度
            original_speed = float(f_match.group(1))
            
            # 计算新速度
            if self.mode == SpeedOverrideMode.PERCENTAGE:
                new_speed = original_speed * (self.percentage / 100.0)
            else:  # FIXED_VALUE
                new_speed = self.fixed_value
            
            # 记录修改后的速度
            self.last_speed = new_speed
            
            # 替换F参数
            modified_line = self.F_PATTERN.sub(f'F{new_speed:.1f}', line)
            
            return modified_line
        
        elif is_move and self.last_speed is not None:
            # 移动指令但没有F参数，注入修改后的速度（模态）
            # 在行尾添加F参数
            # 去掉注释部分
            if ';' in line:
                code_part, comment_part = line.split(';', 1)
                modified_line = f"{code_part.strip()} F{self.last_speed:.1f} ;{comment_part}"
            else:
                modified_line = f"{line.strip()} F{self.last_speed:.1f}"
            
            return modified_line
        
        # 其他情况，返回原行
        return line


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
# Modified LightGCode with Speed Override
# --------------------------------
class ModifiedLightGCode(LightGCode):
    """支持速度覆写的G-code加载器"""
    
    def __init__(self, lines, speed_modifier):
        """
        lines: 原始G-code行列表
        speed_modifier: GcodeSpeedModifier实例
        """
        self.speed_modifier = speed_modifier
        # 不直接修改原始lines，而是在发送时动态修改
        super().__init__(lines)
    
    def __getitem__(self, index):
        """重写索引访问，动态修改速度"""
        original_line = super().__getitem__(index)
        return self.speed_modifier.modify_line(original_line)


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
        
        # 速度修改器
        self.speed_modifier = GcodeSpeedModifier()
        
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
        self.pub_speed_override_enabled = rospy.Publisher('/gcode_sender/speed_override_enabled', Bool, queue_size=10)
        self.pub_speed_override_info = rospy.Publisher('/gcode_sender/speed_override_info', String, queue_size=10)
        
        # ROS Subscribers
        rospy.Subscriber('/gcode_sender/load_file', String, self.load_file_callback)
        
        # ROS Services
        rospy.Service('/gcode_sender/start', Trigger, self.handle_start)
        rospy.Service('/gcode_sender/pause', Trigger, self.handle_pause)
        rospy.Service('/gcode_sender/resume', Trigger, self.handle_resume)
        rospy.Service('/gcode_sender/stop', Trigger, self.handle_stop)
        rospy.Service('/gcode_sender/connect', Trigger, self.handle_connect)
        rospy.Service('/gcode_sender/disconnect', Trigger, self.handle_disconnect)
        
        # 速度覆写服务
        rospy.Service('/gcode_sender/enable_speed_override', SetBool, self.handle_enable_speed_override)
        rospy.Service('/gcode_sender/set_speed_percentage', SetSpeedOverride, self.handle_set_speed_percentage)
        rospy.Service('/gcode_sender/set_speed_fixed', SetSpeedOverride, self.handle_set_speed_fixed)
        
        rospy.loginfo("G-code Sender Node initialized (with Speed Override)")
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
        参考OctoPrint的execute_printer_pause逻辑
        """
        with self.lock:
            # 检查当前状态
            if self.state != PrinterState.PRINTING:
                return TriggerResponse(success=False, message=f"Not printing (current state: {self.state.name})")
            
            # 调用printcore的pause方法
            if self.printer and self.printer.printing:
                rospy.loginfo("Calling printer.pause()...")
                self.printer.pause()
                
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

    def handle_enable_speed_override(self, req):
        """服务：启用/禁用速度覆写"""
        with self.lock:
            if req.data:
                # 默认启用百分比模式，100%
                self.speed_modifier.set_mode(SpeedOverrideMode.PERCENTAGE, 100.0)
                msg = "Speed override enabled (100%)"
            else:
                self.speed_modifier.disable()
                msg = "Speed override disabled"
            
            rospy.loginfo(msg)
            return SetBoolResponse(success=True, message=msg)

    def handle_set_speed_percentage(self, req):
        """服务：设置速度百分比"""
        with self.lock:
            percentage = req.value
            if percentage < 1 or percentage > 200:
                return SetSpeedOverrideResponse(
                    success=False,
                    message=f"Invalid percentage: {percentage}. Must be 1-200"
                )
            
            self.speed_modifier.set_mode(SpeedOverrideMode.PERCENTAGE, percentage)
            msg = f"Speed set to {percentage}% of original"
            rospy.loginfo(msg)
            return SetSpeedOverrideResponse(success=True, message=msg)

    def handle_set_speed_fixed(self, req):
        """服务：设置固定速度值"""
        with self.lock:
            speed = req.value
            if speed < 1:
                return SetSpeedOverrideResponse(
                    success=False,
                    message=f"Invalid speed: {speed}. Must be > 0"
                )
            
            self.speed_modifier.set_mode(SpeedOverrideMode.FIXED_VALUE, speed)
            msg = f"Speed set to fixed value: F{speed}"
            rospy.loginfo(msg)
            return SetSpeedOverrideResponse(success=True, message=msg)

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
            
            # 发布速度覆写信息
            speed_enabled = self.speed_modifier.mode != SpeedOverrideMode.DISABLED
            self.pub_speed_override_enabled.publish(speed_enabled)
            
            if speed_enabled:
                if self.speed_modifier.mode == SpeedOverrideMode.PERCENTAGE:
                    speed_info = f"Speed Override: {self.speed_modifier.percentage}% of original"
                else:
                    speed_info = f"Speed Override: F{self.speed_modifier.fixed_value} (fixed)"
                self.pub_speed_override_info.publish(speed_info)
            
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
                    # 开始打印 - 使用支持速度覆写的G-code加载器
                    try:
                        rospy.loginfo(f"Starting print: {self.gcode_filepath}")
                        rospy.loginfo(f"Total lines to send: {self.total_lines}")
                        
                        # 创建支持速度修改的G-code对象
                        modified_gcode = ModifiedLightGCode(self.gcode_lines, self.speed_modifier)
                        self.printer.startprint(modified_gcode)
                        
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