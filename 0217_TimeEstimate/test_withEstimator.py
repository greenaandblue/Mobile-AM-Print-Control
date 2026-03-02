#!/usr/bin/env python3

"""
test_gcode_sender.py (Enhanced - With Buffer Info Display)

测试脚本：演示如何控制G-code发送节点
支持 pause、resume、stop、quit 功能
显示 buffer 延迟信息
"""

import rospy # type: ignore
import sys
import time
from std_msgs.msg import String, Float32, Int32 # type: ignore
from std_srvs.srv import Trigger # type: ignore

class GcodeSenderTest:
    def __init__(self):
        rospy.init_node('gcode_sender_test', anonymous=True)
        
        # 状态变量
        self.current_status = "Idle"
        self.current_progress = 0.0
        self.current_state = "IDLE"
        
        # Buffer 信息
        self.buffer_delay = 0.0
        self.buffer_info = {}
        
        # 订阅状态更新
        rospy.Subscriber('/gcode_sender/status', String, self.status_callback)
        rospy.Subscriber('/gcode_sender/progress', Float32, self.progress_callback)
        rospy.Subscriber('/gcode_sender/state', String, self.state_callback)
        
        # 订阅 buffer 信息
        rospy.Subscriber('/gcode_sender/buffer_delay', Float32, self.buffer_delay_callback)
        rospy.Subscriber('/gcode_sender/buffer_info', String, self.buffer_info_callback)
        
        # 发布器
        self.pub_file = rospy.Publisher('/gcode_sender/load_file', String, queue_size=10)
    
    def status_callback(self, msg):
        self.current_status = msg.data
    
    def progress_callback(self, msg):
        self.current_progress = msg.data
    
    def state_callback(self, msg):
        self.current_state = msg.data
    
    def buffer_delay_callback(self, msg):
        self.buffer_delay = msg.data
    
    def buffer_info_callback(self, msg):
        try:
            self.buffer_info = eval(msg.data)  # 简化处理
        except:
            pass

    def wait_for_service(self, service_name, timeout=5.0):
        """等待服务可用"""
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
            return True
        except rospy.ROSException:
            print(f"❌ Service {service_name} not available")
            return False

    def call_service(self, service_name):
        """调用服务"""
        try:
            service = rospy.ServiceProxy(service_name, Trigger)
            response = service()
            return response.success, response.message
        except rospy.ServiceException as e:
            print(f"❌ Service call failed: {e}")
            return False, str(e)

    def print_header(self, text):
        """打印标题"""
        print("\n" + "=" * 80)
        print(f"  {text}")
        print("=" * 80)

    def print_section(self, text):
        """打印部分标题"""
        print(f"\n[*] {text}")

    def print_current_status(self):
        """打印当前状态（包含 buffer 信息）"""
        print(f"\n{'─' * 80}")
        print(f"State: {self.current_state} | Progress: {self.current_progress:.1f}%")
        
        # 显示 buffer 信息
        if self.buffer_info:
            print(f"Buffer: {self.buffer_info.get('buffer_count', 0)}/{self.buffer_info.get('buffer_capacity', 0)} moves "
                  f"({self.buffer_info.get('buffer_usage', '0%')})")
            print(f"Estimated pause delay: {self.buffer_delay:.3f}s")
        
        if self.current_status != "Idle":
            print(f"{self.current_status}")
        print(f"{'─' * 80}")

    def run(self):
        self.print_header("G-code Sender Node - Test Script (With Buffer Info)")
        
        # 等待服务
        self.print_section("Waiting for ROS services...")
        services = [
            '/gcode_sender/connect',
            '/gcode_sender/start',
            '/gcode_sender/pause',
            '/gcode_sender/resume',
            '/gcode_sender/stop',
            '/gcode_sender/disconnect'
        ]
        
        all_available = True
        for service in services:
            if self.wait_for_service(service):
                print(f"    ✓ {service}")
            else:
                print(f"    ✗ {service}")
                all_available = False
        
        if not all_available:
            print("\n❌ Error: Some services are not available. Is the node running?")
            print("    Please start the node with: python3 gcode_sender_node_with_estimator.py")
            return
        
        print("\n✓ All services are ready!")
        time.sleep(0.5)
        
        # 发布G-code文件路径
        self.print_section("Loading G-code file")
        
        # 获取G-code文件路径
        if len(sys.argv) > 1:
            gcode_file = sys.argv[1]
        else:
            gcode_file = input("    Enter G-code file path: ").strip()
        
        if not gcode_file:
            print("❌ No file path provided")
            return
        
        print(f"    Publishing file path: {gcode_file}")
        self.pub_file.publish(gcode_file)
        time.sleep(1)
        
        # 连接打印机
        self.print_section("Connecting to printer")
        success, msg = self.call_service('/gcode_sender/connect')
        if success:
            print(f"    ✓ {msg}")
        else:
            print(f"    ✗ {msg}")
            return
        
        time.sleep(1)
        
        # 开始发送
        self.print_section("Starting G-code transmission")
        success, msg = self.call_service('/gcode_sender/start')
        if success:
            print(f"    ✓ {msg}")
        else:
            print(f"    ✗ {msg}")
            return
        
        time.sleep(1)
        
        # 交互式控制
        self.print_header("Interactive Control - Print in Progress")
        print("""
Commands:
    p - Pause printing (shows estimated delay)
    r - Resume printing
    s - Stop printing (cancel)
    i - Show current status (with buffer info)
    b - Show detailed buffer status
    q - Quit (stop and disconnect)

Printer is now printing. Enter a command:
        """)
        
        try:
            while not rospy.is_shutdown():
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'p':
                    print(f"  Current state: {self.current_state}")
                    
                    # 显示当前 buffer 状态
                    if self.buffer_info:
                        print(f"  Buffer before pause: {self.buffer_info.get('buffer_count', 0)}/{self.buffer_info.get('buffer_capacity', 0)} moves")
                        print(f"  Estimated delay: {self.buffer_delay:.3f}s")
                    
                    print("  Sending pause command...")
                    success, msg = self.call_service('/gcode_sender/pause')
                    if success:
                        print(f"    ✓ {msg}")
                        print("    Note: Printer will finish buffer commands before pausing")
                    else:
                        print(f"    ✗ Failed: {msg}")
                
                elif cmd == 'r':
                    print(f"  Current state: {self.current_state}")
                    print("  Sending resume command...")
                    success, msg = self.call_service('/gcode_sender/resume')
                    if success:
                        print(f"    ✓ Resumed: {msg}")
                    else:
                        print(f"    ✗ Failed: {msg}")
                
                elif cmd == 's':
                    print(f"  Current state: {self.current_state}")
                    print("  Sending stop command...")
                    success, msg = self.call_service('/gcode_sender/stop')
                    if success:
                        print(f"    ✓ Stopped: {msg}")
                        print("    Print job cancelled")
                    else:
                        print(f"    ✗ Failed: {msg}")
                    
                    time.sleep(2)
                
                elif cmd == 'i':
                    print("  Current status:")
                    self.print_current_status()
                
                elif cmd == 'b':
                    print("\n  Detailed Buffer Status:")
                    if self.buffer_info:
                        for key, value in sorted(self.buffer_info.items()):
                            print(f"    {key:20s}: {value}")
                    else:
                        print("    No buffer info available (printer may not be printing)")
                    print()
                
                elif cmd == 'q':
                    print(f"\n  Current state: {self.current_state}")
                    print("  Stopping print and disconnecting...")
                    
                    # 只有在打印或暂停状态才需要停止
                    if self.current_state in ['PRINTING', 'PAUSED']:
                        self.call_service('/gcode_sender/stop')
                        time.sleep(2)
                    
                    self.call_service('/gcode_sender/disconnect')
                    print("  ✓ Disconnected. Exiting.\n")
                    break
                
                elif cmd == '':
                    # 空命令，显示状态
                    self.print_current_status()
                
                else:
                    print("    ❌ Invalid command. Use: p, r, s, i, b, or q")
        
        except KeyboardInterrupt:
            print("\n\n  Interrupted by user.")
            print("  Stopping print and disconnecting...")
            if self.current_state in ['PRINTING', 'PAUSED']:
                self.call_service('/gcode_sender/stop')
                time.sleep(1)
            self.call_service('/gcode_sender/disconnect')
            print("  ✓ Disconnected. Exiting.\n")
        except Exception as e:
            print(f"\n\n  Error: {e}")
            print("  Stopping print and disconnecting...")
            if self.current_state in ['PRINTING', 'PAUSED']:
                self.call_service('/gcode_sender/stop')
                time.sleep(1)
            self.call_service('/gcode_sender/disconnect')
            print("  ✓ Disconnected. Exiting.\n")

def main():
    try:
        test = GcodeSenderTest()
        test.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()