#!/usr/bin/env python3

"""
test_gcode_sender.py (Improved - Based on OctoPrint Logic)

测试脚本：演示如何控制G-code发送节点
支持 pause、resume、stop、quit 功能
"""

import rospy
import sys
import time
from std_msgs.msg import String, Float32, Int32
from std_srvs.srv import Trigger

class GcodeSenderTest:
    def __init__(self):
        rospy.init_node('gcode_sender_test', anonymous=True)
        
        # 状态变量
        self.current_status = "Idle"
        self.current_progress = 0.0
        self.current_state = "IDLE"
        
        # 订阅状态更新
        rospy.Subscriber('/gcode_sender/status', String, self.status_callback)
        rospy.Subscriber('/gcode_sender/progress', Float32, self.progress_callback)
        rospy.Subscriber('/gcode_sender/state', String, self.state_callback)
        
        # 发布器
        self.pub_file = rospy.Publisher('/gcode_sender/load_file', String, queue_size=10)
    
    def status_callback(self, msg):
        self.current_status = msg.data
    
    def progress_callback(self, msg):
        self.current_progress = msg.data
    
    def state_callback(self, msg):
        self.current_state = msg.data

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
        print("\n" + "=" * 70)
        print(f"  {text}")
        print("=" * 70)

    def print_section(self, text):
        """打印部分标题"""
        print(f"\n[*] {text}")

    def print_current_status(self):
        """打印当前状态"""
        print(f"\n{'─' * 70}")
        print(f"State: {self.current_state} | Progress: {self.current_progress:.1f}%")
        if self.current_status != "Idle":
            print(f"{self.current_status}")
        print(f"{'─' * 70}")

    def run(self):
        self.print_header("G-code Sender Node - Test Script (OctoPrint-Style)")
        
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
            print("    Please start the node with: rosrun <package> gcode_sender_node.py")
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
    p - Pause printing
    r - Resume printing
    s - Stop printing (cancel)
    i - Show current status
    q - Quit (stop and disconnect)

Printer is now printing. Enter a command:
        """)
        
        try:
            while not rospy.is_shutdown():
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'p':
                    print(f"  Current state: {self.current_state}")
                    print("  Sending pause command...")
                    success, msg = self.call_service('/gcode_sender/pause')
                    if success:
                        print(f"    ✓ Paused: {msg}")
                        print("    Note: Printer will finish current moves before pausing")
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
                    print("    ❌ Invalid command. Use: p, r, s, i, or q")
        
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