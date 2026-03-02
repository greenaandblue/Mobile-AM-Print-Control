#!/usr/bin/env python3

"""
test_gcode_sender.py (With Speed Override)

测试脚本：演示如何控制G-code发送节点
支持：
- pause、resume、stop、quit 功能
- 速度覆写（Speed Override）功能
"""

import rospy
import sys
import time
from std_msgs.msg import String, Bool, Float32, Int32
from std_srvs.srv import Trigger, SetBool
from gcode_sender.srv import SetSpeedOverride

class GcodeSenderTest:
    def __init__(self):
        rospy.init_node('gcode_sender_test', anonymous=True)
        
        # 状态变量
        self.current_status = "Idle"
        self.current_progress = 0.0
        self.current_state = "IDLE"
        self.speed_override_enabled = False
        self.speed_override_info = ""
        
        # 订阅状态更新
        rospy.Subscriber('/gcode_sender/status', String, self.status_callback)
        rospy.Subscriber('/gcode_sender/progress', Float32, self.progress_callback)
        rospy.Subscriber('/gcode_sender/state', String, self.state_callback)
        rospy.Subscriber('/gcode_sender/speed_override_enabled', Bool, self.speed_enabled_callback)
        rospy.Subscriber('/gcode_sender/speed_override_info', String, self.speed_info_callback)
        
        # 发布器
        self.pub_file = rospy.Publisher('/gcode_sender/load_file', String, queue_size=10)
    
    def status_callback(self, msg):
        self.current_status = msg.data
    
    def progress_callback(self, msg):
        self.current_progress = msg.data
    
    def state_callback(self, msg):
        self.current_state = msg.data
    
    def speed_enabled_callback(self, msg):
        self.speed_override_enabled = msg.data
    
    def speed_info_callback(self, msg):
        self.speed_override_info = msg.data

    def wait_for_service(self, service_name, timeout=5.0):
        """等待服务可用"""
        try:
            rospy.wait_for_service(service_name, timeout=timeout)
            return True
        except rospy.ROSException:
            print(f"❌ Service {service_name} not available")
            return False

    def call_service(self, service_name):
        """调用Trigger服务"""
        try:
            service = rospy.ServiceProxy(service_name, Trigger)
            response = service()
            return response.success, response.message
        except rospy.ServiceException as e:
            print(f"❌ Service call failed: {e}")
            return False, str(e)
    
    def call_set_bool_service(self, service_name, value):
        """调用SetBool服务"""
        try:
            service = rospy.ServiceProxy(service_name, SetBool)
            response = service(value)
            return response.success, response.message
        except rospy.ServiceException as e:
            print(f"❌ Service call failed: {e}")
            return False, str(e)
    
    def call_speed_override_service(self, service_name, value):
        """调用SetSpeedOverride服务"""
        try:
            service = rospy.ServiceProxy(service_name, SetSpeedOverride)
            response = service(value)
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
        if self.speed_override_enabled:
            print(f"🚀 {self.speed_override_info}")
        if self.current_status != "Idle":
            print(f"{self.current_status}")
        print(f"{'─' * 70}")

    def interactive_speed_control(self):
        """交互式速度控制界面"""
        print("\n" + "─" * 70)
        print("  Speed Override Control")
        print("─" * 70)
        print("\n1. Enable/Disable speed override")
        print("2. Set speed percentage (e.g., 50% = half speed)")
        print("3. Set fixed speed value (e.g., F1500)")
        print("4. Return to main menu")
        
        choice = input("\nEnter your choice (1-4): ").strip()
        
        if choice == '1':
            current = "enabled" if self.speed_override_enabled else "disabled"
            print(f"\nSpeed override is currently {current}")
            action = input("Enable (e) or Disable (d)? ").strip().lower()
            
            if action == 'e':
                success, msg = self.call_set_bool_service('/gcode_sender/enable_speed_override', True)
                if success:
                    print(f"  ✓ {msg}")
                else:
                    print(f"  ✗ {msg}")
            elif action == 'd':
                success, msg = self.call_set_bool_service('/gcode_sender/enable_speed_override', False)
                if success:
                    print(f"  ✓ {msg}")
                else:
                    print(f"  ✗ {msg}")
        
        elif choice == '2':
            try:
                percentage = float(input("Enter speed percentage (1-200): ").strip())
                success, msg = self.call_speed_override_service('/gcode_sender/set_speed_percentage', percentage)
                if success:
                    print(f"  ✓ {msg}")
                else:
                    print(f"  ✗ {msg}")
            except ValueError:
                print("  ✗ Invalid input")
        
        elif choice == '3':
            try:
                speed = float(input("Enter fixed speed value (e.g., 1500): ").strip())
                success, msg = self.call_speed_override_service('/gcode_sender/set_speed_fixed', speed)
                if success:
                    print(f"  ✓ {msg}")
                else:
                    print(f"  ✗ {msg}")
            except ValueError:
                print("  ✗ Invalid input")

    def run(self):
        self.print_header("G-code Sender Node - Test Script (With Speed Override)")
        
        # 等待服务
        self.print_section("Waiting for ROS services...")
        services = [
            '/gcode_sender/connect',
            '/gcode_sender/start',
            '/gcode_sender/pause',
            '/gcode_sender/resume',
            '/gcode_sender/stop',
            '/gcode_sender/disconnect',
            '/gcode_sender/enable_speed_override',
            '/gcode_sender/set_speed_percentage',
            '/gcode_sender/set_speed_fixed'
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
    o - Speed Override control (modify printing speed)
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
                        print("    Tip: You can now modify speed settings with 'o' command")
                    else:
                        print(f"    ✗ Failed: {msg}")
                
                elif cmd == 'r':
                    print(f"  Current state: {self.current_state}")
                    print("  Sending resume command...")
                    success, msg = self.call_service('/gcode_sender/resume')
                    if success:
                        print(f"    ✓ Resumed: {msg}")
                        if self.speed_override_enabled:
                            print(f"    🚀 {self.speed_override_info}")
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
                
                elif cmd == 'o':
                    self.interactive_speed_control()
                
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
                    print("    ❌ Invalid command. Use: p, r, s, i, o, or q")
        
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