#!/usr/bin/env python3

"""
test_gcode_sender.py (Improved)

测试脚本：演示如何控制G-code发送节点
支持 pause、resume、stop、quit 功能
"""

import rospy
import sys
import time
from std_msgs.msg import String
from std_srvs.srv import Trigger

def wait_for_service(service_name, timeout=5.0):
    """等待服务可用"""
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        return True
    except rospy.ROSException:
        print(f"❌ Service {service_name} not available")
        return False

def call_service(service_name):
    """调用服务"""
    try:
        service = rospy.ServiceProxy(service_name, Trigger)
        response = service()
        return response.success, response.message
    except rospy.ServiceException as e:
        print(f"❌ Service call failed: {e}")
        return False, str(e)

def print_header(text):
    """打印标题"""
    print("\n" + "=" * 70)
    print(f"  {text}")
    print("=" * 70)

def print_section(text):
    """打印部分标题"""
    print(f"\n[*] {text}")

def main():
    rospy.init_node('gcode_sender_test', anonymous=True)
    
    print_header("G-code Sender Node - Test Script")
    
    # 等待服务
    print_section("Waiting for ROS services...")
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
        if wait_for_service(service):
            print(f"    ✓ {service}")
        else:
            print(f"    ✗ {service}")
            all_available = False
    
    if not all_available:
        print("\n❌ Error: Some services are not available. Is the node running?")
        return
    
    print("\n✓ All services are ready!")
    
    # 发布G-code文件路径
    print_section("Loading G-code file")
    pub_file = rospy.Publisher('/gcode_sender/load_file', String, queue_size=10)
    rospy.sleep(1)  # 等待发布者连接
    
    # 获取G-code文件路径
    if len(sys.argv) > 1:
        gcode_file = sys.argv[1]
    else:
        gcode_file = input("    Enter G-code file path: ").strip()
    
    if not gcode_file:
        print("❌ No file path provided")
        return
    
    print(f"    Publishing file path: {gcode_file}")
    pub_file.publish(gcode_file)
    rospy.sleep(1)
    
    # 连接打印机
    print_section("Connecting to printer")
    success, msg = call_service('/gcode_sender/connect')
    if success:
        print(f"    ✓ {msg}")
    else:
        print(f"    ✗ {msg}")
        return
    
    # 开始发送
    print_section("Starting G-code transmission")
    success, msg = call_service('/gcode_sender/start')
    if success:
        print(f"    ✓ {msg}")
    else:
        print(f"    ✗ {msg}")
        return
    
    # 交互式控制
    print_header("Interactive Control - Print in Progress")
    print("""
Commands:
    p - Pause printing
    r - Resume printing
    s - Stop printing
    q - Quit (disconnect and exit)

Printer is now printing. Enter a command:
    """)
    
    try:
        while not rospy.is_shutdown():
            try:
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'p':
                    print("  Sending pause command...")
                    success, msg = call_service('/gcode_sender/pause')
                    if success:
                        print(f"    ✓ Paused: {msg}")
                    else:
                        print(f"    ✗ {msg}")
                    
                elif cmd == 'r':
                    print("  Sending resume command...")
                    success, msg = call_service('/gcode_sender/resume')
                    if success:
                        print(f"    ✓ Resumed: {msg}")
                    else:
                        print(f"    ✗ {msg}")
                    
                elif cmd == 's':
                    print("  Sending stop command...")
                    success, msg = call_service('/gcode_sender/stop')
                    if success:
                        print(f"    ✓ Stopped: {msg}")
                    else:
                        print(f"    ✗ {msg}")
                    
                    print("  Waiting for print to stop...")
                    time.sleep(2)
                    
                elif cmd == 'q':
                    print("  Disconnecting from printer...")
                    call_service('/gcode_sender/stop')
                    time.sleep(1)
                    call_service('/gcode_sender/disconnect')
                    print("  ✓ Disconnected. Exiting.\n")
                    break
                
                elif cmd == '':
                    # 空命令，不做任何操作
                    pass
                
                else:
                    print("    ❌ Invalid command. Use: p, r, s, or q")
    
    except KeyboardInterrupt:
        print("\n\n  Interrupted by user.")
        print("  Stopping print and disconnecting...")
        call_service('/gcode_sender/stop')
        time.sleep(1)
        call_service('/gcode_sender/disconnect')
        print("  ✓ Disconnected. Exiting.\n")

if __name__ == "__main__":
    main()
