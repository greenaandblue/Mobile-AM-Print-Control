#!/usr/bin/env python3

"""
test_gcode_sender.py

测试脚本：演示如何控制G-code发送节点
"""

import rospy
import sys
from std_msgs.msg import String
from std_srvs.srv import Trigger

def wait_for_service(service_name, timeout=5.0):
    """等待服务可用"""
    try:
        rospy.wait_for_service(service_name, timeout=timeout)
        return True
    except rospy.ROSException:
        print(f"Service {service_name} not available")
        return False

def call_service(service_name):
    """调用服务"""
    try:
        service = rospy.ServiceProxy(service_name, Trigger)
        response = service()
        return response.success, response.message
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return False, str(e)

def main():
    rospy.init_node('gcode_sender_test', anonymous=True)
    
    print("=" * 60)
    print("G-code Sender Node Test Script")
    print("=" * 60)
    
    # 等待服务
    print("\n[1] Waiting for services...")
    services = [
        '/gcode_sender/connect',
        '/gcode_sender/start',
        '/gcode_sender/pause',
        '/gcode_sender/resume',
        '/gcode_sender/stop',
        '/gcode_sender/disconnect'
    ]
    
    for service in services:
        if not wait_for_service(service):
            print(f"Error: Service {service} not available. Is the node running?")
            return
    
    print("✓ All services available")
    
    # 发布G-code文件路径
    pub_file = rospy.Publisher('/gcode_sender/load_file', String, queue_size=10)
    rospy.sleep(1)  # 等待发布者连接
    
    # 获取G-code文件路径
    if len(sys.argv) > 1:
        gcode_file = sys.argv[1]
    else:
        gcode_file = input("\n[2] Enter G-code file path: ").strip()
    
    print(f"\n[3] Loading G-code file: {gcode_file}")
    pub_file.publish(gcode_file)
    rospy.sleep(1)
    
    # 连接打印机
    print("\n[4] Connecting to printer...")
    success, msg = call_service('/gcode_sender/connect')
    print(f"   Result: {msg}")
    
    if not success:
        print("Failed to connect. Exiting.")
        return
    
    # 开始发送
    print("\n[5] Starting G-code transmission...")
    success, msg = call_service('/gcode_sender/start')
    print(f"   Result: {msg}")
    
    if not success:
        print("Failed to start. Exiting.")
        return
    
    # 交互式控制
    print("\n" + "=" * 60)
    print("Interactive Control")
    print("=" * 60)
    print("Commands:")
    print("  p - Pause")
    print("  r - Resume")
    print("  s - Stop")
    print("  q - Quit (disconnect)")
    print("=" * 60)
    
    try:
        while not rospy.is_shutdown():
            cmd = input("\nEnter command: ").strip().lower()
            
            if cmd == 'p':
                success, msg = call_service('/gcode_sender/pause')
                print(f"Pause: {msg}")
                
            elif cmd == 'r':
                success, msg = call_service('/gcode_sender/resume')
                print(f"Resume: {msg}")
                
            elif cmd == 's':
                success, msg = call_service('/gcode_sender/stop')
                print(f"Stop: {msg}")
                
            elif cmd == 'q':
                print("\nDisconnecting and exiting...")
                call_service('/gcode_sender/disconnect')
                break
                
            else:
                print("Invalid command")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted. Disconnecting...")
        call_service('/gcode_sender/disconnect')

if __name__ == "__main__":
    main()