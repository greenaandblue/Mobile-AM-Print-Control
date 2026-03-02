#!/usr/bin/env python3

"""
diagnose_ros.py - 诊断ROS环境和gcode_sender_node问题
"""

import os
import sys

print("=" * 60)
print("ROS 环境诊断")
print("=" * 60)

# 1. 检查Python版本
print(f"\n1. Python 版本: {sys.version}")

# 2. 检查ROS环境变量
print("\n2. ROS 环境变量:")
ros_vars = ['ROS_MASTER_URI', 'ROS_DISTRO', 'ROS_ROOT', 'ROS_PACKAGE_PATH']
for var in ros_vars:
    value = os.environ.get(var, "未设置")
    print(f"   {var} = {value}")

# 3. 检查配置文件
print("\n3. 检查配置文件:")
config_file = "printer_config.ini"
if os.path.exists(config_file):
    print(f"   ✓ {config_file} 存在")
    print(f"   路径: {os.path.abspath(config_file)}")
else:
    print(f"   ✗ {config_file} 不存在")

# 4. 尝试导入rospy
print("\n4. 检查 rospy 模块:")
try:
    import rospy
    print(f"   ✓ rospy 导入成功")
    print(f"   rospy 路径: {rospy.__file__}")
except ImportError as e:
    print(f"   ✗ rospy 导入失败: {e}")
    print("   ** 这是问题所在！**")

# 5. 检查printrun
print("\n5. 检查 printrun 模块:")
try:
    from printrun.printcore import printcore
    print(f"   ✓ printrun 导入成功")
except ImportError as e:
    print(f"   ✗ printrun 导入失败: {e}")

# 6. 尝试连接ROS master
print("\n6. 尝试连接 ROS Master:")
try:
    import rospy
    rospy.init_node('diagnose_test', anonymous=True)
    print("   ✓ 成功连接到 ROS Master")
    
    # 列出当前节点
    import rosnode
    nodes = rosnode.get_node_names()
    print(f"   当前运行的节点: {nodes}")
    
except Exception as e:
    print(f"   ✗ 连接失败: {e}")

print("\n" + "=" * 60)
print("诊断完成")
print("=" * 60)