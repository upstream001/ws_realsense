#!/usr/bin/env python3
import subprocess
import time
import math
import os
import datetime
import sys
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角(roll, pitch, yaw)转换为四元数(w, x, y, z)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z

def move_model_cli(model_name, x, y, z, roll=0, pitch=0, yaw=0):
    """
    使用 ign service 命令行工具移动模型
    """
    w, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)
    req_str = f'name: "{model_name}" position {{ x: {x} y: {y} z: {z} }} orientation {{ w: {w} x: {qx} y: {qy} z: {qz} }}'

    cmd = [
        "ign",
        "service",
        "-s",
        "/world/strawberry_world/set_pose",
        "--reqtype",
        "ignition.msgs.Pose",
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        "5000",
        "--req",
        req_str,
    ]

    try:
        result = subprocess.run(cmd, check=False, capture_output=True, text=True)
        return result.returncode == 0
    except Exception as e:
        print(f"Exception occurred moving model: {e}")
        return False

class DataCollector(Node):
    def __init__(self, save_dir):
        super().__init__('data_collector')
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.save_dir = save_dir
        
        # 订阅RGB和深度图像
        self.rgb_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        self.get_logger().info('数据采集节点已启动')

    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def capture(self, index):
        """
        等待图像捕获并保存
        """
        # 等待最新的图像数据
        timeout = 2.0
        start_time = time.time()
        while (self.latest_rgb is None or self.latest_depth is None) and (time.time() - start_time < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.latest_rgb is not None and self.latest_depth is not None:
            rgb_path = os.path.join(self.save_dir, f'rgb_{index:03d}.jpg')
            depth_path = os.path.join(self.save_dir, f'depth_{index:03d}.npy')
            
            cv2.imwrite(rgb_path, self.latest_rgb)
            np.save(depth_path, self.latest_depth)
            
            # 同时保存一个可视化的深度图
            depth_vis = cv2.normalize(np.nan_to_num(self.latest_depth), None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            cv2.imwrite(os.path.join(self.save_dir, f'depth_vis_{index:03d}.png'), depth_vis)
            
            print(f"Saved point {index} data.")
            # 清除缓存，确保下一次采集的是新数据
            self.latest_rgb = None
            self.latest_depth = None
            return True
        else:
            print(f"Failed to capture point {index} (Timeout).")
            return False

def main():
    # 参数解析
    parser = argparse.ArgumentParser(description='围绕目标物体旋转采集数据')
    parser.add_argument('--x', type=float, default=3.0, help='初始相机X位置')
    parser.add_argument('--y', type=float, default=0.0, help='初始相机Y位置')
    parser.add_argument('--z', type=float, default=0.5, help='初始相机Z位置')
    parser.add_argument('--steps', type=int, default=12, help='采集步数')
    parser.add_argument('--target_name', type=str, default='realsense_d405', help='相机模型名称')
    parser.add_argument('--strawberry_xyz', type=float, nargs=3, default=[0.0, 0.0, 0.1], help='草莓(目标)位置')
    
    args = parser.parse_args()

    # 参数设定
    target_model = args.target_name
    strawberry_pos = tuple(args.strawberry_xyz)
    
    # 计算初始的半径和高度
    dx_init = args.x - strawberry_pos[0]
    dy_init = args.y - strawberry_pos[1]
    radius = math.sqrt(dx_init**2 + dy_init**2)
    height = args.z - strawberry_pos[2]
    initial_angle = math.atan2(dy_init, dx_init)
    
    steps = args.steps
    
    # 创建保存目录
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    save_dir = os.path.join(os.getcwd(), f'collected_data_{timestamp}')
    os.makedirs(save_dir, exist_ok=True)
    
    # 初始化 ROS 2
    rclpy.init()
    collector = DataCollector(save_dir)
    
    print(f"目标模型: {target_model}")
    print(f"目标位置: {strawberry_pos}")
    print(f"初始设定: ({args.x}, {args.y}, {args.z}) -> 半径: {radius:.2f}, 相对高度: {height:.2f}")
    print(f"计划采集 {steps} 个点...")
    
    try:
        for i in range(steps):
            # 从初始角度开始旋转
            angle = initial_angle + (2 * math.pi * i) / steps
            
            # 计算相机位置
            cam_x = strawberry_pos[0] + radius * math.cos(angle)
            cam_y = strawberry_pos[1] + radius * math.sin(angle)
            cam_z = strawberry_pos[2] + height
            
            # 计算朝向 (面向草莓)
            dx = strawberry_pos[0] - cam_x
            dy = strawberry_pos[1] - cam_y
            dz = strawberry_pos[2] - cam_z
            dist_xy = math.sqrt(dx**2 + dy**2)
            
            yaw = math.atan2(dy, dx)
            pitch = math.atan2(-dz, dist_xy)
            roll = 0.0
            
            print(f"Moving to step {i+1}/{steps}: pos=({cam_x:.2f}, {cam_y:.2f}, {cam_z:.2f}), yaw={math.degrees(yaw):.1f}°")
            
            # 移动模型
            if move_model_cli(target_model, cam_x, cam_y, cam_z, roll, pitch, yaw):
                # 等待系统稳定和图像传输
                time.sleep(1.5)
                # 记录数据
                collector.capture(i)
            else:
                print(f"Error: Could not move {target_model}")
                
        print(f"\n数据采集完成! 保存在: {save_dir}")
        
    except KeyboardInterrupt:
        print("采集任务被用户中断")
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
