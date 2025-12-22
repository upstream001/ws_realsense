#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import datetime

class TestImageCapture(Node):
    def __init__(self):
        super().__init__('test_image_capture')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 计数器
        self.image_count = 0
        
        # 订阅RGB图像话题
        self.rgb_sub = self.create_subscription(
            Image,
            '/rgb_image',
            self.rgb_callback,
            10)
            
        # 订阅深度图像话题
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_image',
            self.depth_callback,
            10)
            
        self.get_logger().info('Test Image Capture Node 已启动')
        self.get_logger().info('正在接收图像数据...')

    def rgb_callback(self, msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().info(f'接收到RGB图像: {rgb_image.shape}')
            self.image_count += 1
            
            # 保存一张测试图像
            if self.image_count <= 3:  # 只保存前3张
                save_dir = 'test_images'
                os.makedirs(save_dir, exist_ok=True)
                timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                rgb_filename = os.path.join(save_dir, f'test_rgb_{timestamp}.jpg')
                cv2.imwrite(rgb_filename, rgb_image)
                self.get_logger().info(f'保存RGB测试图像: {rgb_filename}')
                
        except Exception as e:
            self.get_logger().error(f'RGB图像转换失败: {e}')

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            self.get_logger().info(f'接收到深度图像: {depth_image.shape}')
            
            # 保存一张测试深度图像
            if self.image_count <= 3:  # 只保存前3张
                save_dir = 'test_images'
                os.makedirs(save_dir, exist_ok=True)
                timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                
                # 保存深度图像为NPY格式
                depth_filename = os.path.join(save_dir, f'test_depth_{timestamp}.npy')
                np.save(depth_filename, depth_image)
                
                # 保存深度图像的可视化版本
                depth_vis_filename = os.path.join(save_dir, f'test_depth_{timestamp}.png')
                # 处理深度图像的NaN值
                depth_clean = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
                # 归一化深度图像以便可视化
                if np.max(depth_clean) > 0:
                    depth_normalized = cv2.normalize(depth_clean, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                else:
                    depth_normalized = np.zeros_like(depth_clean, dtype=np.uint8)
                cv2.imwrite(depth_vis_filename, depth_normalized)
                
                self.get_logger().info(f'保存深度测试图像: {depth_filename}')
                
        except Exception as e:
            self.get_logger().error(f'深度图像转换失败: {e}')

def main():
    # 初始化ROS 2
    rclpy.init()
    
    # 创建节点
    node = TestImageCapture()
    
    try:
        # 运行5秒钟
        rclpy.spin_once(node, timeout_sec=5.0)
        rclpy.spin_once(node, timeout_sec=5.0)
        rclpy.spin_once(node, timeout_sec=5.0)
    except KeyboardInterrupt:
        node.get_logger().info('程序退出')
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()