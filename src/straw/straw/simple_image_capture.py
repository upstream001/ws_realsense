#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import datetime
import select
import sys


class SimpleImageCapture(Node):
    def __init__(self):
        super().__init__('simple_image_capture')

        # 初始化CV Bridge
        self.bridge = CvBridge()

        # 存储最新的图像
        self.latest_rgb = None
        self.latest_depth = None

        # 创建保存目录
        self.save_dir = 'captured_images'
        os.makedirs(self.save_dir, exist_ok=True)

        # 订阅RGB图像话题
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.rgb_callback,
            10)

        # 订阅深度图像话题
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)

        self.get_logger().info('Simple Image Capture Node 已启动')
        self.get_logger().info('正在等待图像数据...')
        self.get_logger().info('按 Enter 键保存图像，按 Ctrl+C 退出')

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().debug(f'接收到RGB图像: {self.latest_rgb.shape}')
        except Exception as e:
            self.get_logger().error(f'RGB图像转换失败: {e}')

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            self.get_logger().debug(f'接收到深度图像: {self.latest_depth.shape}')
        except Exception as e:
            self.get_logger().error(f'深度图像转换失败: {e}')

    def save_images(self):
        # 检查是否有有效的图像数据
        if self.latest_rgb is None:
            self.get_logger().warn('暂无有效RGB图像数据')
            return False
        if self.latest_depth is None:
            self.get_logger().warn('暂无有效深度图像数据')
            return False

        # 生成时间戳
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

        # 保存RGB图像
        rgb_filename = os.path.join(self.save_dir, f'rgb_{timestamp}.jpg')
        cv2.imwrite(rgb_filename, self.latest_rgb)

        # 保存深度图像为NPY格式
        depth_filename = os.path.join(self.save_dir, f'depth_{timestamp}.npy')
        np.save(depth_filename, self.latest_depth)

        # 同时保存深度图像的可视化版本
        depth_vis_filename = os.path.join(
            self.save_dir, f'depth_{timestamp}.png')
        # 处理深度图像的NaN值
        depth_clean = np.nan_to_num(
            self.latest_depth, nan=0.0, posinf=0.0, neginf=0.0)
        # 归一化深度图像以便可视化
        if np.max(depth_clean) > 0:
            depth_normalized = cv2.normalize(
                depth_clean, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        else:
            depth_normalized = np.zeros_like(depth_clean, dtype=np.uint8)
        cv2.imwrite(depth_vis_filename, depth_normalized)

        self.get_logger().info(f'图像已保存: {rgb_filename}, {depth_filename}')
        return True


def main():
    # 初始化ROS 2
    rclpy.init()

    # 创建节点
    node = SimpleImageCapture()

    # 主循环
    try:
        while rclpy.ok():
            # 处理ROS回调
            rclpy.spin_once(node, timeout_sec=0.1)

            # 非阻塞地检查stdin是否有输入
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                # 读取输入（会阻塞直到有输入）
                sys.stdin.readline()
                node.save_images()

    except KeyboardInterrupt:
        node.get_logger().info('程序退出')
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
