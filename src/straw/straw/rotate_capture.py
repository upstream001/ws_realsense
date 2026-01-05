#!/usr/bin/env python3
import subprocess
import time
import math
import os
import datetime
import sys
import argparse
import rclpy
import re
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

    return [w, x, y, z]


def get_model_pose_ign(model_name):
    """使用 ign model -m [name] -p 获取位姿"""
    try:
        cmd = ["ign", "model", "-m", model_name, "-p"]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            return None
        
        output = result.stdout
        matches = re.findall(r'\[\s*(-?\d+\.?\d*(?:e[-+]?\d+)?\s+-?\d+\.?\d*(?:e[-+]?\d+)?\s+-?\d+\.?\d*(?:e[-+]?\d+)?)\s*\]', output)
        
        if len(matches) >= 2:
            pos_vals = [float(v) for v in matches[0].split()]
            rpy_vals = [float(v) for v in matches[1].split()]
            return {'pos': pos_vals, 'rpy': rpy_vals}
    except Exception:
        pass
    return None


def quaternion_inverse(q):
    w, x, y, z = q
    norm_sq = w*w + x*x + y*y + z*z
    return [w/norm_sq, -x/norm_sq, -y/norm_sq, -z/norm_sq]


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ]


def rotate_vector(v, q):
    v_q = [0, v[0], v[1], v[2]]
    q_inv = quaternion_inverse(q)
    res_q = quaternion_multiply(quaternion_multiply(q, v_q), q_inv)
    return res_q[1:]


def quaternion_to_rotation_matrix(q):
    """将四元数 [w, x, y, z] 转换为 3x3 旋转矩阵"""
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,         2*x*z + 2*y*w],
        [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w,         1 - 2*x*x - 2*y*y]
    ])


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


def create_pcd_and_transform(rgb, depth, matrix):
    """
    将深度图处理为对齐后的点云并保存为 ply 格式需要的 header 字符串和数据
    使用 numpy 向量化运算提高效率
    """
    # 相机内参
    fx, fy = 976.736, 976.736
    cx, cy = 640.0, 360.0

    h, w = depth.shape
    i, j = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')

    # 过滤无效像素 (0, nan, inf, 范围限制)
    mask = (depth > 0.1) & (depth < 5.0) & np.isfinite(depth)
    if not np.any(mask):
        return None

    z = depth[mask]
    x = (j[mask] - cx) * z / fx
    y = (i[mask] - cy) * z / fy
    points_opt = np.stack([x, y, z], axis=-1)
    
    # 颜色处理
    colors = rgb[mask][:, ::-1] # BGR to RGB

    # 构造变换矩阵
    # 1. Optical -> Gazebo Link
    R_opt_to_gz = np.array([
        [0, 0, 1, 0],
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])
    # 2. Sensor Link -> Camera Model
    T_sensor_model = np.eye(4)
    T_sensor_model[:3, 3] = [0.01465, 0, 0.021]
    
    # 3. Final transform
    final_T = matrix @ T_sensor_model @ R_opt_to_gz
    
    # 应用变换
    # 将点转换为齐次坐标 [N, 4]
    ones = np.ones((points_opt.shape[0], 1))
    points_homo = np.hstack([points_opt, ones])
    points_trans = (final_T @ points_homo.T).T[:, :3]
    
    return points_trans, colors


def save_ply(path, points, colors):
    """手动保存 PLY 文件以避免依赖 Open3D"""
    header = f"""ply
format ascii 1.0
element vertex {len(points)}
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
"""
    with open(path, 'w') as f:
        f.write(header)
        for p, c in zip(points, colors):
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {int(c[0])} {int(c[1])} {int(c[2])}\n")


class DataCollector(Node):
    def __init__(self, save_dir):
        super().__init__("data_collector")
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.save_dir = save_dir

        self.rgb_sub = self.create_subscription(
            Image, "/camera/rgb/image_raw", self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 10
        )

        self.get_logger().info("数据采集节点已启动")

    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def capture(self, index, matrix=None):
        timeout = 2.0
        start_time = time.time()
        while (self.latest_rgb is None or self.latest_depth is None) and (
            time.time() - start_time < timeout
        ):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_rgb is not None and self.latest_depth is not None:
            # 1. 基础保存
            rgb_path = os.path.join(self.save_dir, f"rgb_{index:03d}.jpg")
            cv2.imwrite(rgb_path, self.latest_rgb)
            
            # 2. 如果有矩阵，直接进行变换并保存点云
            if matrix is not None:
                pcd_data = create_pcd_and_transform(self.latest_rgb, self.latest_depth, matrix)
                if pcd_data:
                    points, colors = pcd_data
                    ply_path = os.path.join(self.save_dir, f"strawberry_pcd_{index:03d}.ply")
                    save_ply(ply_path, points, colors)
                    print(f"Saved aligned pointcloud: {ply_path}")
                else:
                    print(f"Warning: No valid depth points for frame {index}")
            else:
                # 备选：如果没有矩阵，由于用户要求不输出 matrix，我们只存原始深度图
                depth_path = os.path.join(self.save_dir, f"depth_{index:03d}.npy")
                np.save(depth_path, self.latest_depth)

            # 3. 清理缓存
            self.latest_rgb = None
            self.latest_depth = None
            return True
        else:
            print(f"Failed to capture point {index} (Timeout).")
            return False


def main():
    parser = argparse.ArgumentParser(description="围绕目标物体旋转采集数据并直接输出对齐点云")
    parser.add_argument("--x", type=float, default=3.0, help="初始相机X位置")
    parser.add_argument("--y", type=float, default=0.0, help="初始相机Y位置")
    parser.add_argument("--z", type=float, default=-1.0, help="初始相机Z位置")
    parser.add_argument("--steps", type=int, default=12, help="采集步数")
    parser.add_argument("--target_name", type=str, default="realsense_d405", help="相机模型名称")
    parser.add_argument("--strawberry_name", type=str, default="strawberry", help="草莓模型名称")
    parser.add_argument("--strawberry_xyz", type=float, nargs=3, default=[0.0, 0.0, 0.0], help="草莓(目标)大概位置")

    args = parser.parse_args()

    target_model = args.target_name
    strawberry_model = args.strawberry_name
    strawberry_pos_guess = tuple(args.strawberry_xyz)

    dx_init = args.x - strawberry_pos_guess[0]
    dy_init = args.y - strawberry_pos_guess[1]
    radius = math.sqrt(dx_init**2 + dy_init**2)
    height = args.z - strawberry_pos_guess[2]
    initial_angle = math.atan2(dy_init, dx_init)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = os.path.join(os.getcwd(), f"collected_pcd_{timestamp}")
    os.makedirs(save_dir, exist_ok=True)

    rclpy.init()
    collector = DataCollector(save_dir)

    print(f"计划采集 {args.steps} 个点并直接生成对齐后的点云 (.ply)...")

    try:
        for i in range(args.steps):
            angle = initial_angle + (2 * math.pi * i) / args.steps
            cam_x = strawberry_pos_guess[0] + radius * math.cos(angle)
            cam_y = strawberry_pos_guess[1] + radius * math.sin(angle)
            cam_z = strawberry_pos_guess[2] + height

            dx = strawberry_pos_guess[0] - cam_x
            dy = strawberry_pos_guess[1] - cam_y
            dz = strawberry_pos_guess[2] - cam_z
            dist_xy = math.sqrt(dx**2 + dy**2)
            yaw = math.atan2(dy, dx)
            pitch = math.atan2(-dz, dist_xy)

            if not move_model_cli(target_model, cam_x, cam_y, cam_z, 0, pitch, yaw):
                continue
            
            time.sleep(1.5)

            cam_data = get_model_pose_ign(target_model)
            str_data = get_model_pose_ign(strawberry_model)
            
            matrix = None
            if cam_data and str_data:
                p_cam, q_cam = np.array(cam_data['pos']), euler_to_quaternion(*cam_data['rpy'])
                p_str, q_str = np.array(str_data['pos']), euler_to_quaternion(*str_data['rpy'])
                
                R_wc = quaternion_to_rotation_matrix(q_cam)
                T_wc = np.eye(4); T_wc[:3, :3] = R_wc; T_wc[:3, 3] = p_cam
                
                R_ws = quaternion_to_rotation_matrix(q_str)
                T_ws = np.eye(4); T_ws[:3, :3] = R_ws; T_ws[:3, 3] = p_str
                
                matrix = np.linalg.inv(T_ws) @ T_wc
            
            collector.capture(i, matrix)

        print(f"\n任务完成! 结果保存在: {save_dir}")

    except KeyboardInterrupt:
        pass
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
