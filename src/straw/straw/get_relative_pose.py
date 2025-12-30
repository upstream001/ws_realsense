#python3 /home/tianqi/ws_realsense/src/straw/straw/get_relative_pose.py
#!/usr/bin/env python3
"""
获取相机和草莓之间相对位姿的脚本
使用 'ign model -p' 命令获取实时数据
"""

import subprocess
import math
import re

def get_model_pose_ign(model_name):
    """使用 ign model -m [name] -p 获取位姿"""
    try:
        cmd = ["ign", "model", "-m", model_name, "-p"]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            return None
        
        output = result.stdout
        # 使用更精确的正则表达式查找包含数字的方括号内容
        # 我们寻找形如 [1.23 4.56 7.89] 的行
        matches = re.findall(r'\[\s*(-?\d+\.?\d*(?:e[-+]?\d+)?\s+-?\d+\.?\d*(?:e[-+]?\d+)?\s+-?\d+\.?\d*(?:e[-+]?\d+)?)\s*\]', output)
        
        if len(matches) >= 2:
            # 第一组通常是 Position [X Y Z]
            pos_vals = [float(v) for v in matches[0].split()]
            # 第二组通常是 Orientation [R P Y]
            rpy_vals = [float(v) for v in matches[1].split()]
            return {'pos': pos_vals, 'rpy': rpy_vals}
    except Exception as e:
        print(f"获取 {model_name} 位姿出错: {e}")
    return None

def euler_to_quaternion(roll, pitch, yaw):
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

def quaternion_to_euler(q):
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def quaternion_to_rotation_matrix(q):
    """将四元数 [w, x, y, z] 转换为 3x3 旋转矩阵"""
    w, x, y, z = q
    return [
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,         2*x*z + 2*y*w],
        [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,         2*y*z + 2*x*w,         1 - 2*x*x - 2*y*y]
    ]

def main():
    camera_name = "realsense_d405"
    strawberry_name = "strawberry"
    
    print(f"正在获取实时位姿并计算转换矩阵: {camera_name} -> {strawberry_name} ...")
    
    cam_data = get_model_pose_ign(camera_name)
    str_data = get_model_pose_ign(strawberry_name)
    
    if not cam_data or not str_data:
        print("错误: 无法获取模型位姿，请确保仿真正在运行且模型名称正确。")
        return

    p_cam = cam_data['pos']
    rpy_cam = cam_data['rpy']
    p_str = str_data['pos']
    rpy_str = str_data['rpy']

    # 计算相对位姿
    q_cam = euler_to_quaternion(*rpy_cam)
    q_str = euler_to_quaternion(*rpy_str)
    
    # 1. 相对位置 (在相机坐标系下)
    dp_world = [p_str[0] - p_cam[0], p_str[1] - p_cam[1], p_str[2] - p_cam[2]]
    q_cam_inv = quaternion_inverse(q_cam)
    rel_pos = rotate_vector(dp_world, q_cam_inv)
    
    # 2. 相对姿态
    rel_ori_q = quaternion_multiply(q_cam_inv, q_str)
    rel_rpy = quaternion_to_euler(rel_ori_q)
    
    # 3. 构造 4x4 变换矩阵
    rot_matrix = quaternion_to_rotation_matrix(rel_ori_q)
    
    print("-" * 50)
    print(f"相机世界坐标: {p_cam}")
    print(f"草莓世界坐标: {p_str}")
    print("-" * 50)
    print(f"草莓相对于相机的位姿 (相机局部坐标系):")
    print(f"位置 [x, y, z]: {rel_pos[0]:.4f}, {rel_pos[1]:.4f}, {rel_pos[2]:.4f}")
    print(f"旋转 (角度 RPY): {math.degrees(rel_rpy[0]):.2f}, {math.degrees(rel_rpy[1]):.2f}, {math.degrees(rel_rpy[2]):.2f}")
    
    print("\n相机到草莓的 4x4 转换矩阵 (T_cam_strawberry):")
    # 打印格式化的矩阵
    for i in range(3):
        row = rot_matrix[i]
        print(f"| {row[0]:8.4f}  {row[1]:8.4f}  {row[2]:8.4f}  {rel_pos[i]:8.4f} |")
    print(f"| {0:8.4f}  {0:8.4f}  {0:8.4f}  {1:8.4f} |")
    
    print("-" * 50)
    dist = math.sqrt(sum(d**2 for d in dp_world))
    print(f"欧式距离: {dist:.4f} 米")

if __name__ == "__main__":
    main()
