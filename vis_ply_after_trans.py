#
import numpy as np
import open3d as o3d
import os
import argparse
import cv2

def create_pcd_from_depth(rgb, depth, fx, fy, cx, cy):
    """
    将深度图和RGB图转换为点云
    """
    h, w = depth.shape
    i, j = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
    
    # 核心修正：不仅要过滤 NaN 和 0，还要过滤 inf，并限制最大深度（例如 5米）
    # 否则 inf 坐标会导致 Open3D 缩放异常，显示一片空白
    mask = (depth > 0.1) & (depth < 5.0) & np.isfinite(depth)
    
    if np.sum(mask) == 0:
        return None

    z = depth[mask]
    x = (j[mask] - cx) * z / fx
    y = (i[mask] - cy) * z / fy
    
    points = np.stack([x, y, z], axis=-1)
    
    # 颜色处理
    colors = rgb[mask] / 255.0  # Open3D 需要 [0, 1] 范围的颜色
    colors = colors[:, ::-1]  # BGR to RGB
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    return pcd

def main():
    parser = argparse.ArgumentParser(description="可视化对齐后的点云数据")
    parser.add_argument("dir", type=str, help="数据目录路径")
    parser.add_argument("--index", type=int, default=0, help="要可视化的数据索引 (默认: 0)")
    parser.add_argument("--gt", type=str, default="complete_no_leave.ply", help="参考点云路径 (默认: complete_no_leave.ply)")
    parser.add_argument("--save", action="store_true", help="是否保存点云为 .ply")
    args = parser.parse_args()

    data_dir = args.dir
    idx = args.index
    gt_path = args.gt

    # 尝试加载参考点云 (GT)
    gt_pcd = None
    if os.path.exists(gt_path):
        gt_pcd = o3d.io.read_point_cloud(gt_path)
        # 将参考点云设为绿色，方便对比
        gt_pcd.paint_uniform_color([0, 1, 0])
        print(f"已加载参考点云: {gt_path}")
    else:
        print(f"警告: 未找到参考点云 {gt_path}")

    # 相机参数 (从 strawberry_world.sdf 获取)
    fx, fy = 976.736, 976.736
    cx, cy = 640.0, 360.0

    print(f"正在处理帧 {idx} ...")

    rgb_path = os.path.join(data_dir, f"rgb_{idx:03d}.jpg")
    depth_path = os.path.join(data_dir, f"depth_{idx:03d}.npy")
    matrix_path = os.path.join(data_dir, f"trans_matrix_{idx:03d}.npy")

    if not (os.path.exists(rgb_path) and os.path.exists(depth_path) and os.path.exists(matrix_path)):
        print(f"错误: 第 {idx} 帧的数据不完整 (确保 rgb, depth, trans_matrix 都存在)")
        return

    # 加载数据
    rgb = cv2.imread(rgb_path)
    depth = np.load(depth_path)
    matrix = np.load(matrix_path)

    # 1. 生成原始点云 (此时处于相机光学坐标系: Z向前, X向右, Y向下)
    pcd = create_pcd_from_depth(rgb, depth, fx, fy, cx, cy)
    if pcd is None:
        print(f"错误: 第 {idx} 帧没有有效的深度数据")
        return
    
    # 2. 坐标系转换 (光学坐标系 -> Gazebo Link 坐标系)
    # Optical: Z=forward, X=right, Y=down
    # Gazebo:  X=forward, Y=left, Z=up
    # 转换逻辑: X_gz = Z_opt, Y_gz = -X_opt, Z_gz = -Y_opt
    R_opt_to_gz = np.array([
        [0, 0, 1, 0],
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    # 3. 传感器相对于模型中心的偏移 (根据 SDF 中的 <pose>0.01465 0 0.021 0 0 0</pose>)
    T_sensor_model = np.eye(4)
    T_sensor_model[:3, 3] = [0.01465, 0, 0.021]
    
    # 4. 复合转换
    # 顺序: 原始点 -> 转到Gazebo方向 -> 移到模型中心 -> 移动到草莓中心
    # 注意: matrix 是 T_sc (Camera_Model -> Strawberry_Model)
    final_transform = matrix @ T_sensor_model @ R_opt_to_gz
    
    print(f"应用复合变换矩阵对齐...")
    pcd.transform(final_transform)

    if args.save:
        save_path = os.path.join(data_dir, f"aligned_{idx:03d}.ply")
        o3d.io.write_point_cloud(save_path, pcd)
        print(f"点云已保存至: {save_path}")

    # 可视化
    print("打开可视化窗口 (按 'q' 退出)...")
    
    # 添加一个坐标网格参考 (草莓局部坐标系的原点)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    
    vis_list = [pcd, mesh_frame]
    if gt_pcd is not None:
        vis_list.append(gt_pcd)
        
    o3d.visualization.draw_geometries(vis_list)

if __name__ == "__main__":
    main()
