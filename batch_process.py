import numpy as np
import open3d as o3d
import os
import glob


def process_depth_to_pointcloud(depth_path, rgb_path, output_ply_path):
    """
    处理单个深度文件到点云的转换
    """
    # 1. 加载 npy 深度文件
    depth_data = np.load(depth_path)

    # 2. 配置相机内参 (fx, fy, cx, cy)
    # 对应 Gazebo 仿真中的 1280x720 分辨率设置
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width=1280, height=720, fx=976.736, fy=976.736, cx=640.0, cy=360.0
    )

    # 3. 将深度图转换为点云
    depth_o3d = o3d.geometry.Image(depth_data.astype(np.float32))
    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        depth_o3d, intrinsic, depth_scale=1.0, depth_trunc=3.0, stride=4
    )

    # 如果有 RGB 图像，可以给点云上色
    if os.path.exists(rgb_path):
        rgb_data = o3d.io.read_image(rgb_path)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_data,
            depth_o3d,
            depth_scale=1.0,
            depth_trunc=3.0,
            convert_rgb_to_intensity=False,
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    # 4. 下采样到 500 个点
    if len(pcd.points) > 500:
        pcd = pcd.farthest_point_down_sample(500)

    # 5. 统计滤波 - 剔除离群点
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = pcd.select_by_index(ind)

    # 6. 点云归一化
    points = np.asarray(pcd.points)
    if len(points) > 0:
        distances = np.linalg.norm(points, axis=1)
        max_distance = np.max(distances)
        if max_distance > 0:
            normalized_points = points / max_distance
            pcd.points = o3d.utility.Vector3dVector(normalized_points)

    # 7. 保存点云文件为ply格式
    o3d.io.write_point_cloud(output_ply_path, pcd)

    return pcd


def batch_process():
    # 读取文件夹collected_data_merged
    input_folder = "collected_data_merged"
    output_ply_folder = "pointclouds_ply"
    output_rgb_folder = "rgb_images"

    # 创建输出文件夹
    os.makedirs(output_ply_folder, exist_ok=True)
    os.makedirs(output_rgb_folder, exist_ok=True)

    # 获取所有深度文件
    depth_files = glob.glob(os.path.join(input_folder, "depth_*.npy"))
    depth_files.sort()  # 确保文件按顺序处理

    print(f"找到 {len(depth_files)} 个深度文件")

    for i, depth_path in enumerate(depth_files):
        # 构建对应的RGB文件路径
        depth_filename = os.path.basename(depth_path)
        rgb_filename = depth_filename.replace("depth_", "rgb_").replace(".npy", ".jpg")
        rgb_path = os.path.join(input_folder, rgb_filename)

        # 构建输出文件路径
        ply_filename = depth_filename.replace(".npy", ".ply")
        output_ply_path = os.path.join(output_ply_folder, ply_filename)

        # 复制RGB文件到输出文件夹
        if os.path.exists(rgb_path):
            import shutil

            output_rgb_path = os.path.join(output_rgb_folder, rgb_filename)
            shutil.copy2(rgb_path, output_rgb_path)

        print(f"处理文件 {i+1}/{len(depth_files)}: {depth_filename}")

        # 处理深度文件为点云
        try:
            pcd = process_depth_to_pointcloud(depth_path, rgb_path, output_ply_path)
            print(f"  已保存点云到: {output_ply_path}")
        except Exception as e:
            print(f"  处理文件时出错 {depth_path}: {str(e)}")

    print(f"\n批量处理完成！")
    print(f"点云文件保存在: {output_ply_folder}")
    print(f"RGB图片保存在: {output_rgb_folder}")


if __name__ == "__main__":
    batch_process()
