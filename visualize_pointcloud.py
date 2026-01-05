#滤波处理后可视化
import numpy as np
import open3d as o3d
import os

# 1. 加载 npy 深度文件
# 注意：这里使用的是您刚才生成的最新文件，如果文件名变了请手动更新
depth_path = "/home/tianqi/ws_realsense/collected_data_20251225_174027/depth_005.npy"
rgb_path = "/home/tianqi/ws_realsense/collected_data_20251225_174027/rgb_005.jpg"

if not os.path.exists(depth_path):
    print(f"错误: 找不到文件 {depth_path}")
    # 尝试查找目录下最新的文件
    import glob

    list_of_files = glob.glob("/home/tianqi/ws_realsense/captured_images/*.npy")
    if list_of_files:
        depth_path = max(list_of_files, key=os.path.getctime)
        print(f"自动使用最新的深度文件: {depth_path}")
    else:
        exit(1)

depth_data = np.load(depth_path)

# 2. 配置相机内参 (fx, fy, cx, cy)
# 对应 Gazebo 仿真中的 1280x720 分辨率设置
intrinsic = o3d.camera.PinholeCameraIntrinsic(
    width=1280, height=720, fx=976.736, fy=976.736, cx=640.0, cy=360.0
)

# 3. 将深度图转换为点云
# depth_scale=1.0: 因为 Gazebo 仿真的 float32 深度数据单位已经是米
# depth_trunc=3.0: 截断 3 米以外的数据
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
    print(f"已加载 RGB 图像用于着色")


# # 4. 点云滤波 (剔除离群点)
# # 剔除距离点云主体 1 厘米（0.01m）以上的离散点
# # nb_points=16 表示在 1cm 半径内至少要有 16 个点，否则被剔除
# pcd, ind = pcd.remove_radius_outlier(nb_points=16, radius=0.01)
# pcd = pcd.select_by_index(ind)

# # 如果您的意思是只保留深度范围在 1cm 以内的点（例如只看物体表面）：
# points = np.asarray(pcd.points)
# if len(points) > 0:
#     min_z = np.min(points[:, 2])
#     pcd = pcd.select_by_index(np.where(points[:, 2] < min_z + 0.013)[0])

# 5. 下采样到 500 个点
if len(pcd.points) > 500:
    pcd = pcd.farthest_point_down_sample(500)
    print(f"下采样完成，当前点数: {len(pcd.points)}")
else:
    print(f"点数少于 500 ({len(pcd.points)})，跳过下采样")

before_normalization_points = np.asarray(pcd.points)
distances = np.linalg.norm(before_normalization_points, axis=1)
max_distance = np.max(distances)
normalized_points = before_normalization_points / max_distance
transform_info = {
    "max_distance": max_distance,
    "scale_factor": 1.0 / max_distance if max_distance > 0 else 1.0,
}
print(f"点云最大距离: {max_distance} m, 归一化因子: {transform_info['scale_factor']}")

pcd.points = o3d.utility.Vector3dVector(normalized_points)

pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
pcd = pcd.select_by_index(ind)


# 计算平均深度 (cm)
points = np.asarray(pcd.points)
if len(points) > 0:
    # Z轴为深度值 (单位: 米 -> 厘米)
    avg_depth = np.mean(points[:, 2]) * 100
    print(f"过滤后点数: {len(pcd.points)}，平均深度值: {avg_depth:.2f} cm")

    # 计算点云中两点之间的最远距离
    # 使用计算所有点对之间距离的方法，但为避免性能问题，如果点数过多则采样
    if len(points) > 1000:
        # 如果点数过多，随机采样1000个点进行计算
        indices = np.random.choice(len(points), 1000, replace=False)
        sampled_points = points[indices]
    else:
        sampled_points = points

    # 计算最远点对距离
    max_distance = 0
    max_pair = None
    for i in range(len(sampled_points)):
        for j in range(i + 1, len(sampled_points)):
            dist = np.linalg.norm(sampled_points[i] - sampled_points[j])
            if dist > max_distance:
                max_distance = dist
                if len(sampled_points) == len(points):
                    max_pair = (i, j)  # 记录原始索引
                else:
                    # 如果是采样的点，则不能记录原始索引
                    max_pair = ("采样点", "采样点")

    print(f"点云中两点之间的最远距离: {max_distance:.2f} cm")
    if max_pair != ("采样点", "采样点"):
        print(
            f"最远点对坐标: 点{max_pair[0]} {points[max_pair[0]]} 和 点{max_pair[1]} {points[max_pair[1]]}"
        )
else:
    print("警告: 滤波后点云为空")

# 6. 可视化点云
# 翻转点云以符合 Open3D 的坐标系习惯
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

print(f"最终显示点数: {len(pcd.points)}，正在可视化...")
o3d.visualization.draw_geometries([pcd], window_name="D405 Simulation PointCloud")
