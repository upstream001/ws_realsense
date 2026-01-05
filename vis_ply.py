#可视化两个点云做对比
import open3d as o3d
import argparse
import numpy as np
import os


def load_point_cloud(file_path):
    """
    加载点云文件，支持 .ply, .pcd, .npy 格式
    """
    _, file_extension = os.path.splitext(file_path)
    if file_extension in [".pcd", ".ply"]:
        pcd = o3d.io.read_point_cloud(file_path)
    elif file_extension == ".npy":
        pts = np.load(file_path)
        # print(f"加载 .npy 文件: {file_path}, 形状: {pts.shape}, 类型: {pts.dtype}")
        
        # 如果是 (B, N, 3) 形状，取第一个 batch
        if pts.ndim == 3:
            pts = pts[0]
            
        # 如果形状是 (H, W)，可能是深度图
        if pts.ndim == 2 and pts.shape[1] != 3:
            # 使用默认内参 (对应 Gazebo 仿真 1280x720)
            h, w = pts.shape
            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                width=w, height=h, fx=976.736, fy=976.736, cx=w/2.0, cy=h/2.0
            )
            depth_o3d = o3d.geometry.Image(pts.astype(np.float32))
            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                depth_o3d, intrinsic, depth_scale=1.0, depth_trunc=3.0, stride=2
            )
        else:
            # 确保是 (N, 3)
            if pts.ndim != 2 or pts.shape[1] != 3:
                raise ValueError(f"NumPy 数组形状不正确: {pts.shape}，期望 (N, 3) 或 (H, W)")
            
            pcd = o3d.geometry.PointCloud()
            # 明确转换为 float64，避免 pybind11 转换失败
            pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    else:
        raise ValueError(f"不支持的文件格式: {file_extension}")
    
    # 移除无效点 (NaN, Inf)
    pcd = pcd.remove_non_finite_points()
    return pcd


def visualize(file1, file2, offset=False):
    try:
        pcd1 = load_point_cloud(file1)
        pcd2 = load_point_cloud(file2)
    except Exception as e:
        print(f"读取点云时发生错误: {e}")
        return

    if pcd1.is_empty() and pcd2.is_empty():
        print("错误: 两个点云都为空。")
        return

    # 为区分两个点云，分别着色
    # 第一个点云着红色 (Red)
    pcd1.paint_uniform_color([1, 0, 0])
    # 第二个点云着蓝色 (Blue)
    pcd2.paint_uniform_color([0, 0, 1])

    if offset:
        # 计算偏移，使点云并排显示
        bbox = pcd1.get_axis_aligned_bounding_box()
        extent = bbox.get_extent()
        # 沿着 X 轴平移 1.2 倍宽度
        pcd2.translate([extent[0] * 1.2, 0, 0])
        print("已启用偏移: 并排显示。")
    else:
        print("未启用偏移: 重叠显示（方便对比形状差异）。")

    print(f"正在显示:")
    print(f"  红色 (Red):  {file1}")
    print(f"  蓝色 (Blue): {file2}")

    # 打印点数信息
    center1 = pcd1.get_center()
    center2 = pcd2.get_center()
    print(f"点云1 (红色) - 点数: {len(pcd1.points)}, 中心: {center1}")
    print(f"点云2 (蓝色) - 点数: {len(pcd2.points)}, 中心: {center2}")
    print("\n操作提示: 使用鼠标左键旋转，右键平移，滚轮缩放。按 'q' 键退出。")

    # 启动可视化窗口
    o3d.visualization.draw_geometries(
        [pcd1, pcd2],
        window_name="PoinTr 点云对比可视化",
        width=1280,
        height=720,
        left=50,
        top=50,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PoinTr 项目点云对比可视化脚本")
    parser.add_argument(
        "--file1",
        default="/home/tianqi/ws_realsense/partial_v2_processed/000.ply",
        type=str,
        help="第一个点云文件路径 (.ply, .pcd, .npy)",
    )
    parser.add_argument(
        "--file2",
        default="/home/tianqi/ws_realsense/complete_no_leave.ply",
        type=str,
        help="第二个点云文件路径 (.ply, .pcd, .npy)",
    )
    parser.add_argument(
        "--offset", action="store_true", help="是否并排显示点云 (默认重叠显示)"
    )

    args = parser.parse_args()
    visualize(args.file1, args.file2, args.offset)
