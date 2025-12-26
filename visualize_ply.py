#!/usr/bin/env python3
"""
PLY文件可视化脚本
此脚本用于加载和可视化PLY格式的3D点云或网格文件
"""

import argparse
import os
import sys
import numpy as np
import open3d as o3d


def load_and_visualize_ply(
    file_path,
    downsample_points=50000,
    window_name="PLY Visualization",
    flip_axes=None,
    normalize=True,
    show_coordinate_frame=True,
):
    """
    加载并可视化PLY文件

    Args:
        file_path (str): PLY文件路径
        downsample_points (int): 下采样后的点数上限
        window_name (str): 显示窗口名称
        flip_axes (str): 轴翻转选项，如 "xy", "xz", "yz", "xyz" 等
        normalize (bool): 是否归一化点云大小
        show_coordinate_frame (bool): 是否显示坐标系
    """
    # 检查文件是否存在
    if not os.path.exists(file_path):
        print(f"错误: 找不到文件 {file_path}")
        return False

    print(f"正在加载PLY文件: {file_path}")

    # 尝试读取PLY文件
    try:
        # 检测PLY文件类型（点云或网格）
        mesh = o3d.io.read_triangle_mesh(file_path)
        point_cloud = o3d.io.read_point_cloud(file_path)

        # 根据PLY文件内容判断类型
        if len(mesh.vertices) > 0 and len(mesh.triangles) > 0:
            # 这是一个网格文件
            print(
                f"检测到网格文件，顶点数: {len(mesh.vertices)}, 面数: {len(mesh.triangles)}"
            )
            geometry = mesh
            is_mesh = True

        elif len(point_cloud.points) > 0:
            # 这是一个点云文件
            print(f"检测到点云文件，点数: {len(point_cloud.points)}")
            geometry = point_cloud
            is_mesh = False
        else:
            print("错误: 无法识别PLY文件内容")
            return False

        # 计算边界框和中心
        bbox = geometry.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        print(f"几何体中心: {center}")
        print(f"边界框: Min {bbox.min_bound}, Max {bbox.max_bound}")

        # 如果是网格但没有颜色，计算法线
        if is_mesh and len(mesh.vertex_colors) == 0:
            mesh.compute_vertex_normals()

        # 如果是点云且没有颜色，设置默认颜色
        if not is_mesh and len(point_cloud.colors) == 0:
            geometry.paint_uniform_color([0.5, 0.5, 0.5])  # 灰色

        # 如果点数过多，进行下采样以提高性能（仅对点云）
        if not is_mesh and len(geometry.points) > downsample_points:
            print(
                f"点数过多 ({len(geometry.points)})，进行下采样到 {downsample_points} 点"
            )
            geometry = geometry.farthest_point_down_sample(downsample_points)
            print(f"下采样后点数: {len(geometry.points)}")

        # 应用坐标系转换
        if flip_axes:
            transform_matrix = np.eye(4)  # 4x4 单位矩阵
            if "x" in flip_axes.lower():
                transform_matrix[0, 0] = -1
            if "y" in flip_axes.lower():
                transform_matrix[1, 1] = -1
            if "z" in flip_axes.lower():
                transform_matrix[2, 2] = -1

            print(f"应用坐标轴翻转: {flip_axes.upper()}")
            geometry.transform(transform_matrix)

        # 归一化处理
        if normalize:
            points = np.asarray(geometry.points)
            distances = np.linalg.norm(points - center, axis=1)
            max_distance = np.max(distances) if len(distances) > 0 else 1.0

            if max_distance > 0:
                scale_factor = 1.0 / max_distance
                transform_matrix = np.array(
                    [
                        [scale_factor, 0, 0, 0],
                        [0, scale_factor, 0, 0],
                        [0, 0, scale_factor, 0],
                        [0, 0, 0, 1],
                    ]
                )
                geometry.transform(transform_matrix)
                print(f"几何体已归一化，缩放因子: {scale_factor:.4f}")

        # 准备显示的几何体列表
        geometries = [geometry]

        # 添加坐标系
        if show_coordinate_frame:
            coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.3, origin=center
            )
            geometries.append(coordinate_frame)

        # 显示几何体
        print(f"正在可视化...")
        o3d.visualization.draw_geometries(
            geometries,
            window_name=window_name,
            width=1024,
            height=768,
            point_show_normal=False,
        )

        print("可视化完成")
        return True

    except Exception as e:
        print(f"加载PLY文件时出错: {str(e)}")
        return False


def visualize_folder(
    folder_path,
    downsample_points=50000,
    window_name="PLY Visualization",
    flip_axes=None,
    normalize=True,
    show_coordinate_frame=True,
    batch_mode=False,
):
    """
    可视化文件夹中的所有PLY文件

    Args:
        folder_path (str): 包含PLY文件的文件夹路径
        downsample_points (int): 下采样后的点数上限
        window_name (str): 显示窗口名称
        flip_axes (str): 轴翻转选项
        normalize (bool): 是否归一化点云大小
        show_coordinate_frame (bool): 是否显示坐标系
        batch_mode (bool): 是否为批量模式（自动处理所有文件）
    """
    # 获取文件夹中的所有PLY文件
    ply_files = [f for f in os.listdir(folder_path) if f.lower().endswith(".ply")]
    ply_files.sort()  # 排序文件列表

    if not ply_files:
        print(f"错误: 文件夹 {folder_path} 中没有找到PLY文件")
        return False

    print(f"在文件夹 {folder_path} 中找到 {len(ply_files)} 个PLY文件:")
    for i, f in enumerate(ply_files):
        print(f"  {i+1:2d}. {f}")

    # 如果是批量模式，自动处理所有文件
    if batch_mode:
        for i, filename in enumerate(ply_files):
            file_path = os.path.join(folder_path, filename)
            print(f"\n[{i+1}/{len(ply_files)}] 正在处理: {filename}")

            success = load_and_visualize_ply(
                file_path,
                downsample_points=downsample_points,
                window_name=f"{window_name} - {filename}",
                flip_axes=flip_axes,
                normalize=normalize,
                show_coordinate_frame=show_coordinate_frame,
            )

            if not success:
                print(f"跳过文件: {filename}")

            # 询问是否继续
            if i < len(ply_files) - 1:
                response = input("\n继续处理下一个文件? (Y/n): ")
                if response.lower() == "n":
                    break
    else:
        # 交互模式 - 让用户选择要处理的文件
        while True:
            try:
                choice = input(
                    "\n请选择要可视化的文件编号 (1-{}, 输入 'all' 处理所有文件, 'q' 退出): ".format(
                        len(ply_files)
                    )
                )

                if choice.lower() == "q":
                    break
                elif choice.lower() == "all":
                    for i, filename in enumerate(ply_files):
                        file_path = os.path.join(folder_path, filename)
                        print(f"\n[{i+1}/{len(ply_files)}] 正在处理: {filename}")

                        success = load_and_visualize_ply(
                            file_path,
                            downsample_points=downsample_points,
                            window_name=f"{window_name} - {filename}",
                            flip_axes=flip_axes,
                            normalize=normalize,
                            show_coordinate_frame=show_coordinate_frame,
                        )

                        if not success:
                            print(f"跳过文件: {filename}")

                        # 询问是否继续
                        if i < len(ply_files) - 1:
                            response = input("\n继续处理下一个文件? (Y/n): ")
                            if response.lower() == "n":
                                break
                    break
                else:
                    idx = int(choice) - 1
                    if 0 <= idx < len(ply_files):
                        file_path = os.path.join(folder_path, ply_files[idx])
                        print(f"处理文件: {ply_files[idx]}")

                        success = load_and_visualize_ply(
                            file_path,
                            downsample_points=downsample_points,
                            window_name=f"{window_name} - {ply_files[idx]}",
                            flip_axes=flip_axes,
                            normalize=normalize,
                            show_coordinate_frame=show_coordinate_frame,
                        )

                        if not success:
                            print(f"处理文件失败: {ply_files[idx]}")
                    else:
                        print("无效的选择")
            except ValueError:
                print("请输入有效的数字、'all' 或 'q'")

    return True


def main():
    parser = argparse.ArgumentParser(description="PLY文件可视化工具")
    parser.add_argument(
        "file_path", nargs="?", help="PLY文件路径或包含PLY文件的文件夹路径"
    )
    parser.add_argument(
        "--downsample", type=int, default=50000, help="下采样后的最大点数 (默认: 50000)"
    )
    parser.add_argument(
        "--window-name",
        type=str,
        default="PLY Visualization",
        help="显示窗口名称 (默认: PLY Visualization)",
    )
    parser.add_argument(
        "--flip-axes",
        type=str,
        default=None,
        help="轴翻转选项 (例如: x, y, z, xy, xyz)",
    )
    parser.add_argument("--no-normalize", action="store_true", help="不进行归一化处理")
    parser.add_argument(
        "--no-coordinate-frame", action="store_true", help="不显示坐标系"
    )
    parser.add_argument(
        "--batch", action="store_true", help="批量处理模式（自动处理所有PLY文件）"
    )
    parser.add_argument("--folder", type=str, help="指定包含PLY文件的文件夹路径")

    args = parser.parse_args()

    # 确定要处理的路径
    target_path = args.folder if args.folder else args.file_path

    # 如果没有提供路径，尝试使用默认文件夹
    if not target_path:
        default_folder = "/home/tianqi/ws_realsense/pointclouds_ply/"
        if os.path.isdir(default_folder):
            print(f"使用默认文件夹: {default_folder}")
            target_path = default_folder
        else:
            # 尝试查找当前目录下的PLY文件
            ply_files = [f for f in os.listdir(".") if f.lower().endswith(".ply")]
            if ply_files:
                print("在当前目录找到以下PLY文件:")
                for i, f in enumerate(ply_files):
                    print(f"  {i+1}. {f}")

                if len(ply_files) == 1:
                    target_path = ply_files[0]
                    print(f"自动选择文件: {target_path}")
                elif len(ply_files) > 1:
                    choice = input("请选择要可视化的文件编号 (或直接输入文件路径): ")
                    try:
                        idx = int(choice) - 1
                        if 0 <= idx < len(ply_files):
                            target_path = ply_files[idx]
                        else:
                            print("无效的选择")
                            return
                    except ValueError:
                        # 用户输入了文件路径
                        target_path = choice
            else:
                print("错误: 没有提供PLY文件路径，也没有在当前目录找到PLY文件")
                print("使用方法: python visualize_ply.py [PLY文件路径]")
                print(
                    "      或: python visualize_ply.py --folder [包含PLY文件的文件夹路径]"
                )
                return

    # 检查目标是文件还是文件夹
    if os.path.isdir(target_path):
        # 处理文件夹
        success = visualize_folder(
            target_path,
            downsample_points=args.downsample,
            window_name=args.window_name,
            flip_axes=args.flip_axes,
            normalize=not args.no_normalize,
            show_coordinate_frame=not args.no_coordinate_frame,
            batch_mode=args.batch,
        )

        if not success:
            print("文件夹处理失败")
            sys.exit(1)
    elif os.path.isfile(target_path):
        # 处理单个文件
        if not target_path.lower().endswith(".ply"):
            print(f"警告: 文件 {target_path} 可能不是PLY格式")
            response = input("是否继续加载? (y/N): ")
            if response.lower() != "y":
                return

        success = load_and_visualize_ply(
            target_path,
            downsample_points=args.downsample,
            window_name=args.window_name,
            flip_axes=args.flip_axes,
            normalize=not args.no_normalize,
            show_coordinate_frame=not args.no_coordinate_frame,
        )

        if not success:
            print("PLY文件可视化失败")
            sys.exit(1)
    else:
        print(f"错误: 路径 {target_path} 不存在")
        sys.exit(1)


if __name__ == "__main__":
    main()
