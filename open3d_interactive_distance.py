#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import random


class PointCloudDistanceVisualizer:
    def __init__(self, ply_path):
        # 加载PLY文件
        self.pcd = o3d.io.read_point_cloud(ply_path)
        if len(self.pcd.points) < 2:
            raise ValueError("点云中的点数少于2个，无法计算距离")

        print(f"点云加载成功，总点数: {len(self.pcd.points)}")

        # 存储选中的点
        self.selected_points = []
        self.points = np.asarray(self.pcd.points)

        # 设置点云可视化参数
        self.pcd.paint_uniform_color([0.5, 0.5, 0.5])  # 设置点云为灰色

    def calculate_random_distance(self):
        """计算随机两点之间的距离"""
        # 随机选择两个不同的点的索引
        idx1, idx2 = random.sample(range(len(self.points)), 2)

        # 获取这两个点的坐标
        point1 = self.points[idx1]
        point2 = self.points[idx2]

        # 计算两点之间的欧几里得距离
        distance_meters = np.linalg.norm(point1 - point2)

        # 转换为厘米
        distance_cm = distance_meters * 100

        print(f"随机选择的两个点:")
        print(f"  点1 索引: {idx1}, 坐标: {point1}")
        print(f"  点2 索引: {idx2}, 坐标: {point2}")
        print(f"  两点之间的距离: {distance_cm:.4f} 厘米 ({distance_meters:.4f} 米)")

        return distance_cm

    def visualize_and_calculate(self):
        """可视化点云并允许交互式选择点"""
        # 创建可视化窗口
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="PLY点云距离计算器", width=1024, height=768)
        vis.add_geometry(self.pcd)

        # 设置选项以允许点选择
        vis.get_render_option().point_size = 5.0  # 增大点的大小以便选择

        # 运行可视化器
        print("正在启动可视化窗口...")
        print("在窗口中按住Shift键并左键点击点来选择点")
        print("选择完成后按'Q'键退出并计算距离")
        print("按'Q'键退出")

        vis.run()
        vis.destroy_window()

        # 获取选择的点（在窗口关闭后）
        selected_indices = vis.get_picked_points()

        # 将选择的点索引存储到实例变量中
        self.selected_points = selected_indices

        # 如果选择的点数大于2，则只保留前两个
        if len(self.selected_points) > 2:
            self.selected_points = self.selected_points[:2]

        print(f"最终选中点数: {len(self.selected_points)}")

        if len(self.selected_points) == 2:
            # 获取选中的两个点的坐标
            point1_idx = self.selected_points[0]
            point2_idx = self.selected_points[1]
            point1 = self.points[point1_idx]
            point2 = self.points[point2_idx]

            # 计算两点之间的距离
            distance_meters = np.linalg.norm(point1 - point2)
            distance_cm = distance_meters * 100

            print(f"选中的两个点:")
            print(f"  点1 索引: {point1_idx}, 坐标: {point1}")
            print(f"  点2 索引: {point2_idx}, 坐标: {point2}")
            print(f"  两点之间的距离: {distance_cm:.4f} 厘米 ({distance_meters:.4f} 米)")
        elif len(self.selected_points) == 1:
            point_idx = self.selected_points[0]
            print(f"只选择了一个点，索引: {point_idx}, 坐标: {self.points[point_idx]}")
        elif len(self.selected_points) > 2:
            print(f"选择了超过2个点，仅使用前两个点进行计算")
            point1_idx = self.selected_points[0]
            point2_idx = self.selected_points[1]
            point1 = self.points[point1_idx]
            point2 = self.points[point2_idx]

            distance_meters = np.linalg.norm(point1 - point2)
            distance_cm = distance_meters * 100

            print(f"  点1 索引: {point1_idx}, 坐标: {point1}")
            print(f"  点2 索引: {point2_idx}, 坐标: {point2}")
            print(f"  两点之间的距离: {distance_cm:.4f} 厘米 ({distance_meters:.4f} 米)")
        else:
            print("没有选择任何点")

    def run(self):
        """运行应用"""
        print("PLY点云距离计算器")
        print("1. 交互式选择点来计算距离")
        print("2. 或者直接计算随机两点距离")

        while True:
            print("\n选择操作:")
            print("1 - 交互式选择点")
            print("2 - 计算随机两点距离")
            print("3 - 退出")

            choice = input("请输入选择 (1/2/3): ").strip()

            if choice == "1":
                self.visualize_and_calculate()
            elif choice == "2":
                self.calculate_random_distance()
            elif choice == "3":
                print("退出程序")
                break
            else:
                print("无效选择，请重新输入")


def main():
    # PLY文件路径
    ply_path = "/home/tianqi/ws_realsense/1.ply"

    # 创建并运行应用
    try:
        app = PointCloudDistanceVisualizer(ply_path)
        app.run()
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    main()
