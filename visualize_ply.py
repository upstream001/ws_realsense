import open3d as o3d
import os
import glob
import sys
import argparse

def visualize_folder(folder_path):
    """
    可视化指定文件夹中的所有 .ply 文件，按 Enter 键切换到下一个。
    """
    # 获取并排序所有 .ply 文件
    ply_files = sorted(glob.glob(os.path.join(folder_path, "*.ply")))
    
    if not ply_files:
        print(f"错误: 在目录 '{folder_path}' 中未找到 .ply 文件。")
        return

    print(f"找到 {len(ply_files)} 个 .ply 文件。")
    print("-" * 50)
    print("操作指南:")
    print("  [Enter] / [N] : 查看下一个点云")
    print("  [Q]           : 退出程序")
    print("-" * 50)

    current_idx = 0

    # 加载第一个点云
    def load_pcd(file_path):
        pcd = o3d.io.read_point_cloud(file_path)
        if pcd.is_empty():
            print(f"警告: 文件 '{file_path}' 为空或无法读取。")
        return pcd

    pcd = load_pcd(ply_files[current_idx])
    
    # 初始化可视化窗口
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="PLY Folder Visualizer", width=1280, height=720)
    vis.add_geometry(pcd)
    
    print(f"[{current_idx + 1}/{len(ply_files)}] 正在显示: {os.path.basename(ply_files[current_idx])}")

    def update_to_next(vis):
        nonlocal current_idx, pcd
        current_idx = (current_idx + 1) % len(ply_files)
        
        # 加载下一个点云数据
        new_pcd = load_pcd(ply_files[current_idx])
        
        # 更新几何体数据
        pcd.points = new_pcd.points
        pcd.colors = new_pcd.colors
        if new_pcd.has_normals():
            pcd.normals = new_pcd.normals
        else:
            pcd.normals = o3d.utility.Vector3dVector([])
            
        vis.update_geometry(pcd)
        vis.update_renderer()
        
        print(f"[{current_idx + 1}/{len(ply_files)}] 正在显示: {os.path.basename(ply_files[current_idx])}")
        return False

    # 注册按键回调
    # GLFW_KEY_ENTER = 257, GLFW_KEY_KP_ENTER = 258
    vis.register_key_callback(257, update_to_next) # 主回车
    vis.register_key_callback(258, update_to_next) # 小键盘回车
    vis.register_key_callback(ord('N'), update_to_next) # N 键作为备选

    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PLY 文件夹可视化工具")
    parser.add_argument("folder", nargs="?", default=".", help="要可视化的文件夹路径 (默认为当前目录)")
    
    args = parser.parse_args()
    
    target_folder = os.path.abspath(args.folder)
    if not os.path.isdir(target_folder):
        print(f"错误: 路径 '{target_folder}' 不是一个目录。")
        sys.exit(1)
        
    visualize_folder(target_folder)
