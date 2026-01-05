import open3d as o3d
import numpy as np
import os

def main():
    # 路径指向模型文件
    mesh_path = "/home/tianqi/ws_realsense/src/straw/models/strawberry/meshes/strawberry_no_leaves.dae"
    save_path = "/home/tianqi/ws_realsense/strawberry_gt.ply"

    if not os.path.exists(mesh_path):
        print(f"错误: 找不到模型文件 {mesh_path}")
        return

    print(f"正在加载网格模型 (使用 trimesh): {mesh_path}")
    import trimesh
    
    # trimesh 对 .dae 格式的支持更好
    mesh_raw = trimesh.load(mesh_path)
    
    # 如果加载的是场景 (Scene)，合并所有网格
    if isinstance(mesh_raw, trimesh.Scene):
        mesh_trimesh = mesh_raw.dump(concatenate=True)
    else:
        mesh_trimesh = mesh_raw

    # 转换为 Open3D 网格以利用其采样函数
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(mesh_trimesh.vertices)
    mesh.triangles = o3d.utility.Vector3iVector(mesh_trimesh.faces)
    
    if mesh.is_empty():
        print("错误: 转换后的网格为空")
        return

    print("正在从网格采样点云...")
    # 从模型表面均匀采样 100,000 个点
    # 这是获取“真实草莓点云”的最精确方法
    pcd = mesh.sample_points_uniformly(number_of_points=100000)

    # 如果需要，可以将点云涂上一个单一颜色以便区分
    # pcd.paint_uniform_color([0.5, 0.5, 0.5])

    print(f"正在保存点云到: {save_path}")
    o3d.io.write_point_cloud(save_path, pcd)
    
    print("\n完成！")
    print(f"模型点数: {len(pcd.points)}")
    print(f"你现在可以使用这个 {os.path.basename(save_path)} 作为真实值 (Ground Truth) 进行对比。")

if __name__ == "__main__":
    main()
