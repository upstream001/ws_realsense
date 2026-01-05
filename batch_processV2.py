import open3d as o3d  # 建议先导入 open3d
import numpy as np
import os
import glob

def process_ply_v2(input_path, output_path, n_points=2048):
    """
    处理对齐后的 PLY 文件：去噪、下采样、归一化
    """
    # 1. 加载 PLY 文件
    pcd = o3d.io.read_point_cloud(input_path)
    if not pcd.has_points():
        return None

    # 2. 统计滤波 - 剔除离群点
    # 使用与 debug 脚本一致的参数
    pcd_sampled=pcd.farthest_point_down_sample(n_points)
    cl, ind = pcd_sampled.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
    pcd_filtered = pcd_sampled.select_by_index(ind)

    # 3. 归一化 (Centering & Scaling to Unit Sphere)
    # points = np.asarray(pcd_filtered.points)
    
    # if len(points) == 0:
    #     return None

    # a. 将几何中心平移到原点 (去中心化)
    # center = np.mean(points, axis=0)
    # points_centered = points - center
    
    # b. 缩放到单位球 (确保所有点到原点的最大距离为1)
    # dist = np.max(np.linalg.norm(points_centered, axis=1))
    # if dist > 0:
    #     points_normalized = points_centered / dist
    # else:
    #     points_normalized = points_centered
    
    # 构建归一化后的新点云对象，避免内存冲突
    # pcd_norm = o3d.geometry.PointCloud()
    # pcd_norm.points = o3d.utility.Vector3dVector(points_normalized)
    
    # 如果原点云有颜色，直接复制过滤后的颜色（select_by_index 已经处理好了颜色匹配）
    # if pcd_filtered.has_colors():
    #     pcd_norm.colors = pcd_filtered.colors

    # 4. 下采样到固定点数 (FPS - Farthest Point Sampling)
    # if len(pcd_norm.points) > n_points:
    #     pcd_final = pcd_norm.farthest_point_down_sample(n_points)
    # else:
    #     pcd_final = pcd_norm
    
    # 5. 保存处理后的点云
    o3d.io.write_point_cloud(output_path, pcd_filtered)
    return pcd_filtered

def batch_process():
    input_folder = "partial"            
    output_folder = "partial_v2_processed"   
    
    os.makedirs(output_folder, exist_ok=True)

    ply_files = sorted(glob.glob(os.path.join(input_folder, "*.ply")))
    
    if not ply_files:
        print(f"在 {input_folder} 中没有找到 .ply 文件！")
        return

    print(f"找到 {len(ply_files)} 个点云文件待处理 (源目录: {input_folder})")

    for i, file_path in enumerate(ply_files):
        filename = os.path.basename(file_path)
        output_path = os.path.join(output_folder, filename)
        
        print(f"处理进度 [{i+1}/{len(ply_files)}]: {filename}")
        
        try:
            pcd = process_ply_v2(file_path, output_path, n_points=2048)
            if pcd:
                print(f"  -> 成功: 点数已统一为 {len(pcd.points)}")
            else:
                print(f"  -> 警告: {filename} 处理后为空")
        except Exception as e:
            print(f"  -> 出错 {filename}: {str(e)}")

    print(f"\n批量处理完成！")
    print(f"处理后的文件保存在: {output_folder}")

if __name__ == "__main__":
    batch_process()
