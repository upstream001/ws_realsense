import open3d as o3d
import numpy as np
import os

def debug_process(path):
    print(f"Testing {path}...")
    
    print("Step 1: Reading point cloud...")
    pcd = o3d.io.read_point_cloud(path)
    print(f"Points: {len(pcd.points)}")
    
    print("Step 2: Statistical outlier removal...")
    # Try different parameters or check if this crashes
    try:
        pcd_filtered, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        pcd = pcd.select_by_index(ind)
        print(f"Points after filtering: {len(pcd.points)}")
    except Exception as e:
        print(f"Crash in filtering: {e}")
        return

    print("Step 3: Numpy conversion...")
    points = np.asarray(pcd.points)
    print(f"Shape: {points.shape}")

    print("Step 4: FPS downsampling...")
    # FPS in Open3D can sometimes be buggy on certain platforms/versions
    try:
        pcd_down = pcd.farthest_point_down_sample(1024)
        print(f"Points after FPS: {len(pcd_down.points)}")
    except Exception as e:
        print(f"Crash in FPS: {e}")
        return

    print("Process finished successfully.")

if __name__ == "__main__":
    test_path = "/home/tianqi/ws_realsense/partial/000.ply"
    if os.path.exists(test_path):
        debug_process(test_path)
    else:
        print(f"File not found: {test_path}")
