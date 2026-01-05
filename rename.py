import os
import shutil
from pathlib import Path


def rename_and_copy_pcd_only(source_dirs, target_dir):
    """
    只处理 .ply 文件，将其重命名为 partial_XXX.ply 并汇总
    """
    # 创建目标文件夹
    Path(target_dir).mkdir(exist_ok=True, parents=True)

    # 收集所有的 .ply 文件
    all_ply_files = []

    for source_dir in source_dirs:
        if not os.path.exists(source_dir):
            print(f"Warning: Directory {source_dir} does not exist. Skipping.")
            continue
            
        print(f"Scanning {source_dir} for .ply files...")

        # 获取当前文件夹中的所有 .ply 文件并排序
        files = sorted([f for f in os.listdir(source_dir) if f.endswith('.ply')])

        for file in files:
            old_path = os.path.join(source_dir, file)
            all_ply_files.append(old_path)

    # 重新编号并移动
    print(f"Total .ply files found: {len(all_ply_files)}")
    
    for idx, old_path in enumerate(all_ply_files):
        # 重命名为简洁的格式，方便后续处理
        new_filename = f"{idx:03d}.ply"
        new_path = os.path.join(target_dir, new_filename)

        # 复制文件
        shutil.copy2(old_path, new_path)
        if idx < 5 or idx > len(all_ply_files) - 5:
            print(f"  Copied: {os.path.basename(old_path)} -> {new_filename}")
        elif idx == 5:
            print("  ...")

    print(f"\nSuccess: Merged into {target_dir}")


def main():
    # 用户指定的两个目录
    source_dirs = [
        "collected_pcd_20251230_180219",
        "collected_pcd_20251230_173332",
        "collected_pcd_20251230_180338",
        "collected_pcd_20251230_180423"
    ]

    # 合并后的结果目录
    target_dir = "partial" 

    rename_and_copy_pcd_only(source_dirs, target_dir)


if __name__ == "__main__":
    main()
