import os
import shutil
from pathlib import Path


def rename_and_copy_files(source_dirs, target_dir):
    """
    将多个源文件夹中的文件合并并按顺序重命名，然后复制到目标文件夹

    Args:
        source_dirs: 源文件夹路径列表
        target_dir: 目标文件夹路径
    """
    # 创建目标文件夹
    Path(target_dir).mkdir(exist_ok=True)

    # 定义文件类型
    file_types = ["depth", "depth_vis", "rgb"]
    extensions = {"depth": ".npy", "depth_vis": ".png", "rgb": ".jpg"}

    # 收集所有文件
    all_files = []

    # 遍历每个源文件夹
    for source_dir in source_dirs:
        print(f"Processing {source_dir}...")

        # 获取当前文件夹中的所有文件并排序
        files = sorted(os.listdir(source_dir))

        # 添加文件到总列表
        for file in files:
            if any(
                file.startswith(ftype) and file.endswith(extensions[ftype])
                for ftype in file_types
            ):
                old_path = os.path.join(source_dir, file)
                all_files.append((old_path, file))

    # 按文件类型分组
    files_by_type = {ftype: [] for ftype in file_types}

    for old_path, file in all_files:
        for ftype in file_types:
            if file.startswith(ftype) and file.endswith(extensions[ftype]):
                files_by_type[ftype].append((old_path, file))
                break

    # 为每种类型重新编号并复制
    for ftype in file_types:
        for idx, (old_path, original_file) in enumerate(files_by_type[ftype]):
            new_filename = f"{ftype}_{idx:03d}{extensions[ftype]}"
            new_path = os.path.join(target_dir, new_filename)

            # 复制文件
            shutil.copy2(old_path, new_path)
            print(f"  Copied: {original_file} -> {new_filename}")


def main():
    # 定义源文件夹和目标文件夹
    source_dirs = [
        "collected_data_20251225_174027",
        "collected_data_20251225_174748",
    ]

    target_dir = "collected_data_merged"  # 创建新的目标文件夹以避免覆盖原始数据

    rename_and_copy_files(source_dirs, target_dir)
    print(f"\nAll files have been renamed and copied to {target_dir}")


if __name__ == "__main__":
    main()
