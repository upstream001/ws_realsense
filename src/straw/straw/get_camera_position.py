#!/usr/bin/env python3
"""
获取相机当前位置的脚本
按任意键输出相机当前位置
"""

import subprocess
import sys
import select
import tty
import termios
import math


def get_model_pose(model_name):
    """
    获取模型的位置和姿态
    """
    try:
        # 使用ign service获取场景信息
        cmd = [
            "ign",
            "service",
            "-s",
            "/world/strawberry_world/scene/info",
            "--reqtype",
            "ignition.msgs.Empty",
            "--reptype",
            "ignition.msgs.Scene",
            "--timeout",
            "5000",
            "--req",
            "",
        ]

        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

        if result.returncode != 0:
            print(f"Error getting scene info: {result.stderr}")
            return None

        # 解析场景信息以获取模型位置
        scene_info = result.stdout
        return parse_model_pose(scene_info, model_name)

    except subprocess.TimeoutExpired:
        print("获取模型位置超时")
        return None
    except Exception as e:
        print(f"获取模型位置时发生错误: {e}")
        return None


def parse_model_pose(scene_info, model_name):
    """
    从场景信息中解析模型位置
    """
    try:
        lines = scene_info.split("\n")
        in_model = False
        current_model_name = ""
        model_pose = {}

        for i, line in enumerate(lines):
            line = line.strip()

            # 查找模型定义
            if line.startswith("model {"):
                in_model = True
                continue
            elif line == "}":
                if in_model:
                    in_model = False
                    current_model_name = ""
                continue
            elif in_model and line.startswith("name:"):
                # 提取模型名称
                current_model_name = line.split('"')[1] if '"' in line else ""

                if current_model_name == model_name:
                    # 找到目标模型，继续解析其位置
                    j = i + 1
                    while j < len(lines):
                        next_line = lines[j].strip()

                        if next_line == "}":  # 模型定义结束
                            break
                        elif "pose {" in next_line:
                            # 开始解析姿态信息
                            j += 1
                            while j < len(lines) and lines[j].strip() != "}":
                                pose_line = lines[j].strip()

                                if "position {" in pose_line:
                                    # 解析位置
                                    j += 1
                                    pos_x = pos_y = pos_z = 0.0
                                    while j < len(lines) and lines[j].strip() != "}":
                                        pos_line = lines[j].strip()
                                        if pos_line.startswith("x:"):
                                            pos_x = float(pos_line.split()[1])
                                        elif pos_line.startswith("y:"):
                                            pos_y = float(pos_line.split()[1])
                                        elif pos_line.startswith("z:"):
                                            pos_z = float(pos_line.split()[1])
                                        j += 1

                                    print(f"模型 {model_name} 的位置:")
                                    print(f"position:  X,Y,Z:{pos_x},{pos_y},{pos_z}")

                                    # 查找方向
                                    j += 1  # 跳过 '}'
                                    if j < len(lines) and "orientation {" in lines[j]:
                                        j += 1
                                        orient_w = orient_x = orient_y = orient_z = 0.0
                                        while (
                                            j < len(lines) and lines[j].strip() != "}"
                                        ):
                                            orient_line = lines[j].strip()
                                            if orient_line.startswith("w:"):
                                                orient_w = float(orient_line.split()[1])
                                            elif orient_line.startswith("x:"):
                                                orient_x = float(orient_line.split()[1])
                                            elif orient_line.startswith("y:"):
                                                orient_y = float(orient_line.split()[1])
                                            elif orient_line.startswith("z:"):
                                                orient_z = float(orient_line.split()[1])
                                            j += 1

                                        # 转换四元数为欧拉角
                                        roll, pitch, yaw = quaternion_to_euler(
                                            orient_w, orient_x, orient_y, orient_z
                                        )
                                        print(
                                            f"  方向 (w, x, y, z): {orient_w}, {orient_x}, {orient_y}, {orient_z}"
                                        )
                                        print(
                                            f"  欧拉角 (roll, pitch, yaw): {roll:.4f}, {pitch:.4f}, {yaw:.4f}"
                                        )

                                    return {
                                        "position": (pos_x, pos_y, pos_z),
                                        "orientation": (
                                            orient_w,
                                            orient_x,
                                            orient_y,
                                            orient_z,
                                        ),
                                    }
                        j += 1

        print(f"在场景信息中未找到模型 {model_name}")
        return None

    except Exception as e:
        print(f"解析模型位置时发生错误: {e}")
        return None


def quaternion_to_euler(w, x, y, z):
    """
    将四元数转换为欧拉角(roll, pitch, yaw)
    """
    # 计算 roll (x轴旋转)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # 计算 pitch (y轴旋转)
    sinp = 2 * (w * y - z * x)
    if math.fabs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 使用90度
    else:
        pitch = math.asin(sinp)

    # 计算 yaw (z轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def get_model_pose_alternative(model_name):
    """
    另一种获取模型位置的方法 - 使用场景信息
    """
    return get_model_pose(model_name)


def getch():
    """
    非阻塞地获取键盘输入
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    print("相机位置获取脚本")
    print("按 'q' 退出，按其他任意键获取相机位置")
    print("-" * 50)

    model_name = "realsense_d405"

    while True:
        print("\n按任意键获取相机位置 (q退出): ", end="", flush=True)
        char = getch()

        if char.lower() == "q":
            print("\n退出程序")
            break

        print(f"\n获取模型 '{model_name}' 的位置...")

        # 尝试获取模型位置
        success = get_model_pose_alternative(model_name)

        if success:
            print("成功获取位置信息")
        else:
            print("获取位置信息失败")

        print("-" * 50)


if __name__ == "__main__":
    main()
