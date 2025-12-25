import subprocess
import time
import math


def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角(roll, pitch, yaw)转换为四元数(w, x, y, z)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z


def move_model_cli(model_name, x, y, z, roll=0, pitch=0, yaw=0):
    # 根据之前检查的服务，使用正确的服务名称和消息类型
    # 使用 ign service 命令行工具 (适用于 Ignition Gazebo)

    # 将欧拉角转换为四元数
    w, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)

    # 构建Protobuf文本格式的请求 - 注意使用 ignition.msgs.Pose 而不是 EntityPose
    req_str = f'name: "{model_name}" position {{ x: {x} y: {y} z: {z} }} orientation {{ w: {w} x: {qx} y: {qy} z: {qz} }}'

    cmd = [
        "ign",
        "service",
        "-s",
        "/world/strawberry_world/set_pose",
        "--reqtype",
        "ignition.msgs.Pose",
        "--reptype",
        "ignition.msgs.Boolean",
        "--timeout",
        "5000",
        "--req",
        req_str,
    ]

    try:
        result = subprocess.run(cmd, check=False, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Command failed with return code {result.returncode}")
            print(f"Error: {result.stderr}")

            # 如果第一个命令失败，尝试备用服务
            print("Trying alternate service /gazebo/set_pose")

            cmd_alt = [
                "ign",
                "service",
                "-s",
                "/gazebo/set_pose",
                "--reqtype",
                "ignition.msgs.Pose",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                req_str,
            ]
            result_alt = subprocess.run(
                cmd_alt, check=False, capture_output=True, text=True
            )

            if result_alt.returncode != 0:
                print(
                    f"Alternate command also failed with return code {result_alt.returncode}"
                )
                print(f"Error: {result_alt.stderr}")
                return False
            else:
                print(f"Successfully moved {model_name} using alternate service")
                return True
        else:
            print(f"Successfully moved {model_name} using primary service")
            return True

    except Exception as e:
        print(f"Exception occurred: {e}")
        return False


def main():
    # 等待一点时间确保Gazebo服务已准备好
    print("Waiting for Gazebo services to be available...")
    time.sleep(2)

    success = move_model_cli("realsense_d405", 3.0, 0.0, 0.1, 0, 0, 3.1416)
    time.sleep(4)
    success = move_model_cli("realsense_d405", 5.0, 0.0, 1.9261, 0, 0.3150, 3.1416)
    # time.sleep(2)
    # success = move_model_cli("realsense_d405", 4.0095, 0.0, 1.9261, 0, 0.3150, 3.1416)
    if success:
        print("Camera moved successfully!")
    else:
        print("Failed to move camera. Make sure Gazebo is running.")


if __name__ == "__main__":
    main()
