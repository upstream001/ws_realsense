from setuptools import setup

package_name = "straw"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/strawberry_world.launch.py"]),
        ("share/" + package_name + "/worlds", ["worlds/strawberry_world.sdf"]),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
        "open3d",
        "cv-bridge",
        "rclpy",
        "gazebo_msgs",
    ],
    zip_safe=True,
    maintainer="tianqi",
    maintainer_email="tianqi@todo.todo",
    description="RealSense D405 camera control and strawberry detection package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_viewer = straw.camera_viewer:main",
            "simple_image_capture = straw.simple_image_capture:main",
            "test_image_topics = straw.test_image_topics:main",
            "move_camera_command = straw.move_camera_command:main",
            "get_camera_position = straw.get_camera_position:main",
            "rotate_capture = straw.rotate_capture:main",
            "get_relative_pose=straw.get_relative_pose:main"
        ],
    },
)
