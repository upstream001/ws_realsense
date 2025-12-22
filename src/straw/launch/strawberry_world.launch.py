from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to the SDF world file
    world_file = PathJoinSubstitution([
        FindPackageShare('straw'),
        'worlds',
        'strawberry_world.sdf'
    ])

    # Start Ignition Gazebo with the world
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )

    # Bridge to forward Ignition topics to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/rgb/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/depth/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge
    ])
