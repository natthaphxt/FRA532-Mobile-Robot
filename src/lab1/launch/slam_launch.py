from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription(
        [
            Node(
                package="lab1",
                executable="ekf.py",
                name="ekf_node",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_scan"],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "scan_topic": "/scan",
                        "mode": "mapping",
                    }
                ],
                output="screen",
            ),
            Node(
                package="lab1",
                executable="path_logger.py",
                name="path_logger_node",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "output_file": "slam_path.csv", 
                        "log_rate": 10.0, 
                    }
                ],
                output="screen",
            ),
        ]
    )
