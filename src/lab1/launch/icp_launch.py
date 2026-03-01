import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo/Bag) clock if true",
    )
    rviz_config_path = os.path.join(
        get_package_share_directory("lab1"), "config", "seq1.rviz"
    )
    ekf_node = Node(
        package="lab1",
        executable="ekf.py",
        name="ekf_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    icp_node = Node(
        package="lab1",
        executable="icp.py",
        name="icp_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            ekf_node,
            icp_node,
            #rviz_node,
        ]
    )
