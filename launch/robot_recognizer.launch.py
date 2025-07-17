import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    # 1. 声明 Launch Arguments
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="red_standard_robot1", description="节点命名空间"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="True",
        description="是否使用仿真时间 (/clock)"
    )
    declare_pkg_dir = DeclareLaunchArgument(
        "robot_recognizer_dir",
        default_value=get_package_share_directory("rm_robot_recognizer"),
        description="rm_robot_recognizer 包的 share 目录"
    )
    declare_yaml = DeclareLaunchArgument(
        "yaml_dir",
        default_value=PathJoinSubstitution([
            LaunchConfiguration("robot_recognizer_dir"),
            "config", "robot_recognizer.yaml"
        ]),
        description="robot_recognizer 的 YAML 参数文件路径"
    )

    # 2. 定义 Node 并加载参数
    robot_recognizer_node = Node(
        package="rm_robot_recognizer",
        executable="robot_recognizer",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            { "use_sim_time": LaunchConfiguration("use_sim_time") },  # 内联覆盖
            LaunchConfiguration("yaml_dir")                           # 加载 YAML
        ],
        remappings=remappings,
    )

    # 3. 组装 LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_pkg_dir)
    ld.add_action(declare_yaml)
    ld.add_action(robot_recognizer_node)
    return ld    