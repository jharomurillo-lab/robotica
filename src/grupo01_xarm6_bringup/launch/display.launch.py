from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Iniciar joint_state_publisher_gui (usar use_gui:=false al correr ik_node)",
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch",
                "_robot_description.launch.py",
            ])
        ),
        launch_arguments={
            "dof": "6",
            "robot_type": "xarm",
            "hw_ns": "xarm",
            "add_gripper": "false",
            "add_vacuum_gripper": "false",
            "add_bio_gripper": "false",
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch",
                "_rviz_display.launch.py",
            ])
        )
    )

    gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        use_gui_arg,
        description,
        rviz,
        TimerAction(period=1.0, actions=[gui]),
    ])
