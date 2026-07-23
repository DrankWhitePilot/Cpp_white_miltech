from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    move_commit_period_ms = LaunchConfiguration("move_commit_period_ms")
    scenario_path = PathJoinSubstitution(
        [FindPackageShare("underground_world"), "config", scenario]
    )

    world_node = Node(
        package="underground_world",
        executable="underground_world_node",
        name="underground_world_node",
        output="screen",
        parameters=[
            {
                "scenario_path": scenario_path,
                "move_commit_period_ms": ParameterValue(
                    move_commit_period_ms, value_type=int
                ),
            }
        ],
    )

    payload_action = Node(
        package="underground_world",
        executable="payload_action_node",
        name="payload_action",
        output="screen",
    )

    mission_explorer = Node(
        package="underground_world",
        executable="mission_explorer_node",
        name="mission_explorer",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="training_corridor.yaml",
                description="Scenario YAML file from underground_world/config",
            ),
            DeclareLaunchArgument(
                "move_commit_period_ms",
                default_value="50",
                description="Delay before applying queued move commands",
            ),
            TimerAction(period=2.0, actions=[world_node]),
            payload_action,
            mission_explorer,
        ]
    )
