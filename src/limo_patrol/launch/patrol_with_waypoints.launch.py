from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    num_waypoints_arg = DeclareLaunchArgument(
        "num_waypoints",
        default_value="4",
        description="Number of random waypoints to generate",
    )

    # Waypoint generator node
    waypoint_generator_node = Node(
        package="limo_patrol",
        executable="waypoint_generator",
        name="waypoint_generator",
        output="screen",
        parameters=[
            {
                "num_waypoints": LaunchConfiguration("num_waypoints"),
                "x_min": -4.0,
                "x_max": 4.0,
                "y_min": -4.0,
                "y_max": 4.0,
                "safe_radius": 0.5,
                "include_home": True,
            }
        ],
    )

    # Patrol node (waits for waypoints)
    patrol_node = Node(
        package="limo_patrol",
        executable="patrol_node",
        name="patrol_node",
        output="screen",
        parameters=[
            {
                "use_random_goals": False,  # Use waypoints from topic
                "goal_tolerance": 0.3,
                "obstacle_threshold": 0.6,
            }
        ],
    )

    return LaunchDescription(
        [
            num_waypoints_arg,
            waypoint_generator_node,
            # Delay patrol node start to ensure waypoints are published
            TimerAction(period=2.0, actions=[patrol_node]),
        ]
    )
