from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments
    num_waypoints_arg = DeclareLaunchArgument(
        "num_waypoints",
        default_value="4",
        description="Number of random waypoints to generate",
    )

    # Waypoint generator node
    waypoint_generator = Node(
        package="limo_patrol",
        executable="waypoint_generator",
        name="waypoint_generator",
        output="screen",
        parameters=[
            {
                "num_waypoints": LaunchConfiguration("num_waypoints"),
                "x_min": -2.0,
                "x_max": 2.0,
                "y_min": -2.0,
                "y_max": 2.0,
                "safe_radius": 0.5,
                "include_home": True,
            }
        ],
    )

    # Waypoint navigator node (delayed start)
    waypoint_navigator = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="limo_patrol",
                executable="waypoint_navigator",
                name="waypoint_navigator",
                output="screen",
                parameters=[
                    {
                        "goal_tolerance": 0.4,
                        "obstacle_distance": 0.5,
                        "linear_speed": 0.2,
                        "angular_speed": 0.4,
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            num_waypoints_arg,
            waypoint_generator,
            waypoint_navigator,
        ]
    )
