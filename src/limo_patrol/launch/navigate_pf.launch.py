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
                "x_min": -9.0, "x_max": 9.0,
                "y_min": -9.0, "y_max": 9.0,
                "safe_radius": 0.5,
                "include_home": True,
            }
        ],
    )

    # Waypoint pf navigator node
    pf_navigator = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="limo_patrol",
                executable="potential_field_navigator",
                name="potential_field_navigator",
                output="screen",
                parameters=[
                    {
                        "ka": 1.5,        # Increase attraction
                        "kr": 0.3,        # Reduce repulsion to avoid oscillation
                        "obs_radius": 0.8,  # Smaller influence radius
                    }
                ],
            )
        ],
    )

    return LaunchDescription([
        num_waypoints_arg,
        waypoint_generator,
        pf_navigator,
    ])
