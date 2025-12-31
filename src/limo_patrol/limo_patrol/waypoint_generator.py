import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import random
import subprocess
import tempfile
import os
import math


X_MIN, X_MAX = -4.0, 4.0
Y_MIN, Y_MAX = -4.0, 4.0
ROBOT_SAFE_RADIUS = 0.5  # Avoid spawning too close to origin
OBSTACLE_CLEARANCE = 1.0  # Minimum distance between obstacles and waypoints


BOX_SDF = """
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# Waypoint marker - colored cylinder with number
WAYPOINT_MARKER_SDF = """
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="pole">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>1.0</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
      <visual name="sphere">
        <pose>0 0 0.55 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# Colors for waypoints (Green -> Yellow -> Orange -> Blue -> Purple)
WAYPOINT_COLORS = [
    (0.2, 0.8, 0.2),  # 1: Green
    (0.9, 0.9, 0.1),  # 2: Yellow
    (0.9, 0.5, 0.1),  # 3: Orange
    (0.2, 0.4, 0.9),  # 4: Blue
    (0.7, 0.2, 0.9),  # 5: Purple
    (0.1, 0.9, 0.9),  # 6: Cyan
    (0.9, 0.2, 0.6),  # 7: Pink
    (0.5, 0.5, 0.5),  # 8: Gray
]


class WaypointGenerator(Node):
    """Generates random waypoints and spawns obstacles that don't overlap."""

    def __init__(self):
        super().__init__("waypoint_generator")

        # Parameters
        self.declare_parameter("num_waypoints", 4)
        self.declare_parameter("num_obstacles", 4)
        self.declare_parameter("x_min", X_MIN)
        self.declare_parameter("x_max", X_MAX)
        self.declare_parameter("y_min", Y_MIN)
        self.declare_parameter("y_max", Y_MAX)
        self.declare_parameter("safe_radius", ROBOT_SAFE_RADIUS)
        self.declare_parameter("obstacle_clearance", OBSTACLE_CLEARANCE)
        self.declare_parameter("include_home", True)
        self.declare_parameter("spawn_obstacles", True)
        self.declare_parameter("spawn_markers", True)

        self.num_waypoints = self.get_parameter("num_waypoints").value
        self.num_obstacles = self.get_parameter("num_obstacles").value
        self.x_min = self.get_parameter("x_min").value
        self.x_max = self.get_parameter("x_max").value
        self.y_min = self.get_parameter("y_min").value
        self.y_max = self.get_parameter("y_max").value
        self.safe_radius = self.get_parameter("safe_radius").value
        self.obstacle_clearance = self.get_parameter("obstacle_clearance").value
        self.include_home = self.get_parameter("include_home").value
        self.spawn_obstacles = self.get_parameter("spawn_obstacles").value
        self.spawn_markers = self.get_parameter("spawn_markers").value

        # Publisher
        self.waypoint_pub = self.create_publisher(PoseArray, "/patrol_waypoints", 10)

        # Generate waypoints first
        self.waypoints = self.generate_waypoints()
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints")

        # Spawn visual markers for waypoints
        if self.spawn_markers:
            self.spawn_waypoint_markers()

        # Spawn obstacles that don't overlap with waypoints
        if self.spawn_obstacles:
            self.obstacles = self.generate_and_spawn_obstacles()
            self.get_logger().info(f"Spawned {len(self.obstacles)} obstacles")

        # Timer to publish waypoints periodically
        self.timer = self.create_timer(1.0, self.publish_waypoints)

    def generate_waypoints(self):
        """Generate random waypoints avoiding the robot's starting position."""
        waypoints = []
        attempts = 0
        max_attempts = 100

        while len(waypoints) < self.num_waypoints and attempts < max_attempts:
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)

            # Avoid spawning too close to origin (robot start)
            if (x**2 + y**2) ** 0.5 < self.safe_radius:
                attempts += 1
                continue

            waypoints.append((x, y))
            self.get_logger().info(f"Waypoint {len(waypoints)}: ({x:.2f}, {y:.2f})")

        # Add home position at the end if configured
        if self.include_home:
            waypoints.append((0.0, 0.0))
            self.get_logger().info("Added home position (0.0, 0.0) as final waypoint")

        return waypoints

    def spawn_waypoint_markers(self):
        """Spawn colored markers at each waypoint for visualization."""
        self.get_logger().info("Spawning waypoint markers...")

        for i, (x, y) in enumerate(self.waypoints):
            # Get color for this waypoint
            color_idx = i % len(WAYPOINT_COLORS)
            r, g, b = WAYPOINT_COLORS[color_idx]

            # Special color for home (last waypoint if include_home)
            if self.include_home and i == len(self.waypoints) - 1:
                r, g, b = 1.0, 1.0, 1.0  # White for home

            name = f"waypoint_marker_{i + 1}"
            sdf_content = WAYPOINT_MARKER_SDF.format(name=name, r=r, g=g, b=b)

            with tempfile.NamedTemporaryFile(
                delete=False, suffix=".sdf", mode="w"
            ) as f:
                f.write(sdf_content)
                sdf_path = f.name

            try:
                subprocess.run(
                    [
                        "ros2",
                        "run",
                        "gazebo_ros",
                        "spawn_entity.py",
                        "-entity",
                        name,
                        "-file",
                        sdf_path,
                        "-x",
                        str(x),
                        "-y",
                        str(y),
                        "-z",
                        "0.5",
                    ],
                    capture_output=True,
                    timeout=10,
                )
                self.get_logger().info(f"  Marker {i + 1}: ({x:.2f}, {y:.2f})")
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"Timeout spawning marker {i + 1}")
            finally:
                os.remove(sdf_path)

    def generate_and_spawn_obstacles(self):
        """Generate obstacles that don't overlap with waypoints or robot start."""
        obstacles = []
        attempts = 0
        max_attempts = 200

        self.get_logger().info("Spawning obstacles...")

        while len(obstacles) < self.num_obstacles and attempts < max_attempts:
            x = random.uniform(self.x_min, self.x_max)
            y = random.uniform(self.y_min, self.y_max)
            attempts += 1

            # Check distance from robot start
            if math.hypot(x, y) < self.safe_radius + 0.5:
                continue

            # Check distance from all waypoints
            too_close = False
            for wx, wy in self.waypoints:
                if math.hypot(x - wx, y - wy) < self.obstacle_clearance:
                    too_close = True
                    break

            if too_close:
                continue

            # Check distance from other obstacles
            for ox, oy in obstacles:
                if math.hypot(x - ox, y - oy) < 0.8:  # Obstacles shouldn't overlap
                    too_close = True
                    break

            if too_close:
                continue

            # Spawn the obstacle
            name = f"obstacle_{len(obstacles)}"
            self.spawn_box(name, x, y)
            obstacles.append((x, y))
            self.get_logger().info(f"Obstacle {len(obstacles)}: ({x:.2f}, {y:.2f})")

        return obstacles

    def spawn_box(self, name, x, y):
        """Spawn a box obstacle in Gazebo."""
        sdf_content = BOX_SDF.format(name=name)

        with tempfile.NamedTemporaryFile(delete=False, suffix=".sdf", mode="w") as f:
            f.write(sdf_content)
            sdf_path = f.name

        try:
            subprocess.run(
                [
                    "ros2",
                    "run",
                    "gazebo_ros",
                    "spawn_entity.py",
                    "-entity",
                    name,
                    "-file",
                    sdf_path,
                    "-x",
                    str(x),
                    "-y",
                    str(y),
                    "-z",
                    "0.25",
                ],
                capture_output=True,
                timeout=10,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"Timeout spawning {name}")
        finally:
            os.remove(sdf_path)

    def publish_waypoints(self):
        """Publish waypoints as PoseArray."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        for x, y in self.waypoints:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.waypoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
