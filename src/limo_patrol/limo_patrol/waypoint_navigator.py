"""
Waypoint Navigator Node
Navigates through waypoints with obstacle avoidance and re-routing.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math
import time


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__("waypoint_navigator")

        # ---- Parameters ----
        self.declare_parameter("goal_tolerance", 0.4)
        self.declare_parameter("obstacle_distance", 0.5)
        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.4)

        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.obstacle_distance = self.get_parameter("obstacle_distance").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value

        # ---- Publishers ----
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ---- Subscribers ----
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry", self.odom_callback, 10
        )
        self.waypoint_sub = self.create_subscription(
            PoseArray, "/patrol_waypoints", self.waypoint_callback, 10
        )

        # ---- Timer ----
        self.timer = self.create_timer(0.1, self.navigate)

        # ---- State ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0

        # ---- Waypoints ----
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.waypoints_received = False

        # ---- Navigation state ----
        self.state = "WAITING"  # WAITING, NAVIGATING, AVOIDING, DONE
        self.avoid_direction = 1  # 1 = left, -1 = right
        self.avoid_start_time = 0.0
        self.last_log_time = 0.0

        self.get_logger().info(
            "Waypoint Navigator started - waiting for waypoints on /patrol_waypoints"
        )

    # ==================== CALLBACKS ====================

    def waypoint_callback(self, msg: PoseArray):
        """Receive waypoints from waypoint_generator."""
        if self.waypoints_received:
            return

        self.waypoints = [(p.position.x, p.position.y) for p in msg.poses]

        if self.waypoints:
            self.waypoints_received = True
            self.current_waypoint_idx = 0
            self.state = "NAVIGATING"
            self.get_logger().info(f"Received {len(self.waypoints)} waypoints:")
            for i, (x, y) in enumerate(self.waypoints):
                self.get_logger().info(f"  [{i+1}] ({x:.2f}, {y:.2f})")

    def scan_callback(self, msg: LaserScan):
        """Store laser scan data."""
        self.scan_ranges = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    # ==================== HELPERS ====================

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def distance_to(self, gx, gy):
        """Euclidean distance to goal."""
        return math.hypot(gx - self.x, gy - self.y)

    def angle_to(self, gx, gy):
        """Angle error to goal (normalized)."""
        target = math.atan2(gy - self.y, gx - self.x)
        return self.normalize_angle(target - self.yaw)

    def get_scan_at_angle(self, angle_rad):
        """Get scan reading at a specific angle (0 = front)."""
        if not self.scan_ranges:
            return float("inf")

        # Convert angle to index
        idx = int((angle_rad - self.scan_angle_min) / self.scan_angle_increment)
        idx = max(0, min(idx, len(self.scan_ranges) - 1))

        r = self.scan_ranges[idx]
        if math.isinf(r) or math.isnan(r) or r < 0.05:
            return float("inf")
        return r

    def check_front(self):
        """Check if obstacle ahead. Returns (blocked, left_clear, right_clear)."""
        if not self.scan_ranges:
            return False, True, True

        n = len(self.scan_ranges)

        # Front sector (roughly -30 to +30 degrees)
        front_start = n // 3
        front_end = 2 * n // 3
        front = self.scan_ranges[front_start:front_end]

        # Left sector
        left_start = 2 * n // 3
        left_end = n
        left = self.scan_ranges[left_start:left_end]

        # Right sector
        right_start = 0
        right_end = n // 3
        right = self.scan_ranges[right_start:right_end]

        def min_valid(readings):
            valid = [
                r
                for r in readings
                if r > 0.05 and not math.isinf(r) and not math.isnan(r)
            ]
            return min(valid) if valid else float("inf")

        front_min = min_valid(front)
        left_min = min_valid(left)
        right_min = min_valid(right)

        blocked = front_min < self.obstacle_distance
        left_clear = left_min > self.obstacle_distance
        right_clear = right_min > self.obstacle_distance

        return blocked, left_clear, right_clear

    def stop(self):
        """Stop the robot."""
        msg = Twist()
        self.cmd_pub.publish(msg)

    def log_status(self, force=False):
        """Log current status every 2 seconds."""
        now = time.time()
        if force or (now - self.last_log_time > 2.0):
            self.last_log_time = now
            if self.current_waypoint_idx < len(self.waypoints):
                gx, gy = self.waypoints[self.current_waypoint_idx]
                dist = self.distance_to(gx, gy)
                angle = self.angle_to(gx, gy)
                self.get_logger().info(
                    f"[{self.state}] pos=({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.0f}° | "
                    f"goal[{self.current_waypoint_idx+1}]=({gx:.2f},{gy:.2f}) dist={dist:.2f} angle={math.degrees(angle):.0f}°"
                )

    # ==================== MAIN NAVIGATION ====================

    def navigate(self):
        """Main navigation loop."""
        msg = Twist()

        # === WAITING STATE ===
        if self.state == "WAITING":
            self.stop()
            return

        # === DONE STATE ===
        if self.state == "DONE":
            self.stop()
            return

        # === Check if all waypoints done ===
        if self.current_waypoint_idx >= len(self.waypoints):
            if self.state != "DONE":
                self.state = "DONE"
                self.get_logger().info("=" * 50)
                self.get_logger().info("ALL WAYPOINTS REACHED!")
                self.get_logger().info("=" * 50)
            self.stop()
            return

        # Current goal
        gx, gy = self.waypoints[self.current_waypoint_idx]
        dist = self.distance_to(gx, gy)
        angle_error = self.angle_to(gx, gy)

        # Check if waypoint reached
        if dist < self.goal_tolerance:
            self.get_logger().info(
                f"✓ Reached waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} "
                f"at ({gx:.2f}, {gy:.2f})"
            )
            self.current_waypoint_idx += 1
            self.state = "NAVIGATING"
            self.stop()
            return

        # Check obstacles
        blocked, left_clear, right_clear = self.check_front()

        # === AVOIDING STATE ===
        if self.state == "AVOIDING":
            elapsed = time.time() - self.avoid_start_time

            # If front is clear, go back to navigating
            if not blocked:
                self.state = "NAVIGATING"
                self.get_logger().info("Path clear - resuming navigation")
                return

            # Turn in chosen direction
            msg.linear.x = 0.0
            msg.angular.z = self.angular_speed * self.avoid_direction

            # If stuck for too long, try reverse
            if elapsed > 5.0:
                msg.linear.x = -0.1
                msg.angular.z = self.angular_speed * self.avoid_direction

            self.cmd_pub.publish(msg)
            self.log_status()
            return

        # === NAVIGATING STATE ===
        if self.state == "NAVIGATING":
            # Obstacle detected - switch to avoiding
            if blocked:
                self.state = "AVOIDING"
                self.avoid_start_time = time.time()

                # Choose direction: prefer the clearer side
                if left_clear and not right_clear:
                    self.avoid_direction = 1  # Turn left
                elif right_clear and not left_clear:
                    self.avoid_direction = -1  # Turn right
                elif left_clear and right_clear:
                    # Both clear, turn towards goal
                    self.avoid_direction = 1 if angle_error > 0 else -1
                else:
                    # Neither clear, just pick one
                    self.avoid_direction = 1

                direction_name = "LEFT" if self.avoid_direction == 1 else "RIGHT"
                self.get_logger().info(f"Obstacle detected! Turning {direction_name}")
                return

            # No obstacle - navigate to goal
            # First, align with goal
            if abs(angle_error) > 0.2:  # ~11 degrees
                msg.linear.x = 0.0
                msg.angular.z = (
                    self.angular_speed if angle_error > 0 else -self.angular_speed
                )
            else:
                # Move forward with slight correction
                msg.linear.x = self.linear_speed
                msg.angular.z = angle_error * 1.0  # Proportional control
                msg.angular.z = max(
                    min(msg.angular.z, self.angular_speed), -self.angular_speed
                )

            self.cmd_pub.publish(msg)
            self.log_status()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
