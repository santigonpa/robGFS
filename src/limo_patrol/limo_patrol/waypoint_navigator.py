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
        self.avoid_phase = "REVERSE"  # REVERSE, TURN, FOLLOW_WALL
        self.last_log_time = 0.0

        # Avoidance timing parameters
        self.reverse_duration = 1.0  # seconds to reverse
        self.turn_duration = 1.5     # seconds to turn away
        self.follow_wall_timeout = 30.0  # max seconds following wall (increased)

        # Blocked positions to avoid loops
        self.blocked_positions = []  # List of (x, y) positions where obstacles were found
        self.blocked_radius = 0.6    # Radius around blocked position to avoid
        self.max_blocked_positions = 20  # Max positions to remember

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
        """Check if obstacle ahead. Returns (blocked, left_clear, right_clear, left_dist, right_dist)."""
        if not self.scan_ranges:
            return False, True, True, float("inf"), float("inf")

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

        return blocked, left_clear, right_clear, left_min, right_min

    def check_path_to_goal(self, gx, gy):
        """Check if the direct path to goal is clear (no obstacle in that direction)."""
        if not self.scan_ranges:
            return True

        angle_to_goal = self.angle_to(gx, gy)
        
        # Get scan reading in the direction of the goal
        # Convert from robot-relative angle to scan index
        scan_angle = angle_to_goal  # Already relative to robot heading
        
        # Check a cone of ±15 degrees around the goal direction
        cone_half = math.radians(15)
        
        for offset in [0, -cone_half/2, cone_half/2, -cone_half, cone_half]:
            dist = self.get_scan_at_angle(scan_angle + offset)
            goal_dist = self.distance_to(gx, gy)
            # If obstacle is closer than the goal and within obstacle_distance, path is blocked
            if dist < min(goal_dist, self.obstacle_distance * 1.5):
                return False
        
        return True

    def add_blocked_position(self, x, y):
        """Add a position to the blocked list to avoid loops."""
        # Check if already blocked nearby
        for bx, by in self.blocked_positions:
            if math.hypot(x - bx, y - by) < self.blocked_radius:
                return  # Already blocked nearby
        
        self.blocked_positions.append((x, y))
        self.get_logger().info(f"Blocked position added: ({x:.2f}, {y:.2f})")
        
        # Limit the number of blocked positions
        if len(self.blocked_positions) > self.max_blocked_positions:
            self.blocked_positions.pop(0)

    def is_position_blocked(self, x, y):
        """Check if a position is near a blocked area."""
        for bx, by in self.blocked_positions:
            if math.hypot(x - bx, y - by) < self.blocked_radius:
                return True
        return False

    def get_direction_away_from_blocked(self):
        """Get the best direction to turn away from blocked positions."""
        if not self.blocked_positions:
            return 0
        
        # Find the nearest blocked position
        min_dist = float("inf")
        nearest_blocked = None
        for bx, by in self.blocked_positions:
            d = math.hypot(self.x - bx, self.y - by)
            if d < min_dist:
                min_dist = d
                nearest_blocked = (bx, by)
        
        if nearest_blocked is None:
            return 0
        
        # Calculate angle to blocked position
        angle_to_blocked = math.atan2(nearest_blocked[1] - self.y, nearest_blocked[0] - self.x)
        angle_diff = self.normalize_angle(angle_to_blocked - self.yaw)
        
        # Return direction away from blocked (opposite side)
        return -1 if angle_diff > 0 else 1

    def clear_blocked_for_waypoint(self):
        """Clear blocked positions when reaching a new waypoint."""
        if self.blocked_positions:
            self.get_logger().info(f"Clearing {len(self.blocked_positions)} blocked positions")
            self.blocked_positions.clear()

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
            self.clear_blocked_for_waypoint()  # Clear blocked positions for new waypoint
            self.stop()
            return

        # Check obstacles
        blocked, left_clear, right_clear, left_dist, right_dist = self.check_front()

        # === AVOIDING STATE ===
        if self.state == "AVOIDING":
            elapsed = time.time() - self.avoid_start_time

            # Check if path to goal is now clear - if so, resume navigation
            # But also check we're not heading back into a blocked area
            if not blocked and self.check_path_to_goal(gx, gy):
                # Check if the path forward would lead us into a blocked area
                forward_x = self.x + math.cos(self.yaw) * 0.5
                forward_y = self.y + math.sin(self.yaw) * 0.5
                if not self.is_position_blocked(forward_x, forward_y):
                    self.state = "NAVIGATING"
                    self.get_logger().info("Path to goal is clear - resuming navigation")
                    return

            # Timeout protection - skip waypoint if stuck too long
            if elapsed > self.follow_wall_timeout:
                self.get_logger().warn(
                    f"Avoidance timeout! Skipping waypoint {self.current_waypoint_idx + 1}"
                )
                self.current_waypoint_idx += 1
                self.state = "NAVIGATING"
                return

            # Phase 1: REVERSE - back up to create space
            if self.avoid_phase == "REVERSE":
                if elapsed < self.reverse_duration:
                    msg.linear.x = -0.15
                    msg.angular.z = 0.0
                    self.cmd_pub.publish(msg)
                    return
                else:
                    self.avoid_phase = "TURN"
                    self.get_logger().info(f"Reverse done, now turning")

            # Phase 2: TURN - rotate away from obstacle
            if self.avoid_phase == "TURN":
                turn_elapsed = elapsed - self.reverse_duration
                if turn_elapsed < self.turn_duration:
                    msg.linear.x = 0.0
                    msg.angular.z = self.angular_speed * self.avoid_direction
                    self.cmd_pub.publish(msg)
                    return
                else:
                    self.avoid_phase = "FOLLOW_WALL"
                    self.get_logger().info("Turn done, now following wall to go around")

            # Phase 3: FOLLOW_WALL - move forward while keeping obstacle on one side
            if self.avoid_phase == "FOLLOW_WALL":
                # Desired distance to keep from wall/obstacle on the side
                desired_side_dist = self.obstacle_distance * 1.5

                # Get the distance on the side where the obstacle was
                if self.avoid_direction == 1:  # We turned left, obstacle is on right
                    side_dist = right_dist
                else:  # We turned right, obstacle is on left
                    side_dist = left_dist

                # If front is blocked, turn more
                if blocked:
                    msg.linear.x = 0.05
                    msg.angular.z = self.angular_speed * self.avoid_direction * 0.8
                else:
                    # Move forward with wall-following correction
                    msg.linear.x = self.linear_speed * 0.7

                    # Adjust angular velocity to maintain distance from wall
                    if side_dist < desired_side_dist:
                        # Too close to wall, turn away
                        msg.angular.z = self.angular_speed * self.avoid_direction * 0.5
                    elif side_dist > desired_side_dist * 2:
                        # Lost the wall, turn back towards it slightly
                        msg.angular.z = -self.angular_speed * self.avoid_direction * 0.3
                    else:
                        # Good distance, go straight
                        msg.angular.z = 0.0

                self.cmd_pub.publish(msg)
                self.log_status()
                return

        # === NAVIGATING STATE ===
        if self.state == "NAVIGATING":
            # Check if current position is near a blocked area
            if self.is_position_blocked(self.x, self.y):
                # We're in a blocked area, need to move away
                away_dir = self.get_direction_away_from_blocked()
                msg.linear.x = 0.1
                msg.angular.z = self.angular_speed * away_dir if away_dir != 0 else self.angular_speed
                self.cmd_pub.publish(msg)
                self.get_logger().info("Moving away from blocked area...")
                return

            # Obstacle detected - switch to avoiding
            if blocked:
                self.state = "AVOIDING"
                self.avoid_start_time = time.time()
                self.avoid_phase = "REVERSE"

                # Block the current position to avoid returning here
                self.add_blocked_position(self.x, self.y)

                # Choose direction: prefer the clearer side, but also consider blocked positions
                away_from_blocked = self.get_direction_away_from_blocked()
                
                if left_clear and not right_clear:
                    self.avoid_direction = 1  # Turn left
                elif right_clear and not left_clear:
                    self.avoid_direction = -1  # Turn right
                elif left_clear and right_clear:
                    # Both clear - prefer direction away from blocked areas, or towards goal
                    if away_from_blocked != 0:
                        self.avoid_direction = away_from_blocked
                    else:
                        self.avoid_direction = 1 if angle_error > 0 else -1
                else:
                    # Neither clear, choose side with more space or away from blocked
                    if away_from_blocked != 0:
                        self.avoid_direction = away_from_blocked
                    else:
                        self.avoid_direction = 1 if left_dist > right_dist else -1

                direction_name = "LEFT" if self.avoid_direction == 1 else "RIGHT"
                self.get_logger().info(
                    f"Obstacle detected! Starting avoidance maneuver: {direction_name}"
                )
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
