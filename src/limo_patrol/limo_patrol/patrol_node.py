import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math
import random
import time


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PatrolNode(Node):

    def __init__(self):
        super().__init__("patrol_node")

        # ---- Parameters ----
        self.declare_parameter("use_random_goals", True)
        self.declare_parameter("num_random_goals", 4)
        self.declare_parameter("goal_tolerance", 0.3)
        self.declare_parameter("obstacle_threshold", 0.6)

        self.use_random_goals = self.get_parameter("use_random_goals").value
        self.num_random_goals = self.get_parameter("num_random_goals").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.obstacle_threshold = self.get_parameter("obstacle_threshold").value

        # ---- Publisher ----
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ---- Subscribers ----
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.waypoint_sub = self.create_subscription(
            PoseArray, "/patrol_waypoints", self.waypoint_callback, 10
        )

        # ---- Timer ----
        self.timer = self.create_timer(0.1, self.control_loop)

        # ---- Robot state ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.ranges = []

        # ---- Patrol goals ----
        self.home = (0.0, 0.0)
        self.goals = []
        self.current_goal = 0
        self.goals_received = False

        # Generate random goals if configured (fallback)
        if self.use_random_goals:
            self._generate_random_goals()

        # ---- Metrics ----
        self.start_time = time.time()
        self.obstacle_count = 0
        self.patrol_complete = False

        self.get_logger().info("Patrol node started")
        if self.use_random_goals:
            self.get_logger().info(
                "Using random goals (will switch to /patrol_waypoints if received)"
            )
        else:
            self.get_logger().info(
                "Waiting for waypoints on /patrol_waypoints topic..."
            )

    def _generate_random_goals(self):
        """Generate random patrol goals as fallback."""
        self.goals = []
        for _ in range(self.num_random_goals):
            gx = random.uniform(-2.5, 2.5)
            gy = random.uniform(-2.5, 2.5)
            self.goals.append((gx, gy))
        self.goals.append(self.home)
        self.get_logger().info(f"Generated {len(self.goals)} random goals")

    # ---------------- CALLBACKS ----------------

    def waypoint_callback(self, msg: PoseArray):
        """Receive waypoints from external generator."""
        if self.goals_received:
            return  # Only accept waypoints once

        self.goals = []
        for pose in msg.poses:
            self.goals.append((pose.position.x, pose.position.y))

        if self.goals:
            self.goals_received = True
            self.current_goal = 0
            self.start_time = time.time()
            self.get_logger().info(
                f"Received {len(self.goals)} waypoints from /patrol_waypoints"
            )
            for i, (x, y) in enumerate(self.goals):
                self.get_logger().info(f"  Goal {i+1}: ({x:.2f}, {y:.2f})")

    def scan_callback(self, msg):
        self.ranges = msg.ranges

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    # ---------------- UTILS ----------------

    def distance_to_goal(self, gx, gy):
        return math.hypot(gx - self.x, gy - self.y)

    def angle_to_goal(self, gx, gy):
        target_angle = math.atan2(gy - self.y, gx - self.x)
        return normalize_angle(target_angle - self.yaw)

    def obstacle_ahead(self, threshold=None):
        if threshold is None:
            threshold = self.obstacle_threshold

        if not self.ranges:
            return False

        idx_start = len(self.ranges) // 3
        idx_end = 2 * len(self.ranges) // 3
        front = self.ranges[idx_start:idx_end]

        valid_points = [r for r in front if r > 0.05 and not math.isinf(r)]

        if not valid_points:
            return False

        return min(valid_points) < threshold

    # ---------------- MAIN LOOP ----------------

    def control_loop(self):
        msg = Twist()

        # Wait for goals
        if not self.goals:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            return

        # Check if patrol is complete
        if self.current_goal >= len(self.goals):
            if not self.patrol_complete:
                self.patrol_complete = True
                elapsed = time.time() - self.start_time
                self.get_logger().info("=" * 50)
                self.get_logger().info("PATROL COMPLETE!")
                self.get_logger().info(f"Total time: {elapsed:.2f} seconds")
                self.get_logger().info(f"Obstacles avoided: {self.obstacle_count}")
                self.get_logger().info("=" * 50)

            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            return

        gx, gy = self.goals[self.current_goal]

        # ----- REACTIVE LAYER (Obstacle Avoidance) -----
        if self.obstacle_ahead():
            self.get_logger().info("Obstacle detected, turning...")
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.obstacle_count += 1

        # ----- DELIBERATIVE LAYER (Navigation) -----
        else:
            dist = self.distance_to_goal(gx, gy)

            if dist < self.goal_tolerance:
                self.get_logger().info(
                    f"Reached goal {self.current_goal + 1}/{len(self.goals)} "
                    f"at ({gx:.2f}, {gy:.2f})"
                )
                self.current_goal += 1
                return

            angle_error = self.angle_to_goal(gx, gy)

            if abs(angle_error) > 0.5:
                msg.linear.x = 0.0
                msg.angular.z = 0.5 if angle_error > 0 else -0.5
            else:
                msg.linear.x = 0.3
                msg.angular.z = angle_error * 2.0
                msg.angular.z = max(min(msg.angular.z, 1.0), -1.0)

        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
