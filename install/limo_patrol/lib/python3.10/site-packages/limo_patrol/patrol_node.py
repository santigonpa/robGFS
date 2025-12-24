import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math
import random
import time


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        # ---- Publisher ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- Subscribers ----
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

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
        for _ in range(4):
            gx = random.uniform(-3.0, 3.0)
            gy = random.uniform(-3.0, 3.0)
            self.goals.append((gx, gy))

        self.goals.append(self.home)
        self.current_goal = 0

        # ---- Metrics ----
        self.start_time = time.time()
        self.obstacle_count = 0

        self.get_logger().info('Patrol node started')

    # ---------------- CALLBACKS ----------------

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
        return math.atan2(gy - self.y, gx - self.x) - self.yaw

    def obstacle_ahead(self, threshold=0.5):
        if not self.ranges:
            return False

        front = self.ranges[len(self.ranges)//3 : 2*len(self.ranges)//3]
        return min(front) < threshold

    # ---------------- MAIN LOOP ----------------

    def control_loop(self):
        msg = Twist()

        # If patrol finished
        if self.current_goal >= len(self.goals):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)

            total_time = time.time() - self.start_time
            self.get_logger().info(
                f'Patrol finished | Time: {total_time:.2f}s | Obstacles: {self.obstacle_count}'
            )
            return

        gx, gy = self.goals[self.current_goal]

        # ----- REACTIVE LAYER -----
        if self.obstacle_ahead():
            msg.linear.x = 0.0
            msg.angular.z = 0.6
            self.obstacle_count += 1

        # ----- DELIBERATIVE LAYER -----
        else:
            dist = self.distance_to_goal(gx, gy)

            if dist < 0.3:
                self.get_logger().info(
                    f'Reached goal {self.current_goal + 1}/{len(self.goals)}'
                )
                self.current_goal += 1
                return

            angle = self.angle_to_goal(gx, gy)
            msg.linear.x = 0.3
            msg.angular.z = angle

        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

