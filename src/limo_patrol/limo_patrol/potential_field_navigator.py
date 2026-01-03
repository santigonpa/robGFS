"""
Potential Field Navigator - ESTRATEGIA 2
Navegación basada en fuerzas virtuales:
- Atractiva: Hacia el waypoint.
- Repulsiva: Alejándose de los obstáculos detectados por el LIDAR.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math
import time
import numpy as np

class PotentialFieldNavigator(Node):

    def __init__(self):
        super().__init__("potential_field_navigator")

        # ---- Parameters ----
        self.declare_parameter("ka", 1.2)           # Attractive Gain
        self.declare_parameter("kr", 0.4)           # Repulsive Gain
        self.declare_parameter("obs_radius", 1.0)   # Obstacle influence radius
        self.declare_parameter("goal_tolerance", 0.3)
        self.declare_parameter("max_linear_speed", 0.35)
        self.declare_parameter("max_angular_speed", 1.0)

        self.ka = self.get_parameter("ka").value
        self.kr = self.get_parameter("kr").value
        self.obs_radius = self.get_parameter("obs_radius").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value

        # ---- Interfaces ----
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # QoS for sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, "/odometry", self.odom_callback, 10)
        self.waypoint_sub = self.create_subscription(PoseArray, "/patrol_waypoints", self.waypoint_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # ---- State ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0

        self.waypoints = []
        self.current_wp_idx = 0
        self.mission_active = False
        self.mission_completed = False  # Prevent restarting after completion

        self.get_logger().info("Strategy 2: Potential Fields - Started")

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

    def odom_callback(self, msg):
        if not self.odom_received:
            self.get_logger().info("Odometry received successfully.")
            self.odom_received = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def waypoint_callback(self, msg):
        # Do not restart if already completed or active
        if self.mission_active or self.mission_completed:
            return
        self.waypoints = [(p.position.x, p.position.y) for p in msg.poses]
        if self.waypoints:
            self.current_wp_idx = 0
            self.mission_active = True
            self.get_logger().info(f"Received {len(self.waypoints)} waypoints.")

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if not self.mission_active or not self.waypoints:
            return

        # 0. Check mission end
        if self.current_wp_idx >= len(self.waypoints):
            self.stop_robot()
            self.mission_active = False
            self.mission_completed = True  # Mark as completed
            self.get_logger().info("=" * 40)
            self.get_logger().info("Mission Completed! Robot stopped.")
            self.get_logger().info("=" * 40)
            return

        goal_x, goal_y = self.waypoints[self.current_wp_idx]
        
        # 1. Attractive Force (Towards the goal)
        dx = goal_x - self.x
        dy = goal_y - self.y
        dist_goal = math.hypot(dx, dy)

        if dist_goal < self.goal_tolerance:
            self.get_logger().info(f"Waypoint {self.current_wp_idx+1} reached.")
            self.current_wp_idx += 1
            return

        # Unit vector towards the goal
        f_att_x = self.ka * (dx / dist_goal)
        f_att_y = self.ka * (dy / dist_goal)

        # 2. Repulsive Force (Sum of obstacle vectors)
        f_rep_x = 0.0
        f_rep_y = 0.0
        obs_count = 0

        if self.scan_ranges:
            # Analyze a subset of rays for performance (every 5 rays)
            for i, r in enumerate(self.scan_ranges):
                if i % 5 != 0: continue 
                
                # Filter invalid values (inf, nan, 0)
                if not math.isfinite(r) or r <= 0.05:
                    continue
                
                if r < self.obs_radius:
                    # Angle of the obstacle in the robot frame
                    angle_robot = self.scan_angle_min + (i * self.scan_angle_increment)
                    # Angle in the global frame
                    angle_global = angle_robot + self.yaw
                    
                    # Magnitude: increases as it gets closer (quadratic for smoothness)
                    mag = self.kr * ((1.0/r) - (1.0/self.obs_radius)) ** 2
                    
                    # Repulsive vector (opposite to the obstacle)
                    f_rep_x -= mag * math.cos(angle_global)
                    f_rep_y -= mag * math.sin(angle_global)
                    obs_count += 1

        # 3. Total Force
        fx_total = f_att_x + f_rep_x
        fy_total = f_att_y + f_rep_y

        # Debug: every 40 cycles (~2 sec)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
        
        if self._debug_counter % 40 == 0:
            self.get_logger().info(
                f"WP{self.current_wp_idx+1} | Pos:({self.x:.2f},{self.y:.2f}) | "
                f"Goal:({goal_x:.2f},{goal_y:.2f}) | Dist:{dist_goal:.2f} | "
                f"F_att:({f_att_x:.2f},{f_att_y:.2f}) | F_rep:({f_rep_x:.2f},{f_rep_y:.2f}) | "
                f"Obs:{obs_count}"
            )

        # 4. Robot Control
        # We want to orient towards the resultant force
        target_yaw = math.atan2(fy_total, fx_total)
        yaw_error = self.normalize_angle(target_yaw - self.yaw)

        # Linear velocity: proportional to the magnitude of the force, but reduced if the angular error is high
        force_magnitude = math.hypot(fx_total, fy_total)
        
        # If we are very misaligned, reduce linear velocity (but do not stop completely)
        if abs(yaw_error) > 1.2: # ~70 degrees
            linear_vel = 0.05  # Minimum velocity to avoid stopping
        else:
            # Project the force in the current robot direction for smoothness
            linear_vel = min(force_magnitude, self.max_linear_speed) * math.cos(yaw_error)
            linear_vel = max(0.05, linear_vel) # Minimum velocity

        angular_vel = yaw_error * 1.5  # Reduce angular gain for smoothness
        angular_vel = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)

        cmd = Twist()
        cmd.linear.x = float(linear_vel)
        cmd.angular.z = float(angular_vel)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
