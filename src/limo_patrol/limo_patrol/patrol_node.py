import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math
import random
import time

# Función auxiliar para mantener los ángulos entre -pi y pi
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

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
        # Generar puntos aleatorios (ajustado para que no estén demasiado lejos)
        for _ in range(4):
            gx = random.uniform(-2.5, 2.5)
            gy = random.uniform(-2.5, 2.5)
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
        target_angle = math.atan2(gy - self.y, gx - self.x)
        # IMPORTANTE: Normalizar la diferencia de ángulo
        return normalize_angle(target_angle - self.yaw)

    def obstacle_ahead(self, threshold=0.6): # Subí un poco el umbral por seguridad
        if not self.ranges:
            return False

        # Asumiendo configuración estándar (-pi a pi), el frente está en el medio.
        # Tomamos el tercio central.
        idx_start = len(self.ranges) // 3
        idx_end = 2 * len(self.ranges) // 3
        front = self.ranges[idx_start : idx_end]

        # FILTRO CRÍTICO: Ignorar 0.0 (error/cerca) e inf (lejos)
        # Si el valor es muy pequeño (<0.05), suele ser error de lectura o el propio chasis
        valid_points = [r for r in front if r > 0.05 and not math.isinf(r)]

        if not valid_points:
            return False

        return min(valid_points) < threshold

    # ---------------- MAIN LOOP ----------------

    def control_loop(self):
        msg = Twist()

        # Si ya terminó la patrulla
        if self.current_goal >= len(self.goals):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            # Solo loguear una vez o detener el timer para no saturar la terminal
            # self.timer.cancel() 
            return

        gx, gy = self.goals[self.current_goal]

        # ----- REACTIVE LAYER (Evasión) -----
        if self.obstacle_ahead():
            self.get_logger().info('Obstaculo detectado, girando...')
            msg.linear.x = 0.0
            # Girar en sentido antihorario
            msg.angular.z = 0.5
            self.obstacle_count += 1

        # ----- DELIBERATIVE LAYER (Navegación) -----
        else:
            dist = self.distance_to_goal(gx, gy)

            # Tolerancia para llegar al punto
            if dist < 0.3:
                self.get_logger().info(
                    f'Reached goal {self.current_goal + 1}/{len(self.goals)}'
                )
                self.current_goal += 1
                return

            angle_error = self.angle_to_goal(gx, gy)

            # CONTROLADOR PROPORCIONAL SIMPLE
            # Si el error angular es grande, gira rápido in-situ.
            # Si es pequeño, avanza y corrige suavemente.

            if abs(angle_error) > 0.5: # Si el objetivo está a más de ~30 grados
                msg.linear.x = 0.0     # Detenerse para girar (evita círculos grandes)
                msg.angular.z = 0.5 if angle_error > 0 else -0.5
            else:
                msg.linear.x = 0.3
                # Ganancia P (2.0) para corregir el ángulo proporcionalmente
                msg.angular.z = angle_error * 2.0 

                # Limitar la velocidad angular máxima para estabilidad
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

if __name__ == '__main__':
    main()