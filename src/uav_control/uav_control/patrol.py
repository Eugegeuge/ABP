#!/usr/bin/env python3
"""
UAV Patrol - Patrulla Cooperativa (Multi-UAV) Sincronizada
Los drones dividen el área de la playa en N franjas horizontales.
Van al primer waypoint, esperan a que todos lleguen (sincronización),
y luego barren en paralelo sin colisionar.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class UAVPatrol(Node):
    def __init__(self):
        super().__init__('uav_patrol')
        
        self.declare_parameter('uav_id', 1)
        self.declare_parameter('num_uavs', 1)
        self.declare_parameter('target_altitude', 2.5)
        self.declare_parameter('forward_speed', 0.8)
        
        self.uav_id = self.get_parameter('uav_id').value
        self.num_uavs = self.get_parameter('num_uavs').value
        self.target_z = self.get_parameter('target_altitude').value
        self.speed = self.get_parameter('forward_speed').value
        
        prefix = f'/uav{self.uav_id}'
        
        self.cmd_pub = self.create_publisher(Twist, f'{prefix}/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, f'{prefix}/odom', self.odom_callback, 10)
        
        # Sincronización
        self.sync_pub = self.create_publisher(String, '/patrol_sync', 10)
        self.sync_sub = self.create_subscription(String, '/patrol_sync', self.sync_callback, 10)
        self.ready_uavs = set()
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        
        self.waypoints = self.generate_cooperative_waypoints()
        self.current_wp_idx = 0
        
        # Estados: TAKEOFF, GOTO_START, WAIT_SYNC, PATROL, FINISHED
        self.state = 'TAKEOFF'
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'UAV {self.uav_id}/{self.num_uavs} iniciado. Esperando despegue...')

    def generate_cooperative_waypoints(self):
        total_y_min = -7.0
        total_y_max = 7.0
        y_width = (total_y_max - total_y_min) / self.num_uavs
        
        my_y_min = total_y_min + (self.uav_id - 1) * y_width
        my_y_max = total_y_min + self.uav_id * y_width
        
        x_min = 2.0
        x_max = 20.0
        step_y = 3.0
        
        waypoints = []
        current_y = my_y_min + min(step_y / 2.0, y_width / 2.0)
        going_right = True
        
        while current_y <= my_y_max:
            if going_right:
                waypoints.append((x_min, current_y))
                waypoints.append((x_max, current_y))
            else:
                waypoints.append((x_max, current_y))
                waypoints.append((x_min, current_y))
                
            going_right = not going_right
            current_y += step_y
            
        if (current_y - step_y) < (my_y_max - 1.0):
            current_y = my_y_max - 0.5
            if going_right:
                waypoints.append((x_min, current_y))
                waypoints.append((x_max, current_y))
            else:
                waypoints.append((x_max, current_y))
                waypoints.append((x_min, current_y))

        # Regreso a base
        waypoints.append((0.0, (self.uav_id - 1) * 2.0))
        return waypoints

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        self.current_z = pose.position.z
        
        q = pose.orientation
        self.current_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.has_odom = True
        
    def sync_callback(self, msg):
        """Recibe mensajes de que un dron ha llegado al inicio"""
        if msg.data not in self.ready_uavs:
            self.ready_uavs.add(msg.data)
            self.get_logger().info(f'Dron {msg.data} listo para patrullar. ({len(self.ready_uavs)}/{self.num_uavs})')

    def control_loop(self):
        if not self.has_odom:
            return

        twist = Twist()

        # Control de Altura Permanente
        error_z = self.target_z - self.current_z
        twist.linear.z = error_z * 0.8
        twist.linear.z = max(-0.5, min(0.5, twist.linear.z))

        if self.state == 'TAKEOFF':
            if abs(error_z) < 0.2:
                self.get_logger().info('Altitud alcanzada. Yendo al punto de inicio...')
                self.state = 'GOTO_START'
            else:
                self.get_logger().info(f'Despegando... Altura: {self.current_z:.2f}m', throttle_duration_sec=2.0)
                
        elif self.state == 'GOTO_START':
            target_x, target_y = self.waypoints[0]
            
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            
            yaw_error = target_yaw - self.current_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            
            if distance < 1.0:
                self.get_logger().info('Punto de inicio alcanzado. Esperando a los demás drones...')
                self.state = 'WAIT_SYNC'
                return 
                
            twist.angular.z = max(-1.0, min(1.0, yaw_error * 1.5))
            if abs(yaw_error) > 0.4:
                twist.linear.x = 0.0
            else:
                twist.linear.x = max(0.2, min(self.speed, distance * 0.5))
                
        elif self.state == 'WAIT_SYNC':
            # Mantenerse en hover estricto
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            
            # Publicar que estamos listos
            msg = String()
            msg.data = f'uav{self.uav_id}'
            self.sync_pub.publish(msg)
            
            # Comprobar si todos están listos
            if len(self.ready_uavs) == self.num_uavs:
                self.get_logger().info('¡Todos los drones en posición! Arrancando patrulla simultánea...')
                self.current_wp_idx = 1 # Ya estamos en el wp 0
                self.state = 'PATROL'

        elif self.state == 'PATROL':
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info('Misión completada. Regresando a base.')
                self.state = 'FINISHED'
                return
            
            target_x, target_y = self.waypoints[self.current_wp_idx]
            
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            
            yaw_error = target_yaw - self.current_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            
            if distance < 1.0:
                self.get_logger().info(f'Punto {self.current_wp_idx} alcanzado: ({target_x:.1f}, {target_y:.1f})')
                self.current_wp_idx += 1
                return 
                
            twist.angular.z = max(-1.0, min(1.0, yaw_error * 1.5))
            
            if abs(yaw_error) > 0.4:
                twist.linear.x = 0.0 
            else:
                twist.linear.x = max(0.2, min(self.speed, distance * 0.5))

        elif self.state == 'FINISHED':
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = UAVPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
