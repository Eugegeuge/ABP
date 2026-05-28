#!/usr/bin/env python3
"""
UAV Teleop - Control manual del dron con teclado.
Autores: Hugo Lopez Pastor & Hugo Sevilla Martinez

Controles:
    W/S  - Adelante / Atras
    A/D  - Izquierda / Derecha
    Q/E  - Subir / Bajar
    J/L  - Girar izquierda / derecha (yaw)
    ESPACIO - Parada de emergencia (hover)
    ESC  - Salir
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

INSTRUCTIONS = """
--------------------------------------------------
   UAV TELEOP - Control Manual del Dron
--------------------------------------------------
   W/S  : Adelante / Atras
   A/D  : Izquierda / Derecha
   Q/E  : Subir / Bajar
   J/L  : Girar (yaw)
   SPACE: Hover (parada)
   ESC  : Salir
--------------------------------------------------
"""

# Velocidades
LINEAR_SPEED = 1.0    # m/s
VERTICAL_SPEED = 0.8  # m/s
ANGULAR_SPEED = 1.0   # rad/s


class UAVTeleop(Node):
    def __init__(self):
        super().__init__('uav_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Terminal settings for raw key reading
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.current_key = None
        self.get_logger().info('UAV Teleop iniciado. Usa WASD/QE para volar.')

    def get_key(self):
        """Read a key without blocking."""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            return key
        return None

    def timer_callback(self):
        key = self.get_key()
        twist = Twist()

        if key is not None:
            k = key.lower()

            if k == '\x1b':  # ESC
                self.get_logger().info('Saliendo...')
                self.publisher_.publish(Twist())  # stop
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                raise SystemExit

            elif k == 'w':
                twist.linear.x = LINEAR_SPEED
            elif k == 's':
                twist.linear.x = -LINEAR_SPEED
            elif k == 'a':
                twist.linear.y = LINEAR_SPEED
            elif k == 'd':
                twist.linear.y = -LINEAR_SPEED
            elif k == 'q':
                twist.linear.z = VERTICAL_SPEED
            elif k == 'e':
                twist.linear.z = -VERTICAL_SPEED
            elif k == 'j':
                twist.angular.z = ANGULAR_SPEED
            elif k == 'l':
                twist.angular.z = -ANGULAR_SPEED
            elif k == ' ':
                pass  # twist is already all zeros = hover

        # If no key pressed, twist stays at 0 = hover
        self.publisher_.publish(twist)


def main(args=None):
    print(INSTRUCTIONS)
    rclpy.init(args=args)
    node = UAVTeleop()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
