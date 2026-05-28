#!/usr/bin/env python3
"""
UAV Georeferencer - Proyeccion de detecciones de pixeles a coordenadas GPS.
Autores: Hugo Lopez Pastor & Hugo Sevilla Martinez

Este nodo:
1. Recibe detecciones en pixeles (Detection2DArray).
2. Obtiene la altitud (Lidar) y posicion GPS del dron.
3. Calcula el offset en metros usando trigonometria (HFOV).
4. Publica la posicion GPS estimada del residuo.
"""

import rclpy
from rclpy.node import Node
import math
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import NavSatFix, LaserScan
from geometry_msgs.msg import PoseStamped

class GeoreferencerNode(Node):
    def __init__(self):
        super().__init__('georeferencer')

        # Parametros de la camara (deben coincidir con el URDF)
        self.declare_parameter('hfov', 1.089)  # radianes
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        self.hfov = self.get_parameter('hfov').get_parameter_value().double_value
        self.img_w = self.get_parameter('image_width').get_parameter_value().integer_value
        self.img_h = self.get_parameter('image_height').get_parameter_value().integer_value

        # Estado del dron
        self.current_gps = None
        self.current_altitude = 0.0

        # Suscriptores
        self.create_subscription(NavSatFix, 'gps', self.gps_callback, 10)
        self.create_subscription(LaserScan, 'scan_altitude', self.altimeter_callback, 10)
        self.create_subscription(Detection2DArray, 'yolo/detections', self.detections_callback, 10)

        # Publicador para el consolidador de mapas
        self.gps_detection_pub = self.create_publisher(Detection2DArray, 'yolo/detections_georeferenced', 10)

        self.get_logger().info('Nodo Georeferenciador iniciado.')

    def gps_callback(self, msg):
        self.current_gps = msg

    def altimeter_callback(self, msg):
        # El altimetro es un LaserScan de 1 solo rayo
        if len(msg.ranges) > 0:
            val = msg.ranges[0]
            if not math.isinf(val):
                self.current_altitude = val

    def detections_callback(self, msg):
        if self.current_gps is None or self.current_altitude <= 0.1:
            return

        # Preparar mensaje para el consolidador
        gps_msg = Detection2DArray()
        gps_msg.header = msg.header

        for detection in msg.detections:
            # Centro del pixel detectado
            u = detection.bbox.center.position.x
            v = detection.bbox.center.position.y
            
            # 1. Calcular focal length en pixeles basada en HFOV
            f_px = (self.img_w / 2.0) / math.tan(self.hfov / 2.0)
            
            # 2. Calcular offset en metros (asumiendo camara apuntando hacia abajo)
            dx_px = u - (self.img_w / 2.0)
            dy_px = v - (self.img_h / 2.0)
            
            # Offset en metros relativo al dron
            dist_y_m = (dx_px * self.current_altitude) / f_px
            dist_x_m = (dy_px * self.current_altitude) / f_px
            
            # 3. Convertir offset metros a grados GPS (WGS84 aproximado)
            lat_drone = self.current_gps.latitude
            lon_drone = self.current_gps.longitude
            
            offset_lat = dist_x_m / 111132.0
            offset_lon = dist_y_m / (111132.0 * math.cos(math.radians(lat_drone)))
            
            target_lat = lat_drone + offset_lat
            target_lon = lon_drone + offset_lon
            
            # Crear nueva deteccion con datos GPS
            gps_det = detection
            hypothesis = gps_det.results[0]
            hypothesis.pose.pose.position.x = target_lat
            hypothesis.pose.pose.position.y = target_lon
            hypothesis.pose.pose.position.z = self.current_altitude
            
            gps_msg.detections.append(gps_det)

            self.get_logger().info(
                f'OBJETO DETECTADO (ID:{hypothesis.hypothesis.class_id}): Lat={target_lat:.7f}, Lon={target_lon:.7f}',
                throttle_duration_sec=1.0
            )

        self.gps_detection_pub.publish(gps_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GeoreferencerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
