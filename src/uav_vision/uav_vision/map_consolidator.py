#!/usr/bin/env python3
"""
Map Consolidator - Consolida detecciones de basura de N drones
y genera un CSV + un mapa visual con matplotlib.
"""
import rclpy
from rclpy.node import Node
import csv
import os
import math
from datetime import datetime
from vision_msgs.msg import Detection2DArray

class MapConsolidator(Node):
    def __init__(self):
        super().__init__('map_consolidator')
        
        # Parametros
        self.declare_parameter('output_file', 'mapa_residuos.csv')
        self.declare_parameter('min_dist_duplicate', 2.0) # metros
        self.declare_parameter('num_uavs', 3)
        
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.min_dist = self.get_parameter('min_dist_duplicate').get_parameter_value().double_value
        num_uavs = self.get_parameter('num_uavs').get_parameter_value().integer_value
        
        self.detections = [] # Lista de {lat, lon, class, conf, uav}
        
        # Suscribirse a TODOS los drones dinámicamente
        for i in range(1, num_uavs + 1):
            topic = f'/uav{i}/yolo/detections_georeferenced'
            self.create_subscription(
                Detection2DArray,
                topic,
                lambda msg, uav_id=i: self.detection_callback(msg, uav_id),
                10
            )
            self.get_logger().info(f'Escuchando detecciones de uav{i} en {topic}')
        
        # Crear CSV con cabecera si no existe
        if not os.path.exists(self.output_file):
            with open(self.output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'UAV', 'Class', 'Lat', 'Lon', 'Confidence'])
        
        # Timer para actualizar el mapa visual cada 10 segundos
        self.map_timer = self.create_timer(10.0, self.generate_map)
        
        self.get_logger().info(f'Consolidador de mapa iniciado. Archivo: {self.output_file}')

    def haversine_dist(self, lat1, lon1, lat2, lon2):
        """Calcula distancia en metros entre dos puntos GPS."""
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def is_duplicate(self, lat, lon):
        for d in self.detections:
            if self.haversine_dist(lat, lon, d['lat'], d['lon']) < self.min_dist:
                return True
        return False

    def detection_callback(self, msg, uav_id):
        for detection in msg.detections:
            if len(detection.results) == 0:
                continue
            hypothesis = detection.results[0]
            lat = hypothesis.pose.pose.position.x
            lon = hypothesis.pose.pose.position.y
            class_id = hypothesis.hypothesis.class_id
            conf = hypothesis.hypothesis.score
            
            # Evitar duplicados por cercania
            if not self.is_duplicate(lat, lon):
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                
                # Guardar en memoria
                self.detections.append({
                    'lat': lat, 'lon': lon, 
                    'class': class_id, 'conf': conf,
                    'uav': f'uav{uav_id}'
                })
                
                # Guardar en CSV
                with open(self.output_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, f'uav{uav_id}', class_id, 
                                   f'{lat:.7f}', f'{lon:.7f}', f'{conf:.2f}'])
                
                self.get_logger().info(
                    f'NUEVO RESIDUO por uav{uav_id}: clase={class_id} '
                    f'en ({lat:.6f}, {lon:.6f}) conf={conf:.2f}'
                )

    def generate_map(self):
        """Genera un mapa visual PNG con las detecciones acumuladas."""
        if len(self.detections) == 0:
            return
        
        try:
            import matplotlib
            matplotlib.use('Agg')  # Backend sin GUI
            import matplotlib.pyplot as plt
            
            fig, ax = plt.subplots(1, 1, figsize=(12, 8))
            fig.patch.set_facecolor('#1a1a2e')
            ax.set_facecolor('#16213e')
            
            # Colores por UAV
            uav_colors = {
                'uav1': '#e94560',
                'uav2': '#0f3460',
                'uav3': '#53a653',
                'uav4': '#e9a045',
                'uav5': '#a045e9',
            }
            
            # Plotear cada detección
            for d in self.detections:
                color = uav_colors.get(d['uav'], '#ffffff')
                ax.scatter(d['lon'], d['lat'], c=color, s=120, 
                          edgecolors='white', linewidths=0.8, zorder=5,
                          label=d['uav'])
                ax.annotate(f"  {d['class']}", (d['lon'], d['lat']),
                           fontsize=7, color='white', fontweight='bold')
            
            # Leyenda sin duplicados
            handles, labels = ax.get_legend_handles_labels()
            unique = dict(zip(labels, handles))
            ax.legend(unique.values(), unique.keys(), 
                     loc='upper right', facecolor='#0a0a1a', 
                     edgecolor='#e94560', labelcolor='white',
                     fontsize=10, title='Detectado por',
                     title_fontsize=11)
            
            ax.set_xlabel('Longitud', color='white', fontsize=12)
            ax.set_ylabel('Latitud', color='white', fontsize=12)
            ax.set_title(f'Mapa de Residuos Detectados ({len(self.detections)} objetos)',
                        color='white', fontsize=16, fontweight='bold', pad=15)
            ax.tick_params(colors='white', labelsize=9)
            for spine in ax.spines.values():
                spine.set_edgecolor('#e94560')
            
            ax.grid(True, alpha=0.15, color='white')
            
            plt.tight_layout()
            map_path = self.output_file.replace('.csv', '.png')
            plt.savefig(map_path, dpi=150, bbox_inches='tight',
                       facecolor=fig.get_facecolor())
            plt.close(fig)
            
            self.get_logger().info(f'Mapa actualizado: {map_path} ({len(self.detections)} residuos)')
            
        except ImportError:
            self.get_logger().warn('matplotlib no instalado. Instala con: pip install matplotlib')
        except Exception as e:
            self.get_logger().error(f'Error generando mapa: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MapConsolidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Generando mapa final...')
        node.generate_map()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
