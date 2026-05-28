#!/usr/bin/env python3
"""
UAV YOLO Vision - Deteccion de personas en tiempo real.
Autores: Hugo Lopez Pastor & Hugo Sevilla Martinez

Suscribe a /camera/image_raw, ejecuta YOLOv8 y muestra
la imagen anotada con OpenCV. Detecta personas (clase 0).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class YoloVisionNode(Node):
    def __init__(self):
        super().__init__('yolo_vision')

        # Parametros
        self.declare_parameter('input_topic', 'camera/image_raw')
        self.declare_parameter('model', 'taco.pt')  # Cambiado a taco.pt por defecto
        self.declare_parameter('confidence', 0.2)   # Confianza al 20%
        # Si esta vacio, detecta todas las clases del modelo
        self.declare_parameter('target_classes', []) 
        self.declare_parameter('show_gui', True)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        model_path = self.get_parameter('model').get_parameter_value().string_value
        self.confidence = self.get_parameter('confidence').get_parameter_value().double_value
        self.target_classes = self.get_parameter('target_classes').get_parameter_value().integer_array_value
        self.show_gui = self.get_parameter('show_gui').get_parameter_value().bool_value

        import os
        if not os.path.isabs(model_path):
            base_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.join(base_dir, 'models', model_path)

        # YOLO model
        self.get_logger().info(f'Cargando modelo YOLO: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info('Modelo YOLO cargado correctamente')

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )

        # Window Name (with namespace)
        self.window_name = f'YOLO Vision - {self.get_namespace()}'

        # Publishers
        self.annotated_pub = self.create_publisher(Image, 'yolo/image_annotated', 10)
        self.detection_pub = self.create_publisher(Detection2DArray, 'yolo/detections', 10)

        self.get_logger().info('Nodo YOLO Vision iniciado. Esperando imagenes...')

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        # Run YOLO inference
        results = self.model(cv_image, conf=self.confidence, verbose=False)

        # Draw detections
        annotated = cv_image.copy()
        # Prepare Detection2DArray
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])

                # Filter by target classes (if list is not empty)
                if len(self.target_classes) > 0 and cls_id not in self.target_classes:
                    continue

                # Bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = float((x1 + x2) / 2.0)
                cy = float((y1 + y2) / 2.0)
                width = float(x2 - x1)
                height = float(y2 - y1)
                
                # Add to ROS message
                detection = Detection2D()
                detection.header = msg.header
                detection.bbox.center.position.x = cx
                detection.bbox.center.position.y = cy
                detection.bbox.size_x = width
                detection.bbox.size_y = height
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(cls_id)
                hypothesis.hypothesis.score = conf
                detection.results.append(hypothesis)
                detections_msg.detections.append(detection)

                # Label for CV - Override class name to "Basura" for generic detection
                class_name = "Basura" 
                label = f'{class_name} {conf:.2f}'

                # Draw box and label
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                cv2.rectangle(annotated, (x1, y1 - label_size[1] - 10),
                              (x1 + label_size[0], y1), (0, 255, 0), -1)
                cv2.putText(annotated, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

                self.get_logger().info(
                    f'Basura detectada: conf={conf:.2f} pos=({x1},{y1})-({x2},{y2})',
                    throttle_duration_sec=1.0
                )

        # Show with OpenCV (solo si show_gui está activado)
        if self.show_gui:
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)

        # Publish annotated image and detections
        try:
            self.detection_pub.publish(detections_msg)
            
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'Error publicando: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
