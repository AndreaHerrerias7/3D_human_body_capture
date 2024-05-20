import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime, timedelta
from builtin_interfaces.msg import Time

# from astra_camera.msg import DeviceInfo, Extrinsics, Metadata
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import struct
from custom_msgs.msg import PointsCamera, Point

from custom_msgs.msg import PointsCamera, Point

class DetectionAstra(Node):
    def __init__(self):
        super().__init__('data_astra_camera')
        self.get_logger().info('Iniciado')

        # Suscriptor
        """self.image_sub = self.create_subscription(PointCloud2, '/camera1/depth/points', self.data_camera1, 10) # 18072430022
        self.image_sub = self.create_subscription(PointCloud2, '/camera2/depth/points', self.data_camera2, 10) """

        # Suscriptor para solo una cámara
        self.image_sub = self.create_subscription(PointCloud2, '/camera/depth/points', self.data_camera1, 10)

        # Publicador
        self.publisher = self.create_publisher(PointsCamera, '/processed/points', 10)

    def data_camera1(self, msg):
        self.get_logger().info('Mensaje recibido')

        field_names = [field.name for field in msg.fields]

        if 'x' in field_names and 'y' in field_names and 'z' in field_names:
            points = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))

            processed_points = []

            for point in points:
                x, y, z = point
                self.get_logger().info(f'Punto procesado: x={x}, y={y}, z={z}')
                # Crear instancia de Point para cada punto
                point_msg = Point()
                point_msg.x = float(x)
                point_msg.y = float(y)
                point_msg.z = float(z)
                processed_points.append(point_msg)

                if len(processed_points) > 1000:
                    break

            # Publicar puntos procesados
            msg = PointsCamera()
            msg.points1 = processed_points
            self.publisher.publish(msg)
        else:
            self.get_logger().error('Los campos x, y, z no están presentes en el mensaje de PointCloud2')

    def data_camera2(self, msg):
        self.get_logger().info('Mensaje recibido')

        field_names = [field.name for field in msg.fields]

        if 'x' in field_names and 'y' in field_names and 'z' in field_names:
            points = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))

            processed_points = []

            for point in points:
                x, y, z = point
                self.get_logger().info(f'Punto procesado: x={x}, y={y}, z={z}')
                # Crear instancia de Point para cada punto
                point_msg = Point()
                point_msg.x = float(x)
                point_msg.y = float(y)
                point_msg.z = float(z)
                processed_points.append(point_msg)

                if len(processed_points) > 1000:
                    break

            # Publicar puntos procesados
            msg = PointsCamera()
            msg.points2 = processed_points
            self.publisher.publish(msg)
        else:
            self.get_logger().error('Los campos x, y, z no están presentes en el mensaje de PointCloud2')
    
def main(args=None):
    rclpy.init(args=args)
    detection_astra_node = DetectionAstra()
    rclpy.spin(detection_astra_node)
    detection_astra_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
