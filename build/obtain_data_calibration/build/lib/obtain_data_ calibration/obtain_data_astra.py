import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime, timedelta
from builtin_interfaces.msg import Time
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class DetectionAstra(Node):
    def __init__(self):
        super().__init__('data_astra_camera')
        self.get_logger().info('Iniciado')

        # Suscriptor
        self.image_sub = self.create_subscription(PointCloud2, '/camera2/depth/points', self.data_camera1, 10)
        #self.image_sub_body = self.create_subscription(PointCloud2, '/processed/points/body', self.data_camera2, 10)
        
        # Publicador
        self.publisher = self.create_publisher(PointCloud2, '/processed/points', 10)
        #self.publisher_body = self.create_publisher(PointCloud2, '/processed/points/body_c2', 10)

        # Matriz de calibración

        self.k = np.array([
    [ 0.51349915,  0.39198431, -0.76332622,  1.84062  ],
    [-0.36871484,  0.90404816,  0.21620892, -1.037388 ],
    [ 0.77483417,  0.17042661,  0.6087584 ,  0.526757 ],
    [ 0.        ,  0.        ,  0.        ,  1.       ]
])

        
        """self.k = np.array([
            [-0.461651, 0.82762, -0.319255, -1.58399],
            [-0.791298, -0.22156, 0.569876, 0.204414],
            [0.400907, 0.51571, 0.757177, 1.49522],
            [0, 0, 0, 1]
        ])"""


    def data_camera1(self, msg):
        points = np.array(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        self.get_logger().info(f'Puntos recibidos: {len(points)}')
        if len(points) > 0:
            self.get_logger().info(f'Punto 1: {points[0]}')

        processed_points = []
        if len(points) > 0:
            for point in points:
                x, y, z = point
                #self.get_logger().info(f'Punto procesado: x={x}, y={y}, z={z}')

                # Añadir 1 al final del punto para la multiplicación con la matriz 4x4
                point_with_one = np.array([x, y, z, 1.0])

                # Multiplicar por la matriz de calibración
                transformed_point = np.dot(self.k, point_with_one)
                #self.get_logger().info(f'Punto transformado: {transformed_point}')

                # Normalizar el punto
                transformed_point /= transformed_point[3]
                #self.get_logger().info(f'Punto normalizado: {transformed_point}')

                processed_points.append(transformed_point[:3])  # Tomar solo x, y, z transformados
                
                """if len(processed_points) > 50000:
                    break"""

            # Crear el mensaje PointCloud2
            header = msg.header
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            points_data = np.array(processed_points).reshape(-1, 3)
            cloud_msg = pc2.create_cloud(header, fields, points_data)
            #self.get_logger().info(f'Mensaje PointCloud2 creado: {cloud_msg}')

            # Publicar puntos procesados
            self.publisher.publish(cloud_msg)

    def data_camera2(self, msg):
        points = np.array(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        self.get_logger().info(f'Puntos recibidos: {len(points)}')
        if len(points) > 0:
            self.get_logger().info(f'Punto 1: {points[0]}')

        processed_points = []
        if len(points) > 0:
            for point in points:
                x, y, z = point
                #self.get_logger().info(f'Punto procesado: x={x}, y={y}, z={z}')

                # Añadir 1 al final del punto para la multiplicación con la matriz 4x4
                point_with_one = np.array([x, y, z, 1.0])

                # Multiplicar por la matriz de calibración
                transformed_point = np.dot(self.k, point_with_one)
                #self.get_logger().info(f'Punto transformado: {transformed_point}')

                # Normalizar el punto
                transformed_point /= transformed_point[3]
                #self.get_logger().info(f'Punto normalizado: {transformed_point}')

                processed_points.append(transformed_point[:3])

            header = msg.header
            cloud_msg = pc2.create_cloud(header, msg.fields, np.array(processed_points).reshape(-1, 3))
            self.publisher_body.publish(cloud_msg)
            

    
def main(args=None):
    rclpy.init(args=args)
    detection_astra_node = DetectionAstra()
    rclpy.spin(detection_astra_node)
    detection_astra_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
