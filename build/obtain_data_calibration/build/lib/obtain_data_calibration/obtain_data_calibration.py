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
from geometry_msgs.msg import Vector3
from custom_msgs.msg import Articulations, ArticulationList

class DetectionAstra(Node):
    def __init__(self):
        super().__init__('Data_calibrated')
        self.get_logger().info('Iniciado')

        # Suscriptor
        #self.image_sub = self.create_subscription(PointCloud2, '/camera2/depth/points', self.data_camera1, 10)
        self.image_sub_body = self.create_subscription(ArticulationList, '/processed/points/body_c2', self.data_camera2, 10)
        
        # Publicador
        #self.publisher = self.create_publisher(PointCloud2, '/processed/points', 10)
        self.publisher_body = self.create_publisher(ArticulationList, '/processed/points/body_c2_calibrated', 10)

        # Matriz de calibración

        self.k =  np.array([
    [ 0.707278 , -0.0357788, -0.70603  ,  0.95994  ],
    [ 0.0509082,  0.998703 ,  0.000387929, 0.00870791],
    [ 0.7051   , -0.0362171,  0.708182 ,  0.459205 ],
    [ 0.       ,  0.       ,  0.       ,  1.       ]
])


    def data_camera1(self, msg):
        points = np.array(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        #self.get_logger().info(f'Puntos recibidos: {len(points)}')
        #if len(points) > 0:
        #    self.get_logger().info(f'Punto 1: {points[0]}')

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
                
                if len(processed_points) > 400000:
                    break

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
        processed_articulations = []

        for articulation in msg.articulations:
            x, y, z = articulation.coordinates.x, articulation.coordinates.y, articulation.coordinates.z

            # Añadir 1 al final del punto para la multiplicación con la matriz 4x4
            point_with_one = np.array([x, y, z, 1.0])

            # Multiplicar por la matriz de calibración
            transformed_point = np.dot(self.k, point_with_one)

            # Normalizar el punto
            transformed_point /= transformed_point[3]

            # Crear una nueva instancia de CDetectedPerson para almacenar los puntos transformados
            new_articulation = Articulations()
            new_articulation.id = articulation.id
            new_articulation.coordinates = Vector3(
                x=transformed_point[0],
                y=transformed_point[1],
                z=transformed_point[2]
            )
            new_articulation.visibility = articulation.visibility

            # Añadir la nueva articulación procesada a la lista
            processed_articulations.append(new_articulation)

        # Crear un nuevo mensaje ArticulationList y publicar
        new_msg = ArticulationList()
        new_msg.header = msg.header
        new_msg.articulations = processed_articulations

        self.publisher_body.publish(new_msg)

    
def main(args=None):
    rclpy.init(args=args)
    obtain_data_calibration = DetectionAstra()
    rclpy.spin(obtain_data_calibration)
    obtain_data_calibration.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
