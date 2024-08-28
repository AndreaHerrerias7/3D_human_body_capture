import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from custom_msgs.msg import Articulations, ArticulationList
import std_msgs.msg

class PointCloudAverager(Node):
    def __init__(self):
        super().__init__('point_cloud_averager')
        self.get_logger().info('Node initialized')
        
        # Suscripciones a los tópicos calibrados
        self.sub_c1 = self.create_subscription(ArticulationList, '/processed/points/body_c1', self.callback_c1, 10)
        self.sub_c2 = self.create_subscription(ArticulationList, '/processed/points/body_c2_calibrated', self.callback_c2, 10)
        #self.datos_camara1 = self.create_subscription(PointCloud2, '/camera1/depth/color/points', self.data_camera1, 10)

        # Almacenamiento para los puntos recibidos
        self.points_c1 = {}
        self.points_c2 = {}

        # Publicador para la media de los puntos
        self.publisher = self.create_publisher(PointCloud2, '/processed/points/average', 10)
        #self.publisher_prueba = self.create_publisher(PointCloud2, '/processed/points/prueba', 10)

    def data_camera1(self, msg):
        points = np.array(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        # obtener header
        header = msg.header
        #self.get_logger().info(f'Puntos recibidos: {len(points)}')
        """if len(points) > 0:
            self.get_logger().info(f'Punto 1: {points[0]}')"""

        processed_points = []
        if len(points) > 0:
            for point in points:
                x, y, z = point
                #self.get_logger().info(f'Punto procesado: x={x}, y={y}, z={z}')

                # Añadir 1 al final del punto para la multiplicación con la matriz 4x4
                point_with_one = np.array([x, y, z, 1.0])


                # Normalizar el punto
                point_with_one /= point_with_one[3]
                #self.get_logger().info(f'Punto normalizado: {transformed_point}')

                processed_points.append(point_with_one[:3])

                if len(processed_points) > 4000:
                    break
                
            self.publish_average(processed_points,header)
                

    def callback_c1(self, msg):
        header = msg.header
        self.process_message(msg, self.points_c1)
        self.calculate_and_publish_average(header)

    def callback_c2(self, msg):
        header = msg.header
        self.process_message(msg, self.points_c2)
        self.calculate_and_publish_average(header)

    def process_message(self, msg, storage):
        # Procesar cada articulación en el mensaje ArticulationList
        for articulation in msg.articulations:
            point_id = articulation.id
            x, y, z = articulation.coordinates.x, articulation.coordinates.y, articulation.coordinates.z
            visibility = articulation.visibility
            #self.get_logger().info(f'Punto recibido: id={point_id}, x={x}, y={y}, z={z}, visibility={visibility}')
            storage[point_id] = {'coordinates': np.array([x, y, z]), 'visibility': visibility}

    def calculate_and_publish_average(self, header):
        common_ids = set(self.points_c1.keys()).intersection(set(self.points_c2.keys()))
        averaged_points = []

        for point_id in common_ids:
            visibility_c1 = self.points_c1[point_id]['visibility']
            visibility_c2 = self.points_c2[point_id]['visibility']

            if visibility_c1 < 0.9 or visibility_c2 < 0.9:
                # Eliminar punto de las listas si la visibilidad es baja
                self.get_logger().info(f'Eliminando id {point_id} debido a baja visibilidad: visibilidad_c1={visibility_c1}, visibilidad_c2={visibility_c2}')
                del self.points_c1[point_id]
                del self.points_c2[point_id]
                continue

            elif visibility_c1 > 0.9 and visibility_c2 > 0.9:
                self.get_logger().info(f'Usando id {point_id} punto de cámara 1 y 2: {self.points_c1[point_id]["coordinates"]} y {self.points_c2[point_id]["coordinates"]} con visibilidad {visibility_c1} y {visibility_c2}')
                avg_point = (self.points_c1[point_id]['coordinates'] + self.points_c2[point_id]['coordinates']) / 2
                #self.get_logger().info(f'Usando punto de cámara 1 y 2: {avg_point} con visibilidad {visibility_c1} y {visibility_c2}')
                averaged_points.append([avg_point[0], avg_point[1], avg_point[2]])

            elif visibility_c1 > 0.9 and visibility_c2 <= 0.9:
                self.get_logger().info(f'Usando id {point_id} punto de cámara 1: {self.points_c1[point_id]["coordinates"]} con visibilidad {visibility_c1}')
                #self.get_logger().info(f'Usando punto de cámara 1: {self.points_c1[point_id]["coordinates"]} con visibilidad {visibility_c1}')
                averaged_points.append([self.points_c1[point_id]['coordinates'][0], self.points_c1[point_id]['coordinates'][1], self.points_c1[point_id]['coordinates'][2]])
            
            elif visibility_c1 <= 0.9 and visibility_c2 > 0.9:
                self.get_logger().info(f'Usando id {point_id} punto de cámara 2: {self.points_c2[point_id]["coordinates"]} con visibilidad {visibility_c2}')
                #self.get_logger().info(f'Usando punto de cámara 2: {self.points_c2[point_id]["coordinates"]} con visibilidad {visibility_c2}')
                averaged_points.append([self.points_c2[point_id]['coordinates'][0], self.points_c2[point_id]['coordinates'][1], self.points_c2[point_id]['coordinates'][2]])
            
        

        if averaged_points:
            self.publish_average(averaged_points, header)
        


    def publish_average(self, averaged_points,header):
        msg = PointCloud2()

        #header = self.create_header()
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        cloud_msg = pc2.create_cloud(header, msg.fields, np.array(averaged_points).reshape(-1, 3))
        #self.get_logger().info(f'Mensaje PointCloud2 creado: {cloud_msg}')
        self.publisher.publish(cloud_msg)
        # Convertir en PointCloud2 y publicar
        #cloud_msg_prueba = pc2.create_cloud(header, msg.fields, np.array(averaged_points).reshape(-1, 3))
        #self.get_logger().info(f'Mensaje PointCloud2 creado prueba: {cloud_msg_prueba}')
        #self.publisher_prueba.publish(cloud_msg_prueba)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAverager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
