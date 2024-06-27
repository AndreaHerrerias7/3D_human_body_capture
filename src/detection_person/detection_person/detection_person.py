import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
import mediapipe as mp
from cv_bridge import CvBridge

class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('data_astra_camera')
        self.get_logger().info('Iniciado')

        # Suscriptores
        self.depth_sub = self.create_subscription(Image, '/camera2/depth/image_raw', self.data_rgbd, 10)
        self.image_sub = self.create_subscription(Image, '/camera2/color/image_raw', self.camera_rgb_detection, 10)
        
        # Publicador
        self.publisher = self.create_publisher(PointCloud2, '/processed/points/body', 10)

        self.pose = mp.solutions.pose.Pose(static_image_mode=True, model_complexity=2, enable_segmentation=True)
        self.bridge = CvBridge()

        self.current_image = None
        
        self.pose_landmarks_result = []
        self.points_data_2d = []
        self.depth_image = None

    # Detectar pose en la imagen RGB
    def camera_rgb_detection(self, image):
        # Convert the ROS Image message to an OpenCV image.
        cv_image = self.bridge.imgmsg_to_cv2(image)

        # Obtener ancho y alto de la imagen 
        height, width, _ = cv_image.shape

        # Detectar puntos de referencia de pose en el cuadro.
        results = self.pose.process(cv_image)

        pose_landmarks = results.pose_landmarks
        
        # Quitar normalización de los puntos
        if pose_landmarks: 
            for lm in pose_landmarks.landmark:
                lm.x *= width
                lm.y *= height

            self.pose_landmarks_result = pose_landmarks
            #self.get_logger().info('Puntos de referencia de pose detectados')
            #self.get_logger().info(f'Número de puntos detectados: {len(pose_landmarks.landmark)}')
            #self.get_logger().info(f'Los puntos son: {pose_landmarks}')
        else:
            self.pose_landmarks_result = []

    def data_rgbd(self, msg):
        if  self.pose_landmarks_result == []:
            return
        points_meter = []

        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        #self.get_logger().info('Imagen de profundidad recibida')

        # Convertir la imagen a un array de puntos
        points = self.compare_points_with_landmarks(self.depth_image, self.pose_landmarks_result)

        # Convertir de pixel a metros
        points_meter = self.convert_pixel_to_meter(points)

        header = msg.header
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        landmark_points_data = np.array(points_meter, dtype=np.float32)
        cloud_msg = pc2.create_cloud(header, fields, landmark_points_data)
        self.publisher.publish(cloud_msg)

    def compare_points_with_landmarks(self, depth_image, pose_landmarks):     
        height, width = depth_image.shape
        #self.get_logger().info(f'Ancho de la imagen: {width}')

        points_body = []
        for lm in pose_landmarks.landmark:
            u, v = int(lm.x), int(lm.y)
            if 0 <= u < width and 0 <= v < height:
                depth = depth_image[v, u] * 0.001  # Convertir de milímetros a metros
                if depth > 0:
                    points_body.append([u, v, depth])
                    #self.get_logger().info(f'Punto del cuerpo encontrado: (u={u}, v={v}, z={depth})')

        return points_body
    
    def convert_pixel_to_meter(self, points):
        points_meter = []
        fx = 517.301
        fy = 519.291
        cx = 326.785
        cy = 244.563

        for point in points:
            u, v, z = point
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            points_meter.append([x, y, z])
            #self.get_logger().info(f'Punto convertido a metros: (x={x}, y={y}, z={z})')

        return points_meter

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
