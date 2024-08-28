import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import mediapipe as mp
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from custom_msgs.msg import Articulations, ArticulationList

class CDetectedPerson():
    def __init__(self):
        self.id = 0                                 # id of articulation
        self.coordinates = Vector3()                # coordinates of articulation
        self.visibility = 0.0                       # visibility


class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('data_astra_camera')
        self.get_logger().info('Node initialized')

        # Suscriptores
        self.depth_sub_c1 = self.create_subscription(Image, '/camera1/depth/image_raw', self.data_rgbd_c1, 10)
        self.image_sub_c1 = self.create_subscription(Image, '/camera1/color/image_raw', self.camera_rgb_detection_c1, 10)
        self.depth_sub_c2 = self.create_subscription(Image, '/camera2/depth/image_raw', self.data_rgbd_c2, 10)
        self.image_sub_c2 = self.create_subscription(Image, '/camera2/color/image_raw', self.camera_rgb_detection_c2, 10)
        
        # Publicadores
        self.publisher_c1 = self.create_publisher(ArticulationList, '/processed/points/body_c1', 10)
        self.publisher_c2 = self.create_publisher(ArticulationList, '/processed/points/body_c2', 10)

        # Timer para la publicación periódica
        self.timer = self.create_timer(0.5, self.publisher_image)

        # Configuración de MediaPipe y CVBridge
        self.pose = mp.solutions.pose.Pose(static_image_mode=True, model_complexity=2, enable_segmentation=True)
        self.bridge = CvBridge()

        self.pose_landmarks_result_c1 = []
        self.pose_landmarks_result_c2 = []

    def camera_rgb_detection_c1(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image)
        height, width, _ = cv_image.shape
        results = self.pose.process(cv_image)
        pose_landmarks = results.pose_landmarks

        if pose_landmarks:
            mp.solutions.drawing_utils.draw_landmarks(cv_image, pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)
            # cambiar a rgb
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            """cv2.imshow('Pose Detection Camera 1', cv_image)
            cv2.waitKey(1)"""
            for lm in pose_landmarks.landmark:
                lm.x = lm.x * width
                lm.y = lm.y * height

            

            # Guardar resultados
            self.pose_landmarks_result_c1 = pose_landmarks
        else:
            self.pose_landmarks_result_c1 = []

    def camera_rgb_detection_c2(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(image)
        height, width, _ = cv_image.shape
        results = self.pose.process(cv_image)
        pose_landmarks = results.pose_landmarks

        if pose_landmarks:
            mp.solutions.drawing_utils.draw_landmarks(cv_image, pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)
            # cambiar a rgb
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            """cv2.imshow('Pose Detection Camera 2', cv_image)
            cv2.waitKey(1)"""
            for lm in pose_landmarks.landmark:
                lm.x = lm.x * width
                lm.y = lm.y * height
            

            # Guardar resultados
            self.pose_landmarks_result_c2 = pose_landmarks
        else:
            self.pose_landmarks_result_c2 = []

    def data_rgbd_c1(self, msg):
        if not self.pose_landmarks_result_c1:
            return
        
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        """cv2.imshow('Pose Detection Camera 1 depth', depth_image)
        cv2.waitKey(1)"""
        points = self.compare_points_with_landmarks(depth_image, self.pose_landmarks_result_c1)
        points_meter = self.convert_pixel_to_meter(points)
        #self.get_logger().info(f'Points detected in camera 1: {points_meter}')

        articulations_list_msg = ArticulationList()
        articulations_list_msg.header = msg.header


        for id, x, y, z, visibility in points_meter:
            articulation = Articulations()
            articulation.id = id
            articulation.coordinates.x = x
            articulation.coordinates.y = y
            articulation.coordinates.z = z
            articulation.visibility = visibility

            articulations_list_msg.articulations.append(articulation)


        self.publisher_c1.publish(articulations_list_msg)

    def data_rgbd_c2(self, msg):
        if not self.pose_landmarks_result_c2:
            return
        
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        points = self.compare_points_with_landmarks(depth_image, self.pose_landmarks_result_c2)

        #self.get_logger().info(f'Points detected in camera 2: {points}')
        points_meter = self.convert_pixel_to_meter(points)

        articulations_list_msg = ArticulationList()
        articulations_list_msg.header = msg.header


        for id, x, y, z, visibility in points_meter:
            articulation = Articulations()
            articulation.id = id
            articulation.coordinates.x = x
            articulation.coordinates.y = y
            articulation.coordinates.z = z
            articulation.visibility = visibility

            articulations_list_msg.articulations.append(articulation)

        self.publisher_c2.publish(articulations_list_msg)


    def process_depth_data(self, msg, camera_id):
        if camera_id == 1:
            pose_landmarks_result = self.pose_landmarks_result_c1
            publisher = self.publisher_c1
        elif camera_id == 2:
            pose_landmarks_result = self.pose_landmarks_result_c2
            publisher = self.publisher_c2
        else:
            return

        if not pose_landmarks_result:
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        points = self.compare_points_with_landmarks(depth_image, pose_landmarks_result)
        points_meter = self.convert_pixel_to_meter(points)

        
        articulations_list_msg = ArticulationList()
        articulations_list_msg.header = msg.header


        for id, x, y, z, visibility in points_meter:
            articulation = Articulations()
            articulation.id = id
            articulation.coordinates.x = x
            articulation.coordinates.y = y
            articulation.coordinates.z = z
            articulation.visibility = visibility

            articulations_list_msg.articulations.append(articulation)

        publisher.publish(articulations_list_msg)

    def compare_points_with_landmarks(self, depth_image, pose_landmarks):
        if not pose_landmarks:
            return []
        
        height, width = depth_image.shape
        points_body = []
        #self.get_logger().info(f'pose_landmarks: {pose_landmarks}')
        for id, lm in enumerate(pose_landmarks.landmark):
            u, v = int(lm.x), int(lm.y)
            #self.get_logger().info(f'Punto: id={id}, u={u}, v={v}, visibility={lm.visibility}')
            if 0 <= u < width and 0 <= v < height:
                depth = depth_image[v, u] * 0.001  # Convertir de milímetros a metros
                if depth > 0:
                    points_body.append([id, u, v, depth, lm.visibility])
        return points_body

    def convert_pixel_to_meter(self, points):
        points_meter = []
        fx = 517.301
        fy = 519.291
        cx = 326.785
        cy = 244.563

        for point in points:
            id, u, v, z, visibility = point
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            points_meter.append([id, x, y, z, visibility])
            #self.get_logger().info(f'Punto: id={id}, x={x}, y={y}, z={z}, visibility={visibility}')

        return points_meter

    def publisher_image(self):
        # Implementa la lógica de publicación periódica aquí si es necesario
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

