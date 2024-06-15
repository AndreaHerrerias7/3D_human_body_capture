#@markdown To better demonstrate the Pose Landmarker API, we have created a set of visualization tools that will be used in this colab. These will draw the landmarks on a detect person, as well as the expected connections between those markers.

from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
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
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2
import numpy as np


class DetectionHuman(Node):
    def __init__(self):
        super().__init__('detection_pose_human')
        self.get_logger().info('Iniciado')

        # Suscriptor
        self.image_sub = self.create_subscription(PointCloud2, '/camera1/depth/points', self.data_camera1, 10)
        
        # Publicador
        self.publisher = self.create_publisher(PointCloud2, '/processed/points', 10)

        """# Matriz de calibración
        self.k = np.array([
            [1, 0, 0, 0.44],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0.0, 0.0, 0.0, 1.0]
        ])"""

        """self.k =  np.array([
    [0.691733,  -0.114198,  -0.713067,   0.596189],
    [0.139864,   0.989907, -0.0228555,  0.0396064],
    [0.70848,  -0.0839221,   0.700724,   0.274987],
    [0,          0,          0,          1]
])"""


    def data_camera1(self, msg):
        # Create a PoseLandmarker object.
        base_options = python.BaseOptions(model_asset_path='pose_landmarker.task')
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True)
        detector = vision.PoseLandmarker.create_from_options(options)
        # Iniciar camara
        # process image
        # Open the webcam.
        cap = cv2.VideoCapture(0)

        if cap.isOpened():
            success, frame = cap.read()

            # Convert the frame to RGB.
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            # Detect pose landmarks from the frame.
            detection_result = detector.detect(image)

            pose_landmarks_list = detection_result.pose_landmarks

            # Publicar pose_landmarks_list como PointCloud2 
            #self.publish_points(pose_landmarks_list, msg)


            # Crear el mensaje PointCloud2
            header = msg.header
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            points_data = np.array(pose_landmarks_list).reshape(-1, 3)
            cloud_msg = pc2.create_cloud(header, fields, points_data)
            #self.get_logger().info(f'Mensaje PointCloud2 creado: {cloud_msg}')

            # Publicar puntos procesados
            self.publisher.publish(cloud_msg)

    
def main(args=None):
    rclpy.init(args=args)
    detection_astra_node = DetectionHuman()
    rclpy.spin(detection_astra_node)
    detection_astra_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
