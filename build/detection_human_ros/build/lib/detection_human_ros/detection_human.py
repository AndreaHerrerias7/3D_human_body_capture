#@markdown To better demonstrate the Pose Landmarker API, we have created a set of visualization tools that will be used in this colab. These will draw the landmarks on a detect person, as well as the expected connections between those markers.

from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
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

        self.br = CvBridge()

        # Suscriptor
        self.image_sub = self.create_subscription(Image, '/camera1/color/image_raw', self.data_camera1, 10)
        
        # Publicador
        self.publisher = self.create_publisher(PointCloud2, '/processed/points', 10)

        # Create a PoseLandmarker object.
        base_options = python.BaseOptions(model_asset_path='pose_landmarker.task')
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(options)


    def data_camera1(self, image):
        # Convert the ROS Image message to an OpenCV image.
        cv_image = self.br.imgmsg_to_cv2(image)

        # Convert the frame to RGB.
        rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        # Detect pose landmarks from the frame.
        detection_result = self.detector.detect(mp_image)

        pose_landmarks_list = detection_result.pose_landmarks
        self.get_logger().info(f'Pose landmarks detected: {len(pose_landmarks_list)}')
        self.get_logger().info(f'Pose landmarks: {pose_landmarks_list}')

        # Process and publish the pose landmarks as PointCloud2.
        self.publish_points(pose_landmarks_list, image.header)


    def publish_points(self, pose_landmarks_list, header):
        if not pose_landmarks_list:
            #self.get_logger().info('No landmarks detected')
            return

        points = []
        for pose_landmarks in pose_landmarks_list:
            for landmark in pose_landmarks:
                points.append([landmark.x, landmark.y, landmark.z])


        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create the PointCloud2 message.
        cloud_msg = pc2.create_cloud(header, fields, points)

        # Publish the processed points.
        self.publisher.publish(cloud_msg)
        #self.get_logger().info(f'Published PointCloud2 message with {len(points)} points')


def main(args=None):
    rclpy.init(args=args)
    detection_astra_node = DetectionHuman()
    rclpy.spin(detection_astra_node)
    detection_astra_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
