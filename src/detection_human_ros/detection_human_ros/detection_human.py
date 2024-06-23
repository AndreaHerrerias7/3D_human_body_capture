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
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.data_camera1, 10)
        
        # Publicador
        self.publisher = self.create_publisher(PointCloud2, '/processed/points', 10)

        self.pose = mp.solutions.pose.Pose(static_image_mode=True, model_complexity=2, enable_segmentation=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        

    def data_camera1(self, image):
        # Convert the ROS Image message to an OpenCV image.
        cv_image = self.br.imgmsg_to_cv2(image)

        # Convert the frame to RGB.
        #rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        rgb_frame = cv_image

        # Detect pose landmarks from the frame.
        results = self.pose.process(rgb_frame)

        pose_landmarks = results.pose_landmarks
        if pose_landmarks:
            self.get_logger().info(f'Pose landmarks detected')
            for idx, landmark in enumerate(pose_landmarks.landmark):
                self.get_logger().info(f'Landmark {idx}: (x={landmark.x}, y={landmark.y}, z={landmark.z})')

            # Draw the pose landmarks on the image.
            annotated_image = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
            mp.solutions.drawing_utils.draw_landmarks(
                annotated_image,
                pose_landmarks,
                mp.solutions.pose.POSE_CONNECTIONS,
                mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
            )


            # Process and publish the pose landmarks as PointCloud2.
            self.publish_points(pose_landmarks, image.header)


    def publish_points(self, pose_landmarks, header):
        if not pose_landmarks:
            return

        points = []
        for landmark in pose_landmarks.landmark:
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

def main(args=None):
    rclpy.init(args=args)
    detection_astra_node = DetectionHuman()
    rclpy.spin(detection_astra_node)
    detection_astra_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
