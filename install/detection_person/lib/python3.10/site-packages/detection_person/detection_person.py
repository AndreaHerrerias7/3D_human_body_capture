import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('data_astra_camera')
        self.get_logger().info('Iniciado')

        # Suscriptores
        self.depth_sub = self.create_subscription(PointCloud2, '/camera/depth/points', self.data_camera1, 10)
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.camera_rgb_detection, 10)
        
        # Publicador
        self.publisher = self.create_publisher(PointCloud2, '/processed/points', 10)

        self.pose = mp.solutions.pose.Pose(static_image_mode=True, model_complexity=2, enable_segmentation=True)
        self.bridge = CvBridge()

        self.current_image = None
        
        self.pose_landmarks_result = []
        self.points_data_2d = []
        self.image_rgbd = None

    # Detectar pose en la imagen RGB
    def camera_rgb_detection(self, image):
        # Convert the ROS Image message to an OpenCV image.
        cv_image = self.bridge.imgmsg_to_cv2(image)

        # Obtener ancho y alto de la imagen 
        height, width, _ = cv_image.shape
        # self.get_logger().info(f'Ancho: {width}, Alto: {height}')

        # Detect pose landmarks from the frame.
        results = self.pose.process(cv_image)

        pose_landmarks = results.pose_landmarks
        
        # Quitar normalización de los puntos
        if pose_landmarks: 
            for lm in pose_landmarks.landmark:
                lm.x *= width
                lm.y *= height

        self.pose_landmarks_result = pose_landmarks

    def data_camera1(self, msg):
        if not self.pose_landmarks_result:
            #self.get_logger().warn('No pose landmarks available.')
            return

        points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        processed_points = []

        min_z = min(points, key=lambda point: point[2])[2]

        for point in points:
            x, y, z = point
            if z < 1.2 * min_z:
                processed_points.append([x, y, z])

        if len(processed_points) == 0:
            #self.get_logger().warn('No points found in the specified z range.')
            return

        points_data = np.array(processed_points, dtype=np.float32)
        

        if points_data.ndim == 2 and points_data.shape[1] == 3:
            self.image_rgbd = self.convert_to_image(points_data)

            """# Ver imagen
            cv2.imshow('Imagen', self.image_rgbd)
            cv2.waitKey(10000)"""

            points_body_px = self.compare_points_with_landmarks(self.image_rgbd, self.pose_landmarks_result)
            #self.get_logger().info(f'Puntos del cuerpo en pixeles: {points_body_px}')
            #if points_body_px:
                #self.get_logger().info(f'Puntos del cuerpo en pixeles: {len(points_body_px)}')
            
            points_body = self.convert_to_pointcloud(points_body_px)
            #self.get_logger().info(f'Puntos del cuerpo real: {points_body}')
            if points_body:
                #self.get_logger().info(f'Puntos del cuerpo real: {len(points_body)}')
                header = msg.header
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
                ]
                landmark_points_data = np.array(points_body, dtype=np.float32)
                cloud_msg = pc2.create_cloud(header, fields, landmark_points_data)
                self.publisher.publish(cloud_msg)

    def convert_to_image(self, points):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        fx = 517.301
        fy = 519.291
        cx = 326.785
        cy = 244.563

        for point in points:
            x, y, z = point
            u = int(fx * x / z + cx)
            v = int(fy * y / z + cy)
            if 0 <= u < 640 and 0 <= v < 480:
                image[v, u] = 255
                # guardar la v,u y z en un array de self.point_data_2d
                self.points_data_2d.append([u, v, z])
                #self.get_logger().info(f'Punto: (u={u}, v={v}, z={z})')

        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    def compare_points_with_landmarks(self, image, pose_landmarks):
        if not pose_landmarks:
            return []
        
        nose = pose_landmarks.landmark[0]
        #self.get_logger().info(f'Nose: (x={nose.x}, y={nose.y}, z={nose.z})')

        points_body = []
        for lm in pose_landmarks.landmark:
            # Ver valores de los landmarks
            #self.get_logger().info(f'Landmark: (x={int(lm.x)}, y={int(lm.y)}, z={lm.z})')
            u, v = int(lm.x), int(lm.y)
            if 0 <= u < 640 and 0 <= v < 480:
                depth = self.get_depth_from_image(u, v)
                #self.get_logger().info(f'Profundidad: {depth}')
                #self.get_logger().info(f'Punto con la profundidad {self.points_data_2d}')
                if depth:
                    points_body.append([u, v, depth])
                    self.get_logger().info('Punto del cuerpo encontrado')

        return points_body

    def get_depth_from_image(self, u, v):
        # Para cada punto en la imagen, recorrer el array de self.points_data_2d y comparar si el punto de la imagen es igual al punto del array
        for point in self.points_data_2d:
            if (point[0] <= 1.2*u or point[0] >= 0.9*u) and (point[1] <= 1.2*v or point[1] >= 0.9*v):
                self.get_logger().info(f'Punto encontrado en la imagen: (u={u}, v={v}, z={point[2]})')
                return point[2]
            else:
                return None

    def convert_to_pointcloud(self, points):
        if not points: 
            return []

        fx = 517.301
        fy = 519.291
        cx = 326.785
        cy = 244.563

        points_3d = []
        for point in points:
            u, v, z = point
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            points_3d.append([x, y, z])

        return points_3d

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
