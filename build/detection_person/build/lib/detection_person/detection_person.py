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

        # Suscriptor
        self.image_sub = self.create_subscription(PointCloud2, '/camera/depth/points', self.data_camera1, 10)

        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.camera_rgb_detection, 10)
        
        # Publicador
        self.publisher = self.create_publisher(PointCloud2, '/processed/points', 10)

        self.pose = mp.solutions.pose.Pose(static_image_mode=True, model_complexity=2, enable_segmentation=True, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.bridge = CvBridge()

        self.current_image = None
        
        self.min_x = np.inf
        self.min_y = np.inf
        self.max_x = -np.inf
        self.max_y = -np.inf
        self.pose_landmarks_result = None
        self.points_data_2d = None

    # Detectar pose en la imagen RGB
    def camera_rgb_detection(self, image):
        # Convert the ROS Image message to an OpenCV image.
        cv_image = self.bridge.imgmsg_to_cv2(image)

        # Obtener ancho y alto de la imagen 
        height, width, _ = cv_image.shape
        self.get_logger().info(f'Ancho1: {width}, Alto: {height}')

        # Detect pose landmarks from the frame.
        results = self.pose.process(cv_image)

        pose_landmarks = results.pose_landmarks
        

        #Para dibujarlo
        """if pose_landmarks:
            #self.get_logger().info(f'Pose landmarks detected')
            #for idx, landmark in enumerate(pose_landmarks.landmark):
                #self.get_logger().info(f'Landmark {idx}: (x={landmark.x}, y={landmark.y}, z={landmark.z})')

            # Draw the pose landmarks on the image.
            annotated_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            mp.solutions.drawing_utils.draw_landmarks(
                annotated_image,
                pose_landmarks,
                mp.solutions.pose.POSE_CONNECTIONS,
                mp.solutions.drawing_utils.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2),
                mp.solutions.drawing_utils.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
            )

            # Mostrar la imagen
            cv2.imshow('Image', annotated_image)
            cv2.waitKey(5000)"""
            
            



        # Quitar normalización de los puntos
        if pose_landmarks: 
            for lm in pose_landmarks.landmark:
                lm.x *= width
                lm.y *= height
                #self.get_logger().info(f'Landmark  (x={lm.x}, y={lm.y})')

        self.pose_landmarks_result = pose_landmarks

        # Datos obtenidos pie
        # Landmark 31: (x=75.81861877441406, y=579.90966796875, z=0.15833468735218048)
        # Landmark 32: (x=59.77082061767578, y=554.6194458007812, z=0.33150044083595276)



    def data_camera1(self, msg):
        point_body = None
        points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        #self.get_logger().info(f'Puntos recibidos: {len(points)}')
        #self.get_logger().info(f'Punto 1: {points[0]}')

        # Obtener punto con la z más pequeña
        min_z = np.inf # Inicializar con infinito
        min_point = None
        processed_points = []
        

        min_z = min(points, key=lambda point: point[2])[2]

        # Obtener puntos con z menor que 1.5 * min_z
        for point in points:
            x, y, z = point
            if z < 1.2 * min_z:
                processed_points.append([x, y, z])


        #self.get_logger().info(f'Tamaño de puntos procesados: {len(processed_points)}')

        if len(processed_points) == 0:
            #self.get_logger().warn('No points found in the specified z range.')
            return

        points_data = np.array(processed_points, dtype=np.float32)

        if points_data.ndim == 2 and points_data.shape[1] == 3:
            image = self.convert_to_image(points_data)

            # Mostrar la imagen
            # cv2.imshow('Image', image)
            # cv2.waitKey(5000)
            # cv2.destroyAllWindows()       

            points_body_px = []

            # Llamar a funcion para comparar puntos de la imagen con los landmarks
            points_body_px = self.compare_points_with_landmarks(image, self.pose_landmarks_result, msg)
            if points_body_px:
                self.get_logger().info(f'Puntos del cuerpo en pixeles: {len(points_body_px)}')
            
            points_body = self.convert_to_pointcloud(points_body_px)
            if points_body:
                self.get_logger().info(f'Puntos del cuerpo: {len(points_body)}')
            # Publicar puntos

            #self.get_logger().info(f'Detected {len(landmarks_meters)} landmarks')

            if point_body: 
                header = msg.header
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
                ]
                landmark_points_data = np.array(points_body, dtype=np.float32)
                cloud_msg = pc2.create_cloud(header, fields, landmark_points_data)
                self.publisher.publish(cloud_msg)
        """else:
            #self.get_logger().error(f'Error en la forma de points_data: {points_data.shape}')"""
        
    def convert_to_image(self, points):
        # Crear una imagen vacía con el tamaño deseado (640x480)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # fx: Focal length of the camera along the x-axis
        fx = 517.301
        # fy: Focal length of the camera along the y-axis
        fy = 519.291
        # cx: X-coordinate of the principal point/optical center of the camera in pixels
        cx = 326.785
        # cy: Y-coordinate of the principal point/optical center of the camera in pixels
        cy = 244.563

        k = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0, 0, 1]])
        
        
        # Rellenar la imagen con los puntos
        for point in points:
            x, y, z = point
            u = int(fx * x / z + cx)
            v = int(fy * y / z + cy)
            self.points_data_2d = np.array([u, v, z])
            if 0 <= u < 480 and 0 <= v < 640:
                image[v, u] = 255

        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    """def convert_landmarks_to_meters(self, landmarks, min_x, max_x, min_y, max_y):
        landmarks_meters = []
        for lm in landmarks:
            x = lm.x * (max_x - min_x) + min_x
            y = lm.y * (max_y - min_y) + min_y
            z = lm.z  # Suponiendo que z ya está en metros
            landmarks_meters.append([x, y, z])
        return landmarks_meters"""
    
    def compare_points_with_landmarks(self, image, pose_landmarks, msg):
        if not pose_landmarks:
            return
        
        # Obtener punto de la nariz(landmark 0)
        nose = pose_landmarks.landmark[0]
        self.get_logger().info(f'Nose: (x={nose.x}, y={nose.y}, z={nose.z})')

        # Restarle en la y -40
        nose.y -= 40

        if all(0 <= coord < dim for coord, dim in zip((nose.y, nose.x), image.shape[:2])) and (image[int(nose.y), int(nose.x)] != 255).all():
            return


        points_body = []
        for lm in pose_landmarks.landmark:
            u, v = int(lm.x), int(lm.y)
            if 0 <= u < 480 and 0 <= v < 640:
                    depth = self.points_data_2d[2]
                    points_body.append([lm.x, lm.y, depth])
                    self.get_logger().info('Punto del cuerpo encontrado')

        return points_body
    
    def convert_to_pointcloud(self, points):
        if not points: 
            return
        # fx: Focal length of the camera along the x-axis
        fx = 517.301
        # fy: Focal length of the camera along the y-axis
        fy = 519.291
        # cx: X-coordinate of the principal point/optical center of the camera in pixels
        cx = 326.785
        # cy: Y-coordinate of the principal point/optical center of the camera in pixels
        cy = 244.563

        k = np.array([[fx, 0, cx],
                      [0, fy, cy],
                      [0, 0, 1]])
        
        points_3d = []
        # Rellenar la imagen con los puntos
        if len(points) > 0: 
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
