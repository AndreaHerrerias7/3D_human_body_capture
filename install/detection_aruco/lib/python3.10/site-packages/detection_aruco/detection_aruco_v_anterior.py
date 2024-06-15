"""Apriltag"""


import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Vector3
from datetime import datetime, timedelta
from builtin_interfaces.msg import Time
from apriltag_msgs.msg import AprilTagDetectionArray

from custom_msgs.msg import Correspondences, Plane, PlaneMatch

TH_TSTAMPS = 0.01

class CDetectedPlane():
    def __init__(self) -> None:
        self.id = 0                             # id of the detected planes
        self.d = 0.0                            # distance of the plane
        self.normal = np.array([0.0,0.0,0.0])   # normal of the plane

class CArucoObservation():
    def __init__(self) -> None:
        self.cam = -1           # camera id from where it was observed
        self.tstamp = Time()    # timestamp initialization
        self.planes = list()    # list of planes in this observation of class CDetectedPlane
    
    def __str__(self):
        mystr = f"ArucoObservation from cam {self.cam} at {str(self.tstamp)} with planes "
        for plane in self.planes:
            mystr += f"[{plane.id}] : d = {plane.d} | normal = {plane.normal}\n"
        return mystr

class DrawAruco():
    def __init__(self) -> None: 
        self.a = 0

    def drawMarker(self, image, corners, ids):
        cv_image_bgr = cv2.aruco.drawDetectedMarkers(image, corners, ids)

        return cv_image_bgr

    def drawCenter(self, image, corners, ids):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                corners=markerCorner.reshape((4,2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                image = cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

        return image
    
    def drawAxis(self, image, ids, corners, K, distorsion, marker_length):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_length, K, distorsion)
                rvecs_matrix, _ = cv2.Rodrigues(rvecs)
                image = cv2.aruco.drawAxis(image, K, distorsion, rvecs_matrix, tvecs, marker_length)

        return image


    
class DetectionAruco(Node):
    def __init__(self):
        super().__init__('Detection_Aruco')
        self.get_logger().info(f'Iniciado')

        self.br = CvBridge()

        ### Parameters for RGB image processing of ASTRA ORBBEC
        # fx: Focal length of the camera along the x-axis
        self.fx = 517.301
        # fy: Focal length of the camera along the y-axis
        self.fy = 519.291
        # cx: X-coordinate of the principal point/optical center of the camera in pixels
        self.cx = 326.785
        # cy: Y-coordinate of the principal point/optical center of the camera in pixels
        self.cy = 244.563

        self.k = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0, 0, 1]])
        
        self.distorsion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))


        """ Parámetros Aruco opencv"""
        self.marker_length = 17.2

        """Parámetros Detector Aruco"""
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        self.observations = [None, None] # cameras one and two

        self.subscription = self.create_subscription(AprilTagDetectionArray, 'detections', self.tag_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera1/color/image_raw', self.on_camera1, 10)
        self.image_sub = self.create_subscription(Image, '/camera2/color/image_raw', self.on_camera2, 10)
        self.publisher = self.create_publisher(Correspondences, '/detection_aruco/topic', 10)

        # timer to send info every 5 secs
        self.timer = self.create_timer(1.0, self.publisher_image)
 
    def tag_callback(self, msg):
        self.centres = []
        for detection in msg.detections:
            self.centres.append((int(detection.centre.x), int(detection.centre.y)))

    def on_camera1(self, image):
        # self.get_logger().info(f'Camara 1')
        if self.centres:
            # process image
            cv_image = self.br.imgmsg_to_cv2(image)
            cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  

            for centre in self.centres: 
                cv2.circle(cv_image_bgr, (centre[0], centre[1]), 5, (0, 0, 255), -1)

            cv2.imshow('Imagen 1', cv_image_bgr)
            cv2.waitKey(1)
        

            """# store plane information
            plane = CDetectedPlane()
            plane.id = markerID
            plane.d = np.linalg.norm(tvecs)
            plane.normal = rvecs_matrix[:, 2]

            # add it to the observation
            obs.planes.append(plane)

            # debug
            self.get_logger().info(f'obs cam1, {obs}')

            # store observation (and implicitly set ready flag)
            self.observations[0] = obs"""



    def on_camera2(self, image):
        
        a = 0

        
    def publisher_image(self):
        # polling until two observations are ready
        if self.observations[0] and self.observations[1]:
            #self.get_logger().info(f'Observations2, {abs(self.observations[1].tstamp)}')
            #self.get_logger().info(f'Valor absoluto, {abs(self.observations[0].tstamp - self.observations[1].tstamp)}')
            if abs(self.observations[0].tstamp.sec - self.observations[1].tstamp.sec) < TH_TSTAMPS:
                # build msg to publish
                msg = Correspondences()

                for plane_0 in self.observations[0].planes:
                    for plane_1 in self.observations[1].planes:
                        if plane_0.id == plane_1.id:
                            # PlaneMatch found                         
                            plane_match = PlaneMatch()

                            plane_match.first = Plane()
                            plane_match.first.d = plane_0.d
                            plane_match.first.n.x = plane_0.normal[0]
                            plane_match.first.n.y = plane_0.normal[1]
                            plane_match.first.n.z = plane_0.normal[2]

                            plane_match.second = Plane()
                            plane_match.second.d = plane_1.d
                            plane_match.second.n.x = plane_1.normal[0]
                            plane_match.second.n.y = plane_1.normal[1]
                            plane_match.second.n.z = plane_1.normal[2]
                            

                            msg.correspondences.append(plane_match)
                msg.first_label = '/camera1/color/image_raw'
                msg.second_label = '/camera2/color/image_raw'

                if len(msg.correspondences) > 0:
                    self.publisher.publish(msg)
                
            # reset observations
            self.observations[0] = None
            self.observations[1] = None
                      

def main(args=None):
    rclpy.init(args=args)
    detection_aruco_node = DetectionAruco()
    rclpy.spin(detection_aruco_node)
    detection_aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#######################
#######################
#######################
#######################
#######################
#######################
#######################
#######################
#######################
#######################
#######################
"""""VERSION CORRECTA ARUCOS"""""
#######################
#######################
#######################
#######################
#######################
#######################
#######################
#######################
#######################
#######################
#######################

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Vector3
from datetime import datetime, timedelta
from builtin_interfaces.msg import Time

from custom_msgs.msg import Correspondences, Plane, PlaneMatch

TH_TSTAMPS = 0.01

class CDetectedPlane():
    def __init__(self) -> None:
        self.id = 0                             # id of the detected planes
        self.d = 0.0                            # distance of the plane
        self.normal = np.array([0.0,0.0,0.0])   # normal of the plane

class CArucoObservation():
    def __init__(self) -> None:
        self.cam = -1           # camera id from where it was observed
        self.tstamp = Time()    # timestamp initialization
        self.planes = list()    # list of planes in this observation of class CDetectedPlane
    
    def __str__(self):
        mystr = f"ArucoObservation from cam {self.cam} at {str(self.tstamp)} with planes "
        for plane in self.planes:
            mystr += f"[{plane.id}] : d = {plane.d} | normal = {plane.normal}\n"
        return mystr

class DrawAruco():
    def __init__(self) -> None: 
        self.a = 0

    def drawMarker(self, image, corners, ids):
        cv_image_bgr = cv2.aruco.drawDetectedMarkers(image, corners, ids)

        return cv_image_bgr

    def drawCenter(self, image, corners, ids):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                corners=markerCorner.reshape((4,2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                image = cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

        return image
    
    def drawAxis(self, image, ids, corners, K, distorsion, marker_length):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_length, K, distorsion)
                rvecs_matrix, _ = cv2.Rodrigues(rvecs)
                image = cv2.aruco.drawAxis(image, K, distorsion, rvecs_matrix, tvecs, marker_length)

        return image


    
class DetectionAruco(Node):
    def __init__(self):
        super().__init__('Detection_Aruco')
        self.get_logger().info(f'Iniciado')

        self.br = CvBridge()

        # Parámetros camara usb
        # self.K = np.array(((676.699523, 0., 283.804190),
        #                    (0., 676.358437, 247.200716),
        #                    (0., 0., 1.)))
        # self.distorsion = np.array((0.019135, -0.44869, 0.000693, -0.014817, 0.0))

        # camera matrix camera 
        """self.K_1 = np.array([[739.901716, 0.000000, 348.296261],
                           [0.000000, 730.526241, 193.897742],
                           [0.000000, 0.000000, 1.000000]])

        # distortion camera 
        self.distorsion_1 = np.array([0.251910, -0.443212, 0.005339, 0.024901, 0.000000])

        # Parámetros cámara nueva (tiene la pegatina)
        self.K_2 = np.array([[613.957058, 0., 323.62420],
                            [0., 618.575469, 216.459925],
                            [0., 0., 1.]])
        self.distorsion_2 = np.array([-0.083010, 0.161326, -0.007532, 0.000727, 0.0])"""
        ### Parameters for RGB image processing of ASTRA ORBBEC
        # fx: Focal length of the camera along the x-axis
        self.fx = 517.301
        # fy: Focal length of the camera along the y-axis
        self.fy = 519.291
        # cx: X-coordinate of the principal point/optical center of the camera in pixels
        self.cx = 326.785
        # cy: Y-coordinate of the principal point/optical center of the camera in pixels
        self.cy = 244.563

        self.k = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0, 0, 1]])
        
        self.distorsion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))


        """ Parámetros Aruco opencv"""
        self.marker_length = 17.2

        """Parámetros Detector Aruco"""
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        self.observations = [None, None] # cameras one and two

        self.image_sub = self.create_subscription(Image, '/camera1/color/image_raw', self.on_camera1, 10)
        self.image_sub = self.create_subscription(Image, '/camera2/color/image_raw', self.on_camera2, 10)
        self.publisher = self.create_publisher(Correspondences, '/custom_msgs/msg/correspondences', 10)

        # timer to send info every 5 secs
        self.timer = self.create_timer(1.0, self.publisher_image)

    def is_valid_rotation_matrix(self, R):
        return np.allclose(np.dot(R, R.T), np.eye(3), atol=1e-6) and np.isclose(np.linalg.det(R), 1.0, atol=1e-6)


    def on_camera1(self, image):
        # self.get_logger().info(f'Camara 1')

        # process image
        cv_image = self.br.imgmsg_to_cv2(image)
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  


        # initialize observation
        obs = CArucoObservation()
        obs.cam = 0
        obs.tstamp = image.header.stamp


        drawImage = DrawAruco()

        # Detecta las esquinas e ids de los marcadores
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image_bgr, self.arucoDict, parameters=self.arucoParams)

        if self.observations[0]:
            return

        cv_image_bgr = drawImage.drawMarker(cv_image_bgr, corners, ids)
        cv_image_bgr = drawImage.drawCenter(cv_image_bgr, corners, ids)
        cv_image_bgr = drawImage.drawAxis(cv_image_bgr, ids, corners, self.k, self.distorsion, self.marker_length)

        cv2.imshow('Imagen 1', cv_image_bgr)
        cv2.waitKey(1)

        # Si se detecta algún marcador
        if ids is not None:
            # Si detecta los tres a la vez ids te devuelve: [[0][1][2]]
            ids = ids.flatten()

            # Recorre los marcadores detectados
            # En cada for se cogerá el id con sus esquinas correspondientes
            for (markerCorner, markerID) in zip(corners, ids):
                self.get_logger().info(f'Esquinas, {markerCorner}')

                # Calcula la posición y orientación del marcador
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.marker_length, self.k, self.distorsion)

                #self.get_logger().info(f'Marcador detectado, {markerID}')
                # self.get_logger().info(f'Matriz rotación , {rvecs}')
                # Aplicar Rodrigues
                rvecs_matrix, _ = cv2.Rodrigues(rvecs)
                # self.get_logger().info(f'Marcador detectado, {rvecs_matrix}')
                # Devuelve rvecs_matrix un array (3x3) -> Matriz rotacion
                                     #   un array (3x9) -> Derivada de la matriz de rotación respecto al vector de rotación
                                     #                     Esta es una matriz Jacobiana para ver cuánto varia la matriz de rotación 

                # Verifica la matriz de rotación
                if not self.is_valid_rotation_matrix(rvecs_matrix):
                    self.get_logger().error(f'Invalid rotation matrix for marker {markerID}')
                    continue  # Skip this marker

                """# Verifica la matriz de rotación
                if not np.allclose(np.dot(rvecs_matrix, rvecs_matrix.T), np.eye(3), atol=1e-6):
                    self.get_logger().error(f'Invalid rotation matrix for marker {markerID}')   
                    continue  # Skip this marker"""

                #self.get_logger().info(f'Matriz rotación Rodrigues, {rvecs_matrix}')

                # store plane information
                plane = CDetectedPlane()
                plane.id = markerID
                plane.d = (np.linalg.norm(tvecs)/100)
                plane.normal = rvecs_matrix[:, 2]
                plane.normal = rvecs_matrix[:, 2] / np.linalg.norm(rvecs_matrix[:, 2])  # Normalize the normal


                # add it to the observation
                obs.planes.append(plane)

            # debug
            self.get_logger().info(f'obs cam1, {obs}')

            # store observation (and implicitly set ready flag)
            self.observations[0] = obs



    def on_camera2(self, image):
        
        
        # process image
        cv_image = self.br.imgmsg_to_cv2(image)
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Dibujar Marcador, centro y texto
        drawImage = DrawAruco()
        


        # initialize observation
        obs = CArucoObservation()
        obs.cam = 0
        obs.tstamp = image.header.stamp


        if self.observations[1]:
            # self.get_logger().info(f'Detecciones hechas tal vez con la otra camara, {self.observations[1]}')
            return
        
        # Detecta las esquinas e ids de los marcadores
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image_bgr, self.arucoDict, parameters=self.arucoParams)

        # Dibujar ejes
        #cv_image_bgr = cv2.aruco.drawDetectedMarkers(cv_image_bgr, corners, ids)

        cv_image_bgr = drawImage.drawMarker(cv_image_bgr, corners, ids)
        cv_image_bgr = drawImage.drawCenter(cv_image_bgr, corners, ids)
        cv_image_bgr = drawImage.drawAxis(cv_image_bgr, ids, corners, self.k, self.distorsion, self.marker_length)

        cv2.imshow('Imagen 2', cv_image_bgr)
        cv2.waitKey(1)


        
        # Si se detecta algún marcador
        if ids is not None:
            # Si detecta los tres a la vez ids te devuelve: [[0][1][2]]

            ids = ids.flatten()

            # Recorre los marcadores detectados
            # En cada for se cogerá el id con sus esquinas correspondientes
            for (markerCorner, markerID) in zip(corners, ids): 

                # Calcula la posición y orientación del marcador
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.marker_length, self.k, self.distorsion)


                #self.get_logger().info(f'Marcador detectado, {markerID}')

                #Aplicar Rodrigues
                rvecs_matrix, matrix_der = cv2.Rodrigues(rvecs)
                # self.get_logger().info(f'Marcador detectado, {rvecs_matrix}')
                # Deevuelve rvecs_matrix un array (3x3) -> Matriz rotacion
                                     #   un array (3x9) -> Derivada de la matriz de rotación respecto al vector de rotación
                                     #                     Esta es una matriz Jacobiana para ver cuánto varia la matriz de rotación    
                # self.get_logger().info(f'Vector translación, {rvecs_matrix}')

                

                # store plane information
                plane = CDetectedPlane()
                plane.id = markerID
                plane.d = (np.linalg.norm(tvecs)/100)
                plane.normal = rvecs_matrix[:,2] # np.dot(rvecs_matrix, np.array([0, 0, 1]))

                # add it to the observation
                obs.planes.append(plane)


            # debug
            self.get_logger().info(f'obs cam2, {obs}')

            # store observation (and implicitly set ready flag)
            self.observations[1] = obs

        
    def publisher_image(self):
        # polling until two observations are ready
        if self.observations[0] and self.observations[1]:
            #self.get_logger().info(f'Observations2, {abs(self.observations[1].tstamp)}')
            #self.get_logger().info(f'Valor absoluto, {abs(self.observations[0].tstamp - self.observations[1].tstamp)}')
            if abs(self.observations[0].tstamp.sec - self.observations[1].tstamp.sec) < TH_TSTAMPS:
                # build msg to publish
                msg = Correspondences()

                for plane_0 in self.observations[0].planes:
                    for plane_1 in self.observations[1].planes:
                        if plane_0.id == plane_1.id:
                            # PlaneMatch found                         
                            plane_match = PlaneMatch()

                            plane_match.first = Plane()
                            plane_match.first.d = plane_0.d
                            plane_match.first.n.x = plane_0.normal[0]
                            plane_match.first.n.y = plane_0.normal[1]
                            plane_match.first.n.z = plane_0.normal[2]

                            plane_match.second = Plane()
                            plane_match.second.d = plane_1.d
                            plane_match.second.n.x = plane_1.normal[0]
                            plane_match.second.n.y = plane_1.normal[1]
                            plane_match.second.n.z = plane_1.normal[2]
                            

                            msg.correspondences.append(plane_match)
                msg.first_label = '/camera1/color/image_raw'
                msg.second_label = '/camera2/color/image_raw'

                if len(msg.correspondences) > 0:
                    self.publisher.publish(msg)
                
            # reset observations
            self.observations[0] = None
            self.observations[1] = None

    
                      

def main(args=None):
    rclpy.init(args=args)
    detection_aruco_node = DetectionAruco()
    rclpy.spin(detection_aruco_node)
    detection_aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

















import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Vector3
from datetime import datetime, timedelta
from builtin_interfaces.msg import Time
from apriltag_msgs.msg import AprilTagDetectionArray

from custom_msgs.msg import Correspondences, Plane, PlaneMatch

TH_TSTAMPS = 0.01

class CDetectedPlane():
    def __init__(self) -> None:
        self.id = 0                             # id of the detected planes
        self.d = 0.0                            # distance of the plane
        self.normal = np.array([0.0,0.0,0.0])   # normal of the plane

class CArucoObservation():
    def __init__(self) -> None:
        self.cam = -1           # camera id from where it was observed
        self.tstamp = Time()    # timestamp initialization
        self.planes = list()    # list of planes in this observation of class CDetectedPlane
    
    def __str__(self):
        mystr = f"ArucoObservation from cam {self.cam} at {str(self.tstamp)} with planes "
        for plane in self.planes:
            mystr += f"[{plane.id}] : d = {plane.d} | normal = {plane.normal}\n"
        return mystr

class DrawAruco():
    def __init__(self) -> None: 
        self.a = 0

    def drawMarker(self, image, corners, ids):
        cv_image_bgr = cv2.aruco.drawDetectedMarkers(image, corners, ids)

        return cv_image_bgr

    def drawCenter(self, image, corners, ids):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                corners=markerCorner.reshape((4,2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                image = cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

        return image
    
    def drawAxis(self, image, ids, corners, K, distorsion, marker_length):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_length, K, distorsion)
                rvecs_matrix, _ = cv2.Rodrigues(rvecs)
                image = cv2.aruco.drawAxis(image, K, distorsion, rvecs_matrix, tvecs, marker_length)

        return image


    
class DetectionAruco(Node):
    def __init__(self):
        super().__init__('Detection_Aruco')
        self.get_logger().info(f'Iniciado')

        self.br = CvBridge()

        ### Parameters for RGB image processing of ASTRA ORBBEC
        # fx: Focal length of the camera along the x-axis
        self.fx = 517.301
        # fy: Focal length of the camera along the y-axis
        self.fy = 519.291
        # cx: X-coordinate of the principal point/optical center of the camera in pixels
        self.cx = 326.785
        # cy: Y-coordinate of the principal point/optical center of the camera in pixels
        self.cy = 244.563

        self.k = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0, 0, 1]])
        
        self.distorsion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))

        self.dets = []
        """ Parámetros Aruco opencv"""
        self.marker_length = 17.2

        """Parámetros Detector Aruco"""
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        self.observations = [None, None] # cameras one and two

        self.subscription = self.create_subscription(AprilTagDetectionArray, 'detections', self.tag_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera1/color/image_raw', self.on_camera1, 10)
        self.image_sub = self.create_subscription(Image, '/camera2/color/image_raw', self.on_camera2, 10)
        self.publisher = self.create_publisher(Correspondences, '/detection_aruco/topic', 10)

        # timer to send info every 5 secs
        self.timer = self.create_timer(1.0, self.publisher_image)
 
    def tag_callback(self, dets):
		#for d in dets:
        self.dets = dets
			#if not d in self.dets_list:
        if dets.detections:
            msg = "tag_%s\tt(ns)=%s\n" %(str(dets.detections[-1].id), str(self.get_clock().now().nanoseconds))
			# Append-adds at las
            self.get_logger().info(msg)

            # Convert homography list to numpy array
            H_list = dets.detections[-1].homography
            
            H = np.array(H_list).reshape(3, 3)
            self.get_logger().info(f'Homografia, {H}')
            # Normaliza la homografía para que la norma de las dos primeras columnas sea 1

            #norm = np.linalg.norm(np.linalg.inv(self.k).dot(H[:, 0]))
            #H = H / norm

            # Normaliza la homografía
            H_normalized = np.dot(np.linalg.inv(self.k), H)

            # Extrae las dos primeras columnas de la homografía normalizada
            h1 = H_normalized[:, 0]
            h2 = H_normalized[:, 1]

            # Calcula el producto cruzado entre h1 y h2
            r1 = np.cross(h1, h2)
            r1 /= np.linalg.norm(r1)

            # Calcula el producto cruzado entre h1 y r1
            r2 = np.cross(h1, r1)
            r2 /= np.linalg.norm(r2)

            # Calcula el producto cruzado entre r1 y r2 para obtener r3
            r3 = np.cross(r1, r2)

            # Construye la matriz de rotación
            R = np.column_stack((r1, r2, r3))

            # Calcula la tercera columna de la matriz de traslación
            t = H_normalized[:, 2] / np.linalg.norm(r1)

            self.get_logger().info(f'Rotacion calculada de homografia, {R}')
            self.get_logger().info(f'Translacion calculada de homografia, {t}')

            self.get_logger().info(f'Vector z, {r3}')

            # Descomposición de la homografía
            R1 = np.linalg.inv(self.k).dot(H[:, 0])
            R2 = np.linalg.inv(self.k).dot(H[:, 1])
            T = np.linalg.inv(self.k).dot(H[:, 2])

            # Obtén la tercera columna de la matriz de rotación usando el producto cruzado
            R3 = np.cross(R1, R2)

            # Construye la matriz de rotación
            R = np.column_stack((R1, R2, R3))

            # Asegúrate de que R es una matriz de rotación válida usando descomposición SVD
            U, _, Vt = np.linalg.svd(R)
            R = U @ Vt

            # Utilizar Rodrigues 
            rvec, _ = cv2.Rodrigues(R)

            # Asegúrate de que la determinante de R sea 1 (mismo sentido de rotación)
            if np.linalg.det(R) < 0:
                R[:, 2] = -R[:, 2]

            self.tag_array = []

            retval, R, self.t, N = cv2.decomposeHomographyMat(H, self.k)


            # Me quedo con el ultimo valor devuelto de R, T y N
            self.R = R[-1]
            self.t = self.t[-1]
            self.N = N[-1]

            self.get_logger().info(f'Normal, {self.N}')
            self.get_logger().info(f'Rotacion, {self.R}')
            self.get_logger().info(f'Translacion, {self.t}')
            self.get_logger().info(f'trans otra, {T}')

            # Dibujar en markerCorners las detections corners
            markerCorner = [dets.detections[-1].corners]
            # Suponiendo que 'dets' contiene tus detecciones y 'corners_msg' es la lista de puntos de esquinas
            corners_msg = dets.detections[-1].corners

            # Convertir las coordenadas de las esquinas al formato necesario para estimatePoseSingleMarkers
            markerCorner_list = []
            for point_msg in corners_msg:
                corners_np = np.array([[point_msg.x, point_msg.y]], dtype=np.float32)
                markerCorner_list.append(corners_np)

            corners_msg = dets.detections[-1].corners
            markerCorner_list = np.array([[point_msg.x, point_msg.y] for point_msg in corners_msg], dtype=np.float32)

            
            self.get_logger().info(f'ESquinas, {markerCorner}')
            self.get_logger().info(f'ESquinas _list, {markerCorner_list}')

            # Calcular la pose del marcador
            rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner_list, self.marker_length, self.k, self.distorsion)

            
            
            self.get_logger().info(f'Posicion, {tvecs}')
            self.get_logger().info(f'Rotacion, {rvecs}')
            d = (np.linalg.norm(tvecs)/100)

            self.get_logger().info(f'Distancia, {d}')   
            

    def on_camera1(self, image):
        # process image
        cv_image = self.br.imgmsg_to_cv2(image)
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  

        
        cv2.imshow('Imagen 1', cv_image_bgr)
        cv2.waitKey(1)
        

        """# store plane information
        plane = CDetectedPlane()
        plane.id = markerID
        plane.d = np.linalg.norm(tvecs)
        plane.normal = rvecs_matrix[:, 2]

        # add it to the observation
        obs.planes.append(plane)

        # debug
        self.get_logger().info(f'obs cam1, {obs}')

        # store observation (and implicitly set ready flag)
        self.observations[0] = obs"""



    def on_camera2(self, image):
        
        a = 0

        
    def publisher_image(self):
        # polling until two observations are ready
        if self.observations[0] and self.observations[1]:
            #self.get_logger().info(f'Observations2, {abs(self.observations[1].tstamp)}')
            #self.get_logger().info(f'Valor absoluto, {abs(self.observations[0].tstamp - self.observations[1].tstamp)}')
            if abs(self.observations[0].tstamp.sec - self.observations[1].tstamp.sec) < TH_TSTAMPS:
                # build msg to publish
                msg = Correspondences()

                for plane_0 in self.observations[0].planes:
                    for plane_1 in self.observations[1].planes:
                        if plane_0.id == plane_1.id:
                            # PlaneMatch found                         
                            plane_match = PlaneMatch()

                            plane_match.first = Plane()
                            plane_match.first.d = plane_0.d
                            plane_match.first.n.x = plane_0.normal[0]
                            plane_match.first.n.y = plane_0.normal[1]
                            plane_match.first.n.z = plane_0.normal[2]

                            plane_match.second = Plane()
                            plane_match.second.d = plane_1.d
                            plane_match.second.n.x = plane_1.normal[0]
                            plane_match.second.n.y = plane_1.normal[1]
                            plane_match.second.n.z = plane_1.normal[2]
                            

                            msg.correspondences.append(plane_match)
                msg.first_label = '/camera1/color/image_raw'
                msg.second_label = '/camera2/color/image_raw'

                if len(msg.correspondences) > 0:
                    self.publisher.publish(msg)
                
            # reset observations
            self.observations[0] = None
            self.observations[1] = None
                      

def main(args=None):
    rclpy.init(args=args)
    detection_aruco_node = DetectionAruco()
    rclpy.spin(detection_aruco_node)
    detection_aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






















import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Vector3
from datetime import datetime, timedelta
from builtin_interfaces.msg import Time
from apriltag_msgs.msg import AprilTagDetectionArray

from custom_msgs.msg import Correspondences, Plane, PlaneMatch

TH_TSTAMPS = 0.01

class CDetectedPlane():
    def __init__(self) -> None:
        self.id = 0                             # id of the detected planes
        self.d = 0.0                            # distance of the plane
        self.normal = np.array([0.0,0.0,0.0])   # normal of the plane

class CArucoObservation():
    def __init__(self) -> None:
        self.cam = -1           # camera id from where it was observed
        self.tstamp = Time()    # timestamp initialization
        self.planes = list()    # list of planes in this observation of class CDetectedPlane
    
    def __str__(self):
        mystr = f"ArucoObservation from cam {self.cam} at {str(self.tstamp)} with planes "
        for plane in self.planes:
            mystr += f"[{plane.id}] : d = {plane.d} | normal = {plane.normal}\n"
        return mystr

class DrawAruco():
    def __init__(self) -> None: 
        self.a = 0

    def drawMarker(self, image, corners, ids):
        cv_image_bgr = cv2.aruco.drawDetectedMarkers(image, corners, ids)

        return cv_image_bgr

    def drawCenter(self, image, corners, ids):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                corners=markerCorner.reshape((4,2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                image = cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

        return image
    
    def drawAxis(self, image, ids, corners, K, distorsion, marker_length):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_length, K, distorsion)
                rvecs_matrix, _ = cv2.Rodrigues(rvecs)
                image = cv2.aruco.drawAxis(image, K, distorsion, rvecs_matrix, tvecs, marker_length)

        return image


    
class DetectionAruco(Node):
    def __init__(self):
        super().__init__('Detection_Aruco')
        self.get_logger().info(f'Iniciado')

        self.br = CvBridge()

        ### Parameters for RGB image processing of ASTRA ORBBEC
        # fx: Focal length of the camera along the x-axis
        self.fx = 517.301
        # fy: Focal length of the camera along the y-axis
        self.fy = 519.291
        # cx: X-coordinate of the principal point/optical center of the camera in pixels
        self.cx = 326.785
        # cy: Y-coordinate of the principal point/optical center of the camera in pixels
        self.cy = 244.563

        self.k = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0, 0, 1]])
        
        self.distorsion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))

        self.dets = []
        """ Parámetros Aruco opencv"""
        self.marker_length = 17.2

        """Parámetros Detector Aruco"""
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        self.observations = [None, None] # cameras one and two

        self.subscription = self.create_subscription(AprilTagDetectionArray, 'detections', self.tag_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera1/color/image_raw', self.on_camera1, 10)
        self.image_sub = self.create_subscription(Image, '/camera2/color/image_raw', self.on_camera2, 10)
        self.publisher = self.create_publisher(Correspondences, '/detection_aruco/topic', 10)

        # timer to send info every 5 secs
        self.timer = self.create_timer(1.0, self.publisher_image)
 
    def tag_callback(self, dets):
		#for d in dets:
        self.dets = dets
			#if not d in self.dets_list:
        if dets.detections:
            msg = "tag_%s\tt(ns)=%s\n" %(str(dets.detections[-1].id), str(self.get_clock().now().nanoseconds))
			# Append-adds at las
            self.get_logger().info(msg)

            # Convert homography list to numpy array
            H_list = dets.detections[-1].homography
            
            H = np.array(H_list).reshape(3, 3)
            self.get_logger().info(f'Homografia, {H}')
            # Normaliza la homografía para que la norma de las dos primeras columnas sea 1

            #norm = np.linalg.norm(np.linalg.inv(self.k).dot(H[:, 0]))
            #H = H / norm

            # Normaliza la homografía
            H_normalized = np.dot(np.linalg.inv(self.k), H)

            # Extrae las dos primeras columnas de la homografía normalizada
            h1 = H_normalized[:, 0]
            h2 = H_normalized[:, 1]

            # Calcula el producto cruzado entre h1 y h2
            r1 = np.cross(h1, h2)
            r1 /= np.linalg.norm(r1)

            # Calcula el producto cruzado entre h1 y r1
            r2 = np.cross(h1, r1)
            r2 /= np.linalg.norm(r2)

            # Calcula el producto cruzado entre r1 y r2 para obtener r3
            r3 = np.cross(r1, r2)

            # Construye la matriz de rotación
            R = np.column_stack((r1, r2, r3))

            # Calcula la tercera columna de la matriz de traslación
            t = H_normalized[:, 2] / np.linalg.norm(r1)

            self.get_logger().info(f'Rotacion calculada de homografia, {R}')
            self.get_logger().info(f'Translacion calculada de homografia, {t}')

            self.get_logger().info(f'Vector z, {r3}')

            # Descomposición de la homografía
            R1 = np.linalg.inv(self.k).dot(H[:, 0])
            R2 = np.linalg.inv(self.k).dot(H[:, 1])
            T = np.linalg.inv(self.k).dot(H[:, 2])

            # Obtén la tercera columna de la matriz de rotación usando el producto cruzado
            R3 = np.cross(R1, R2)

            # Construye la matriz de rotación
            R = np.column_stack((R1, R2, R3))

            # Asegúrate de que R es una matriz de rotación válida usando descomposición SVD
            U, _, Vt = np.linalg.svd(R)
            R = U @ Vt

            # Utilizar Rodrigues 
            rvec, _ = cv2.Rodrigues(R)

            # Asegúrate de que la determinante de R sea 1 (mismo sentido de rotación)
            if np.linalg.det(R) < 0:
                R[:, 2] = -R[:, 2]

            self.tag_array = []

            retval, R, self.t, N = cv2.decomposeHomographyMat(H, self.k)


            # Me quedo con el ultimo valor devuelto de R, T y N
            self.R = R[-1]
            self.t = self.t[-1]
            self.N = N[-1]

            self.get_logger().info(f'Normal, {self.N}')
            self.get_logger().info(f'Rotacion, {self.R}')
            self.get_logger().info(f'Translacion, {self.t}')
            self.get_logger().info(f'trans otra, {T}')

            # Dibujar en markerCorners las detections corners
            markerCorner = [dets.detections[-1].corners]
            # Suponiendo que 'dets' contiene tus detecciones y 'corners_msg' es la lista de puntos de esquinas
            corners_msg = dets.detections[-1].corners

            # Convertir las coordenadas de las esquinas al formato necesario para estimatePoseSingleMarkers
            markerCorner_list = []
            for point_msg in corners_msg:
                corners_np = np.array([[point_msg.x, point_msg.y]], dtype=np.float32)
                markerCorner_list.append(corners_np)

            corners_msg = dets.detections[-1].corners
            markerCorner_list = np.array([[point_msg.x, point_msg.y] for point_msg in corners_msg], dtype=np.float32)

            
            self.get_logger().info(f'ESquinas, {markerCorner}')
            self.get_logger().info(f'ESquinas _list, {markerCorner_list}')

            # Calcular la pose del marcador
            rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner_list, self.marker_length, self.k, self.distorsion)

            
            
            self.get_logger().info(f'Posicion, {tvecs}')
            self.get_logger().info(f'Rotacion, {rvecs}')
            d = (np.linalg.norm(tvecs)/100)

            self.get_logger().info(f'Distancia, {d}')   
            

    def on_camera1(self, image):
        # process image
        cv_image = self.br.imgmsg_to_cv2(image)
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  

        
        cv2.imshow('Imagen 1', cv_image_bgr)
        cv2.waitKey(1)
        

        """# store plane information
        plane = CDetectedPlane()
        plane.id = markerID
        plane.d = np.linalg.norm(tvecs)
        plane.normal = rvecs_matrix[:, 2]

        # add it to the observation
        obs.planes.append(plane)

        # debug
        self.get_logger().info(f'obs cam1, {obs}')

        # store observation (and implicitly set ready flag)
        self.observations[0] = obs"""



    def on_camera2(self, image):
        
        a = 0

        
    def publisher_image(self):
        # polling until two observations are ready
        if self.observations[0] and self.observations[1]:
            #self.get_logger().info(f'Observations2, {abs(self.observations[1].tstamp)}')
            #self.get_logger().info(f'Valor absoluto, {abs(self.observations[0].tstamp - self.observations[1].tstamp)}')
            if abs(self.observations[0].tstamp.sec - self.observations[1].tstamp.sec) < TH_TSTAMPS:
                # build msg to publish
                msg = Correspondences()

                for plane_0 in self.observations[0].planes:
                    for plane_1 in self.observations[1].planes:
                        if plane_0.id == plane_1.id:
                            # PlaneMatch found                         
                            plane_match = PlaneMatch()

                            plane_match.first = Plane()
                            plane_match.first.d = plane_0.d
                            plane_match.first.n.x = plane_0.normal[0]
                            plane_match.first.n.y = plane_0.normal[1]
                            plane_match.first.n.z = plane_0.normal[2]

                            plane_match.second = Plane()
                            plane_match.second.d = plane_1.d
                            plane_match.second.n.x = plane_1.normal[0]
                            plane_match.second.n.y = plane_1.normal[1]
                            plane_match.second.n.z = plane_1.normal[2]
                            

                            msg.correspondences.append(plane_match)
                msg.first_label = '/camera1/color/image_raw'
                msg.second_label = '/camera2/color/image_raw'

                if len(msg.correspondences) > 0:
                    self.publisher.publish(msg)
                
            # reset observations
            self.observations[0] = None
            self.observations[1] = None
                      

def main(args=None):
    rclpy.init(args=args)
    detection_aruco_node = DetectionAruco()
    rclpy.spin(detection_aruco_node)
    detection_aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()












"""""""CODIGO BIEN APRILTAGGGS"""""""




import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Vector3
from datetime import datetime, timedelta
from builtin_interfaces.msg import Time
from apriltag_msgs.msg import AprilTagDetectionArray

from custom_msgs.msg import Correspondences, Plane, PlaneMatch

TH_TSTAMPS = 0.01

class CDetectedPlane():
    def __init__(self) -> None:
        self.id = 0                             # id of the detected planes
        self.d = 0.0                            # distance of the plane
        self.normal = np.array([0.0,0.0,0.0])   # normal of the plane

class CArucoObservation():
    def __init__(self) -> None:
        self.cam = -1           # camera id from where it was observed
        self.tstamp = Time()    # timestamp initialization
        self.planes = list()    # list of planes in this observation of class CDetectedPlane
    
    def __str__(self):
        mystr = f"ArucoObservation from cam {self.cam} at {str(self.tstamp)} with planes "
        for plane in self.planes:
            mystr += f"[{plane.id}] : d = {plane.d} | normal = {plane.normal}\n"
        return mystr

class DrawAruco():
    def __init__(self) -> None: 
        self.a = 0

    def drawMarker(self, image, corners, ids):
        cv_image_bgr = cv2.aruco.drawDetectedMarkers(image, corners, ids)

        return cv_image_bgr

    def drawCenter(self, image, corners, ids):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                corners=markerCorner.reshape((4,2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                image = cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

        return image
    
    def drawAxis(self, image, ids, corners, K, distorsion, marker_length):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_length, K, distorsion)
                rvecs_matrix, _ = cv2.Rodrigues(rvecs)
                image = cv2.aruco.drawAxis(image, K, distorsion, rvecs_matrix, tvecs, marker_length)

        return image


    
class DetectionAruco(Node):
    def __init__(self):
        super().__init__('Detection_Aruco')
        self.get_logger().info(f'Iniciado')

        self.br = CvBridge()

        ### Parameters for RGB image processing of ASTRA ORBBEC
        # fx: Focal length of the camera along the x-axis
        self.fx = 517.301
        # fy: Focal length of the camera along the y-axis
        self.fy = 519.291
        # cx: X-coordinate of the principal point/optical center of the camera in pixels
        self.cx = 326.785
        # cy: Y-coordinate of the principal point/optical center of the camera in pixels
        self.cy = 244.563

        self.k = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0, 0, 1]])
        
        self.distorsion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))

        self.dets = []
        """ Parámetros Aruco opencv"""
        self.marker_length = 17.2

        """Parámetros Detector Aruco"""
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        self.observations = [None, None] # cameras one and two

        self.subscription = self.create_subscription(AprilTagDetectionArray, 'detections', self.tag_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera1/color/image_raw', self.on_camera1, 10)
        self.image_sub = self.create_subscription(Image, '/camera2/color/image_raw', self.on_camera2, 10)
        self.publisher = self.create_publisher(Correspondences, '/detection_aruco/topic', 10)

        # timer to send info every 5 secs
        self.timer = self.create_timer(1.0, self.publisher_image)
 
    def tag_callback(self, dets):
		#for d in dets:
        self.dets = dets
			#if not d in self.dets_list:
        if dets.detections:
            msg = "tag_%s\tt(ns)=%s\n" %(str(dets.detections[-1].id), str(self.get_clock().now().nanoseconds))
			# Append-adds at las
            self.get_logger().info(msg)

            # Convert homography list to numpy array
            H_list = dets.detections[-1].homography
            
            H = np.array(H_list).reshape(3, 3)
            self.get_logger().info(f'Homografia, {H}')
            # Normaliza la homografía para que la norma de las dos primeras columnas sea 1

            #norm = np.linalg.norm(np.linalg.inv(self.k).dot(H[:, 0]))
            #H = H / norm

            # Normaliza la homografía
            H_normalized = np.dot(np.linalg.inv(self.k), H)

            # Extrae las dos primeras columnas de la homografía normalizada
            h1 = H_normalized[:, 0]
            h2 = H_normalized[:, 1]

            # Calcula el producto cruzado entre h1 y h2
            r1 = np.cross(h1, h2)
            r1 /= np.linalg.norm(r1)

            # Calcula el producto cruzado entre h1 y r1
            r2 = np.cross(h1, r1)
            r2 /= np.linalg.norm(r2)

            # Calcula el producto cruzado entre r1 y r2 para obtener r3
            r3 = np.cross(r1, r2)

            # Construye la matriz de rotación
            R = np.column_stack((r1, r2, r3))

            # Calcula la tercera columna de la matriz de traslación
            t = H_normalized[:, 2] / np.linalg.norm(r1)

            self.get_logger().info(f'Rotacion calculada de homografia, {R}')
            self.get_logger().info(f'Translacion calculada de homografia, {t}')

            self.get_logger().info(f'Vector z, {r3}')

            # Descomposición de la homografía
            R1 = np.linalg.inv(self.k).dot(H[:, 0])
            R2 = np.linalg.inv(self.k).dot(H[:, 1])
            T = np.linalg.inv(self.k).dot(H[:, 2])

            # Obtén la tercera columna de la matriz de rotación usando el producto cruzado
            R3 = np.cross(R1, R2)

            # Construye la matriz de rotación
            R = np.column_stack((R1, R2, R3))

            # Asegúrate de que R es una matriz de rotación válida usando descomposición SVD
            U, _, Vt = np.linalg.svd(R)
            R = U @ Vt

            # Utilizar Rodrigues 
            rvec, _ = cv2.Rodrigues(R)

            # Asegúrate de que la determinante de R sea 1 (mismo sentido de rotación)
            if np.linalg.det(R) < 0:
                R[:, 2] = -R[:, 2]

            self.tag_array = []

            retval, R, self.t, N = cv2.decomposeHomographyMat(H, self.k)


            # Me quedo con el ultimo valor devuelto de R, T y N
            self.R = R[-1]
            self.t = self.t[-1]
            self.N = N[-1]

            self.get_logger().info(f'Normal, {self.N}')
            self.get_logger().info(f'Rotacion, {self.R}')
            self.get_logger().info(f'Translacion, {self.t}')
            self.get_logger().info(f'trans otra, {T}')

            # Dibujar en markerCorners las detections corners
            markerCorner = [dets.detections[-1].corners]
            # Suponiendo que 'dets' contiene tus detecciones y 'corners_msg' es la lista de puntos de esquinas
            corners_msg = dets.detections[-1].corners

            # Convertir las coordenadas de las esquinas al formato necesario para estimatePoseSingleMarkers
            markerCorner_list = []
            for point_msg in corners_msg:
                corners_np = np.array([[point_msg.x, point_msg.y]], dtype=np.float32)
                markerCorner_list.append(corners_np)

            corners_msg = dets.detections[-1].corners
            markerCorner_list = np.array([[point_msg.x, point_msg.y] for point_msg in corners_msg], dtype=np.float32)
            markerCorner_list = np.array(markerCorner_list).reshape((1, 4, 2))
            
            self.get_logger().info(f'ESquinas, {markerCorner}')
            self.get_logger().info(f'ESquinas _list, {markerCorner_list}')

            # Calcular la pose del marcador
            rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner_list, self.marker_length, self.k, self.distorsion)

            d = (np.linalg.norm(tvecs)/100)

            self.get_logger().info(f'Distancia, {d}')   
            

    def on_camera1(self, image):
        # process image
        cv_image = self.br.imgmsg_to_cv2(image)
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  

        
        cv2.imshow('Imagen 1', cv_image_bgr)
        cv2.waitKey(1)
        

        """# store plane information
        plane = CDetectedPlane()
        plane.id = markerID
        plane.d = np.linalg.norm(tvecs)
        plane.normal = rvecs_matrix[:, 2]

        # add it to the observation
        obs.planes.append(plane)

        # debug
        self.get_logger().info(f'obs cam1, {obs}')

        # store observation (and implicitly set ready flag)
        self.observations[0] = obs"""



    def on_camera2(self, image):
        
        a = 0

        
    def publisher_image(self):
        # polling until two observations are ready
        if self.observations[0] and self.observations[1]:
            #self.get_logger().info(f'Observations2, {abs(self.observations[1].tstamp)}')
            #self.get_logger().info(f'Valor absoluto, {abs(self.observations[0].tstamp - self.observations[1].tstamp)}')
            if abs(self.observations[0].tstamp.sec - self.observations[1].tstamp.sec) < TH_TSTAMPS:
                # build msg to publish
                msg = Correspondences()

                for plane_0 in self.observations[0].planes:
                    for plane_1 in self.observations[1].planes:
                        if plane_0.id == plane_1.id:
                            # PlaneMatch found                         
                            plane_match = PlaneMatch()

                            plane_match.first = Plane()
                            plane_match.first.d = plane_0.d
                            plane_match.first.n.x = plane_0.normal[0]
                            plane_match.first.n.y = plane_0.normal[1]
                            plane_match.first.n.z = plane_0.normal[2]

                            plane_match.second = Plane()
                            plane_match.second.d = plane_1.d
                            plane_match.second.n.x = plane_1.normal[0]
                            plane_match.second.n.y = plane_1.normal[1]
                            plane_match.second.n.z = plane_1.normal[2]
                            

                            msg.correspondences.append(plane_match)
                msg.first_label = '/camera1/color/image_raw'
                msg.second_label = '/camera2/color/image_raw'

                if len(msg.correspondences) > 0:
                    self.publisher.publish(msg)
                
            # reset observations
            self.observations[0] = None
            self.observations[1] = None
                      

def main(args=None):
    rclpy.init(args=args)
    detection_aruco_node = DetectionAruco()
    rclpy.spin(detection_aruco_node)
    detection_aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()








#################### APRILTAG ULTIMA VERSION #############################################  
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################


import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Vector3
from datetime import datetime, timedelta
from builtin_interfaces.msg import Time
from apriltag_msgs.msg import AprilTagDetectionArray

from custom_msgs.msg import Correspondences, Plane, PlaneMatch

TH_TSTAMPS = 0.01

class CDetectedPlane():
    def __init__(self) -> None:
        self.id = 0                             # id of the detected planes
        self.d = 0.0                            # distance of the plane
        self.normal = np.array([0.0, 0.0, 0.0]) # normal of the plane

class CArucoObservation():
    def __init__(self) -> None:
        self.cam = -1           # camera id from where it was observed
        self.tstamp = Time()    # timestamp initialization
        self.planes = list()    # list of planes in this observation of class CDetectedPlane
    
    def __str__(self):
        mystr = f"ArucoObservation from cam {self.cam} at {str(self.tstamp)} with planes "
        for plane in self.planes:
            mystr += f"[{plane.id}] : d = {plane.d} | normal = {plane.normal}\n"
        return mystr

class DrawAruco():
    def __init__(self) -> None: 
        self.a = 0

    def drawMarker(self, image, corners, ids):
        cv_image_bgr = cv2.aruco.drawDetectedMarkers(image, corners, ids)
        return cv_image_bgr

    def drawCenter(self, image, corners, ids):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                image = cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
        return image
    
    def drawAxis(self, image, ids, corners, K, distorsion, marker_length):
        if ids is not None: 
            id = ids.flatten()
            for (markerCorner, markerID) in zip(corners, id):
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_length, K, distorsion)
                rvecs_matrix, _ = cv2.Rodrigues(rvecs)
                image = cv2.aruco.drawAxis(image, K, distorsion, rvecs_matrix, tvecs, marker_length)
        return image

class DetectionAruco(Node):
    def __init__(self):
        super().__init__('Detection_Aruco')
        self.get_logger().info(f'Iniciado')
        
        self.br = CvBridge()

        ### Parameters for RGB image processing of ASTRA ORBBEC
        self.fx = 517.301
        self.fy = 519.291
        self.cx = 326.785
        self.cy = 244.563
        self.k = np.array([[self.fx, 0, self.cx],
                           [0, self.fy, self.cy],
                           [0, 0, 1]])
        self.distorsion = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))

        self.dets1 = []
        self.dets2 = []
        if len(self.dets1) != 0:
            self.get_logger().info(f'Todo regulinchi ,{self.dets1}')
        self.marker_length = 17.2

        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        self.observations = [None, None] # cameras one and two

        self.subscription1 = self.create_subscription(AprilTagDetectionArray, 'camera1/detections', self.tag_callback1, 10)
        self.subscription2 = self.create_subscription(AprilTagDetectionArray, 'camera2/detections', self.tag_callback2, 10)
        self.image_sub1 = self.create_subscription(Image, '/camera1/color/image_raw', self.on_camera1, 10)
        self.image_sub2 = self.create_subscription(Image, '/camera2/color/image_raw', self.on_camera2, 10)
        self.publisher = self.create_publisher(Correspondences, '/custom_msgs/msg/correspondences', 10)

        self.timer = self.create_timer(1.0, self.publisher_image)
 
    def tag_callback1(self, dets):
        self.dets1 = dets
        self.process_observations()

    def tag_callback2(self, dets):
        self.dets2 = dets
        self.process_observations()

    def process_observations(self):
        if self.dets1 and self.dets2:
            time_diff = abs(self.dets1.header.stamp.sec - self.dets2.header.stamp.sec) + abs(self.dets1.header.stamp.nanosec - self.dets2.header.stamp.nanosec) / 1e9
            if time_diff <= TH_TSTAMPS:
                self.publisher_image()

    def on_camera1(self, image):
        #self.get_logger().info('Received image from camera 1')
        cv_image = self.br.imgmsg_to_cv2(image)
        if self.dets1.detections:
            for detection in self.dets1.detections:
                H_list = detection.homography
                H = np.array(H_list).reshape(3, 3)

                retval, R, t, N = cv2.decomposeHomographyMat(H, self.k)
                self.R = R[-1]
                self.t = t[-1]
                self.N = N[-1]

                corners_msg = detection.corners
                markerCorner_list = np.array([[point_msg.x, point_msg.y] for point_msg in corners_msg], dtype=np.float32).reshape((1, 4, 2))

                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner_list, self.marker_length, self.k, self.distorsion)
                d = (np.linalg.norm(tvecs) / 100)

                plane = CDetectedPlane()
                plane.id = detection.id 
                plane.d = d
                plane.normal = self.N

                obs = CArucoObservation()
                obs.cam = 1
                obs.tstamp = self.get_clock().now().to_msg()
                obs.planes.append(plane)
                self.get_logger().info(f'obs cam 1, {obs}')

                self.observations[0] = obs

    def on_camera2(self, image):
        #self.get_logger().info('Received image from camera 2')
        cv_image = self.br.imgmsg_to_cv2(image)
        if self.dets2.detections:
            for detection in self.dets2.detections:
                H_list = detection.homography
                H = np.array(H_list).reshape(3, 3)

                retval, R, t, N = cv2.decomposeHomographyMat(H, self.k)
                self.R = R[-1]
                self.t = t[-1]
                self.N = N[-1]

                corners_msg = detection.corners
                markerCorner_list = np.array([[point_msg.x, point_msg.y] for point_msg in corners_msg], dtype=np.float32).reshape((1, 4, 2))

                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner_list, self.marker_length, self.k, self.distorsion)
                d = (np.linalg.norm(tvecs) / 100)

                plane = CDetectedPlane()
                plane.id = detection.id 
                plane.d = d
                plane.normal = self.N

                obs = CArucoObservation()
                obs.cam = 2
                obs.tstamp = self.get_clock().now().to_msg()
                obs.planes.append(plane)
                self.get_logger().info(f'obs cam 2, {obs}')

                self.observations[1] = obs

    def publisher_image(self):
        if self.observations[0] and self.observations[1]:
            if abs(self.observations[0].tstamp.sec - self.observations[1].tstamp.sec) <= TH_TSTAMPS:
                self.get_logger().info(f'Detected planes from both cameras')
                # build msg to publish
                msg = Correspondences()

                for plane_0 in self.observations[0].planes:
                    for plane_1 in self.observations[1].planes:
                        if plane_0.id == plane_1.id:
                            # PlaneMatch found                         
                            plane_match = PlaneMatch()

                            plane_match.first = Plane()
                            plane_match.first.d = plane_0.d
                            plane_match.first.n.x = float(plane_0.normal[0])
                            plane_match.first.n.y = float(plane_0.normal[1])
                            plane_match.first.n.z = float(plane_0.normal[2])

                            plane_match.second = Plane()
                            plane_match.second.d = plane_1.d
                            plane_match.second.n.x = float(plane_1.normal[0])
                            plane_match.second.n.y = float(plane_1.normal[1])
                            plane_match.second.n.z = float(plane_1.normal[2])
                            

                            msg.correspondences.append(plane_match)
                msg.first_label = '/camera1/color/image_raw'
                msg.second_label = '/camera2/color/image_raw'

                if len(msg.correspondences) > 0:
                    self.publisher.publish(msg)
                
            # reset observations
            self.observations[0] = None
            self.observations[1] = None
                      

def main(args=None):
    rclpy.init(args=args)
    detection_aruco = DetectionAruco()
    rclpy.spin(detection_aruco)
    detection_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()