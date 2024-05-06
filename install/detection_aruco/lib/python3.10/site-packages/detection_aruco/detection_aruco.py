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

from datos_camara.msg import Num
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
        """        axis = np.float32([[0,0,0], [0,marker_length,0], [marker_length,marker_length,0], [marker_length,0,0],
                                [0,0,-marker_length],[0,marker_length,-marker_length],[marker_length,marker_length,-marker_length],[marker_length,0,-marker_length] ]).reshape(-1,3)
            axisImgpts, _ = cv2.projectPoints(axis, R, t, k, distorsion)
            axisImgpts = tuple(map(tuple, axisImgpts.reshape(8,2)))
            # Draw axis lines
            image = cv2.line(image, axisImgpts[0], axisImgpts[1], (255,0,0), 5)
            image = cv2.line(image, axisImgpts[0], axisImgpts[2], (0,255,0), 5)
            image = cv2.line(image, axisImgpts[0], axisImgpts[3], (0,0,255), 5)
            return image
        """

    
class DetectionAruco(Node):
    def __init__(self):
        super().__init__('apriltag_subscriber')
        self.get_logger().info(f'Iniciado')

        self.br = CvBridge()

        # Parámetros camara usb
        # self.K = np.array(((676.699523, 0., 283.804190),
        #                    (0., 676.358437, 247.200716),
        #                    (0., 0., 1.)))
        # self.distorsion = np.array((0.019135, -0.44869, 0.000693, -0.014817, 0.0))

        # Parámetros cámara nueva
        self.K = np.array([[613.957058, 0., 323.62420],
                            [0., 618.575469, 216.459925],
                            [0., 0., 1.]])
        self.distorsion = np.array([-0.083010, 0.161326, -0.007532, 0.000727, 0.0])
        
        self.R = np.eye(3)
        self.t = np.zeros((3, 1))

        """ Parámetros Aruco"""
        self.marker_length = 17.2

        """Parámetros Detector Aruco"""
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        self.observations = [None, None] # cameras one and two

        self.image_sub = self.create_subscription(Image, '/usb_cam_0/image_raw', self.on_camera1, 10)
        self.image_sub = self.create_subscription(Image, '/usb_cam_1/image_raw', self.on_camera2, 10)
        self.publisher = self.create_publisher(Correspondences, '/detection_aruco/topic', 10)

        # timer to send info every 5 secs
        self.timer = self.create_timer(1.0, self.publisher_image)


    def on_camera1(self, image):
        self.get_logger().info(f'Camara 1 - prueba funciona bien')

        # process image
        cv_image = self.br.imgmsg_to_cv2(image)
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_YUV2BGR_YUYV)

        # initialize observation
        obs = CArucoObservation()
        obs.cam = 0
        obs.tstamp = image.header.stamp

        drawImage = DrawAruco()

        # Detecta las esquinas e ids de los marcadores
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image_bgr, self.arucoDict, parameters=self.arucoParams)

        if self.observations[0]:
            self.get_logger().info(f'Detecciones hechas tal vez esteeeeee, {self.observations[0]}')
            return

        cv_image_bgr = drawImage.drawMarker(cv_image_bgr, corners, ids)
        cv_image_bgr = drawImage.drawCenter(cv_image_bgr, corners, ids)
        cv_image_bgr = drawImage.drawAxis(cv_image_bgr, ids, corners, self.K, self.distorsion, self.marker_length)

        cv2.imshow('Imagen 1', cv_image_bgr)
        cv2.waitKey(1)



        # Si se detecta algún marcador
        if ids is not None:
            # Si detecta los tres a la vez ids te devuelve: [[0][1][2]]
            ids = ids.flatten()

            # Recorre los marcadores detectados
            # En cada for se cogerá el id con sus esquinas correspondientes
            for (markerCorner, markerID) in zip(corners, ids):

                # Calcula la posición y orientación del marcador
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.marker_length, self.K, self.distorsion)

                self.get_logger().info(f'Marcador detectado, {markerID}')

                # Aplicar Rodrigues
                rvecs_matrix, _ = cv2.Rodrigues(rvecs)
                # self.get_logger().info(f'Marcador detectado, {rvecs_matrix}')
                # Devuelve rvecs_matrix un array (3x3) -> Matriz rotacion
                                     #   un array (3x9) -> Derivada de la matriz de rotación respecto al vector de rotación
                                     #                     Esta es una matriz Jacobiana para ver cuánto varia la matriz de rotación    


                # store plane information
                plane = CDetectedPlane()
                plane.id = markerID
                plane.d = np.linalg.norm(tvecs)
                plane.normal = rvecs_matrix[:, 2]

                # add it to the observation
                obs.planes.append(plane)

            # debug
            self.get_logger().info(f'obs, {obs}')

            # store observation (and implicitly set ready flag)
            self.observations[0] = obs

        # Mostrar la imagen

   


    def on_camera2(self, image):
        
        self.get_logger().info(f'Camara 1 - a ver si funciona')
        # check if we already have an observation
        
        # process image
        cv_image = self.br.imgmsg_to_cv2(image)
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_YUV2BGR_YUYV)

        # Dibujar Marcador, centro y texto
        drawImage = DrawAruco()
        


        # initialize observation
        obs = CArucoObservation()
        obs.cam = 0
        obs.tstamp = image.header.stamp


        if self.observations[1]:
            self.get_logger().info(f'Detecciones hechas tal vez con la otra camara, {self.observations[1]}')
            return
        
        # Detecta las esquinas e ids de los marcadores
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image_bgr, self.arucoDict, parameters=self.arucoParams)

        # Dibujar ejes
        #cv_image_bgr = cv2.aruco.drawDetectedMarkers(cv_image_bgr, corners, ids)

        cv_image_bgr = drawImage.drawMarker(cv_image_bgr, corners, ids)
        cv_image_bgr = drawImage.drawCenter(cv_image_bgr, corners, ids)
        cv_image_bgr = drawImage.drawAxis(cv_image_bgr, ids, corners, self.K, self.distorsion, self.marker_length)

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
                rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.marker_length, self.K, self.distorsion)


                self.get_logger().info(f'Marcador detectado, {markerID}')

                #Aplicar Rodrigues
                rvecs_matrix, _ = cv2.Rodrigues(rvecs)
                # self.get_logger().info(f'Marcador detectado, {rvecs_matrix}')
                # Deevuelve rvecs_matrix un array (3x3) -> Matriz rotacion
                                     #   un array (3x9) -> Derivada de la matriz de rotación respecto al vector de rotación
                                     #                     Esta es una matriz Jacobiana para ver cuánto varia la matriz de rotación    
                self.get_logger().info(f'Vector translación, {rvecs_matrix}')


                # store plane information
                plane = CDetectedPlane()
                plane.id = markerID
                plane.d = np.linalg.norm(tvecs)
                plane.normal = rvecs_matrix[:,2] # np.dot(rvecs_matrix, np.array([0, 0, 1]))

                # add it to the observation
                obs.planes.append(plane)


            # debug
            self.get_logger().info(f'obs, {obs}')

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