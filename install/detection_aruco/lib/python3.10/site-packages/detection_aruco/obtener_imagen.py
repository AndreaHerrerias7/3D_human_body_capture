import cv2
import numpy as np

# Definir parámetros de la cámara
K = np.array([[613.957058, 0., 323.62420],
              [0., 618.575469, 216.459925],
              [0., 0., 1.]])
distorsion = np.array([-0.083010, 0.161326, -0.007532, 0.000727, 0.0])

# Obtener el diccionario de marcadores ArUco
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
arucoParams = cv2.aruco.DetectorParameters_create()

# Inicializar la cámara
cap = cv2.VideoCapture(0)

while True:
    # Capturar imagen de la cámara
    ret, frame = cap.read()
    
    if ret:
        # Convertir la imagen a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detectar marcadores ArUco
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
        
        if ids is not None:
            # Si se detectan marcadores
            for i in range(len(ids)):
                # Calcular la pose de cada marcador
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 17.2, K, distorsion)
                
                # Mostrar resultados
                print(f"Marcador ID: {ids[i]}")
                print(f"rvecs (Matriz de Rotación):\n{rvecs}")
                print(f"tvecs (Vector de Traslación):\n{tvecs}\n")
                
                # Dibujar ejes 3D en el marcador
                cv2.aruco.drawAxis(frame, K, distorsion, rvecs, tvecs, 10)
                
        # Mostrar la imagen con los marcadores detectados y los ejes 3D
        cv2.imshow('Frame', frame)
    
    # Salir si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la cámara y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()
