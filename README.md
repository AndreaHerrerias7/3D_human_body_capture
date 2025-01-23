# Proyecto ROS2 - Calibración y Detección de Personas

Este proyecto tiene como objetivo realizar la calibración de cámaras y la detección de articulaciones del cuerpo humano en un entorno tridimensional utilizando ROS2, varios paquetes de cámaras, y la librería MediaPipe. El flujo de trabajo se divide en varias fases, que incluyen la calibración intrínseca, la calibración extrínseca y la reconstrucción del cuerpo utilizando múltiples cámaras.

## Paquetes Utilizados

- **`astra_camera`**: Proporciona la imagen en color y en profundidad de las cámaras Astra.
- **`calib_correspondences`**: Realiza la calibración entre sensores utilizando métodos de mínimos cuadrados.
- **`custom_msgs`**: Contiene los mensajes personalizados utilizados en la comunicación entre nodos.
- **`detection_aruco`**: Realiza la detección de los marcadores Aruco, importante para la calibración de las cámaras.
- **`detection_person`**: Detecta las articulaciones del cuerpo utilizando MediaPipe y las cámaras Astra.
- **`get_body_reconstruction`**: Calcula la reconstrucción tridimensional del cuerpo a partir de las coordenadas de las articulaciones.
- **`obtain_data_calibration`**: Aplica la calibración extrínseca a los puntos de las articulaciones detectadas.

## Estructura del Proyecto

### 1. Calibración Intrínseca
La calibración intrínseca se realiza mediante la detección de un marcador Aruco y la medición de la distancia entre el centro del marcador y el centro óptico de la cámara. Este proceso asegura que la información que se envía sobre la distancia y la rotación sea correcta.

Para su funcionamiento, se deberá de lanzar el siguiente comando estableciendo el tamaño del tablero contando las esquinas interiores y el tamaño qeu tienen los cuadrados en metros.

```bash
ros2 run usb_cam usb_cam_node_exe -- ros - args -p " video_device :=/ dev / video0 "
ros2 run camera_calibration cameracalibrator -- size 5 x8 -- square 0.034 \ -- ros - args -r image :=/ image_raw


### 2. Calibración Extrínseca
La calibración extrínseca se lleva a cabo utilizando el paquete **`calib_correspondences`**, que permite calcular la rotación y la translación entre los diferentes sensores de la cámara. Para obtener la transformación entre cámaras, se utilizan los siguientes métodos:
- **Método de Mínimos Cuadrados**: Estima la rotación y la translación maximizando la verosimilitud logarítmica.
- **Método Iterativo Levenberg-Marquardt**: Utiliza una optimización robusta para manejar errores ruidosos.

Los parámetros de configuración incluyen el ángulo máximo, la distancia máxima, y el tipo de algoritmo a utilizar para la calibración.

### 3. Detección de Persona
Con las cámaras calibradas, se procede a detectar las articulaciones del cuerpo utilizando el paquete **`detection_person`**. Se usan los topics:
- `/camera/color/image_raw`: Imagen RGB para la detección de las articulaciones.
- `/camera/depth/image_raw`: Imagen de profundidad para obtener las coordenadas tridimensionales de las articulaciones.

El paquete utiliza **MediaPipe** para identificar las articulaciones y calcula las coordenadas (x, y, z) de cada una.

### 4. Reconstrucción del Cuerpo
El paquete **`get_body_reconstruction`** es responsable de calcular y publicar la media de las coordenadas de las articulaciones visibles desde ambas cámaras, logrando así una representación en 3D del cuerpo. También maneja la visibilidad de las articulaciones y publica los puntos que son visibles solo en una de las cámaras.

## Comandos de Ejecución

Para ejecutar el sistema, sigue los pasos a continuación:

1. **Lanzar la cámara**:
   ```bash
   ros2 launch astra_camera multi_astra.launch.py
