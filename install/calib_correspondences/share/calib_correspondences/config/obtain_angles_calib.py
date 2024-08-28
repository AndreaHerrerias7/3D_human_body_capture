import numpy as np
from scipy.spatial.transform import Rotation as R

# Dada la matriz de transformación
R_matrix = np.array([
    [0.490012, 0.315367, -0.81267, 1.74929],
    [-0.253816, 0.943488, 0.21309, -0.436628],
    [0.833946, 0.101852, 0.542366, 0.193558],
    [0, 0, 0, 1]
])

# Extraemos la submatriz de rotación 3x3
rot_matrix = R_matrix[:3, :3]

# Calculamos los ángulos de Euler (suponiendo orden ZYX)
rotation = R.from_matrix(rot_matrix)
euler_angles = rotation.as_euler('xyz', degrees=True)

print("Rotación en Roll (X):", euler_angles[0], "grados")
print("Rotación en Pitch (Y):", euler_angles[1], "grados")
print("Rotación en Yaw (Z):", euler_angles[2], "grados")
