#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import numpy as np

class RawPC2Parser(Node):
    def __init__(self):
        super().__init__('raw_pc2_parser')
        self.sub = self.create_subscription(
            PointCloud2,
            '/octomap_voxels_info',
            self.cb,
            10)

    def cb(self, msg: PointCloud2):
        # cuántos puntos hay
        total_pts = msg.width * msg.height
        if total_pts == 0:
            self.get_logger().warn("No hay puntos")
            return

        # obtener offsets de x,y,z
        # asumimos que los tres primeros campos son x,y,z
        offs = [msg.fields[i].offset for i in range(3)]
        step = msg.point_step

        coords = []
        # recorre cada punto en raw msg.data
        for i in range(total_pts):
            base = i * step
            raw = msg.data[base:base + step]
            # desempaquetar 3 floats little‐endian
            x = struct.unpack_from('<f', raw, offs[0])[0]
            y = struct.unpack_from('<f', raw, offs[1])[0]
            z = struct.unpack_from('<f', raw, offs[2])[0]
            coords.append((x, y, z))

        arr = np.array(coords, dtype=np.float32)  # N×3
        self.get_logger().info(f"Parsed {arr.shape[0]} points, first 5:")
        for p in arr[:5]:
            self.get_logger().info(f"  x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}")

def main():
    rclpy.init()
    node = RawPC2Parser()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()

'''
#----------------------------------------------------
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from pydualsense import pydualsense, TriggerModes
import math
import numpy as np
import overlap
import sys
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse


# Variable global para el contador
count = 0
ds = pydualsense()  # Crear una instancia del controlador
force_toSet_last=0
force_toSet_left_last=0
force_toSet_right_last=0
feedbackFactor = 1000000.0

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])

def calculate_parallelepiped_points(center, dimensions, orientation):
    # Definir los 8 vértices del paralelepípedo en el espacio local
    dx, dy, dz = dimensions[0]/2, dimensions[1]/2, dimensions[2]/2
    vertices = np.array([
        [-dx, -dy, -dz],
        [dx, -dy, -dz],
        [dx, dy, -dz],
        [-dx, dy, -dz],
        [-dx, -dy, dz],
        [dx, -dy, dz],
        [dx, dy, dz],
        [-dx, dy, dz]
    ])
    
    quaternion_list = [orientation.w, orientation.x, orientation.y, orientation.z]

    # Aplicar la orientación a los vértices
    rotation_matrix = quaternion_to_rotation_matrix(quaternion_list)
    rotated_vertices = np.dot(vertices, rotation_matrix.T)
    
    # Trasladar los vértices a la posición del centro
    translated_vertices = rotated_vertices + center
    
    return translated_vertices

# Función para desplazar el paralelepípedo a lo largo de su orientación
def displace_parallelepiped(center, displacement, orientation):
    # Convertir el desplazamiento local a global usando la orientación
    quaternion_list = [orientation.w, orientation.x, orientation.y, orientation.z]
    rotation_matrix = quaternion_to_rotation_matrix(quaternion_list)
    global_displacement = np.dot(displacement, rotation_matrix.T)

    # Convertir el centro (objeto Point) a un array numpy
    center_array = np.array([center.x, center.y, center.z])

    # Aplicar el desplazamiento al centro
    displaced_center = center_array + global_displacement
    
    return displaced_center

def calcular_distancia(p1, p2):
    return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)

def transform(x):
    if x >= 0.3:
        return 0
    elif x <= 0.1:
        return 255
    else:
        # Interpolación lineal entre 0.1 y 0.3
        return int((0.3 - x) * (255 / (0.3 - 0.1)))

def overlaptetra(box_points, sphere1, sphere2):
    # Formato de Sphere es centro [(x,y,z),radio]
    # radio_virtual Radio real + enlargement
    radio_virtual = 0.03 + 0.02
    esfera1 = overlap.Sphere((sphere1.x, sphere1.y, sphere1.z), radio_virtual)
    esfera2 = overlap.Sphere((sphere2.x, sphere2.y, sphere2.z), radio_virtual)
    # box_points debe ser [(x, y, z)*8]
    tetra = overlap.Hexahedron(box_points)
    inter1 = overlap.overlap(esfera1, tetra)
    inter2 = overlap.overlap(esfera2, tetra)
    total = (int)(feedbackFactor*(inter1 + inter2))
    #rospy.loginfo("Total %d: inter1=%f, inter2=%f", total, inter1, inter2)
    if (total > 255):
        return 255, inter1 + inter2
    if (total < 0):
        return 0, inter1 + inter2
    return (int)(total), (inter1 + inter2)

def callback2(data):
    try:
        onix_link6_index = data.name.index('onix::link6')
        bolas2_link0_index = data.name.index('Bolas_2::link_0')
        bolas2_link1_index = data.name.index('Bolas_2::link_1')
        bolas2_link2_index = data.name.index('Bolas_2::link_2')
    except ValueError as e:
        rospy.logerr('Link not found in LinkStates: %s', e)
        return

    #volume intersection calculation
    tetra_dimensions = [0.07, 0.1, 0.15]
    tetra_dimensions_inside = [0.06, 0.16, 0.1]

    local_displacement_right = np.array([0, 0.06, 0.08])
    local_displacement_inside = np.array([-0.005, 0, 0.08])
    local_displacement_left = np.array([0, -0.06, 0.08])

    onix_link6_pose = data.pose[onix_link6_index].position
    onix_link6_orientation = data.pose[onix_link6_index].orientation
    bolas2_link0_pose = data.pose[bolas2_link0_index].position
    bolas2_link1_pose = data.pose[bolas2_link1_index].position

    global force_toSet_left_last  # Declarar la variable global para poder modificarla
    global force_toSet_right_last  # Declarar la variable global para poder modificarla

    # Set feedback Right
    new_center_right = displace_parallelepiped(onix_link6_pose, local_displacement_right, onix_link6_orientation)
    displaced_points_right = calculate_parallelepiped_points(new_center_right, tetra_dimensions, onix_link6_orientation)
    #rospy.loginfo("Hap right:")
    hap_right, volIntR = overlaptetra(displaced_points_right, bolas2_link0_pose, bolas2_link1_pose)
    if(hap_right != force_toSet_right_last):
        ds.triggerR.setForce(1, hap_right)
        force_toSet_right_last=hap_right
        rospy.loginfo("Set force Right: %f", hap_right)

    # Set feedback left
    #for i, point in enumerate(displaced_points_right):
    #    rospy.loginfo("Vértice right %d: x=%f, y=%f, z=%f", i+1, point[0], point[1], point[2])
    new_center_left = displace_parallelepiped(onix_link6_pose, local_displacement_left, onix_link6_orientation)
    displaced_points_left = calculate_parallelepiped_points(new_center_left, tetra_dimensions, onix_link6_orientation)
    #rospy.loginfo("Hap left:")
    hap_left, volIntL = overlaptetra(displaced_points_left, bolas2_link0_pose, bolas2_link1_pose)
    if(hap_left != force_toSet_left_last):
        ds.triggerL.setForce(1, hap_left)
        force_toSet_left_last=hap_left
        rospy.loginfo("Set force Left: %f", hap_left)
    
    new_center_inside = displace_parallelepiped(onix_link6_pose, local_displacement_inside, onix_link6_orientation)
    #rospy.loginfo(f"New Center Inside Coordinates: x = {new_center_inside[0]}, y = {new_center_inside[1]}, z = {new_center_inside[2]}")
    displaced_points_inside = calculate_parallelepiped_points(new_center_inside, tetra_dimensions_inside, onix_link6_orientation)
    #rospy.loginfo("Hap Inside:")
    # Calcular volumen inside
    #for i, point in enumerate(displaced_points_inside):
        #rospy.loginfo("Vértice right %d: x=%f, y=%f, z=%f", i+1, point[0], point[1], point[2])

def listener():
    #rospy.init_node('distance_calculator', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback2, queue_size=2)
    rospy.spin()


if __name__ == '__main__':
    #global feedbackFactor
    if len(sys.argv) > 2:
        param_value = float(sys.argv[1])
        pos_object = int(sys.argv[2])
        feedbackFactor = param_value * feedbackFactor
        print(f"Received parameter: {param_value}")
        print(f"feedbackFactor: {feedbackFactor}")
        print(f"Received pos parameter: {pos_object}")
    else:
        print("No enough parameters received.")
    
    rospy.init_node('ps5_controller')  # Iniciar el nodo ROS

    ds.init()  # Inicializar el controlador

    ds.triggerL.setMode(TriggerModes.Rigid)  # Establecer el modo del gatillo
    ds.triggerR.setMode(TriggerModes.Rigid)  # Establecer el modo del gatillo

    listener()
    
    try:
        ds.triggerL.setForce(0)
        ds.triggerR.setForce(0)
        ds.triggerR.setMode(TriggerModes.Off)
        ds.triggerL.setMode(TriggerModes.Off)
        ds.close()
        rospy.loginfo("closed x2/n")
    except OSError as e:
        if str(e) == "Trying to perform action on closed device.":
            rospy.loginfo("Confirmation of dualsense closed/n")
        else:
            raise
'''
