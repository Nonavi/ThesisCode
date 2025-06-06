#Back up creado, en esta funcion se detecta el octagono correctamente, pero no se calcula de manera correcta la inteseccion y eso me preocupa...
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Vector3, Point
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import struct
import overlap

def quaternion_matrix(q):
    x, y, z, w = q
    n = x*x + y*y + z*z + w*w
    if n < 1e-8:
        return np.eye(3)
    s = 2.0 / n
    xs, ys, zs = x*s, y*s, z*s
    wx, wy, wz = w*xs, w*ys, w*zs
    xx, xy, xz = x*xs, x*ys, x*zs
    yy, yz, zz = y*ys, y*zs, z*zs
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)]
    ], dtype=np.float64)

def calculate_parallelepiped_points(center, dimensions, orientation):
    dx, dy, dz = dimensions[0]/2, dimensions[1]/2, dimensions[2]/2
    local = np.array([
        [-dx, -dy, -dz],
        [ dx, -dy, -dz],
        [ dx,  dy, -dz],
        [-dx,  dy, -dz],
        [-dx, -dy,  dz],
        [ dx, -dy,  dz],
        [ dx,  dy,  dz],
        [-dx,  dy,  dz],
    ], dtype=np.float64)
    R = quaternion_matrix(orientation)
    return (R @ local.T).T + np.array(center, dtype=np.float64)

def create_octagonal_hexahedra(cx, cy, cz, r, h):
    hexahedra = []
    z_min = cz - h / 2.0
    z_max = cz + h / 2.0

    for i in range(4):
        angle0 = np.deg2rad(i * 90)
        angle1 = angle0 + np.deg2rad(45)
        angle2 = angle0 + np.deg2rad(90)

        x0, y0 = cx, cy
        x1, y1 = cx + r * np.cos(angle0), cy + r * np.sin(angle0)
        x2, y2 = cx + r * np.cos(angle1), cy + r * np.sin(angle1)
        x3, y3 = cx + r * np.cos(angle2), cy + r * np.sin(angle2)

        v0 = np.array([x0, y0, z_min])
        v1 = np.array([x1, y1, z_min])
        v2 = np.array([x2, y2, z_min])
        v3 = np.array([x3, y3, z_min])

        v4 = np.array([x0, y0, z_max])
        v5 = np.array([x1, y1, z_max])
        v6 = np.array([x2, y2, z_max])
        v7 = np.array([x3, y3, z_max])

        verts = np.array([v0, v1, v2, v3, v4, v5, v6, v7])
        hexahedra.append(verts)

    return hexahedra

def escalar(valor, max_valor=0.0003):
    return int(max(0, min(255, (valor / max_valor) * 255)))

class Sphere:
    def __init__(self, center, radius, side):
        self.center = np.array(center)
        self.radius = radius
        self.side = side

class RawPC2Parser(Node):
    def __init__(self):
        super().__init__('raw_pc2_parser')
        self.box_right_verts = None
        self.box_left_verts = None
        self.point_sphere_radius = 0.03
        self.sphere_spacing = 0.01
        self.cylinder_hexahedra = []
        self.gripper_spheres = []

        self.br = TransformBroadcaster(self)
        self.trigger_pub = self.create_publisher(Vector3, '/trigger_force', 10)
        self.sphere_pub = self.create_publisher(MarkerArray, '/gripper_sphere_markers', 10)
        self.hex_marker_pub = self.create_publisher(MarkerArray, '/hexahedron_markers', 10)

        self.create_subscription(PoseStamped, '/rviz/moveit/move_marker/goal_link_tcp', self.pose_cb, 10)
        self.create_subscription(PointCloud2, '/octomap_voxels_info', self.octomap_cb, 10)
        self.create_subscription(CollisionObject, '/detected_objects', self.cylinder_cb, 10)

    def pose_cb(self, msg: PoseStamped):
        self.gripper_spheres.clear()
        frame_id = msg.header.frame_id
        dimensions = (0.023, 0.037, 0.1)
        orientation = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        R = quaternion_matrix(orientation)
        right_vec = -R[:, 1]
        back_vec = -R[:, 2]

        raw_center = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        offset = 0.0425
        backward_offset = 0.04

        center_right = raw_center + right_vec * offset + back_vec * backward_offset
        center_left  = raw_center - right_vec * offset + back_vec * backward_offset

        self.publish_sphere_markers(center_right, orientation, 'right_gripper', 'right')
        self.publish_sphere_markers(center_left, orientation, 'left_gripper', 'left')

    def publish_sphere_markers(self, center, orientation, prefix, side):
        radius = self.point_sphere_radius
        spacing = self.sphere_spacing
        dx, dy, dz = 0.023/2, 0.037/2, 0.1/2
        R = quaternion_matrix(orientation)

        x_vals = np.arange(-dx + spacing/2, dx, spacing)
        y_vals = np.arange(-dy + spacing/2, dy, spacing)
        z_vals = np.arange(-dz + spacing/2, dz, spacing)

        markers = MarkerArray()
        idx = 0
        for x in x_vals:
            for y in y_vals:
                for z in z_vals:
                    local = np.array([x, y, z])
                    world = R @ local + np.array(center)

                    self.gripper_spheres.append(Sphere(world, radius, side))

                    m = Marker()
                    m.header.frame_id = "world"
                    m.ns = prefix
                    m.id = idx
                    m.type = Marker.SPHERE
                    m.action = Marker.ADD
                    m.pose.position.x = float(world[0])
                    m.pose.position.y = float(world[1])
                    m.pose.position.z = float(world[2])
                    m.pose.orientation.w = 1.0
                    m.scale.x = m.scale.y = m.scale.z = radius * 2
                    m.color.r = 0.0
                    m.color.g = 1.0
                    m.color.b = 1.0
                    m.color.a = 0.6
                    markers.markers.append(m)
                    idx += 1

        self.sphere_pub.publish(markers)

    def _broadcast_box_frames(self, verts, parent_frame, orientation, prefix):
        for i, v in enumerate(verts):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent_frame
            t.child_frame_id = f'{prefix}_v{i}'
            t.transform.translation.x = float(v[0])
            t.transform.translation.y = float(v[1])
            t.transform.translation.z = float(v[2])
            t.transform.rotation.x = orientation[0]
            t.transform.rotation.y = orientation[1]
            t.transform.rotation.z = orientation[2]
            t.transform.rotation.w = orientation[3]
            self.br.sendTransform(t)

    def _broadcast_hexahedron(self, verts, frame_prefix, color):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = now
        m.ns = frame_prefix
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.002
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0

        connections = [
            (0,1),(1,2),(2,3),(3,0),
            (4,5),(5,6),(6,7),(7,4),
            (0,4),(1,5),(2,6),(3,7)
        ]
        for a, b in connections:
            for i in [a, b]:
                pt = verts[i]
                p = Point()
                p.x, p.y, p.z = pt
                m.points.append(p)

        markers.markers.append(m)
        self.hex_marker_pub.publish(markers)

    def cylinder_cb(self, msg: CollisionObject):
        self.cylinder_hexahedra = []
        color_map = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0)   # Yellow
        ]
        for idx, (prim, pose) in enumerate(zip(msg.primitives, msg.primitive_poses)):
            if prim.type != SolidPrimitive.CYLINDER:
                continue
            h = prim.dimensions[0]
            r = prim.dimensions[1]
            cz = pose.position.z
            hexa_list = create_octagonal_hexahedra(pose.position.x, pose.position.y, cz, r, h)
            for j, verts in enumerate(hexa_list):
                color = color_map[j % len(color_map)]
                self._broadcast_hexahedron(verts, f"cylinder_{idx}_hex_{j}", color)
                self.cylinder_hexahedra.append(overlap.Hexahedron(verts))

    def octomap_cb(self, msg: PointCloud2):
        if not self.gripper_spheres:
            return

        overlap_left = 0.0
        overlap_right = 0.0
        for sphere in self.gripper_spheres:
            for hex in self.cylinder_hexahedra:
                value = overlap.overlap(overlap.Sphere(sphere.center, sphere.radius), hex)
                if sphere.side == 'left':
                    overlap_left += value
                else:
                    overlap_right += value

        fuerza_L = escalar(overlap_left)
        fuerza_R = escalar(overlap_right)

        msg_out = Vector3()
        msg_out.x = float(fuerza_L)
        msg_out.y = float(fuerza_R)
        msg_out.z = 0.0

        self.trigger_pub.publish(msg_out)
        self.get_logger().info(
            f"overlap_left={overlap_left:.12f}, overlap_right={overlap_right:.12f} -> L={fuerza_L}, R={fuerza_R}"
        )

def main():
    rclpy.init()
    parser_node = RawPC2Parser()
    try:
        rclpy.spin(parser_node)
    finally:
        parser_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
