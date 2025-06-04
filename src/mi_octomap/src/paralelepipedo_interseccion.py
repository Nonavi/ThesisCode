#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from geometry_msgs.msg import PoseStamped, Vector3, Point
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
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


class Sphere:
    def __init__(self, center, radius, side):
        self.center = np.array(center)
        self.radius = radius
        self.side = side


class RawPC2Parser(Node):
    def __init__(self):
        super().__init__('raw_pc2_parser')
        self.point_sphere_radius = 0.03
        self.sphere_spacing = 0.01 * 2
        self.gripper_spheres = []
        self.cylinder_hexahedra_by_id = {}

        self.br = TransformBroadcaster(self)
        self.trigger_pub = self.create_publisher(Vector3, '/trigger_force', 10)
        self.hex_marker_pub = self.create_publisher(MarkerArray, '/hexahedron_markers', 10)
        self.goal_marker_pub = self.create_publisher(MarkerArray, '/goal_marker', 10)
        self.sphere_pub = self.create_publisher(MarkerArray, '/gripper_sphere_markers', 10)

        self.create_subscription(PoseStamped, '/rviz/moveit/move_marker/goal_link_tcp', self.pose_cb, 10)
        self.create_subscription(CollisionObject, '/detected_objects', self.cylinder_cb, 10)

        self.create_timer(0.1, self.compute_overlap)

    def pose_cb(self, msg: PoseStamped):
        self.gripper_spheres.clear()
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

        self.create_spheres(center_right, orientation, 'right')
        self.create_spheres(center_left, orientation, 'left')

        goal_marker = Marker()
        goal_marker.header.frame_id = "world"
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = "goal"
        goal_marker.id = 0
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = float(msg.pose.position.x)
        goal_marker.pose.position.y = float(msg.pose.position.y)
        goal_marker.pose.position.z = float(msg.pose.position.z)
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.02
        goal_marker.color.r = 1.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        self.goal_marker_pub.publish(MarkerArray(markers=[goal_marker]))

    def create_spheres(self, center, orientation, side):
        radius = self.point_sphere_radius
        spacing = self.sphere_spacing
        dx, dy, dz = 0.023/2, 0.037/2, 0.1/2
        R = quaternion_matrix(orientation)

        x_vals = np.arange(-dx + spacing/2, dx, spacing)
        y_vals = np.arange(-dy + spacing/2, dy, spacing)
        z_vals = np.arange(-dz + spacing/2, dz, spacing)

        marker_array = MarkerArray()
        idx = 0

        for x in x_vals:
            for y in y_vals:
                for z in z_vals:
                    local = np.array([x, y, z])
                    world = R @ local + np.array(center)
                    self.gripper_spheres.append(Sphere(world, radius, side))

                    m = Marker()
                    m.header.frame_id = "world"
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.ns = f"spheres_{side}"
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
                    marker_array.markers.append(m)
                    idx += 1

        self.sphere_pub.publish(marker_array)

    def cylinder_cb(self, msg: CollisionObject):
        all_markers = []
        color_list = [ (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 1.0, 0.0) ]
        self.cylinder_hexahedra_by_id[msg.id] = []

        for idx, (prim, pose) in enumerate(zip(msg.primitives, msg.primitive_poses)):
            if prim.type != SolidPrimitive.CYLINDER:
                continue
            h = prim.dimensions[0]
            r = prim.dimensions[1]
            cz = pose.position.z
            hexa_list = create_octagonal_hexahedra(pose.position.x, pose.position.y, cz, r, h)
            for j, verts in enumerate(hexa_list):
                self.cylinder_hexahedra_by_id[msg.id].append(overlap.Hexahedron(verts))

                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"cylinder_{j}"
                marker.id = idx * 10 + j
                marker.type = Marker.LINE_LIST
                marker.action = Marker.ADD
                marker.scale.x = 0.002
                r_col, g_col, b_col = color_list[j % 4]
                marker.color.r = r_col
                marker.color.g = g_col
                marker.color.b = b_col
                marker.color.a = 1.0

                connections = [
                    (0,1),(1,2),(2,3),(3,0),
                    (4,5),(5,6),(6,7),(7,4),
                    (0,4),(1,5),(2,6),(3,7)
                ]
                for a, b in connections:
                    marker.points.append(Point(x=verts[a][0], y=verts[a][1], z=verts[a][2]))
                    marker.points.append(Point(x=verts[b][0], y=verts[b][1], z=verts[b][2]))

                all_markers.append(marker)

        self.hex_marker_pub.publish(MarkerArray(markers=all_markers))

    def compute_overlap(self):
        if not self.gripper_spheres or not self.cylinder_hexahedra_by_id:
            return

        overlap_left = 0.0
        overlap_right = 0.0
        for sphere in self.gripper_spheres:
            for id_group in self.cylinder_hexahedra_by_id.values():
                for i, hex in enumerate(id_group):
                    value = overlap.overlap(overlap.Sphere(sphere.center, sphere.radius), hex)
                    if value > 0:
                        self.get_logger().info(
                            f"Sphere {sphere.side} at {sphere.center} intersects Hex with volume {value:.12f}"
                        )
                    if sphere.side == 'left':
                        overlap_left += value
                    else:
                        overlap_right += value

        fuerza_L = int(max(0, min(255, (overlap_left / 0.0003) * 255)))
        fuerza_R = int(max(0, min(255, (overlap_right / 0.0003) * 255)))

        msg_out = Vector3()
        msg_out.x = float(fuerza_L)
        msg_out.y = float(fuerza_R)
        msg_out.z = 0.0
        self.trigger_pub.publish(msg_out)

        self.get_logger().info(
            f"Total: overlap_left={overlap_left:.12f}, overlap_right={overlap_right:.12f} -> L={fuerza_L}, R={fuerza_R}"
        )


def main():
    rclpy.init()
    node = RawPC2Parser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
