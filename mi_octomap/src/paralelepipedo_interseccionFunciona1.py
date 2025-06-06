#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import struct
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
        [1.0 - (yy + zz),      xy - wz,           xz + wy],
        [     xy + wz,     1.0 - (xx + zz),       yz - wx],
        [     xz - wy,         yz + wx,      1.0 - (xx + yy)]
    ], dtype=np.float64)

def calculate_parallelepiped_points(center, dimensions, orientation):
    dx, dy, dz = dimensions[0] / 2, dimensions[1] / 2, dimensions[2] / 2
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

class RawPC2Parser(Node):
    def __init__(self):
        super().__init__('raw_pc2_parser')
        self.box_right_verts = None
        self.box_left_verts = None
        self.point_sphere_radius = 0.0254

        # Transform broadcaster for RViz
        self.br = TransformBroadcaster(self)

        self.create_subscription(
            PoseStamped,
            '/rviz/moveit/move_marker/goal_link_tcp',
            self.pose_cb, 10
        )
        self.create_subscription(
            PointCloud2,
            '/octomap_voxels_info',
            self.octomap_cb, 10
        )

    def pose_cb(self, msg: PoseStamped):
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
        raw_center = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        offset = 0.0425
        center_right = raw_center + right_vec * offset
        center_left = raw_center - right_vec * offset

        self.box_right_verts = calculate_parallelepiped_points(center_right, dimensions, orientation)
        self.box_left_verts  = calculate_parallelepiped_points(center_left,  dimensions, orientation)

        # broadcast frames for each vertex
        self._broadcast_box_frames(self.box_right_verts, frame_id, orientation, 'box_right')
        self._broadcast_box_frames(self.box_left_verts, frame_id, orientation, 'box_left')

    def _broadcast_box_frames(self, verts, parent_frame, orientation, prefix):
        """Publish TF frames for each vertex."""
        for i, v in enumerate(verts):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent_frame
            t.child_frame_id = f'{prefix}_v{i}'
            t.transform.translation.x = float(v[0])
            t.transform.translation.y = float(v[1])
            t.transform.translation.z = float(v[2])
            # Use the orientation of the box for the frame
            t.transform.rotation.x = orientation[0]
            t.transform.rotation.y = orientation[1]
            t.transform.rotation.z = orientation[2]
            t.transform.rotation.w = orientation[3]
            self.br.sendTransform(t)

    def octomap_cb(self, msg: PointCloud2):
        if self.box_right_verts is None or self.box_left_verts is None:
            return

        total = msg.width * msg.height
        if total == 0:
            return

        offs = [f.offset for f in msg.fields[:3]]
        step = msg.point_step
        overlap_right = overlap_left = 0.0

        # build hexahedra once
        box_r = overlap.Hexahedron(self.box_right_verts)
        box_l = overlap.Hexahedron(self.box_left_verts)

        for i in range(total):
            base = i * step
            raw = msg.data[base:base + step]
            x = struct.unpack_from('<f', raw, offs[0])[0]
            y = struct.unpack_from('<f', raw, offs[1])[0]
            z = struct.unpack_from('<f', raw, offs[2])[0]

            sph = overlap.Sphere((float(x), float(y), float(z)), self.point_sphere_radius)
            overlap_right += overlap.overlap(sph, box_r)
            overlap_left  += overlap.overlap(sph, box_l)

        self.get_logger().info(f"overlap_right={overlap_right:.6f}, overlap_left={overlap_left:.6f}")


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
