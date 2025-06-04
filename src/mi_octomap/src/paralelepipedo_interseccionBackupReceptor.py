#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped   # ① importar PoseStamped
import struct
import numpy as np

class RawPC2Parser(Node):
    def __init__(self):
        super().__init__('raw_pc2_parser')
        # Suscripción al octomap
        self.sub = self.create_subscription(
            PointCloud2,
            '/octomap_voxels_info',
            self.cb,
            10)
        # Suscripción al goal pose de MoveIt/RViz
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/rviz/moveit/move_marker/goal_link_tcp',
            self.pose_cb,
            10)

    def cb(self, msg: PointCloud2):
        # tu código existente para procesar PointCloud2...
        total_pts = msg.width * msg.height
        if total_pts == 0:
            self.get_logger().warn("No hay puntos")
            return
        offs = [msg.fields[i].offset for i in range(3)]
        step = msg.point_step
        coords = []
        for i in range(total_pts):
            base = i * step
            raw = msg.data[base:base + step]
            x = struct.unpack_from('<f', raw, offs[0])[0]
            y = struct.unpack_from('<f', raw, offs[1])[0]
            z = struct.unpack_from('<f', raw, offs[2])[0]
            coords.append((x, y, z))
        arr = np.array(coords, dtype=np.float32)
        self.get_logger().info(f"Parsed {arr.shape[0]} points, first 5:")
        for p in arr[:5]:
            self.get_logger().info(f"  x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}")

    def pose_cb(self, msg: PoseStamped):
        # Imprime en formato parecido al echo de ROS 2
        self.get_logger().info('---')
        self.get_logger().info(f"header:\n  stamp:\n    sec: {msg.header.stamp.sec}\n    nanosec: {msg.header.stamp.nanosec}\n  frame_id: {msg.header.frame_id}")
        self.get_logger().info("pose:")
        self.get_logger().info(f"  position:\n    x: {msg.pose.position.x}\n    y: {msg.pose.position.y}\n    z: {msg.pose.position.z}")
        self.get_logger().info("  orientation:")
        self.get_logger().info(f"    x: {msg.pose.orientation.x}\n    y: {msg.pose.orientation.y}\n    z: {msg.pose.orientation.z}\n    w: {msg.pose.orientation.w}")
        self.get_logger().info('---')

def main():
    rclpy.init()
    node = RawPC2Parser()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()


