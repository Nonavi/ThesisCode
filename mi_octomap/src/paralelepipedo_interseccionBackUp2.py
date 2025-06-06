#!/usr/bin/env python3
# Este codigo recibe el octomap parseado de cpp en python y puede trabajar con el.
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
