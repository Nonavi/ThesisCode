#!/usr/bin/env python3
# Este codigo hace lo que hacia antes, con las librerias correctas leia octomap, pero rompia todo el resto.
import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
import octomap
import numpy as np

class OctomapProcessor(Node):
    def __init__(self):
        super().__init__('octomap_processor')
        self.subscription = self.create_subscription(
            Octomap,
            '/octomap_binary',
            self.octomap_callback,
            10)
        self.octree = octomap.OcTree(0.1)  # 0.1 is the resolution

    def octomap_callback(self, msg):
        # Convert ROS message to OcTree using binaryMsgToMap
        tree = octomap.OcTree.binaryMsgToMap(msg.resolution, 
                                             msg.id.encode('utf-8'), 
                                             msg.binary, 
                                             bytes(msg.data)
                                             )
    
        if tree is None:
            self.get_logger().warn("Failed to create OcTree from message")
            return
    
        # Process the OcTree
        for node in tree.begin_tree():
            if tree.isNodeOccupied(node):
                # Process occupied nodes
                coord = node.getCoordinate()
                self.get_logger().info(f"Occupied node at {coord[0]}, {coord[1]}, {coord[2]}")

        # You can also update your own OcTree
        occupied_points, _ = tree.extractPointCloud()
        if len(occupied_points) > 0:
            self.octree.insertPointCloud(
                occupied_points,
                np.array([0.0, 0.0, 0.0])  # origin
            )
    
        self.get_logger().info(f"Updated OcTree, now has {self.octree.size()} nodes")

def main(args=None):
    rclpy.init(args=args)
    octomap_processor = OctomapProcessor()
    rclpy.spin(octomap_processor)
    octomap_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
