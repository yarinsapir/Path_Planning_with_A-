#!/usr/bin/env python3
"""
Static Map Publisher Node
Author: Yarin Sapir

Publishes a pre-built occupancy grid map (or loads a YAML/PGM map).
Used for testing the A* planner without nav2_map_server.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

import numpy as np


def build_demo_map():
    """
    Build a 40x40 cell map (resolution=0.25m → 10m x 10m world).
    0  = free, 100 = obstacle, -1 = unknown.
    """
    size = 40
    grid = np.zeros((size, size), dtype=np.int8)

    # Outer walls
    grid[0, :] = 100
    grid[-1, :] = 100
    grid[:, 0] = 100
    grid[:, -1] = 100

    # Horizontal wall with a gap
    grid[15, 5:30] = 100
    grid[15, 18:22] = 0   # gap

    # Vertical wall with a gap
    grid[20:38, 25] = 100
    grid[28:32, 25] = 0   # gap

    # Small obstacle box
    grid[6:10, 8:13] = 100

    # Another obstacle
    grid[25:30, 8:12] = 100

    return grid, size, 0.25   # data, size, resolution


class MapPublisherNode(Node):

    def __init__(self):
        super().__init__('map_publisher')

        # Latched QoS — subscriber gets the map even if they connect later
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)

        # Build and publish once, then keep a timer to re-publish for late subscribers
        self.map_msg = self._build_map_msg()
        self.pub.publish(self.map_msg)
        self.get_logger().info('Map published (latched). Map is 10m x 10m at 0.25m/cell.')

        self.timer = self.create_timer(5.0, self._republish)

    def _build_map_msg(self):
        grid, size, res = build_demo_map()

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.resolution = res
        msg.info.width = size
        msg.info.height = size
        msg.info.origin = Pose()
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.flatten().tolist()
        return msg

    def _republish(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
