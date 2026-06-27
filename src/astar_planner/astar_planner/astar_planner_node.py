#!/usr/bin/env python3
"""
A* Path Planning Node for ROS 2
Author: Yarin Sapir
Course: Software Development for Autonomous Vehicles

This node:
1. Subscribes to /map (OccupancyGrid)
2. Subscribes to /initialpose (start position)
3. Subscribes to /goal_pose (goal position)
4. Runs A* algorithm on the occupancy grid
5. Publishes /planned_path (Path)
6. Publishes /astar_markers (MarkerArray) for RViz visualization
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import heapq
import math
import time


class AStarPlannerNode(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # Parameters
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('heuristic', 'euclidean')  # 'euclidean' or 'manhattan'

        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.allow_diagonal = self.get_parameter('allow_diagonal').value
        self.heuristic_type = self.get_parameter('heuristic').value

        # State
        self.map_data = None
        self.map_info = None
        self.start_pose = None
        self.goal_pose = None

        # QoS for map subscription (latched)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.start_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.start_callback, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/astar_markers', 10)

        self.get_logger().info('A* Planner Node started. Waiting for map...')

    # ─────────────────────────── Callbacks ───────────────────────────

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg.data
        self.map_info = msg.info
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height}, '
            f'resolution={msg.info.resolution:.3f} m/cell'
        )
        self._try_plan()

    def start_callback(self, msg: PoseWithCovarianceStamped):
        self.start_pose = msg.pose.pose
        self.get_logger().info(
            f'Start set: ({self.start_pose.position.x:.2f}, {self.start_pose.position.y:.2f})'
        )
        self._try_plan()

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg.pose
        self.get_logger().info(
            f'Goal set: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})'
        )
        self._try_plan()

    # ─────────────────────────── Planning ────────────────────────────

    def _try_plan(self):
        if self.map_data is None or self.start_pose is None or self.goal_pose is None:
            return

        start_cell = self._world_to_grid(
            self.start_pose.position.x, self.start_pose.position.y)
        goal_cell = self._world_to_grid(
            self.goal_pose.position.x, self.goal_pose.position.y)

        if not self._is_valid(*start_cell):
            self.get_logger().error('Start position is outside the map!')
            return
        if not self._is_valid(*goal_cell):
            self.get_logger().error('Goal position is outside the map!')
            return
        if self._is_obstacle(*start_cell):
            self.get_logger().warn('Start position is inside an obstacle!')
        if self._is_obstacle(*goal_cell):
            self.get_logger().warn('Goal position is inside an obstacle!')

        self.get_logger().info(
            f'Planning from {start_cell} to {goal_cell}...'
        )

        t0 = time.time()
        path_cells, explored = self._astar(start_cell, goal_cell)
        elapsed = (time.time() - t0) * 1000

        if path_cells is None:
            self.get_logger().error('A* found NO path!')
            return

        self.get_logger().info(
            f'Path found in {elapsed:.1f} ms | '
            f'{len(path_cells)} cells | {len(explored)} cells explored'
        )

        self._publish_path(path_cells)
        self._publish_markers(path_cells, explored, start_cell, goal_cell)

    def _astar(self, start, goal):
        """
        A* search on the occupancy grid.
        Returns (path_cells, explored_cells) or (None, explored_cells).
        """
        h = self.map_info.height
        w = self.map_info.width

        open_heap = []   # (f, g, node)
        g_score = {start: 0.0}
        came_from = {}
        explored = set()

        heapq.heappush(open_heap, (self._heuristic(start, goal), 0.0, start))

        while open_heap:
            f, g, current = heapq.heappop(open_heap)

            if current in explored:
                continue
            explored.add(current)

            if current == goal:
                return self._reconstruct_path(came_from, goal), explored

            for neighbor, cost in self._neighbors(current):
                if neighbor in explored:
                    continue
                tentative_g = g + cost
                if tentative_g < g_score.get(neighbor, math.inf):
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f_new = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f_new, tentative_g, neighbor))

        return None, explored

    def _neighbors(self, cell):
        r, c = cell
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]
        if self.allow_diagonal:
            moves += [(-1, -1, 1.414), (-1, 1, 1.414),
                      (1, -1, 1.414), (1, 1, 1.414)]

        result = []
        for dr, dc, cost in moves:
            nr, nc = r + dr, c + dc
            if self._is_valid(nr, nc) and not self._is_obstacle(nr, nc):
                result.append(((nr, nc), cost))
        return result

    def _heuristic(self, a, b):
        if self.heuristic_type == 'manhattan':
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        else:  # euclidean
            return math.hypot(a[0] - b[0], a[1] - b[1])

    def _reconstruct_path(self, came_from, goal):
        path = []
        node = goal
        while node in came_from:
            path.append(node)
            node = came_from[node]
        path.append(node)
        path.reverse()
        return path

    # ─────────────────────────── Grid utils ──────────────────────────

    def _world_to_grid(self, wx, wy):
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        col = int((wx - ox) / res)
        row = int((wy - oy) / res)
        return (row, col)

    def _grid_to_world(self, row, col):
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        wx = ox + (col + 0.5) * res
        wy = oy + (row + 0.5) * res
        return wx, wy

    def _is_valid(self, row, col):
        return 0 <= row < self.map_info.height and 0 <= col < self.map_info.width

    def _is_obstacle(self, row, col):
        idx = row * self.map_info.width + col
        val = self.map_data[idx]
        return val >= self.obstacle_threshold or val == -1

    # ─────────────────────────── Publishing ──────────────────────────

    def _publish_path(self, path_cells):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        for row, col in path_cells:
            wx, wy = self._grid_to_world(row, col)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.path_pub.publish(msg)
        self.get_logger().info(f'Published path with {len(path_cells)} waypoints')

    def _publish_markers(self, path_cells, explored, start, goal):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1. Explored cells (light blue)
        explored_marker = Marker()
        explored_marker.header.frame_id = 'map'
        explored_marker.header.stamp = now
        explored_marker.ns = 'explored'
        explored_marker.id = 0
        explored_marker.type = Marker.POINTS
        explored_marker.action = Marker.ADD
        explored_marker.scale.x = self.map_info.resolution * 0.8
        explored_marker.scale.y = self.map_info.resolution * 0.8
        explored_marker.color = ColorRGBA(r=0.5, g=0.8, b=1.0, a=0.4)

        for row, col in explored:
            wx, wy = self._grid_to_world(row, col)
            explored_marker.points.append(Point(x=wx, y=wy, z=0.0))
        markers.markers.append(explored_marker)

        # 2. Path cells (green)
        path_marker = Marker()
        path_marker.header.frame_id = 'map'
        path_marker.header.stamp = now
        path_marker.ns = 'path'
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = self.map_info.resolution * 0.5
        path_marker.color = ColorRGBA(r=0.0, g=0.9, b=0.2, a=1.0)

        for row, col in path_cells:
            wx, wy = self._grid_to_world(row, col)
            path_marker.points.append(Point(x=wx, y=wy, z=0.0))
        markers.markers.append(path_marker)

        # 3. Start marker (blue sphere)
        start_marker = Marker()
        start_marker.header.frame_id = 'map'
        start_marker.header.stamp = now
        start_marker.ns = 'start'
        start_marker.id = 2
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        wx, wy = self._grid_to_world(*start)
        start_marker.pose.position.x = wx
        start_marker.pose.position.y = wy
        start_marker.pose.position.z = 0.1
        start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.3
        start_marker.color = ColorRGBA(r=0.0, g=0.2, b=1.0, a=1.0)
        markers.markers.append(start_marker)

        # 4. Goal marker (red sphere)
        goal_marker = Marker()
        goal_marker.header.frame_id = 'map'
        goal_marker.header.stamp = now
        goal_marker.ns = 'goal'
        goal_marker.id = 3
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        wx, wy = self._grid_to_world(*goal)
        goal_marker.pose.position.x = wx
        goal_marker.pose.position.y = wy
        goal_marker.pose.position.z = 0.1
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3
        goal_marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
        markers.markers.append(goal_marker)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
