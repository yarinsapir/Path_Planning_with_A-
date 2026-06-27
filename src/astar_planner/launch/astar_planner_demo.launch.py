"""
Launch file: astar_planner_demo.launch.py
Author: Yarin Sapir

Launches:
  1. map_publisher  — publishes the demo occupancy grid
  2. astar_planner  — A* planning node
  3. rviz2          — visualization (optional, set launch_rviz:=true)

Usage:
  ros2 launch astar_planner astar_planner_demo.launch.py
  ros2 launch astar_planner astar_planner_demo.launch.py launch_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('astar_planner')

    # ── Arguments ──────────────────────────────────────────────
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='false',
        description='Set to true to launch RViz2 automatically'
    )
    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold', default_value='50',
        description='Occupancy value (0-100) above which a cell is an obstacle'
    )
    allow_diagonal_arg = DeclareLaunchArgument(
        'allow_diagonal', default_value='true',
        description='Whether A* may move diagonally'
    )
    heuristic_arg = DeclareLaunchArgument(
        'heuristic', default_value='euclidean',
        description='Heuristic type: euclidean or manhattan'
    )

    # ── Nodes ───────────────────────────────────────────────────
    map_publisher_node = Node(
        package='astar_planner',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
    )

    astar_node = Node(
        package='astar_planner',
        executable='astar_planner',
        name='astar_planner',
        output='screen',
        parameters=[{
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'allow_diagonal':     LaunchConfiguration('allow_diagonal'),
            'heuristic':          LaunchConfiguration('heuristic'),
        }],
    )

    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'astar_rviz.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen',
    )

    return LaunchDescription([
        launch_rviz_arg,
        obstacle_threshold_arg,
        allow_diagonal_arg,
        heuristic_arg,
        LogInfo(msg='Starting A* Path Planner — Yarin Sapir'),
        map_publisher_node,
        astar_node,
        rviz_node,
    ])
