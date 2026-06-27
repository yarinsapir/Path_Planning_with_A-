# A* Path Planning in ROS 2

**Course Project — Software Development for Autonomous Vehicles**  
**Author: Yarin Sapir**

---

## Overview

This project implements an **A\* (A-star) path planning algorithm** as a ROS 2 node that plans collision-free paths on an occupancy grid map.

The planner:
- Subscribes to a `/map` (OccupancyGrid) topic
- Receives a start pose via `/initialpose` and a goal via `/goal_pose`
- Runs A\* search on the grid and publishes the resulting path on `/planned_path`
- Visualizes explored cells, the final path, start, and goal in RViz2 via `/astar_markers`

---

## System Architecture

```
┌─────────────────┐       /map        ┌──────────────────────┐
│  map_publisher  │ ─────────────────▶│                      │
│  (OccupancyGrid)│                   │   astar_planner      │──▶ /planned_path
└─────────────────┘                   │   (A* Algorithm)     │──▶ /astar_markers
                                       │                      │
    /initialpose ───────────────────▶│                      │
    /goal_pose   ───────────────────▶│                      │
                                       └──────────────────────┘
                                                   │
                                                   ▼
                                              [ RViz2 ]
```

---

## ROS 2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Subscribe | The occupancy grid map |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Subscribe | Start position (set via RViz2) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Subscribe | Goal position (set via RViz2) |
| `/planned_path` | `nav_msgs/Path` | Publish | The A\* planned path |
| `/astar_markers` | `visualization_msgs/MarkerArray` | Publish | Visualization markers |

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `obstacle_threshold` | `50` | Occupancy value above which a cell is treated as obstacle (0–100) |
| `allow_diagonal` | `true` | Whether A\* can move diagonally |
| `heuristic` | `euclidean` | Heuristic: `euclidean` or `manhattan` |

---

## A\* Algorithm

The A\* algorithm finds the shortest path between two cells on the grid:

```
f(n) = g(n) + h(n)
  g(n) = actual cost from start to n
  h(n) = heuristic estimate from n to goal
```

- **Straight moves** cost 1.0
- **Diagonal moves** cost √2 ≈ 1.414
- Heuristic: Euclidean distance (or Manhattan if configured)

---

## Installation & Build

### Prerequisites
- ROS 2 Humble (or Foxy/Iron)
- Python 3.8+

### Build

```bash
# Clone the repository
git clone https://github.com/YarinSapir/ros2-astar-path-planner.git
cd ros2-astar-path-planner/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select astar_planner

# Source
source install/setup.bash
```

---

## Running the Demo

### Terminal 1 — Launch nodes
```bash
source install/setup.bash
ros2 launch astar_planner astar_planner_demo.launch.py
```

### Terminal 2 — Open RViz2 (or add `launch_rviz:=true` above)
```bash
ros2 run rviz2 rviz2 -d src/astar_planner/config/astar_rviz.rviz
```

### Set Start and Goal in RViz2
1. Click **"2D Pose Estimate"** → click on the map to set **start**
2. Click **"2D Goal Pose"** → click on the map to set **goal**
3. Watch the path appear!

### Or publish manually via CLI:
```bash
# Set start
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"}, pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}'

# Set goal
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 8.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}}}'
```

---

## Running on The Construct (app.theconstructsim.com)

1. Open **The Construct** → New ROSject → ROS 2 Humble
2. Clone this repo into `~/ros2_ws/src/`
3. `cd ~/ros2_ws && colcon build && source install/setup.bash`
4. `ros2 launch astar_planner astar_planner_demo.launch.py`
5. Open the **RViz2 tool** in The Construct's Graphical Tools
6. Add displays: Map (`/map`), Path (`/planned_path`), MarkerArray (`/astar_markers`)
7. Use **2D Pose Estimate** and **2D Goal Pose** to set start/goal

---

## Project Structure

```
ros2_ws/
└── src/
    └── astar_planner/
        ├── astar_planner/
        │   ├── __init__.py
        │   ├── astar_planner_node.py   ← Core A* algorithm + ROS 2 node
        │   └── map_publisher_node.py   ← Demo occupancy grid publisher
        ├── launch/
        │   └── astar_planner_demo.launch.py
        ├── config/
        │   └── astar_rviz.rviz
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

---

## References

- [ROS 2 Navigation Concepts](https://docs.nav2.org/)
- [A\* Algorithm — Hart, Nilsson, Raphael (1968)](https://ieeexplore.ieee.org/document/4082128)
- [OccupancyGrid message](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
- Course material: Software Development for Autonomous Vehicles
