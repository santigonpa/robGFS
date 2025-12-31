# Limo Patrol Package

A ROS2 Humble package for autonomous patrol with obstacle avoidance for the Limo robot.

## Overview

This package provides:
- **Waypoint Generator**: Generates random patrol waypoints
- **Waypoint Navigator**: Navigates through waypoints with obstacle avoidance and re-routing
- **Patrol Node**: Legacy patrol node
- **Obstacle Spawner**: Spawns random obstacles in Gazebo simulation

## Architecture

```
┌─────────────────────┐     /patrol_waypoints     ┌─────────────────────┐
│ waypoint_generator  │ ──────────────────────────▶│  waypoint_navigator │
└─────────────────────┘      (PoseArray)          └─────────────────────┘
                                                           │
                                                           │ /cmd_vel
                                                           ▼
                                                    ┌─────────────┐
                                                    │  Limo Robot │
                                                    └─────────────┘
                                                           │
                              /scan, /odom                 │
                        ◀──────────────────────────────────┘
```

## Installation

1. Navigate to your ROS2 workspace and build:
```bash
cd ~/robGFS
colcon build --packages-select limo_patrol
source install/setup.bash
```

## Usage

### Prerequisites

Make sure Gazebo simulation is running with the Limo robot:

```bash
# Terminal 1: Launch Gazebo with Limo
ros2 launch limo_car ackermann_gazebo.launch.py
```

---

## Running Nodes

### Option 1: Navigate Waypoints (Recommended)

Runs waypoint generator + navigator with obstacle avoidance:

```bash
ros2 launch limo_patrol navigate_waypoints.launch.py
```

With custom number of waypoints:
```bash
ros2 launch limo_patrol navigate_waypoints.launch.py num_waypoints:=6
```

### Option 2: Simple Patrol (Legacy)

```bash
ros2 launch limo_patrol patrol.launch.py
```

---

## Running Nodes Individually

### Waypoint Generator

Generates random waypoints and publishes on `/patrol_waypoints`:

```bash
ros2 run limo_patrol waypoint_generator
```

With parameters:
```bash
ros2 run limo_patrol waypoint_generator --ros-args \
    -p num_waypoints:=4 \
    -p x_min:=-2.0 \
    -p x_max:=2.0 \
    -p include_home:=true
```

### Waypoint Navigator

Navigates to waypoints with obstacle avoidance:

```bash
ros2 run limo_patrol waypoint_navigator
```

With parameters:
```bash
ros2 run limo_patrol waypoint_navigator --ros-args \
    -p goal_tolerance:=0.4 \
    -p obstacle_distance:=0.5 \
    -p linear_speed:=0.2 \
    -p angular_speed:=0.4
```

### Obstacle Spawner

Spawns random obstacles in Gazebo:

```bash
ros2 run limo_patrol obstacle_spawner
```

---

## Topics

### Published Topics

| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/patrol_waypoints` | `geometry_msgs/PoseArray` | waypoint_generator | Generated waypoints |
| `/cmd_vel` | `geometry_msgs/Twist` | waypoint_navigator | Velocity commands |

### Subscribed Topics

| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/patrol_waypoints` | `geometry_msgs/PoseArray` | waypoint_navigator | Waypoints to visit |
| `/scan` | `sensor_msgs/LaserScan` | waypoint_navigator | Laser scan for obstacles |
| `/odom` | `nav_msgs/Odometry` | waypoint_navigator | Robot odometry |

---

## Parameters

### Waypoint Generator

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_waypoints` | int | 4 | Number of waypoints |
| `x_min` | float | -2.5 | Minimum X coordinate |
| `x_max` | float | 2.5 | Maximum X coordinate |
| `y_min` | float | -2.5 | Minimum Y coordinate |
| `y_max` | float | 2.5 | Maximum Y coordinate |
| `safe_radius` | float | 0.5 | Radius around origin to avoid |
| `include_home` | bool | true | Add (0,0) as final waypoint |

### Waypoint Navigator

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `goal_tolerance` | float | 0.4 | Distance to consider goal reached |
| `obstacle_distance` | float | 0.5 | Distance to trigger avoidance |
| `linear_speed` | float | 0.2 | Forward speed (m/s) |
| `angular_speed` | float | 0.4 | Turning speed (rad/s) |

---

## Navigation States

The `waypoint_navigator` uses a state machine:

| State | Description |
|-------|-------------|
| `WAITING` | Waiting for waypoints |
| `NAVIGATING` | Moving towards current waypoint |
| `AVOIDING` | Obstacle detected, turning to find clear path |
| `DONE` | All waypoints reached |

---

## Monitoring

```bash
# View waypoints
ros2 topic echo /patrol_waypoints

# View robot commands
ros2 topic echo /cmd_vel

# Check nodes
ros2 node list
```

## License

Apache-2.0
