# Limo Patrol Package

A ROS2 Humble package for autonomous patrol with obstacle avoidance for the Limo robot.

## Overview

This package provides two navigation strategies:

### Strategy 1: State Machine Navigator (`waypoint_navigator`)
- Uses a state machine (WAITING → NAVIGATING → AVOIDING → DONE)
- Reactive obstacle avoidance with wall-following behavior
- Good for structured environments

### Strategy 2: Potential Field Navigator (`potential_field_navigator`)
- Uses Artificial Potential Fields (APF)
- Attractive force towards waypoints + Repulsive force from obstacles
- Smoother trajectories, better for open environments

**Other nodes:**
- **Waypoint Generator**: Generates random patrol waypoints
- **Obstacle Spawner**: Spawns random obstacles in Gazebo simulation

## Architecture

```
┌─────────────────────┐     /patrol_waypoints     ┌─────────────────────────────┐
│ waypoint_generator  │ ──────────────────────────▶│  waypoint_navigator         │
└─────────────────────┘      (PoseArray)          │  - OR -                     │
                                                  │  potential_field_navigator  │
                                                  └─────────────────────────────┘
                                                           │
                                                           │ /cmd_vel
                                                           ▼
                                                    ┌─────────────┐
                                                    │  Limo Robot │
                                                    └─────────────┘
                                                           │
                              /scan, /odometry             │
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
ros2 launch limo_description gazebo_models_diff.launch.py
```

---

## Running Nodes

### Option 1: State Machine Navigator (Recommended for cluttered environments)

Runs waypoint generator + navigator with reactive obstacle avoidance:

```bash
ros2 launch limo_patrol navigate_waypoints.launch.py
```

With custom number of waypoints:
```bash
ros2 launch limo_patrol navigate_waypoints.launch.py num_waypoints:=6
```

### Option 2: Potential Field Navigator (Recommended for open environments)

Runs waypoint generator + potential field navigator with smooth trajectories:

```bash
ros2 launch limo_patrol navigate_pf.launch.py
```

With custom number of waypoints:
```bash
ros2 launch limo_patrol navigate_pf.launch.py num_waypoints:=6
```

### Option 3: Simple Patrol (Legacy)

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

### Potential Field Navigator

Navigates using artificial potential fields:

```bash
ros2 run limo_patrol potential_field_navigator
```

With parameters:
```bash
ros2 run limo_patrol potential_field_navigator --ros-args \
    -p ka:=1.5 \
    -p kr:=0.3 \
    -p obs_radius:=0.8 \
    -p goal_tolerance:=0.3 \
    -p max_linear_speed:=0.35 \
    -p max_angular_speed:=1.0
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
| `/patrol_waypoints` | `geometry_msgs/PoseArray` | waypoint_navigator, potential_field_navigator | Waypoints to visit |
| `/scan` | `sensor_msgs/LaserScan` | waypoint_navigator, potential_field_navigator | Laser scan for obstacles |
| `/odometry` | `nav_msgs/Odometry` | waypoint_navigator, potential_field_navigator | Robot odometry |

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

### Potential Field Navigator

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ka` | float | 1.2 | Attractive force gain (towards goal) |
| `kr` | float | 0.4 | Repulsive force gain (from obstacles) |
| `obs_radius` | float | 1.0 | Obstacle influence radius (meters) |
| `goal_tolerance` | float | 0.3 | Distance to consider goal reached |
| `max_linear_speed` | float | 0.35 | Maximum forward speed (m/s) |
| `max_angular_speed` | float | 1.0 | Maximum turning speed (rad/s) |

---

## Navigation States

### Waypoint Navigator (State Machine)

| State | Description |
|-------|-------------|
| `WAITING` | Waiting for waypoints |
| `NAVIGATING` | Moving towards current waypoint |
| `AVOIDING` | Obstacle detected, turning to find clear path |
| `DONE` | All waypoints reached |

### Potential Field Navigator (APF)

Uses continuous force calculations:
- **Attractive Force**: Pulls robot towards waypoint (proportional to distance)
- **Repulsive Force**: Pushes robot away from obstacles (inverse proportional to distance²)
- **Resultant Force**: Sum of all forces determines velocity commands

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
