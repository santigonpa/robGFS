#!/usr/bin/env python3
"""Spawn an enclosure of cones around a center (circle or rectangle).

Usage examples:
  python3 spawn_enclosure.py --shape circle --radius 3 --n 16
  python3 spawn_enclosure.py --shape rect --width 6 --height 4 --spacing 0.6

You can also integrate this into the ROS2 package (add entry point and colcon build).
"""
import math
import subprocess
import tempfile
import os
import argparse


CONTAINER_SDF = """
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.25</radius>
            <length>0.7</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.25</radius>
            <length>0.7</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 0.5 0 1</ambient>
          <diffuse>1 0.5 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def spawn_entity(name: str, x: float, y: float, z: float = 0.35) -> bool:
    sdf_content = CONTAINER_SDF.format(name=name)
    with tempfile.NamedTemporaryFile(delete=False, suffix='.sdf') as f:
        f.write(sdf_content.encode('utf-8'))
        sdf_path = f.name

    cmd = [
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', name,
        '-file', sdf_path,
        '-x', str(x),
        '-y', str(y),
        '-z', str(z)
    ]
    print(f"[enclosure] Spawning {name} at x={x:.2f}, y={y:.2f}, z={z:.2f}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    os.remove(sdf_path)
    if result.returncode != 0:
        print(f"[enclosure][ERROR] spawn_entity failed for {name}: {result.stderr}")
        return False
    print(f"[enclosure] spawn_entity succeeded for {name}")
    return True


def circle_positions(center_x, center_y, radius, n):
    for i in range(n):
        theta = 2 * math.pi * i / n
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        yield x, y


def rect_positions(center_x, center_y, width, height, spacing):
    # produce points around the rectangle perimeter with given spacing
    w2 = width / 2.0
    h2 = height / 2.0
    # four edges: bottom (left->right), right (bottom->top), top (right->left), left (top->bottom)
    def edge_points(x1, y1, x2, y2, spacing):
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist == 0:
            return
        steps = max(1, int(math.floor(dist / spacing)))
        for i in range(steps):
            t = i / steps
            yield x1 + (x2 - x1) * t, y1 + (y2 - y1) * t

    corners = [
        (center_x - w2, center_y - h2),
        (center_x + w2, center_y - h2),
        (center_x + w2, center_y + h2),
        (center_x - w2, center_y + h2),
    ]

    for a, b in zip(corners, corners[1:] + corners[:1]):
        for p in edge_points(a[0], a[1], b[0], b[1], spacing):
            yield p


def build_enclosure(args):
    center_x = args.center_x
    center_y = args.center_y

    positions = []
    if args.shape == 'circle':
        positions = list(circle_positions(center_x, center_y, args.radius, args.n))
    else:
        positions = list(rect_positions(center_x, center_y, args.width, args.height, args.spacing))

    # Optionally skip positions that fall inside robot safe radius
    final_positions = []
    for i, (x, y) in enumerate(positions):
        # avoid placing cones too close to center if requested
        dx = x - center_x
        dy = y - center_y
        if args.inner_margin and math.hypot(dx, dy) < args.inner_margin:
            continue
        final_positions.append((x, y))

    print(f"[enclosure] Will spawn {len(final_positions)} cones")
    for i, (x, y) in enumerate(final_positions):
        name = f"enclosure_cone_{i}"
        spawn_entity(name, x, y, args.z)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--shape', choices=['circle', 'rect'], default='circle')
    p.add_argument('--center-x', type=float, default=0.0)
    p.add_argument('--center-y', type=float, default=0.0)
    # circle
    p.add_argument('--radius', type=float, default=3.0)
    p.add_argument('--n', type=int, default=16)
    # rect
    p.add_argument('--width', type=float, default=6.0)
    p.add_argument('--height', type=float, default=4.0)
    p.add_argument('--spacing', type=float, default=0.6)
    # other
    p.add_argument('--z', type=float, default=0.35)
    p.add_argument('--inner-margin', type=float, default=0.0,
                   help='do not place cones inside this radius from center')
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()
    build_enclosure(args)
