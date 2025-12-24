import random
import subprocess
import tempfile
import os


X_MIN, X_MAX = -4.0, 4.0
Y_MIN, Y_MAX = -4.0, 4.0
ROBOT_SAFE_RADIUS = 1.0


BOX_SDF = """
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.8 0.8 0.8</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.8 0.8 0.8</size>
          </box>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def spawn_box(name, x, y):
    sdf_content = BOX_SDF.format(name=name)

    with tempfile.NamedTemporaryFile(delete=False, suffix='.sdf') as f:
        f.write(sdf_content.encode('utf-8'))
        sdf_path = f.name

    subprocess.run([
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', name,
        '-file', sdf_path,
        '-x', str(x),
        '-y', str(y),
        '-z', '0.4'
    ])

    os.remove(sdf_path)


def main():
    print('[Spawner] Generating random obstacles...')

    num = 4
    i = 0

    while i < num:
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)

        # evitar spawnear cerca del robot
        if (x**2 + y**2)**0.5 < ROBOT_SAFE_RADIUS:
            continue

        spawn_box(f'obstacle_{i}', x, y)
        i += 1

    print('[Spawner] Obstacles spawned')


if __name__ == '__main__':
    main()
