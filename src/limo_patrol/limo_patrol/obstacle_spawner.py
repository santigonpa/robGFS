import random
import subprocess
import tempfile
import os


X_MIN, X_MAX = -4.0, 4.0
Y_MIN, Y_MAX = -4.0, 4.0
ROBOT_SAFE_RADIUS = 1.0


CONE_SDF = """
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cone>
            <radius>0.25</radius>
            <length>0.7</length>
          </cone>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cone>
            <radius>0.25</radius>
            <length>0.7</length>
          </cone>
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


def spawn_cone(name, x, y):
    sdf_content = CONE_SDF.format(name=name)

    with tempfile.NamedTemporaryFile(delete=False, suffix='.sdf') as f:
        f.write(sdf_content.encode('utf-8'))
        sdf_path = f.name

    subprocess.run([
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', name,
        '-file', sdf_path,
        '-x', str(x),
        '-y', str(y),
        '-z', '0.35'  # mitad de la altura del cono
    ])

    os.remove(sdf_path)


def main():
    print('[Spawner] Generando conos de trafico aleatorios...')

    num = 4
    i = 0

    while i < num:
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)

        if (x**2 + y**2)**0.5 < ROBOT_SAFE_RADIUS:
            continue

        spawn_cone(f'cone_{i}', x, y)
        i += 1

    print('[Spawner] Conos de trafico creados')


if __name__ == '__main__':
    main()

