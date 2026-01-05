# Limo Patrol

Paquete ROS2 para patrullaje autónomo con evasión de obstáculos.

## Instalación

```bash
cd ~/robGFS
colcon build --packages-select limo_patrol
source install/setup.bash
```

## Uso

### 1. Primero lanzar Gazebo con el robot:
```bash
ros2 launch limo_description gazebo_models_diff.launch.py
```

### 2. Ejecutar navegación (elegir una opción):

| Comando | Descripción |
|---------|-------------|
| `ros2 launch limo_patrol navigate_waypoints.launch.py` | Navegación con evasión reactiva (máquina de estados) |
| `ros2 launch limo_patrol navigate_pf.launch.py` | Navegación con campos potenciales (trayectorias suaves) |
| `ros2 launch limo_patrol patrol.launch.py` | Patrullaje simple |

Parámetro opcional: `num_waypoints:=6`

## Nodos individuales

| Nodo | Comando | Descripción |
|------|---------|-------------|
| Generador de waypoints | `ros2 run limo_patrol waypoint_generator` | Genera puntos de patrullaje aleatorios |
| Navegador reactivo | `ros2 run limo_patrol waypoint_navigator` | Navega evadiendo obstáculos con giros |
| Navegador APF | `ros2 run limo_patrol potential_field_navigator` | Navega usando campos potenciales |
| Spawner de obstáculos | `ros2 run limo_patrol obstacle_spawner` | Genera obstáculos en Gazebo |

## Monitoreo

```bash
ros2 topic echo /patrol_waypoints   # Ver waypoints
ros2 topic echo /cmd_vel            # Ver comandos de velocidad
```

