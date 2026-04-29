# limo_patrol - Taller de Robótica

Este repositorio contiene el desarrollo de un sistema de navegación autónoma para un robot móvil realizado en el marco del **Taller de Robótica 2025** del Instituto de Computación (INCO), Facultad de Ingeniería, Universidad de la República (Udelar), Uruguay.

## Descripción del Proyecto

El objetivo principal es implementar una misión de patrulla autónoma donde un robot parte desde una posición inicial (**Home**), recorre una serie de puntos de control (waypoints) generados aleatoriamente dentro de un recinto delimitado y retorna a la base una vez finalizada la tarea. El sistema utiliza un sensor LIDAR para la detección y evasión de obstáculos.

## Estrategias de Navegación

Se implementaron y compararon dos enfoques principales:

1.  **Navegación Reactiva (Máquina de Estados Finitos):**
    * Estructurada en estados: Espera, Navegación, Evasión y Finalización.
    * Utiliza una capa reactiva que detecta obstáculos mediante el LIDAR y ejecuta maniobras de retroceso y giro.
    * Ofrece alta robustez y previsibilidad en entornos complejos.

2.  **Campos de Potencial Artificiales (APF):**
    * Basada en fuerzas virtuales: una fuerza atractiva hacia el objetivo y fuerzas repulsivas desde los obstáculos.
    * Genera trayectorias más fluidas ("smooth motion") y eficientes en tiempo.
    * Sujeta a mínimos locales en configuraciones específicas de obstáculos.

## Estructura del Paquete

* `src/limo_patrol/limo_patrol/patrol_node.py`: Nodo principal que gestiona la lógica de navegación y el bucle de control.
* `src/limo_patrol/limo_patrol/obstacle_spawner.py`: Herramienta para generar obstáculos (conos) de forma aleatoria en el entorno de simulación.
* `src/limo_patrol/limo_patrol/spawn_enclosure.py`: Script para crear recintos perimetrales (circulares o rectangulares) mediante conos.
* `src/limo_patrol/launch/patrol.launch.py`: Archivo de lanzamiento para iniciar el nodo de patrulla.

## Requisitos y Configuración

El proyecto está diseñado para funcionar en un entorno **ROS 2 Humble** con el simulador **Gazebo**.

### Dependencias
* `rclpy`
* `geometry_msgs`
* `sensor_msgs`
* `nav_msgs`

## Evaluación y Resultados

El sistema fue evaluado mediante métricas como el tiempo total de misión, cantidad de maniobras de evasión y suavidad de la trayectoria. Los resultados mostraron que la estrategia de **Campos de Potencial (APF)** redujo significativamente el tiempo de misión (por ejemplo, de 9 a 3 minutos en escenarios de prueba), aunque la **Máquina de Estados** proporcionó una mayor seguridad ante bloqueos inesperados.

---
*Taller de Robótica - 2025 - INCO - Fing - Udelar*
