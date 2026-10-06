# Primer Parcial Práctico: Cinemática Directa e Inversa del Robot UFACTORY xArm 6 con ROS 2 Jazzy

* **Asignatura:** IMT-342 Robótica - Universidad Católica Boliviana "San Pablo"
* **Docente:** Carlos Daniel Aguilar Mancachi
* **Grupo:** 01
* **Autores:** Ivan y Rodrigo
* **Robot asignado:** UFACTORY xArm 6 (6 DOF)

## 1. Objetivo del Trabajo
Modelar la cinemática directa (convención Denavit-Hartenberg estándar) y la cinemática inversa posicional (mediante el Jacobiano geométrico y la pseudoinversa de Moore-Penrose) del manipulador UFACTORY xArm 6, implementando nodos propios en ROS 2 Jazzy (`fk_node` e `ik_node`) y validando su funcionamiento en RViz2.

## 2. Software y Versiones Requeridas
* **Sistema Operativo:** Ubuntu 24.04 LTS
* **Middleware:** ROS 2 Jazzy Jalisco + `rmw_cyclonedds_cpp`
* **Lenguaje y librerías:** Python 3, `numpy`, `rclpy`, `sensor_msgs`, `geometry_msgs`
* **Visualización:** `rviz2`, `robot_state_publisher`, `joint_state_publisher_gui`

## 3. Instrucciones de Clonación e Instalación Reproducible
En una computadora con Ubuntu 24.04 y ROS 2 Jazzy, ejecutar:

```bash
git clone https://github.com/TU_USUARIO/grupo_01_xarm6_ws.git
cd grupo_01_xarm6_ws
chmod +x instalar.sh
./instalar.sh
```

El script `instalar.sh` instala las dependencias del sistema, recupera automáticamente el paquete de descripción oficial `xarm_ros2` (rama `jazzy`) dentro de `src/xarm_ros2` y compila el workspace con `colcon build --symlink-install`.

## 4. Ejecución del Proyecto

### 4.1 Prueba de Cinemática Directa (FK) con Joint State Publisher GUI
**Terminal 1 (Visualización y GUI):**
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
ros2 launch grupo01_xarm6_bringup display.launch.py use_gui:=true
```

**Terminal 2 (Nodo FK):**
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
ros2 run grupo01_xarm6_kinematics fk_node
```

### 4.2 Prueba de Cinemática Inversa (IK) mediante el tópico `/target`
*(Nota: se lanza con `use_gui:=false` para evitar conflicto de múltiples publicadores en `/joint_states`).*

**Terminal 1 (RViz2 sin GUI):**
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
ros2 launch grupo01_xarm6_bringup display.launch.py use_gui:=false
```

**Terminal 2 (Nodo IK):**
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
ros2 run grupo01_xarm6_kinematics ik_node
```

**Terminal 3 (Nodo FK para verificación numérica):**
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
ros2 run grupo01_xarm6_kinematics fk_node
```

**Terminal 4 (Enviar objetivos cartesianos alcanzables):**
```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Caso 1:
ros2 topic pub /target geometry_msgs/msg/Point "{x: 0.35, y: 0.15, z: 0.30}" --once

# Caso 2:
ros2 topic pub /target geometry_msgs/msg/Point "{x: 0.40, y: -0.20, z: 0.45}" --once

# Caso 3:
ros2 topic pub /target geometry_msgs/msg/Point "{x: 0.25, y: 0.25, z: 0.20}" --once
```

## 5. Tópicos y Estructura del Repositorio
* **Tópicos utilizados:** `/target` (`geometry_msgs/msg/Point`) y `/joint_states` (`sensor_msgs/msg/JointState`).
* **Estructura:**
  * `src/grupo01_xarm6_bringup/`: Launch file `display.launch.py`.
  * `src/grupo01_xarm6_kinematics/`: Nodos `fk_node.py` e `ik_node.py`.
  * `dependencias.repos` e `instalar.sh`: Recuperación del modelo oficial y compilación.
  * `docs/`: Informe final en PDF.
