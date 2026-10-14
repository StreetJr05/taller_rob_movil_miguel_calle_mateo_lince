# Robot diferencial en Gazebo — `pkg_rob_dif`

Robot móvil de tracción diferencial con LiDAR 2D y cámara, simulado en Gazebo Harmonic con ROS 2 Jazzy, que recorre una pista propia con `teleop_twist_keyboard`. Incluye un nodo propio de cinemática de las llantas que integra la pose a partir de `/joint_states` y la compara con la odometría del plugin DiffDrive y con la pose real del simulador.

## Dependencias

- Ubuntu 24.04
- ROS 2 Jazzy (`ros-jazzy-desktop`)
- Gazebo Harmonic e integración ROS–Gazebo:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                 ros-jazzy-teleop-twist-keyboard
```

O, desde la raíz del workspace, con `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -y
```

## Compilación

```bash
git clone <URL_DEL_REPOSITORIO> ws_rob_dif_gaz
cd ws_rob_dif_gaz
source /opt/ros/jazzy/setup.bash
colcon build
```

## Ejecución

**Terminal 1** — simulación completa (Gazebo, bridge, robot_state_publisher, spawn, nodo de cinemática y RViz2):

```bash
cd ws_rob_dif_gaz
source install/setup.bash
ros2 launch pkg_rob_dif rob_dif.launch.py
```

**Terminal 2** — teleoperación (esta terminal debe tener el foco del teclado):

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Estructura del paquete

| Carpeta | Contenido |
|---|---|
| `urdf/rob_dif.urdf.xacro` | Modelo parametrizado: chasis, dos ruedas motrices, caster, LiDAR, cámara y plugins de Gazebo (DiffDrive, JointStatePublisher, OdometryPublisher) |
| `worlds/pista.sdf` | Pista: curvas, chicane, estrechamiento de 0.75 m, 6 obstáculos y muros con colisión |
| `config/bridge.yaml` | Topics puenteados entre Gazebo y ROS 2 |
| `scripts/cinematica_llantas.py` | Nodo propio de cinemática de las llantas |
| `launch/rob_dif.launch.py` | Launch único de todo el sistema |
| `rviz/rob_dif.rviz` | Configuración de RViz2 (modelo, TF, LaserScan, odometría, cámara) |

## Topics

| Topic ROS 2 | Tipo | Dirección | Origen |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | teleop → DiffDrive |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS | DiffDrive |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo → ROS | DiffDrive (`odom → base_footprint`) |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → ROS | JointStatePublisher |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS | LiDAR (`lidar_link`) |
| `/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | Cámara (`camera_optical_frame`) |
| `/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → ROS | Cámara (`camera_optical_frame`) |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS | Tiempo de simulación |
| `/ground_truth` | `nav_msgs/Odometry` | Gazebo → ROS | Pose real (OdometryPublisher), solo para evaluación |
| `/odom_cinematica` | `nav_msgs/Odometry` | ROS | Nodo `cinematica_llantas` |
| `/cinematica/error_odom` | `geometry_msgs/Vector3Stamped` | ROS | Error (x, y, θ) nodo propio vs `/odom` |
| `/cinematica/error_gt` | `geometry_msgs/Vector3Stamped` | ROS | Error (x, y, θ) nodo propio vs pose real |

## Nodo de cinemática de las llantas

Lee las velocidades angulares de las ruedas `w = [wD, wI]ᵀ` de `/joint_states`, calcula `u = [v, ω]ᵀ = J·w`, luego `q̇ = S(θ)·u` e integra la pose `q = [x, y, θ]ᵀ`. Cada 5 s imprime el error RMS y máximo frente a `/odom` y frente a la pose real.

Para guardar los datos en CSV (útil para gráficas del reporte), agregue el parámetro `csv_path` al nodo en el launch, por ejemplo `'csv_path': '/tmp/cinematica.csv'`.

Para graficar el error en vivo:

```bash
ros2 run rqt_plot rqt_plot /cinematica/error_odom/vector/x /cinematica/error_odom/vector/y /cinematica/error_odom/vector/z
```

## Comprobaciones útiles

```bash
ros2 topic list
ros2 topic hz /scan                 # ~10 Hz
ros2 run tf2_tools view_frames      # genera un PDF del árbol TF
ros2 run rqt_image_view rqt_image_view /image_raw
```
