# Proyecto ABP: Detección y Mapeo Georreferenciado de Residuos mediante Enjambre UAV

Este repositorio contiene el desarrollo del proyecto de Aprendizaje Basado en Problemas (ABP) para la asignatura de **Sistemas Multirobot** del Grado en Ingeniería Robótica. 

El objetivo del proyecto es simular un enjambre descentralizado de drones (UAVs) que coordinan sus trayectorias para explorar y detectar de forma automática residuos en la playa de una simulación en Gazebo, generando finalmente un mapa global con la localización GPS exacta de cada residuo.

---

## Estructura del Repositorio

* **`src/`**: Carpeta principal con los paquetes de ROS 2:
  * **`uav_description/`**: Modelos modularizados en XACRO/URDF con las físicas del dron y sus respectivos sensores (Cámara RGB, Lidar, Altímetro, GPS, IMU).
  * **`uav_gazebo/`**: Escenarios de simulación (playa exterior con residuos fotorrealistas en 3D) y archivos de lanzamiento en Gazebo Harmonic.
  * **`uav_control/`**: Rutina de barrido *lawnmower* coordinado con máquina de estados y protocolo de sincronización por red sin controlador central.
  * **`uav_vision/`**: Inferencia visual YOLOv8, lógica de georreferenciación (traducción píxel a coordenadas GPS) y consolidación global en el mapa final.
* **`img/`**: Imágenes y diagramas auxiliares de la memoria.
* **`memoria.tex` / `memoria.pdf`**: Documentación técnica y memoria del proyecto en LaTeX y PDF compilado.
* **`presentacion.pdf`**: Presentación de diapositivas del proyecto.

---

## Requisitos del Sistema

El desarrollo y pruebas se han realizado en **Ubuntu 24.04 LTS** con las siguientes herramientas:
* **ROS 2** Jazzy Jalisco
* **Gazebo** Harmonic
* **RViz2** (para visualización cinemática)
* **Python 3** con librerías:
  * `ultralytics` (YOLOv8)
  * `opencv-python`
  * `matplotlib`

---

## Manual de Ejecución

Para desplegar y arrancar la simulación completa, compila el espacio de trabajo y ejecuta las siguientes instrucciones en terminales independientes:

### 1. Preparación del Entorno
Desde el directorio del workspace:
```bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Terminal 1: Lanzar la simulación física (Gazebo)
Instancia el mundo costero y despliega los drones (por defecto `robots:=3`):
```bash
ros2 launch uav_gazebo multi_uav.launch.py robots:=3
```

### 3. Terminal 2: Inferencia de Visión y Mapeo
Inicia los nodos de detección de YOLOv8, cálculo GPS de residuos y actualización del mapa. Se recomienda desactivar la visualización por pantalla para optimizar recursos:
```bash
ros2 launch uav_vision multi_vision.launch.py robots:=3 show_gui:=false
```

### 4. Terminal 3: Patrulla Sincronizada
Ordena el despegue e inicio simultáneo del barrido de las franjas de la costa de manera coordinada:
```bash
ros2 launch uav_control multi_patrol.launch.py robots:=3
```

El enjambre guardará y actualizará cada 10 segundos las detecciones consolidadas (eliminando duplicados mediante la fórmula del Haversine) en los archivos `mapa_residuos.csv` y `mapa_residuos.png` generados en la raíz del workspace.

---

## Autores
* **Hugo Sevilla Martínez** (GitHub: [@Eugegeuge](https://github.com/Eugegeuge))
* **Hugo López Pastor**
