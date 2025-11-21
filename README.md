

# DQN con Aceleración GPU (CUDA) integrado a Webots

## 1. Descripción general

Este proyecto implementa un **sistema de Aprendizaje por Refuerzo Profundo (Deep Q-Learning, DQN)** para el control de un robot móvil **e-puck** en el simulador **Webots**, incorporando **aceleración por GPU mediante CUDA** a través de **PyTorch**.

El sistema se diseña bajo una **arquitectura desacoplada**, donde:

* Webots ejecuta la simulación física, sensores y actuadores.
* Un proceso externo (Learner) se encarga del aprendizaje y la inferencia del modelo DQN utilizando GPU.
* Ambos procesos se comunican mediante sockets TCP, permitiendo entrenar en GPU sin afectar el tiempo real de la simulación.

Este enfoque permite escalar el entrenamiento, mejorar el rendimiento y evitar las limitaciones de ejecutar aprendizaje intensivo dentro del controlador de Webots.

---

## 2. Arquitectura del sistema

### 2.1 Componentes principales

* **Controller Webots (Python)**

  * Lectura de sensores de proximidad.
  * Construcción del estado del entorno.
  * Ejecución de acciones sobre los motores.
  * Cálculo de recompensa y detección de objetivo.
  * Envío de estados y transiciones al Learner.

* **Learner Externo (PyTorch + CUDA)**

  * Implementación del DQN (red online y red objetivo).
  * Gestión del Replay Buffer.
  * Política ε-greedy.
  * Entrenamiento del modelo en GPU.
  * Selección de acciones.

* **Canal de Comunicación (TCP/IP)**

  * Protocolo binario ligero y eficiente.
  * Mensajes estructurados para interacción agente–entorno.

---

## 3. Protocolo de comunicación

La comunicación entre Webots y el Learner se realiza mediante mensajes TCP estructurados.

### 3.1 Tipos de mensajes

| Tipo       | Código | Descripción                             |
| ---------- | ------ | --------------------------------------- |
| REQ_ACT    | 1      | Solicitud de acción desde Webots        |
| TRANSITION | 2      | Envío de transición (s, a, r, s', done) |
| RESET      | 3      | Reinicio del episodio (opcional)        |

### 3.2 Formato general del mensaje

```
[msg_type (1 byte) | payload_length (4 bytes) | payload]
```

### 3.3 Formato de transición

```
s (5 × float32)
a (uint32)
r (float32)
s' (5 × float32)
done (uint8)
=======
# Light Detection and Ranging (LiDAR)

## 1. Introducción
El presente laboratorio tiene como propósito la implementación de un sistema de escaneo y mapeo 2D utilizando un sensor **LiDAR (Light Detection and Ranging)** dentro del entorno de simulación **Webots**, en conjunto con **ROS 2 Humble** y el paquete **Cartographer**.  
El objetivo principal es adquirir mediciones de distancia mediante un sensor láser, procesarlas para detectar obstáculos y generar un mapa de ocupación bidimensional del entorno simulado.

El experimento permite comprender los fundamentos del mapeo simultáneo (**SLAM, Simultaneous Localization and Mapping**) y la interacción entre los tópicos de ROS relacionados con sensado, odometría y publicación de mapas.

---

## 2. Objetivos

- Comprender el principio de funcionamiento de los sensores LiDAR.
- Configurar un entorno de simulación en **Webots** integrado con **ROS 2**.
- Utilizar el paquete **Cartographer** para la construcción de mapas 2D en tiempo real.
- Evaluar la calidad del mapa generado mediante exploración teleoperada.
- Guardar y documentar el mapa resultante en formato estándar de ROS (`.pgm` y `.yaml`).

---

## 3. Entorno de desarrollo

**Hardware**
- CPU con soporte para virtualización
- Windows 10/11
- WSL 2 habilitado
- Conexión a Internet estable

**Software**
- Ubuntu 22.04 (WSL 2)
- ROS 2 Humble Hawksbill
- Webots R2025a o superior
- Cartographer ROS 2
- Nav2 Map Server
- `colcon`, `rosdep`, `git`, `curl`, `pip`

---

## 4. Instalación del entorno

### 4.1. Configuración inicial de Ubuntu 22.04 en WSL 2
```bash
wsl --install -d Ubuntu-22.04
wsl --set-default Ubuntu-22.04
````

### 4.2. Preparación de paquetes base

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common curl git python3-pip -y
sudo add-apt-repository universe
sudo apt update
pip install -U colcon-common-extensions rosdep
export PATH="$HOME/.local/bin:$PATH"
sudo rosdep init || true
rosdep update
```

### 4.3. Instalación de ROS 2 Humble Desktop

```bash
sudo mkdir -p /usr/share/keyrings
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://packages.ros.org/ubuntu $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verificación:

```bash
ros2 run demo_nodes_cpp talker
```

### 4.4. Instalación de Webots y bridge ROS 2

Webots se instala desde Windows:
[https://cyberbotics.com](https://cyberbotics.com/#download)

Luego, en Ubuntu:

```bash
mkdir -p ~/webots_ws/src
cd ~/webots_ws/src
git clone https://github.com/cyberbotics/webots_ros2.git
cd ~/webots_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### 4.5. Instalación de Cartographer y herramientas de mapeo

```bash
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros \
                 ros-humble-slam-toolbox ros-humble-nav2-map-server -y
>>>>>>> 922d7c3 (Update README with LiDAR mapping system details)
```

---

<<<<<<< HEAD
## 4. Representación del estado

El estado del entorno se define como un vector continuo de dimensión 5:

| Componente | Descripción                        |
| ---------- | ---------------------------------- |
| s_left     | Obstáculo detectado a la izquierda |
| s_front    | Obstáculo frontal                  |
| s_right    | Obstáculo a la derecha             |
| s_corner   | Obstáculo en esquina               |
| dist_norm  | Distancia normalizada al objetivo  |

```
state = [s_left, s_front, s_right, s_corner, dist_norm]
```

---

## 5. Espacio de acciones

El agente dispone de un conjunto discreto de acciones:

| Acción  | Código | Descripción          |
| ------- | ------ | -------------------- |
| FORWARD | 0      | Avanzar recto        |
| LEFT    | 1      | Girar a la izquierda |
| RIGHT   | 2      | Girar a la derecha   |

---

## 6. Función de recompensa

La función de recompensa está diseñada para incentivar el acercamiento al objetivo y la evitación de obstáculos.

### 6.1 Reglas de recompensa

| Evento                             | Recompensa |
| ---------------------------------- | ---------- |
| Paso normal                        | −0.01      |
| Obstáculo frontal                  | −1.5       |
| Obstáculo lateral                  | −0.3       |
| Obstáculo en esquina               | −1.0       |
| Reducción de distancia al objetivo | +1.2       |
| Contacto con la meta               | +20.0      |

---

## 7. Red neuronal DQN

La red neuronal utilizada sigue la arquitectura:

```
Entrada (5)
 → Capa densa (32)
 → ReLU
 → Capa densa (3)
```

Se emplean dos redes:

* **Red Online**: se actualiza en cada paso de entrenamiento.
* **Red Objetivo (Target Network)**: se actualiza periódicamente para estabilizar el aprendizaje.

---

## 8. Entrenamiento con CUDA

El entrenamiento del agente se realiza utilizando PyTorch con soporte CUDA, lo que permite:

* Backpropagation acelerado por GPU.
* Inferencia rápida de acciones.
* Procesamiento eficiente de lotes de experiencias.

La selección del dispositivo es automática:

```python
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
```

---

## 9. Mecanismo anti-atasco

Para evitar estados de bloqueo del robot:

* Se monitorea la mejora de la distancia al objetivo.
* Si no se detecta progreso durante un número determinado de pasos:

  * Se activa un modo de escape con giros forzados.
  * Se reinician métricas internas de seguimiento.

Este mecanismo mejora la exploración del entorno y la estabilidad del aprendizaje.

---

## 10. Ejecución del sistema

### 10.1 Requisitos

* Webots (R2023 o superior)
* Python 3.8 o superior
* PyTorch con soporte CUDA
* GPU NVIDIA con drivers CUDA correctamente instalados

### 10.2 Pasos de ejecución

1. Ejecutar el Learner:

   ```bash
   python learner_cuda.py
   ```

2. Iniciar la simulación en Webots:

   * Asignar `controller_webots.py` como controlador del robot.
   * Ejecutar la simulación.

3. Verificar en consola:

   ```
   Using device: cuda
   ```


---

## 11. Explicación del código

Esta sección describe la estructura y el funcionamiento del código desarrollado, diferenciando claramente las responsabilidades del **controller de Webots** y del **Learner externo con aceleración GPU (CUDA)**.

---

### 11.1 Controller Webots (`controller_webots.py`)

El controller de Webots cumple el rol de **interfaz entre el entorno físico simulado y el agente de aprendizaje**. No realiza entrenamiento ni inferencia pesada, lo que garantiza estabilidad temporal en la simulación.

#### 11.1.1 Inicialización del entorno

En esta etapa se inicializan:

* El objeto `Supervisor` de Webots.
* Los motores de las ruedas en modo de velocidad.
* Los sensores de proximidad (`ps0` a `ps7`).
* Los nodos del robot y del objetivo (definido como `TARGET`).
* El cálculo del tamaño del bounding box del objetivo para la detección de contacto.

Esta información es necesaria para construir el estado del entorno y evaluar cuándo la meta ha sido alcanzada.

---

#### 11.1.2 Lectura de sensores y construcción del estado

La función `sense()` agrupa las lecturas de los sensores de proximidad y las discretiza según umbrales definidos:

* Detección de obstáculos a la izquierda, frente y derecha.
* Identificación de situaciones de esquina.

Posteriormente, la función `build_state()` combina esta información discreta con la **distancia normalizada al objetivo**, generando un vector de estado continuo de dimensión 5:

```
[s_left, s_front, s_right, s_corner, dist_norm]
```

Este vector es el que se envía al Learner para la selección de la acción.

---

#### 11.1.3 Comunicación con el Learner

El controller establece una conexión TCP con el proceso externo de aprendizaje.
En cada iteración del bucle principal:

1. Se envía el estado actual mediante un mensaje `REQ_ACT`.
2. Se recibe la acción seleccionada por el modelo DQN.
3. Se ejecuta la acción sobre los motores del robot.

Esta separación permite que el proceso de inferencia se ejecute en GPU sin afectar el ciclo de simulación.

---

#### 11.1.4 Ejecución de acciones

La función `act(action)` traduce las acciones discretas en comandos de velocidad para las ruedas:

* Avance recto.
* Giro a la izquierda.
* Giro a la derecha.

Este esquema simplifica el espacio de control y facilita el aprendizaje del agente.

---

#### 11.1.5 Cálculo de recompensa y detección de meta

Después de ejecutar la acción, el controller:

* Calcula el nuevo estado.
* Evalúa la recompensa mediante la función `compute_reward()`.

La recompensa considera:

* Penalizaciones por cercanía a obstáculos.
* Incentivos por reducción de distancia al objetivo.
* Recompensa elevada al detectar el **primer contacto válido con la meta**, usando un chequeo geométrico entre el robot y el bounding box del objetivo.

Cuando se alcanza la meta, se marca el episodio como finalizado (`done = True`).

---

#### 11.1.6 Envío de transiciones

Cada transición `(s, a, r, s', done)` se envía al Learner mediante un mensaje `TRANSITION`.
El controller **no almacena ni procesa** estas transiciones, delegando completamente el aprendizaje al proceso externo.

---

#### 11.1.7 Mecanismo anti-atasco

El controller incluye un mecanismo para detectar estados de bloqueo:

* Se monitorea la mejora de la distancia al objetivo.
* Si no hay progreso durante varios pasos consecutivos:

  * Se activa un modo de escape basado en giros forzados.
  * Se evita enviar acciones al Learner durante este periodo.

Este mecanismo mejora la exploración y evita episodios estancados.

---

### 11.2 Learner Externo con CUDA (`learner_cuda.py`)

El Learner externo implementa el **agente DQN completo**, ejecutando tanto la inferencia como el entrenamiento en GPU.

---

#### 11.2.1 Inicialización y selección de dispositivo

Al iniciar el proceso, el Learner:

* Detecta automáticamente la disponibilidad de CUDA.
* Carga el modelo DQN en GPU cuando está disponible.
* Inicializa la red online, la red objetivo y el optimizador.

Esto garantiza un uso eficiente de los recursos computacionales.

---

#### 11.2.2 Arquitectura del modelo

El modelo DQN está compuesto por:

* Una capa de entrada de 5 neuronas.
* Una capa oculta de 32 neuronas con activación ReLU.
* Una capa de salida con 3 valores Q, uno por cada acción posible.

Esta arquitectura equilibra simplicidad y capacidad de representación.

---

#### 11.2.3 Política de selección de acciones

La selección de acciones se realiza mediante una política **ε-greedy**:

* Con probabilidad ε se selecciona una acción aleatoria (exploración).
* Con probabilidad 1 − ε se selecciona la acción con mayor valor Q (explotación).

El valor de ε decae progresivamente a lo largo del entrenamiento.

---

#### 11.2.4 Replay Buffer

El Learner mantiene un buffer de experiencias con capacidad limitada.
Las transiciones recibidas desde Webots se almacenan en este buffer y se muestrean aleatoriamente durante el entrenamiento, reduciendo la correlación temporal entre muestras.

---

#### 11.2.5 Entrenamiento del DQN

El entrenamiento sigue el esquema clásico de DQN:

1. Se muestrea un batch de transiciones.
2. Se calculan los valores Q actuales con la red online.
3. Se estiman los valores objetivo usando la red target.
4. Se minimiza el error cuadrático medio entre Q estimado y objetivo.
5. Se actualizan los pesos mediante backpropagation en GPU.

La red objetivo se actualiza periódicamente para mejorar la estabilidad.

---

#### 11.2.6 Actualización de la red objetivo y decaimiento de ε

Cada cierto número de pasos:

* La red target copia los pesos de la red online.
* El parámetro ε se reduce gradualmente hasta un valor mínimo.

Estos mecanismos ayudan a estabilizar el proceso de aprendizaje.

---

### 11.3 Flujo general de ejecución

El flujo completo del sistema es el siguiente:

1. Webots genera el estado actual del entorno.
2. El estado se envía al Learner.
3. El Learner selecciona la acción usando el DQN en GPU.
4. Webots ejecuta la acción y calcula la recompensa.
5. La transición se envía al Learner para entrenamiento.
6. El ciclo se repite hasta alcanzar la meta.


---

## 12. Conclusión

El proyecto presenta una integración eficiente entre robótica móvil, aprendizaje por refuerzo profundo y aceleración por GPU mediante CUDA. La arquitectura desacoplada permite entrenar modelos complejos de forma estable y escalable, manteniendo el control en tiempo real del simulador Webots. Esta solución resulta adecuada para contextos académicos y de investigación experimental en sistemas autónomos.
=======
## 5. Procedimiento experimental

### 5.1. Ejecución de simulación con LiDAR

En una terminal:

```bash
ros2 launch webots_ros2_husarion rosbot2r_webots.launch.py
```

Esto abre Webots con el modelo **ROSbot 2R**, el cual incluye un sensor LiDAR y odometría.

### 5.2. Lanzamiento del mapeo 2D (Cartographer)

En una segunda terminal:

```bash
cd ~/webots_ws
source install/setup.bash
ros2 launch cartographer_ros cartographer.launch.py use_sim_time:=true
```

### 5.3. Control manual del robot

En una tercera terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Mover el robot a lo largo del entorno hasta cubrir todas las paredes y obstáculos visibles.
Cartographer construirá el mapa en tiempo real a partir del tópico `/scan`.

### 5.4. Almacenamiento del mapa

Cuando el área esté completamente explorada:

```bash
ros2 run nav2_map_server map_saver_cli -f mapa_webots
```

Archivos generados:

```
mapa_webots.pgm
mapa_webots.yaml
```

Estos describen la **ocupación** (0–100 %) de cada celda del entorno en coordenadas cartesianas 2D.

---

## 6. Resultados 

* Publicación continua del tópico `/map` (`nav_msgs/OccupancyGrid`) en ROS 2.
* Construcción dinámica de un mapa con resolución definida por el parámetro `resolution` de Cartographer.
* Generación correcta de los archivos `mapa_webots.pgm` y `mapa_webots.yaml`.
* Visualización del mapa en RViz 2 mediante:

  ```bash
  rviz2
  ```

  y agregando las visualizaciones:

  * `Map`  → `/map`
  * `LaserScan` → `/scan`
  * `TF` → transformaciones del robot

---

## 7. Análisis técnico

El sensor LiDAR empleado en Webots mide distancias mediante la emisión de pulsos láser y el cálculo del tiempo de vuelo (ToF).
Los datos se publican como un `sensor_msgs/LaserScan` con resolución angular fija.
El nodo de **Cartographer** realiza:

1. **Scan matching:** correlación entre mediciones sucesivas para estimar desplazamiento relativo.
2. **Pose graph optimization:** minimización global del error acumulado.
3. **Occupancy grid mapping:** actualización probabilística de celdas mediante log-odds.

La exploración debe cubrir completamente las fronteras del entorno para obtener un mapa cerrado y coherente.
En caso de ruido o superposición de escaneos, el ajuste de parámetros como `scan_matcher.translation_weight` y `resolution` permite mejorar la consistencia del mapa.

---

## 8. Conclusiones

* Se logró la integración de **Webots + ROS 2** para la adquisición de datos LiDAR y la generación de mapas de ocupación 2D.
* El paquete **Cartographer** demostró un rendimiento estable en WSL 2, permitiendo la creación de mapas precisos en tiempo real.
* El uso de herramientas estándar de ROS 2 facilita la exportación, análisis y reutilización de mapas para navegación autónoma.
* La metodología puede extenderse a SLAM 3D mediante la incorporación de sensores multieje o cámaras de profundidad.

---

## 9. Referencias

* [Webots ROS 2 Integration](https://github.com/cyberbotics/webots_ros2)
* [ROS 2 Documentation – Humble Hawksbill](https://docs.ros.org/en/humble/)
* [Cartographer ROS 2](https://github.com/cartographer-project/cartographer_ros)
* [Husarion ROSbot 2R Simulation](https://github.com/husarion/webots_ros2_husarion)
* [Nav2 Map Server](https://navigation.ros.org/)
