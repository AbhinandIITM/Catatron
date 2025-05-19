## Configurar espacio de trabajo

```bash
    mkdir -p ~/catatron_ws/src
    cd  ~/catatron_ws/src
    git clone https://github.com/morg1207/catatron_ros2.git
```

## Descargar dependencias y compilar

```bash
    cd  ~/catatron_ws
    rosdep init
    sudo apt update
    rosdep update --rosdistro $ROS_DISTRO
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
    colcon build --symlink-install

    sudo apt install pip -y
    pip install pynput 
```

## Ejecutar simulación

Terminal 1
```bash
    cd  ~/catatron_ws/
    source install/setup.bash
    ros2 launch catatron_gazebo catatron_gazebo.launch.py
```
Terminal 2
```bash
    cd  ~/catatron_ws/
    source install/setup.bash
    ros2 run catatron_joystick keyboard_sim_joy
```


Aquí está la documentación organizada y mejorada para tu proyecto, lista para incluirse en tu repositorio de Git:

---

## **Joystick Simulado para ROS2**

Este nodo permite simular un joystick utilizando un teclado para enviar comandos a un tópico `/joy`, emulando un dispositivo físico. Este controlador puede usarse para manipular robots o sistemas que acepten mensajes del tipo `sensor_msgs/Joy`.

---

### **Configuración de Controles**

#### **Botones (Buttons)**
| Número de Botón | Tipo de Control | Descripción               |
|------------------|-----------------|---------------------------|
| `button[0]`      | Rest            | Reinicia el estado del robot. |
| `button[1]`      | Trot            | Control en modo trote.         |
| `button[2]`      | Crawl           | Control en modo crawl.         |
| `button[3]`      | Stand           | Control para posición de pie.  |

#### **Ejes (Axes)**
Los ejes controlan distintas funcionalidades dependiendo del modo activo.

- **Modo Rest**:
  - `axis[7]`: Control de posición del cuerpo.
  - `axis[6]`: Control de posición del cuerpo.
  - `axis[1]`: Control de posición del cuerpo.
  - `axis[0]`: Control de orientación.
  - `axis[4]`: Control de orientación.
  - `axis[3]`: Control de orientación.

- **Modo Trot**:
  - `axis[4]`: Velocidad en el eje **X**.
  - `axis[3]`: Velocidad en el eje **Y**.
  - `axis[0]`: Velocidad en el eje de giro (**yaw**).

- **Modo Crawl**:
  - `axis[4]`: Velocidad en el eje **X**.
  - `axis[0]`: Velocidad en el eje de giro (**yaw**).

- **Modo Stand**:
  - `axis[7]`: Control de posición.
  - `axis[1]`: 
  - `axis[0]`: 
  - `axis[4]`: 
  - `axis[3]`: 

---

### **Asignación de Teclas**

#### **Botones**
| Botón     | Tecla |
|-----------|-------|
| `button[0]` | `"1"` |
| `button[1]` | `"2"` |
| `button[2]` | `"3"` |
| `button[3]` | `"4"` |
| `button[4]` | `"5"` |
| `button[5]` | `"6"` |

#### **Ejes**
| Eje         | Decrementar | Incrementar |
|-------------|-------------|-------------|
| `axis[0]`   | `"j"`       | `"l"`       |
| `axis[1]`   | `","`       | `"i"`       |
| `axis[2]`   | `"m"`       | `"u"`       |
| `axis[3]`   | `"a"`       | `"d"`       |
| `axis[4]`   | `"x"`       | `"w"`       |
| `axis[5]`   | `"h"`       | `"y"`       |
| `axis[6]`   | `"g"`       | `"t"`       |
| `axis[7]`   | `"f"`       | `"r"`       |

---

