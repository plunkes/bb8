# bb8_control

Pacote de controle autônomo do robô para a disciplina SSC0712. O robô explora o ambiente seguindo paredes, detecta uma bandeira pela câmera semântica e navega até ela.

## Requisitos

```
ROS 2 Humble
ros-humble-ros-gz-bridge
ros-humble-ros-gz-sim
ros-humble-ign-ros2-control
ros-humble-diff-drive-controller
ros-humble-joint-state-broadcaster
ros-humble-position-controllers
ros-humble-robot-state-publisher
python3-opencv
python3-numpy
ros-humble-cv-bridge
```

Além disso é necessário o pacote prm_2026, uma modificação do [pacote da disciplina](https://github.com/matheusbg8/prm_2026).

Instalar dependências:

```bash
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-ign-ros2-control \
  ros-humble-diff-drive-controller ros-humble-joint-state-broadcaster \
  ros-humble-position-controllers ros-humble-cv-bridge \
  python3-opencv python3-numpy
```

## Compilar

```bash
colcon build --packages-select bb8_control
source install/setup.bash
```

## Rodar

```bash
ros2 launch bb8_control missao_completa_launch.py
```

## Arquitetura e fluxo de controle

O sistema tem dois nodos: `vision_processor` e `controle_robo`.

**vision_processor** lê `/robot_cam/labels_map` (câmera semântica do Gazebo) a cada frame. Para cada pixel, verifica se o label corresponde à bandeira (label 25 = azul). Se a área de pixels detectados for maior que 40, calcula o centroide horizontal e converte para bearing em radianos. Publica em `/vision/flag_detection` (Pose2D: x=centroide, y=área em pixels, theta=1 se detectada) e `/vision/flag_bearing` (Float32).

**controle_robo** roda a 20 Hz e implementa a FSM:

```
SEGUINDO_PAREDE
  ├─ sem parede no LIDAR (> 1.8m)  → avança reto a 0.6 m/s
  ├─ parede à direita detectada     → controle P: ω = -2.2 × (dist_direita - 0.5m)
  ├─ obstáculo frontal < 0.4m      → para e gira (override de segurança)
  ├─ célula visitada ≥ 6 vezes     → EVITANDO_LOOP
  └─ bandeira detectada (≥ 40px)   → DETECTOU_BANDEIRA

DETECTOU_BANDEIRA  (1 tick, apenas para logar a transição)
  └─ sempre → NAVEGANDO_PARA_BANDEIRA

NAVEGANDO_PARA_BANDEIRA
  ├─ bandeira perdida por > 10 ticks → SEGUINDO_PAREDE
  ├─ alinhado (bearing < 0.20 rad) E área ≥ 200px → POSICIONANDO_FINAL
  ├─ obstáculo frontal < 0.65m      → modo contorno: wall-follow no lado mais livre
  │    até o caminho desobstruir, depois retoma navegação direta
  ├─ obstáculo frontal < 0.4m      → para e gira (segurança)
  ├─ não alinhado → gira: ω = 1.8 × bearing
  └─ alinhado, caminho livre → avança a 0.6 m/s

POSICIONANDO_FINAL
  ├─ frente > 0.75m → avança devagar (0.15 m/s) com correção de bearing
  ├─ frente ≤ 0.75m E bandeira detectada E área ≥ 200px → para, estende braço, "Congratulations"
  └─ frente ≤ 0.75m E condição não atendida → era cilindro, volta a NAVEGANDO

EVITANDO_LOOP
  ├─ fase 0: gira esquerda 2.5s
  └─ fase 1: avança reto 3.5s → SEGUINDO_PAREDE
```

O braço fica **retraído** durante toda a navegação e só se estende ao confirmar a missão completa. Posições: retraído `[-1.5, -1.5, 0, 0]`, estendido `[0, 0, 0, 0]`. Ordem: `[gripper_extension, arm_elbow, right_gripper_joint, left_gripper_joint]`.

A posição do robô vem de `/odom_gt`, produzido pelo nodo `ground_truth_odometry` do pacote `prm_2026` a partir do ground truth do Gazebo.

## Tópicos relevantes

| Tópico | Tipo | Direção |
|---|---|---|
| `/scan` | LaserScan | entrada |
| `/odom_gt` | Odometry | entrada |
| `/robot_cam/labels_map` | Image | entrada |
| `/vision/flag_detection` | Pose2D | saída vision |
| `/vision/flag_bearing` | Float32 | saída vision |
| `/cmd_vel` | Twist | saída controle |
| `/gripper_controller/commands` | Float64MultiArray | saída controle |

## Versão com exploração melhorada

Uma branch do robo que utiliza [frontier exploration](https://github.com/plunkes/bb8/tree/exploracao_autonoma) para encontrar a bandeira
