# bb8_control

Pacote ROS 2 (Humble) de **controle autônomo de um robô tipo BB8** para
**exploração de ambiente por fronteiras** e **coleta de uma flag** em simulação
Gazebo (Ignition).

O robô explora autonomamente a arena usando SLAM + Nav2 + exploração de
fronteiras (`m-explore`), detecta a bandeira por segmentação semântica da câmera
e a coleta com um braço/gripper, tudo orquestrado por uma FSM.

> **Branch:** baseada em `explore_flag` (stack SLAM + Nav2 + m-explore que
> substitui o antigo wall-follower reativo). O pacote é **independente**: contém
> a sua própria descrição do robô (URDF/Xacro), configs de controle, SLAM, Nav2 e
> exploração, e os launch files. Depende do `prm_2026` apenas para o **mundo e
> modelos** do Gazebo, e do `m-explore-ros2` para o nó `explore_lite`.

## Conteúdo do pacote

```
bb8_control/
├── bb8_control/            # nós Python
│   ├── controle_robo.py        # FSM orquestradora
│   ├── vision_processor.py     # detecção da bandeira (segmentação)
│   ├── gripper_server.py       # serviço do gripper / postura do braço
│   ├── scan_masker.py          # mascara setor frontal do LIDAR (/scan_filtered)
│   └── odom_gt_publisher.py    # odometria ground-truth (TF odom->base_link)
├── description/robot.urdf.xacro  # descrição do robô (migrada do prm_2026)
├── config/                       # controller_config, slam_toolbox, nav2, explore
├── behavior_trees/navigate_no_backup.xml
├── rviz/rviz_config.rviz
└── launch/
    ├── explore_and_catch_flag.launch.py   # MISSÃO COMPLETA (entrypoint)
    ├── simulation.launch.py               # Gazebo + mundo (prm_2026)
    ├── spawn_robot.launch.py              # spawn + controladores + pontes + RViz
    └── robot_state_publisher.launch.py    # RSP (descrição local)
```

## Dependências externas (clonar em `src/`)

Além das dependências ROS resolvidas via `rosdep` (Nav2, slam_toolbox,
ros_gz_*, etc.), este pacote precisa de **dois repositórios externos** clonados
no `src/` do workspace:

| Repositório | Para quê | URL |
|-------------|----------|-----|
| `prm_2026` | mundo e modelos do Gazebo | https://github.com/matheusbg8/prm_2026 |
| `m-explore-ros2` | nó `explore_lite` (exploração de fronteiras) | https://github.com/robo-friends/m-explore-ros2 |

## Instalação

```bash
# 1. Criar o workspace e o src/
mkdir -p ~/prm_ws/src
cd ~/prm_ws/src

# 2. Clonar este pacote
git clone <URL-deste-repo> bb8_control

# 3. Clonar as dependências externas (mundo + exploração)
git clone https://github.com/matheusbg8/prm_2026.git
git clone https://github.com/robo-friends/m-explore-ros2.git

# 4. Instalar dependências ROS
cd ~/prm_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --symlink-install

# 6. Source
source install/local_setup.bash
```

## Como executar

```bash
cd ~/prm_ws
colcon build --symlink-install        # build do workspace
source install/local_setup.bash       # source do ambiente

# Missão completa: Gazebo + robô + SLAM + Nav2 + exploração + visão + FSM
ros2 launch bb8_control explore_and_catch_flag.launch.py
```

### Launches auxiliares (debug)

```bash
# Só o simulador + mundo
ros2 launch bb8_control simulation.launch.py world:=arena_cilindros.sdf

# Só o robô (spawn + controladores + pontes + RViz) — exige a simulação rodando
ros2 launch bb8_control spawn_robot.launch.py rviz:=true

# Só o robot_state_publisher (descrição/TFs)
ros2 launch bb8_control robot_state_publisher.launch.py
```

## Arquitetura (resumo)

- **Árvore de TF:** `map -> odom` (slam_toolbox), `odom -> base_link`
  (`odom_gt_publisher`, pose ground-truth do Gazebo), `base_link -> sensores`
  (URDF).
- **Exploração:** `explore_lite` consome o `/map` cru do SLAM e envia goals
  `NavigateToPose` ao Nav2; a FSM pausa/retoma via `explore/resume`.
- **Visão:** `vision_processor` detecta a bandeira na câmera de segmentação
  (`/robot_cam/labels_map`).
- **Coleta:** `gripper_server` + juntas do braço (`shoulder_pitch`, `arm_elbow`,
  `gripper_extension`) no `gripper_controller`.

Veja [`troubleshooting_notes.md`](troubleshooting_notes.md) para caveats de
caminhos, variáveis do Gazebo, pontes e a feature de exploração do m-explore.
