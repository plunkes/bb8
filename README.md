# bb8_control

Pacote ROS 2 (Humble) de **controle autônomo de um robô tipo BB8** para
**exploração de ambiente por fronteiras** e **coleta de uma flag** em simulação
Gazebo (Ignition).

O robô explora autonomamente a arena usando SLAM + Nav2 + exploração de
fronteiras (`m-explore`), detecta a bandeira por segmentação semântica da câmera
e a coleta com um braço/gripper, tudo orquestrado por uma FSM.

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
├── behavior_trees/
│   ├── navigate_no_backup.xml      # exploração/flag: sem ré nem spin (câmera precisa ver a flag)
│   └── navigate_with_backup.xml    # volta à base: ré (BackUp) + spin liberados no recovery
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
# 1. Clonar este pacote
git clone <URL-deste-repo> bb8_control

# 2. Clonar as dependências externas (mundo + exploração)
git clone https://github.com/matheusbg8/prm_2026.git
git clone https://github.com/robo-friends/m-explore-ros2.git

# 3. Instalar dependências ROS
cd ~/prm_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install

# 5. Source
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

## Máquina de estados (FSM)

A FSM (`controle_robo.py`) orquestra toda a missão. Estados e transições:

| Estado | O que faz | Sai para |
|--------|-----------|----------|
| **INICIALIZANDO** | Espera o Nav2 (`navigate_to_pose`) subir e grava a pose de origem (base) no `map`. | EXPLORANDO |
| **EXPLORANDO** | Libera o `explore_lite` (`explore/resume`), que manda goals de fronteira ao Nav2. Vigia a câmera (flag) e a detecção de "encravado". | NAVEGANDO_PARA_BANDEIRA (viu a flag) · BUSCANDO_PAREDE (encravado) |
| **NAVEGANDO_PARA_BANDEIRA** | Pausa o explore; marca a flag no `map` (bearing câmera + range câmera/LIDAR) e manda goals Nav2 p/ perto dela. De LONGE recalcula a marca; PERTO (≤ `vs_dist`) para de recalcular e troca de estado. | POSICIONANDO_FINAL (perto/alinhado) · EXPLORANDO (perdeu a flag) |
| **POSICIONANDO_FINAL** | Captura em 5 fases: recua p/ folga + estende o braço → centra/avança na haste → fecha a garra → ergue a flag → confirma a flag na mão (câmera) e aumenta o footprint (caixa → retângulo). | ALINHANDO_RETORNO (pegou) · NAVEGANDO_PARA_BANDEIRA (falhou, re-aproxima) |
| **ALINHANDO_RETORNO** | Gira NO LUGAR p/ encarar a base, p/ voltar DE FRENTE (rápido) em vez de dar ré o caminho todo. Sem espaço p/ girar o casco grande a tempo → marca a volta com ré liberada (fallback). | RETORNANDO_ORIGEM |
| **RETORNANDO_ORIGEM** | Nav2 (mais rápido) até o standoff da origem. Volta só de frente (ou ré, se não girou). Inflação dos costmaps AUMENTADA (confia no mapa: braço up mascara o LIDAR frontal). BT com recovery de ré/spin. | DEPOSITANDO (chegou) · fim (best effort, esgotou retries) |
| **DEPOSITANDO** | Perto da base: visão da plataforma (label 28) centra + avança devagar; TF até o centro da origem diz quando parar. | SUCESSO |
| **SUCESSO** | No centro: abaixa o braço, abre a garra (solta a flag) e imprime a vitória. | — (missão completa) |
| **BUSCANDO_PAREDE** | Recovery: encravado em área aberta (explore manda goal pro próprio lugar). Avança e desvia p/ o lado mais aberto até achar parede nova p/ mapear. | NAVEGANDO_PARA_BANDEIRA (viu a flag) · EXPLORANDO (esgotou o avanço) |

Fluxo nominal: `INICIALIZANDO → EXPLORANDO → NAVEGANDO_PARA_BANDEIRA → POSICIONANDO_FINAL → ALINHANDO_RETORNO → RETORNANDO_ORIGEM → DEPOSITANDO → SUCESSO`.

O Nav2 é dono do `/cmd_vel` na navegação; nas fases de perto (servo, captura, recovery
manual, giro de alinhamento, depósito) a FSM publica `/cmd_vel` direto. Tuning dos
estados em `config/fsm_params.yaml`.

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
