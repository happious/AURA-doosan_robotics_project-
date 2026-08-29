# AURA — 공항형 협동 AMR 시스템

TurtleBot4 2대(**robot1**, **robot3**)를 중앙 노드로 조율하여 **가이드 · 러기지 어시스트 ·
순찰 · 응급(AED) 대응**을 수행하는 ROS2 기반 협동 로봇 시스템. 고정 CCTV가 쓰러짐/혼잡/인원수를
감지하고, 관리자·승객용 Flask UI가 분리되어 있다.

- **robot1** — 응급/AED 대응. 중앙과 액션·서비스로 **완전 연결**.
- **robot3** — 가이드/러기지 어시스트. 로컬 인지·모션 노드로 동작하며 중앙과 **부분 연결**.

전체 구조와 토픽 연결은 [`docs/architecture.md`](docs/architecture.md),
[`docs/topics.md`](docs/topics.md) 참고.

---

## 리포 구조

```
aura_ws/
├── src/
│   ├── fleet_interfaces/     # 공용 msg/srv/action (ament_cmake)
│   ├── fleet_central/        # 중앙: dispatcher · db_manager · crowd_keepout
│   ├── robot1_control/       # robot1 미션 FSM (완전 연결)
│   ├── robot3_control/       # robot3 로컬 자율 스택 (부분 연결)
│   └── cctv_perception/      # CCTV 쓰러짐/혼잡/인원수 감지
├── ui/
│   ├── admin_ui/             # 관제 대시보드 (Flask, 7000)
│   └── passenger_ui/         # 승객 키오스크 (Flask, AMR1 5001 / AMR3 5003)
├── third_party/
│   └── UniDepth/             # git submodule (직접 커밋 안 함)
├── maps/                     # 공용 Nav2 맵 (new_map.pgm/.yaml)
├── docs/
├── requirements.txt          # pip 통합 의존성
└── .gitignore
```

`src/`는 colcon 빌드 대상(ROS2 패키지), `ui/`는 순수 Flask 앱이라 별도로 실행한다.

---

## 1. 사전 요구사항

- **Ubuntu 22.04 + ROS2 Humble**
- **TurtleBot4** 2대 + Nav2 스택 (bringup, localization, navigation)
- **Python 3.10** (ROS2 Humble 기본)
- CCTV/전방 카메라: USB 카메라 또는 OAK-D (robot3 러기지 추적용)
- (선택) NVIDIA GPU + CUDA — YOLO/UniDepth 가속

---

## 2. 설치

### 2-1. 클론 + 서브모듈

```bash
cd ~
git clone --recurse-submodules <YOUR_REPO_URL> aura_ws
cd aura_ws
# 이미 클론했다면:
git submodule update --init --recursive
```

### 2-2. Python 의존성

가상환경 사용을 권장한다(ROS2와 함께 쓸 때는 `--system-site-packages`).

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate

pip install -r requirements.txt
# robot3 후방 카메라 단안 깊이 추정용 UniDepthV2
pip install -e third_party/UniDepth
```

> `numpy<2.0`으로 고정되어 있다. torch/ultralytics와의 호환을 위한 것이며,
> GPU용 torch는 https://pytorch.org 의 CUDA 빌드 안내를 따르라.

### 2-3. ROS 의존성 (apt)

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
# 개별 설치가 필요하면 예:
# sudo apt install ros-humble-nav2-msgs ros-humble-irobot-create-msgs \
#                  ros-humble-message-filters ros-humble-tf2-ros
```

### 2-4. colcon 빌드

`fleet_interfaces`가 먼저 빌드되어야 나머지가 이를 참조할 수 있다. colcon이 의존성 순서를
자동 계산하지만, 인터페이스만 먼저 빌드하면 확실하다.

```bash
cd ~/aura_ws
source /opt/ros/humble/setup.bash

# 1) 인터페이스 먼저
colcon build --packages-select fleet_interfaces
source install/setup.bash

# 2) 나머지 전체
colcon build --symlink-install
source install/setup.bash
```

이후 새 터미널마다:

```bash
source /opt/ros/humble/setup.bash
source ~/aura_ws/install/setup.bash
```

---

## 3. 실행

각 컴포넌트는 **별도 터미널**에서 실행한다(모든 터미널에서 위 `source` 2줄 먼저).
아래는 권장 기동 순서다.

### 0) 로봇 bringup + Nav2 (선행)

TurtleBot4 bringup, localization, Nav2를 각 로봇에 대해 먼저 올린다(맵: `maps/new_map.yaml`).
이 부분은 로봇 팀의 bringup launch를 사용하며 본 리포 범위 밖이다. `/robot1/navigate_to_pose`,
`/robot3/navigate_to_pose`가 떠 있어야 미션이 동작한다.

### 1) CCTV 인지

```bash
ros2 run cctv_perception cctv_detector_node
```

> YOLO11-pose 가중치(`yolo11x-pose.pt`)가 필요하다. 가중치는 저장소에 포함하지 않으므로
> 작업 디렉터리에 두거나 노드 상단 `MODEL_PATH`를 수정한다. 카메라 인덱스는 `CAMERA_INDEX`.

### 2) 중앙 계층

```bash
# 배차 두뇌
ros2 run fleet_central fleet_dispatcher_node

# SQLite 기록 (DB 경로 지정 가능)
AURA_DB_PATH=$HOME/aura_data/amr_system.db ros2 run fleet_central db_manager_node

# 혼잡구역 keepout 코스트맵 마스크
ros2 run fleet_central crowd_keepout_mask_node
```

### 3) robot1 (응급/AED 대응)

```bash
ros2 run robot1_control robot1_mission_fsm_node
```

> AED 탑재 여부/네임스페이스가 파라미터화되어 있으면 예:
> `ros2 run robot1_control robot1_mission_fsm_node --ros-args -p aed_loaded:=true`

### 4) robot3 (가이드/러기지) — 로컬 스택

robot3는 아래 노드들을 함께 띄운다.

```bash
# 가이드 (후방 카메라)
ros2 run robot3_control guide_rear_tracker_node
ros2 run robot3_control guide_motion_node

# 러기지 어시스트 (전방 RGB-D 카메라)
ros2 run robot3_control luggage_rgbd_tracker_node
ros2 run robot3_control luggage_follower_node

# 순찰 + hot_place 출동
ros2 run robot3_control robot3_patrol_node

# 미션 FSM (응급 선점/최종 접근)
ros2 run robot3_control robot3_mission_fsm_node --ros-args -p aed_loaded:=false
```

### 5) 관리자 UI (포트 7000)

```bash
cd ui/admin_ui
pip install -r requirements.txt        # 최초 1회
bash run.sh
# 접속: http://127.0.0.1:7000
```

환경변수: `AURA_DB_PATH`(SQLite 경로), `AURA_ADMIN_CCTV_TOPIC`(기본 `/cctv/image_raw/compressed`),
`AURA_ADMIN_PORT`(기본 7000). ROS가 없어도 DB 전용 모드로 뜬다.

### 6) 승객 UI (AMR1 5001 / AMR3 5003)

```bash
cd ui/passenger_ui
pip install -r requirements.txt        # 최초 1회

# 개별 실행
python3 run_amr1.py     # AMR1 → robot1, http://127.0.0.1:5001
python3 run_amr3.py     # AMR3 → robot3, http://127.0.0.1:5003

# 둘 동시 실행
python3 run_all_ui.py
```

`launcher.py`가 ROS2 환경을 자동 source한다. rclpy를 못 찾으면 print-only 모드로 실행된다.

---

## 4. 주요 환경변수

| 변수 | 대상 | 기본값 | 설명 |
|---|---|---|---|
| `AURA_DB_PATH` | db_manager, admin_ui | `~/Downloads/data/amr_system.db` | SQLite 경로 |
| `AURA_ADMIN_PORT` | admin_ui | `7000` | 관제 UI 포트 |
| `AURA_ADMIN_CCTV_TOPIC` | admin_ui | `/cctv/image_raw/compressed` | CCTV 토픽 |
| `AURA_ROBOT_ID` | passenger_ui | `AMR1` | `AMR1`/`AMR3` |
| `AURA_PORT` | passenger_ui | `5001` | 승객 UI 포트 |
| `AURA_ALIGNMENT_INPUT_MODE` | passenger_ui | — | `manual`/`tracking_int32` |

승객 UI는 보통 `run_amr1.py`/`run_amr3.py`가 위 값을 자동 설정하므로 직접 만질 일은 적다.

---

## 5. 기동 순서 체크리스트

1. 두 로봇 bringup + Nav2 (`navigate_to_pose` 액션 확인)
2. `cctv_detector_node`
3. `fleet_dispatcher_node` → `db_manager_node` → `crowd_keepout_mask_node`
4. `robot1_mission_fsm_node`
5. robot3 노드 6종
6. `admin_ui` → `passenger_ui`

`ros2 topic list`로 `/fall_detection`, `/population`, `/robot1/status`,
`/tracking_web`, `/tracking_rgbd`가 보이면 정상 연결이다.

---

## 6. 트러블슈팅

- **`Package 'fleet_interfaces' not found`** → 인터페이스를 먼저 빌드하고 `source install/setup.bash`
  했는지 확인 (설치 2-4 참고).
- **`ModuleNotFoundError: unidepth`** → `pip install -e third_party/UniDepth` 및
  `git submodule update --init` 확인.
- **YOLO 가중치 로드 실패** → `yolo11x-pose.pt`가 실행 위치에 있는지, 노드의 `MODEL_PATH` 확인.
- **승객 UI가 print-only 모드** → 해당 터미널에서 ROS2 setup을 source 후 실행.
- **admin UI에 CCTV가 안 뜸** → `cctv_detector_node`가 실행 중이고 `AURA_ADMIN_CCTV_TOPIC`이
  일치하는지 확인.
- **numpy 관련 에러** → `numpy<2.0` 준수 확인.

---

## 7. 라이선스 / 크레딧

- `third_party/UniDepth`: 업스트림(https://github.com/lpiccinelli-eth/UniDepth) 라이선스를 따른다.
- YOLO11: Ultralytics 라이선스를 따른다.
- 본 프로젝트 코드 라이선스는 리포 정책에 맞게 지정하라(패키지 기본값 Apache-2.0).
