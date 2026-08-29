# AURA 관리자 UI 기본판

SQLite 기록 DB와 ROS2 CCTV 영상을 한 화면에서 확인하는 읽기 전용 관리자 대시보드입니다.

## 표시 항목

- AMR1 / AMR3 최신 상태와 위치, 배터리, AED 탑재 여부
- 진행 중 승객 서비스
- 응급상황 발생·출동·종료 기록
- `/cctv/image_raw/compressed` 실시간 CCTV 영상 (`sensor_msgs/msg/CompressedImage`)
- `/population` 공항 전체 인원수 (`std_msgs/msg/Int32`)
- DB에 저장된 낙상 감지 스냅샷
- 최근 시스템 이벤트 로그

관리자 UI는 실제 로봇 제어 명령을 발행하지 않습니다.

## 기본 경로와 토픽

```text
DB: ~/Downloads/data/amr_system.db
CCTV: /cctv/image_raw/compressed
인원수: /population
웹 포트: 7000
```

## 필요 패키지

```bash
sudo apt update
sudo apt install python3-flask
```

Python 가상환경을 사용할 경우:

```bash
python3 -m pip install -r requirements.txt
```

## 실행

DB 기록 노드와 카메라 노드가 실행 중인 상태에서 새 터미널을 엽니다.

```bash
source /opt/ros/humble/setup.bash
source ~/rokey_ws/install/setup.bash
export ROS_DOMAIN_ID=3

cd ~/Downloads/aura_admin_ui
python3 app.py
```

브라우저:

```text
http://127.0.0.1:7000
```

같은 네트워크의 다른 PC에서 접속할 때는:

```bash
hostname -I
```

으로 나온 IP를 사용합니다.

```text
http://중앙PC_IP:7000
```

## 경로·토픽 변경

```bash
export AURA_DB_PATH=/home/taehwan/Downloads/data/amr_system.db
export AURA_ADMIN_CCTV_TOPIC=/cctv/image_raw/compressed
export AURA_ADMIN_POPULATION_TOPIC=/population
export AURA_ADMIN_PORT=7000
python3 app.py
```

## 확인 명령

```bash
ros2 topic type /cctv/image_raw/compressed
ros2 topic hz /cctv/image_raw/compressed
ros2 topic type /population
ros2 topic echo /population
ros2 node list | grep aura_admin
```

카메라 타입은 다음이어야 합니다.

```text
sensor_msgs/msg/CompressedImage
```

## 참고

- 실시간 CCTV 영상은 DB에 저장하지 않고 ROS2 토픽에서 직접 표시합니다.
- `/cctv/fall_detection_image`로 저장된 낙상 장면은 `event_images` 테이블의 파일 경로를 읽어 표시합니다.
- DB 파일이 없어도 `/population`과 CCTV는 ROS2 토픽에서 계속 표시됩니다.
- DB 전용 영역(로봇·서비스·응급 기록·스냅샷·로그)만 연결 대기 상태로 표시됩니다.


인원수 토픽 타입은 다음이어야 합니다.

```text
std_msgs/msg/Int32
```


## /population 즉시 갱신

이 버전은 `/population`을 1초 주기 API 폴링에 의존하지 않습니다.
ROS2 콜백에서 새 `Int32` 메시지를 받으면 Flask SSE(`/api/population/stream`)로
브라우저에 즉시 전달합니다. DB 연결 여부와 무관하게 인원수 카드가 갱신됩니다.

테스트:

```bash
ros2 topic pub -r 2 /population std_msgs/msg/Int32 "{data: 127}"
```

브라우저가 이전 JavaScript를 캐시하면 `Ctrl+Shift+R`로 강력 새로고침합니다.
