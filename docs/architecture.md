# AURA 아키텍처

AURA는 공항형 환경에서 TurtleBot4 2대(**robot1**, **robot3**)를 중앙 노드로 조율하여
**가이드 / 러기지 어시스트 / 순찰 / 응급 대응** 역할을 수행하는 협동 로봇 시스템이다.

시스템은 4개 계층 + 1개 공용 계약으로 구성된다.

```
① UI 계층        admin_ui(7000) · passenger_ui(5001/5003)   ← Flask
② 중앙 계층      fleet_dispatcher · db_manager · crowd_keepout
③ 로봇 계층      robot1_control(완전연결) · robot3_control(부분연결)
④ 인지 계층      cctv_perception (쓰러짐·혼잡·인원수)
공용 계약        fleet_interfaces (msg / srv / action)
```

---

## 공용 계약 — `fleet_interfaces`

시스템 전체가 공유하는 인터페이스 정의. 모든 상위 패키지가 여기에 의존한다.

| 종류 | 이름 | 요약 |
|---|---|---|
| msg | `RobotStatus` | robot_id, mode(7종), battery_pct, pose, `aed_loaded`, available |
| action | `ExecuteMission` | (mode, dest_x, dest_y, priority) → (success, final_state) / feedback(current_mode, progress_pct) |
| srv | `EmergencyStop` | reason → success |

`RobotStatus.mode`: `IDLE(0) · GUIDE(1) · LUGGAGE_ASSIST(2) · EMERGENCY_DISPATCH(3) · ERROR(4) · CHARGING(5) · HOTPLACE_DISPATCH(6)`

---

## ② 중앙 계층 — `fleet_central`

세 노드가 논리적으로 한 덩어리를 이룬다.

- **`fleet_dispatcher_node`** — 배차 두뇌.
  - 승객 UI 이벤트(`/aura/robot_select`, `/aura/service_request`, `/aura/service_end`)를 구독해
    GUIDE / LUGGAGE_ASSIST를 트리거한다. AMR1→robot1, AMR3→robot3 고정 매핑.
  - `/fall_detection` + `/fall_detection_point` 감지 시 `RobotStatus.aed_loaded==True`인
    로봇을 실시간으로 찾아 `ExecuteMission` 액션 goal을 보낸다(역할 고정 아님).
  - 미션 종료 결과를 `/aura/arrival_status`로 UI에 통보한다.
- **`db_manager_node`** — dispatcher를 수정하지 않고 옆에서 주요 토픽을 구독해 SQLite(`amr_system.db`)에
  기록하는 독립 노드. admin UI가 이 DB를 읽는다. DB 경로는 `AURA_DB_PATH`로 변경.
- **`crowd_keepout_mask_node`** — `/hot_place`(PoseArray) + map을 받아 Nav2용 keepout 코스트맵
  마스크를 발행하는 내비게이션 보조 노드. 혼잡구역 주변 반경을 keepout으로 지정.

---

## ③ 로봇 계층

### robot1 — `robot1_control` (중앙 완전 연결)

응급/AED 대응 로봇. 단일 노드 `robot1_mission_fsm_node`가 FSM 전체를 담당한다.

- `ExecuteMission` **액션 서버** + `EmergencyStop` **서비스 서버** + `RobotStatus` 발행.
- 6점 무한 순찰 → GUIDE / HOTPLACE_DISPATCH / EMERGENCY_DISPATCH 대응 → 순찰 복귀.
- 즉시 제어는 승객 UI와 `/rb1_standby`, `/rb1_service_end`, `/emergency_end`(Bool) 직결.
- `/robot1/navigate_to_pose`(Nav2) 사용, 응급 최종 접근은 `/rgbd_fall_person_point` 좌표 이용.

### robot3 — `robot3_control` (중앙 부분 연결)

가이드/러기지 어시스트 로봇. **액션 서버가 없고**, 자기 로컬 인지·모션 노드들로 동작한다.
중앙과는 `/service_cancel`, `/start_patrol` 등 개별 토픽으로 최소한만 연결된다.

| 노드 | 역할 | 핵심 토픽 |
|---|---|---|
| `guide_rear_tracker_node` | **가이드 후방 카메라** 추적 (UniDepthV2 단안 깊이) | pub `/tracking_web`, `/target_depth`, `/turn_complete` |
| `guide_motion_node` | **가이드 모션** 컨트롤러 | sub 추적 → pub `/robot3/cmd_vel`; `/robot3/mission_cmd` |
| `luggage_rgbd_tracker_node` | **러기지 전방 OAK-D RGB-D** 추적 | pub `/tracking_rgbd`, `/target_depth_rgbd`; `/carrying`로 언락 |
| `luggage_follower_node` | **러기지 팔로잉** (앞선 승객 추종) | sub `/tracking_rgbd` → pub `/robot3/cmd_vel` |
| `robot3_patrol_node` | 순찰 + `/hot_place` 출동 + `/rb3_standby` 대기 | sub `/hot_place`, `/start_patrol`; pub `/rb3_standby_done` |
| `robot3_mission_fsm_node` | 미션 FSM 래퍼(응급 선점/최종 접근) | sub `/rgbd_fall_person_point`, `/service_cancel` |

> 방향 규칙: **후방 카메라 = 가이드**(로봇이 앞장, 승객이 뒤따름 → `tracking_web`),
> **전방 카메라 = 러기지**(승객이 앞장, 로봇이 추종 → `tracking_rgbd`).

---

## ④ 인지 계층 — `cctv_perception`

고정 CCTV 노드 `cctv_detector_node` (YOLO11-pose):

- 쓰러짐 감지 → `/fall_detection`(Bool) + `/fall_detection_point`(PointStamped)
- 인원수 카운트 → `/population`(Int32)
- 혼잡구역 감지 → `/hot_place`(PoseArray)
- 어노테이션 이미지 → `/cctv/image_raw/compressed`, `/cctv/fall_detection_image`, `/cctv/hot_place_image`

중앙(dispatcher, crowd_keepout)과 admin UI 양쪽에 데이터를 공급하는 소스다.

---

## ① UI 계층

### admin_ui (포트 7000)

관제 대시보드. `camera_bridge`가 CCTV 압축 이미지 + `/population`을 구독해 MJPEG로 스트리밍하고,
`database`가 `db_manager`가 쌓은 SQLite를 조회한다. ROS 없이 DB 전용 모드로도 실행 가능.

### passenger_ui (AMR1 포트 5001 / AMR3 포트 5003)

승객 키오스크. Flask app-factory 구조이며 로봇별로 통신 방식이 다르다.

| | AMR1 (robot1) | AMR3 (robot3) |
|---|---|---|
| 포트 | 5001 | 5003 |
| 정렬 모드 | `manual` (카메라 정렬 없음) | `tracking_int32` (비전 노드 Int32 사용) |
| 중앙 통신 | `/aura/*` String(JSON) | `/aura/*` String(JSON) |
| 로봇 직접 명령 | `/rb1_standby`·`/rb1_service_end`·`/emergency_end` Bool | `/rb3_standby`·`/service_end`·`/carrying`·`/turn_around` Bool |

`run_all_ui.py`로 두 UI를 동시에 실행한다.
