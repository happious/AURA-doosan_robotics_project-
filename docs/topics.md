# 토픽 레퍼런스

주요 ROS2 토픽과 발행/구독 주체를 정리한다. (네임스페이스가 붙는 것은 예: `/robot1/...`)

## 인지 → 중앙/UI (CCTV 이벤트)

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/fall_detection` | `std_msgs/Bool` | cctv_detector | fleet_dispatcher, db_manager, passenger_ui |
| `/fall_detection_point` | `geometry_msgs/PointStamped` | cctv_detector | fleet_dispatcher, db_manager |
| `/population` | `std_msgs/Int32` | cctv_detector | admin_ui, fleet_dispatcher(relay) |
| `/hot_place` | `geometry_msgs/PoseArray` | cctv_detector | crowd_keepout, robot3_patrol, fleet_dispatcher |
| `/cctv/image_raw/compressed` | `sensor_msgs/CompressedImage` | cctv_detector | admin_ui(camera_bridge) |
| `/cctv/fall_detection_image` | `sensor_msgs/CompressedImage` | cctv_detector | (기록/표시용) |
| `/cctv/hot_place_image` | `sensor_msgs/CompressedImage` | cctv_detector | (기록/표시용) |

## 승객 UI ↔ 중앙

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/aura/robot_select` | `std_msgs/String`(JSON) | passenger_ui | fleet_dispatcher(로그) |
| `/aura/service_request` | `std_msgs/String`(JSON) | passenger_ui | fleet_dispatcher |
| `/aura/service_end` | `std_msgs/String`(JSON) | passenger_ui | fleet_dispatcher |
| `/aura/arrival_status` | `std_msgs/String`(JSON) | fleet_dispatcher | passenger_ui |

## 중앙 ↔ robot1 (완전 연결)

| 인터페이스 | 타입 | 서버 | 클라이언트 |
|---|---|---|---|
| `/robot1/execute_mission` | `fleet_interfaces/ExecuteMission` (action) | robot1 | fleet_dispatcher |
| `/robot1/emergency_stop` | `fleet_interfaces/EmergencyStop` (srv) | robot1 | fleet_dispatcher |
| `/robot1/status` | `fleet_interfaces/RobotStatus` | robot1 | fleet_dispatcher, db_manager |

### robot1 즉시 제어 (UI 직결, Bool)

`/rb1_standby`, `/rb1_service_end`, `/emergency_end` — passenger_ui(AMR1) → robot1
`/rgbd_fall_person_point` — 응급 최종 접근 좌표 (센서 → robot1)

## robot3 로컬 (부분 연결)

### 가이드 (후방 카메라)

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/tracking_web` | `std_msgs/Int32` | guide_rear_tracker | guide_motion, passenger_ui(AMR3) |
| `/tracking_web_center_pixel` | `std_msgs/Float32` | guide_rear_tracker | guide_motion |
| `/target_depth` | `std_msgs/Float32` | guide_rear_tracker | guide_motion |
| `/turn_around` | `std_msgs/Bool` | (UI/모션) | guide_motion |
| `/turn_complete` | `std_msgs/Bool` | guide_rear_tracker | guide_motion, passenger_ui(AMR3) |

### 러기지 (전방 RGB-D 카메라)

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/tracking_rgbd` | `std_msgs/Int32` | luggage_rgbd_tracker | luggage_follower, passenger_ui(AMR3) |
| `/tracking_rgbd_center_pixel` | `std_msgs/Float32` | luggage_rgbd_tracker | luggage_follower |
| `/target_depth_rgbd` | `std_msgs/Float32` | luggage_rgbd_tracker | luggage_follower |
| `/carrying` | `std_msgs/Bool` | passenger_ui(AMR3) | luggage_rgbd_tracker, luggage_follower |

### robot3 공통 / 순찰 / 미션

| 토픽 | 타입 | 비고 |
|---|---|---|
| `/robot3/cmd_vel` | `geometry_msgs/Twist` | guide_motion / luggage_follower 출력 |
| `/robot3/navigate_to_pose` | Nav2 action | 목표 이동 |
| `/robot3/mission_cmd` | `std_msgs/String` | 미션 명령 |
| `/robot3/mission_complete` | `std_msgs/Bool` | 미션 완료 |
| `/start_patrol` | `std_msgs/Bool` | 순찰 시작 |
| `/rb3_standby` | `std_msgs/Bool` | 대기 (UI → robot3) |
| `/rb3_standby_done` | `std_msgs/Bool` | 대기 완료 통지 |
| `/service_end` | `std_msgs/Bool` | 서비스 종료 (UI → robot3) |
| `/service_cancel` | `std_msgs/Bool` | 미션 취소 (dispatcher → robot3) |

> 전체 목록이 아니라 계층 간 연결을 이해하기 위한 핵심 토픽 위주다.
> 정확한 시그니처는 각 노드 소스의 `create_publisher` / `create_subscription`를 확인하라.
