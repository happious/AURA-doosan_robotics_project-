# AURA 승객 UI V19 — AMR3 GUIDE 추적 화면 전환 수정

사용자가 제공한 12개 화면 디자인을 유지하면서 AMR3 GUIDE 재인식 반복,
LUGGAGE_ASSIST 전방 인식 안내, DDS ACK 1회 발행 및 상태 전환을 정리한 버전이다.
UI에서는 카메라 영상을 표시하지 않고 비전 결과 토픽만 사용한다.

## 실행

```bash
cd ~/rokey_ws/aura_mobile_ui_module
source /opt/ros/humble/setup.bash
source ~/rokey_ws/install/setup.bash
python3 run_all_ui.py
```

AMR3만 실행:

```bash
python3 run_amr3.py
```

## AMR3 직접 명령

다음 네 토픽은 `std_msgs/msg/Bool`, `data: true`로 발행한다.

```text
/rb3_standby
/turn_around
/carrying
/service_end
```

전송 순서:

```text
subscriber 탐색
→ Bool(True) 정확히 1회 publish
→ RELIABLE DDS ACK 확인
→ 자동 재전송 없음
```

QoS:

```text
RELIABLE
VOLATILE
KEEP_LAST
DEPTH=1
```

`/turn_around`는 반복 발행하지 않으므로 중복 180도 회전을 방지한다.

## AMR3 GUIDE 확정 흐름

```text
목적지 선택
→ PREALIGN: 로봇 전면에서 승객 정렬
→ 정렬 완료 버튼
→ TURNING_TO_REAR 상태로 먼저 변경
→ /turn_around=True 1회
→ 로봇 180도 회전 시간 동안에도 /tracking_web=0/1/2 계속 수신
→ /tracking_web=1이 연속 기준 프레임 이상이면 즉시 DETECTED
→ 회전 종료까지 기준 프레임을 만족하지 못하면 LOST
```

### `/tracking_web=1`

```text
TURNING_TO_REAR 또는 WAITING 중 1 연속 기준 프레임 충족
→ DETECTED 상태 고정(latch)
→ /confirm-alignment 자동 이동
→ /aura/service_request 발행
→ 05_Mission_Progress 운행 화면으로 자동 전환
```

### `/tracking_web=0` 또는 `2`

```text
회전 종료까지 1 연속 기준 프레임 미충족
→ LOST
→ /turn_around=True 1회
→ RETURNING_TO_FRONT
→ 전면 복귀 회전 시간 동안 tracking_web 무시
→ PREALIGN(retry)
→ 정렬 완료 버튼이 있는 재정렬 화면
```

회전 시간 기본값은 4초다. 실제 장비 회전 시간에 맞춰 조절한다.

```bash
export AURA_GUIDE_TURN_AROUND_SECONDS=4.0
```

`/tracking_web=1` 성공 기준은 기본 3프레임 연속이다. 테스트용으로 한 번만 pub해서 확인하려면 1로 낮출 수 있다.

```bash
export AURA_TRACKING_WEB_CONFIRM_FRAMES=3
```


기존 `AURA_GUIDE_TRACKING_GRACE` 환경변수도 호환한다.

정렬 페이지에서는 `alignment_watch.js`를 PREALIGN부터 항상 실행하며,
정적 파일 캐시를 피하기 위해 V19 버전 쿼리를 사용한다. 또한 단순 `GET /` 요청이
진행 중인 전역 정렬 상태를 IDLE로 초기화하지 않는다.

## 서비스 종료 후 추적값 처리

사용자 종료 또는 inactivity timeout 이후 상태는 `IDLE`로 초기화된다.
비전 컴퓨터가 `/tracking_web=1`을 계속 발행해도 UI는 다음처럼 처리한다.

```text
IDLE + tracking_web 값
→ 화면 전환 없음
→ /turn_around 발행 없음
→ /aura/service_request 발행 없음
→ 반복 로그 출력 없음
```

## AMR3 LUGGAGE_ASSIST 확정 흐름

별도의 `04_2` 카메라 정렬 화면은 사용하지 않는다.

```text
짐 들기 선택
→ 짐 적재 화면
→ “짐을 실은 뒤 로봇 앞에 서주세요” 표시
→ 짐 싣기 완료
→ /carrying=True 1회
→ 같은 화면에서 /tracking_rgbd 대기
```

`/tracking_rgbd` 처리:

```text
0 또는 2
→ “다시 인식해 주세요”
→ “다시 로봇 앞에 서주세요” 표시

1
→ 성공 상태 고정(latch)
→ “인식이 완료되었습니다”
→ “이제 출발하시면 됩니다” 표시
→ 약 2초 후 /aura/service_request 발행
→ 05_Mission_Progress 화면으로 이동
```

## AMR3 목적지

```text
화장품 가게 → goal1_1
편의점       → goal1_2
```

주류 판매점과 화장실은 준비 중 상태다.

## 확인 명령

```bash
ros2 topic echo /rb3_standby std_msgs/msg/Bool
ros2 topic echo /turn_around std_msgs/msg/Bool
ros2 topic echo /carrying std_msgs/msg/Bool
ros2 topic echo /service_end std_msgs/msg/Bool
ros2 topic echo /tracking_web std_msgs/msg/Int32
ros2 topic echo /tracking_rgbd std_msgs/msg/Int32
```

정상 GUIDE 로그 예시:

```text
[AURA UI ROUTE] GUIDE 정렬 완료 요청 수신
[AURA UI STATE] GUIDE alignment PREALIGN -> TURNING_TO_REAR
[AURA UI PUBLISH] /turn_around -> True
[AURA UI ACK] /turn_around -> True
[AURA UI TRACKING] /tracking_web=1 accepted=True, status=DETECTED
[AURA UI ROUTE] GUIDE 인식 성공 확인 -> 서비스 진행 화면 전환
[AURA UI PUBLISH] /aura/service_request -> {...}
```

DDS ACK는 메시지가 매칭된 구독 측 DDS 미들웨어까지 전달됐다는 의미이며,
실제 로봇 동작 완료 자체를 의미하지는 않는다.

## Responsive UI update

The passenger screens now load `app/static/css/responsive.css` after their existing page styles.
It removes the old fixed 1080×3240 scaling behavior and provides fluid layouts for phones,
tablets, desktop browsers, safe-area insets, narrow devices, and landscape orientation.
No ROS topic, route, state, or mission logic was changed.
