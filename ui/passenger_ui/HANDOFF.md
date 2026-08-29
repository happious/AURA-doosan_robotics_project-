# 다음 대화용 인계 요약

최신 파일: `aura_mobile_ui_module_logic_v19_tracking_flow_fixed.zip`

V19 핵심 수정:

- GUIDE 상태 흐름을 명시적으로 분리
  - `PREALIGN`
  - `TURNING_TO_REAR`
  - `WAITING`
  - `DETECTED` 또는 `LOST`
  - 실패 시 `RETURNING_TO_FRONT`
  - 전면 복귀 후 `PREALIGN(retry)`
- `/turn_around` 발행 후 설정된 회전 시간 동안 `/tracking_web=0/1/2`를 모두 무시
- 회전 시간이 지난 뒤부터 tracking_web 판정
- `tracking_web=1`
  - DETECTED latch
  - `/confirm-alignment` 자동 이동
  - `/aura/service_request` 발행
  - 05_Mission_Progress 운행 화면 표시
- `tracking_web=0/2`
  - `/turn_around=True` 1회 재발행
  - 전면 복귀 회전 중 화면
  - 회전 완료 후 정렬 완료 버튼이 있는 retry 화면
- GUIDE 정렬 페이지에서 `alignment_watch.js`를 PREALIGN부터 항상 로드
- JS 캐시 방지 `?v=19`
- 단순 `GET /`가 진행 중인 전역 alignment 상태를 IDLE로 초기화하던 문제 제거
- 회전 시간 설정
  - `AURA_GUIDE_TURN_AROUND_SECONDS=4.0` 기본
  - 기존 `AURA_GUIDE_TRACKING_GRACE`도 호환

검증 완료:

- Python compileall
- alignment_watch.js Node 구문 검사
- GUIDE 성공 통합 테스트
  - 회전 중 1 무시
  - 회전 완료 후 1 수신
  - DETECTED
  - service_request
  - mission_progress
- GUIDE 실패/재시도 통합 테스트
  - 회전 완료 후 0/2
  - LOST
  - 전면 복귀 turn_around
  - RETURNING_TO_FRONT
  - PREALIGN retry 버튼 화면
- 정렬 중 GET / 상태 보존 테스트
