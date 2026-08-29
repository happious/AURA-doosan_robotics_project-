from __future__ import annotations

import threading
import time
from copy import deepcopy
from datetime import datetime
from typing import Any


class MissionStateStore:
    """Flask 요청 스레드와 ROS2 spin 스레드가 공유하는 상태 저장소."""

    def __init__(self, robot_id: str):
        self._lock = threading.Lock()
        self._state = self._initial_state(robot_id)

    @staticmethod
    def _initial_state(robot_id: str) -> dict[str, Any]:
        return {
            "active": False,
            "arrived": False,
            "awaiting_service_end": False,
            "robot_id": robot_id,
            "request_id": None,
            "mission_id": None,
            "service_type": "",
            "mode": "",
            "goal_id": None,
            "destination_label": "",
            "arrival_payload": None,
            "arrival_time": None,
            # 정렬/추적 상태
            "alignment_active": False,
            "alignment_service_type": "",
            "alignment_mode": "",
            "alignment_camera": "",
            "alignment_detector": "",
            "alignment_status": "IDLE",
            "alignment_detected": False,
            "alignment_retry": False,
            "alignment_attempt": 0,
            "alignment_request_id": None,
            "alignment_started_at": None,
            "alignment_deadline_monotonic": None,
            "alignment_timeout_seconds": 0.0,
            "alignment_last_payload": None,
            "alignment_last_signal": None,
            "alignment_tracking_value": 0,
            "alignment_tracking_topic": "",
            "alignment_tracking_confirm_frames": 3,
            "alignment_tracking_one_streak": 0,
            "alignment_tracking_one_max_streak": 0,
            "alignment_tracking_sample_count": 0,
            "alignment_accept_tracking_after_monotonic": None,
            # GUIDE 회전 상태. /turn_complete=True 이후에만 tracking_web 판정을 시작한다.
            "alignment_turn_direction": "",
            "alignment_turn_deadline_monotonic": None,
            # GUIDE 후방 인식 실패 시 전방으로 되돌리는 /turn_around 중복 발행 방지
            "alignment_return_turn_sent": False,
            # 응급상황
            "emergency_active": False,
            "emergency_type": "",
            "emergency_payload": None,
            "emergency_time": None,
            "interrupted_state": None,
            # 중앙 노드가 JSON으로 보내는 최신 상태
            "robot_mode": "PATROLLING",
            "battery": None,
            "online": None,
            "robot_status_payload": None,
            "mission_status_payload": None,
        }

    def get(self) -> dict[str, Any]:
        with self._lock:
            self._expire_alignment_locked()
            return deepcopy(self._state)

    def update(self, **updates: Any) -> dict[str, Any]:
        with self._lock:
            self._state.update(updates)
            return deepcopy(self._state)

    def reset(self, robot_id: str) -> dict[str, Any]:
        with self._lock:
            self._state = self._initial_state(robot_id)
            return deepcopy(self._state)


    def prepare_manual_alignment(
        self,
        *,
        service_type: str,
        mode: str,
        camera: str,
        detector: str,
        retry: bool,
        tracking_topic: str = "",
    ) -> dict[str, Any]:
        """사용자가 로봇 전방에 직접 선 뒤 버튼을 누르는 준비 상태를 만든다.

        GUIDE(tracking_int32)에서는 목적지를 고른 즉시 후방 추적을 시작하지 않는다.
        먼저 PREALIGN 상태로 화면을 보여주고, 사용자가 '정렬 완료'를 누른 뒤에만
        /turn_around를 발행하고 TURNING_TO_REAR 상태로 전환한다.
        """
        now = datetime.now().isoformat(timespec="seconds")
        with self._lock:
            self._state.update(
                alignment_active=True,
                alignment_service_type=service_type,
                alignment_mode=mode,
                alignment_camera=camera,
                alignment_detector=detector,
                alignment_status="PREALIGN",
                alignment_detected=False,
                alignment_retry=bool(retry),
                alignment_request_id=None,
                alignment_started_at=now,
                alignment_deadline_monotonic=None,
                alignment_timeout_seconds=0.0,
                alignment_last_payload=None,
                alignment_last_signal=None,
                alignment_tracking_value=0,
                alignment_tracking_topic=tracking_topic,
                alignment_tracking_confirm_frames=3,
                alignment_tracking_one_streak=0,
                alignment_tracking_one_max_streak=0,
                alignment_tracking_sample_count=0,
                alignment_accept_tracking_after_monotonic=None,
                alignment_turn_direction="",
                alignment_turn_deadline_monotonic=None,
                alignment_return_turn_sent=False,
            )
            return deepcopy(self._state)


    def prepare_manual_confirm_alignment(
        self,
        *,
        service_type: str,
        mode: str,
        retry: bool = False,
    ) -> dict[str, Any]:
        """AMR1처럼 카메라 검출 없이 사람이 직접 확인하는 정렬 상태.

        화면에 들어오는 즉시 버튼을 누를 수 있도록 DETECTED 상태로 둔다.
        실제 서비스 시작은 사용자가 정렬 완료 버튼을 눌렀을 때만 발생한다.
        """
        now = datetime.now().isoformat(timespec="seconds")
        with self._lock:
            attempt = int(self._state.get("alignment_attempt") or 0) + 1
            self._state.update(
                alignment_active=True,
                alignment_service_type=service_type,
                alignment_mode=mode,
                alignment_camera="NONE",
                alignment_detector="MANUAL_CONFIRM",
                alignment_status="DETECTED",
                alignment_detected=True,
                alignment_retry=bool(retry),
                alignment_attempt=attempt,
                alignment_request_id=None,
                alignment_started_at=now,
                alignment_deadline_monotonic=None,
                alignment_timeout_seconds=0.0,
                alignment_last_payload={"source": "manual_ui_confirmation"},
                alignment_last_signal=True,
                alignment_tracking_value=0,
                alignment_tracking_topic="",
                alignment_tracking_confirm_frames=3,
                alignment_tracking_one_streak=0,
                alignment_tracking_one_max_streak=0,
                alignment_tracking_sample_count=0,
                alignment_accept_tracking_after_monotonic=None,
                alignment_turn_direction="",
                alignment_turn_deadline_monotonic=None,
                alignment_return_turn_sent=False,
            )
            return deepcopy(self._state)

    def record_robot_status(self, payload: dict[str, Any]) -> bool:
        """중앙에서 받은 /aura/robot_status JSON을 최신 UI 상태로 저장한다."""
        with self._lock:
            incoming_robot = payload.get("robot_id")
            current_robot = self._state.get("robot_id")
            if incoming_robot and current_robot and incoming_robot != current_robot:
                return False
            self._state.update(
                robot_mode=(payload.get("mode") or payload.get("status") or self._state.get("robot_mode")),
                battery=payload.get("battery", self._state.get("battery")),
                online=payload.get("online", self._state.get("online")),
                robot_status_payload=deepcopy(payload),
            )
            return True

    def record_mission_status(self, payload: dict[str, Any]) -> bool:
        """중앙에서 받은 /aura/mission_status JSON을 현재 미션과 매칭해 저장한다."""
        with self._lock:
            incoming_robot = payload.get("robot_id")
            current_robot = self._state.get("robot_id")
            if incoming_robot and current_robot and incoming_robot != current_robot:
                return False
            current_request = self._state.get("request_id")
            incoming_request = payload.get("request_id")
            if incoming_request and current_request and incoming_request != current_request:
                return False
            self._state["mission_status_payload"] = deepcopy(payload)
            if payload.get("mission_id"):
                self._state["mission_id"] = payload.get("mission_id")
            return True

    def start_alignment(
        self,
        *,
        service_type: str,
        mode: str,
        camera: str,
        detector: str,
        retry: bool,
        request_id: str,
        timeout_seconds: float,
        tracking_topic: str = "",
        tracking_grace_seconds: float = 0.0,
        tracking_confirm_frames: int = 3,
    ) -> dict[str, Any]:
        now = datetime.now().isoformat(timespec="seconds")
        timeout_value = max(0.0, float(timeout_seconds))
        now_monotonic = time.monotonic()
        confirm_frames = max(1, int(tracking_confirm_frames or 1))
        is_guide = service_type.upper() == "GUIDE"
        # GUIDE는 /turn_complete 이후 무기한 retargeting을 기다린다.
        # LUGGAGE_ASSIST만 기존 timeout을 유지한다.
        deadline = None if is_guide else (
            now_monotonic + timeout_value if timeout_value > 0.0 else None
        )
        with self._lock:
            attempt = int(self._state.get("alignment_attempt") or 0) + 1
            self._state.update(
                alignment_active=True,
                alignment_service_type=service_type,
                alignment_mode=mode,
                alignment_camera=camera,
                alignment_detector=detector,
                # GUIDE는 최초 회전 완료 토픽(/turn_complete=True)을 받을 때까지 대기한다.
                alignment_status="TURNING_TO_REAR" if is_guide else "WAITING",
                alignment_detected=False,
                alignment_retry=False if is_guide else bool(retry),
                alignment_attempt=attempt,
                alignment_request_id=request_id,
                alignment_started_at=now,
                alignment_deadline_monotonic=deadline,
                alignment_timeout_seconds=timeout_value,
                alignment_last_payload=None,
                alignment_last_signal=None,
                alignment_tracking_value=0,
                alignment_tracking_topic=tracking_topic,
                alignment_tracking_confirm_frames=confirm_frames,
                alignment_tracking_one_streak=0,
                alignment_tracking_one_max_streak=0,
                alignment_tracking_sample_count=0,
                alignment_accept_tracking_after_monotonic=None,
                alignment_turn_direction="TO_REAR" if is_guide else "",
                alignment_turn_deadline_monotonic=None,
                alignment_return_turn_sent=False,
            )
            return deepcopy(self._state)

    def _expire_alignment_locked(self) -> None:
        if not self._state.get("alignment_active"):
            return

        status = str(self._state.get("alignment_status") or "IDLE")
        if status != "WAITING":
            return

        # GUIDE는 최초 회전 완료 후 비전 노드가 계속 retargeting하므로 시간 초과,
        # LOST, 재회전, 재정렬로 전환하지 않는다.
        current_service = str(self._state.get("alignment_service_type") or "").upper()
        if current_service == "GUIDE":
            return

        deadline = self._state.get("alignment_deadline_monotonic")
        if deadline is not None and time.monotonic() >= float(deadline):
            self._state.update(
                alignment_status="FAILED",
                alignment_detected=False,
                alignment_last_signal=False,
            )

    def mark_guide_turn_complete(self) -> bool:
        """Robot3의 /turn_complete=True 이후 GUIDE tracking_web 판정을 시작한다."""
        with self._lock:
            if (
                not self._state.get("alignment_active")
                or str(self._state.get("alignment_service_type") or "").upper() != "GUIDE"
                or self._state.get("alignment_status") != "TURNING_TO_REAR"
            ):
                return False
            self._state.update(
                alignment_status="WAITING",
                alignment_detected=False,
                alignment_deadline_monotonic=None,
                alignment_tracking_one_streak=0,
                alignment_tracking_one_max_streak=0,
                alignment_tracking_sample_count=0,
                alignment_accept_tracking_after_monotonic=None,
                alignment_turn_direction="",
                alignment_turn_deadline_monotonic=None,
                alignment_retry=False,
            )
            return True

    def record_alignment_signal(
        self,
        *,
        detected: bool,
        source: str,
        payload: dict[str, Any] | None = None,
        expected_service_type: str | None = None,
    ) -> bool:
        """기존 Bool 검출 방식의 정렬 결과를 반영한다."""
        with self._lock:
            self._expire_alignment_locked()
            if not self._state.get("alignment_active"):
                return False
            if self._state.get("alignment_status") not in {"WAITING", "DETECTED"}:
                return False
            if expected_service_type:
                current = str(self._state.get("alignment_service_type") or "").upper()
                if current != expected_service_type.upper():
                    return False

            data = dict(payload or {})
            data.setdefault("source", source)
            self._state["alignment_last_payload"] = data
            self._state["alignment_last_signal"] = bool(detected)

            if detected:
                self._state.update(
                    alignment_status="DETECTED",
                    alignment_detected=True,
                )
            return True

    def record_tracking_state(
        self,
        *,
        value: int,
        source: str,
        expected_service_type: str,
    ) -> bool:
        """로봇3 비전 노드의 Int32 추적 상태를 반영한다.

        0: 트래킹 안 함/검출 대기
        1: 타깃이 카메라에 있으며 트래킹 중
        2: 트래킹 중 타깃이 카메라에서 사라짐

        GUIDE에서는 /turn_complete=True 전까지 들어온 값은 판정하지 않는다.
        이후 WAITING에서 1이 설정 프레임 수 이상 연속으로 들어오면 DETECTED가 된다.
        0/2는 retargeting 대기 상태로 유지하며 재회전이나 재정렬을 유발하지 않는다.
        """
        if value not in {0, 1, 2}:
            return False

        with self._lock:
            self._expire_alignment_locked()
            if not self._state.get("alignment_active"):
                return False

            current_service = str(self._state.get("alignment_service_type") or "").upper()
            if current_service != expected_service_type.upper():
                return False

            status = str(self._state.get("alignment_status") or "IDLE")

            # 한 번 DETECTED가 되면 결과를 latch한다.
            # 직후 들어오는 0/2 때문에 서비스 시작 전 상태가 뒤집히면 안 된다.
            if status == "DETECTED":
                return False

            if current_service == "GUIDE":
                if status == "TURNING_TO_REAR":
                    # 회전 중에는 후방 웹캠 방향이 아직 안정되지 않았으므로
                    # 0/1/2 모두 최종 판정에 넣지 않는다. 최신값만 UI 표시용으로 저장한다.
                    self._state.update(
                        alignment_tracking_value=int(value),
                        alignment_tracking_topic=source,
                        alignment_last_signal=int(value),
                        alignment_last_payload={
                            "source": source,
                            "tracking_value": int(value),
                            "ignored_while_turning": True,
                        },
                    )
                    return False

                if status != "WAITING":
                    return False

                previous_streak = int(self._state.get("alignment_tracking_one_streak") or 0)
                next_streak = previous_streak + 1 if value == 1 else 0
                next_max_streak = max(
                    int(self._state.get("alignment_tracking_one_max_streak") or 0),
                    next_streak,
                )
                required = max(
                    1, int(self._state.get("alignment_tracking_confirm_frames") or 1)
                )
                sample_count = int(self._state.get("alignment_tracking_sample_count") or 0) + 1

                self._state.update(
                    alignment_tracking_value=int(value),
                    alignment_tracking_topic=source,
                    alignment_tracking_one_streak=next_streak,
                    alignment_tracking_one_max_streak=next_max_streak,
                    alignment_tracking_sample_count=sample_count,
                    alignment_last_signal=int(value),
                    alignment_last_payload={
                        "source": source,
                        "tracking_value": int(value),
                        "one_streak": next_streak,
                        "required_frames": required,
                        "sample_count": sample_count,
                    },
                )

                if next_streak >= required:
                    self._state.update(
                        alignment_status="DETECTED",
                        alignment_detected=True,
                        alignment_deadline_monotonic=None,
                        alignment_accept_tracking_after_monotonic=None,
                        alignment_turn_direction="",
                        alignment_turn_deadline_monotonic=None,
                    )
                    return True

                # 0/2는 후방 비전 retargeting 중인 정상 대기 상태다.
                # 로봇 방향과 현재 화면을 유지하고 다음 tracking_web 값을 기다린다.
                return value == 1

            # LUGGAGE_ASSIST는 기존대로 WAITING 상태에서 /tracking_rgbd=1이면 성공,
            # 0 또는 2이면 같은 짐 적재 화면에서 계속 대기한다.
            if status not in {"WAITING", "DETECTED"}:
                return False

            self._state.update(
                alignment_tracking_value=int(value),
                alignment_tracking_topic=source,
                alignment_last_signal=int(value),
                alignment_last_payload={
                    "source": source,
                    "tracking_value": int(value),
                },
            )

            if value == 1:
                self._state.update(
                    alignment_status="DETECTED",
                    alignment_detected=True,
                    alignment_deadline_monotonic=None,
                    alignment_accept_tracking_after_monotonic=None,
                    alignment_turn_direction="",
                    alignment_turn_deadline_monotonic=None,
                )
                return True

            self._state.update(
                alignment_status="WAITING",
                alignment_detected=False,
            )
            return True


    def alignment_can_confirm(self, service_type: str) -> bool:
        with self._lock:
            self._expire_alignment_locked()
            return bool(
                self._state.get("alignment_active")
                and self._state.get("alignment_detected")
                and self._state.get("alignment_status") == "DETECTED"
                and str(self._state.get("alignment_service_type") or "").upper()
                == service_type.upper()
            )

    def finish_alignment(self) -> dict[str, Any]:
        with self._lock:
            self._state.update(
                alignment_active=False,
                alignment_status="CONFIRMED",
                alignment_detected=True,
                alignment_deadline_monotonic=None,
                alignment_accept_tracking_after_monotonic=None,
                alignment_turn_direction="",
                alignment_turn_deadline_monotonic=None,
            )
            return deepcopy(self._state)

    def mark_mission_active(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._state.update(
                active=True,
                arrived=False,
                awaiting_service_end=False,
                request_id=payload.get("request_id"),
                mission_id=payload.get("mission_id"),
                service_type=payload.get("service_type") or "",
                mode=payload.get("mode") or "",
                goal_id=payload.get("goal_id"),
                destination_label=payload.get("destination_label") or "",
                arrival_payload=None,
                arrival_time=None,
                alignment_active=False,
                alignment_status="CONFIRMED",
            )
            return deepcopy(self._state)

    def mark_manual_arrival(self, source: str = "passenger_ui") -> bool:
        with self._lock:
            if self._state.get("emergency_active") or not self._state.get("active"):
                return False
            self._state.update(
                arrived=True,
                awaiting_service_end=True,
                arrival_payload={"source": source, "arrival_status": "COMPLETED"},
                arrival_time=datetime.now().isoformat(timespec="seconds"),
            )
            return True

    def accept_arrival(self, payload: dict[str, Any]) -> bool:
        with self._lock:
            if self._state.get("emergency_active") or not self._state.get("active"):
                return False

            current_robot = self._state.get("robot_id")
            incoming_robot = payload.get("robot_id")
            if incoming_robot and current_robot and incoming_robot != current_robot:
                return False

            current_mission = self._state.get("mission_id")
            incoming_mission = payload.get("mission_id")
            if incoming_mission and current_mission and incoming_mission != current_mission:
                return False

            service_type = payload.get("service_type") or self._state.get("service_type") or "GUIDE"
            mode = payload.get("mode") or self._state.get("mode") or (
                "NAVIGATION" if service_type == "GUIDE" else "FOLLOWING"
            )
            self._state.update(
                active=True,
                arrived=True,
                awaiting_service_end=True,
                robot_id=incoming_robot or current_robot,
                service_type=service_type,
                mode=mode,
                goal_id=payload.get("goal_id") or self._state.get("goal_id"),
                destination_label=(
                    payload.get("destination_label")
                    or self._state.get("destination_label")
                    or "목적지"
                ),
                arrival_payload=deepcopy(payload),
                arrival_time=datetime.now().isoformat(timespec="seconds"),
            )
            return True

    def activate_emergency(self, payload: dict[str, Any]) -> tuple[bool, dict[str, Any]]:
        with self._lock:
            if self._state.get("emergency_active"):
                previous = self._state.get("interrupted_state") or self._state
                return False, deepcopy(previous)

            previous = deepcopy(self._state)
            self._state.update(
                active=False,
                arrived=False,
                awaiting_service_end=False,
                alignment_active=False,
                alignment_status="INTERRUPTED",
                emergency_active=True,
                emergency_type=str(payload.get("event_type") or "FALL_DETECTED"),
                emergency_payload=deepcopy(payload),
                emergency_time=datetime.now().isoformat(timespec="seconds"),
                interrupted_state=previous,
            )
            return True, previous

    def clear_emergency(self, robot_id: str) -> dict[str, Any]:
        with self._lock:
            self._state = self._initial_state(robot_id)
            return deepcopy(self._state)
