from __future__ import annotations

from datetime import datetime
from typing import Any
import uuid


class AuraMessageService:
    """UI 이벤트를 ROS2 std_msgs/msg/String(JSON) payload로 변환한다.

    AMR1은 미션/상태 이벤트를 String(JSON)으로 중앙에 전달한다.
    단, 첫 터치 즉시 정지용 /rb1_standby는 AuraRosBridge에서 Bool(True) 직접 명령으로 보낸다.
    AMR3도 /aura/service_request, /aura/service_end 등 중앙 상태 관리용
    이벤트는 동일한 String(JSON) 구조를 유지한다. 로봇3 직접 Bool(True)
    명령은 AuraRosBridge에서 별도로 발행한다.
    """

    SCHEMA_VERSION = "1.0"

    def __init__(self, bridge, config):
        self.bridge = bridge
        self.config = config

    @staticmethod
    def make_request_id(prefix: str) -> str:
        now = datetime.now().strftime("%Y%m%d%H%M%S")
        unique = uuid.uuid4().hex[:6].upper()
        return f"{prefix}-{now}-{unique}"

    @classmethod
    def _base_payload(cls) -> dict[str, Any]:
        return {
            "schema_version": cls.SCHEMA_VERSION,
            "source": "passenger_mobile_ui",
            "ui_language": "ko",
            "sent_at": datetime.now().isoformat(timespec="seconds"),
        }

    def publish_standby_request(self, robot_id: str) -> dict[str, Any]:
        """구형 호환용 String(JSON) standby 요청. 현재 AMR1 정지용으로는 사용하지 않는다."""
        payload = self._base_payload()
        payload.update(
            event_type="STANDBY_REQUEST",
            request_id=self.make_request_id("STANDBY"),
            robot_id=robot_id,
            from_mode="PATROLLING",
            requested_mode="STANDBY",
            reason="USER_TOUCH",
            request_status="REQUESTED",
        )
        self.bridge.publish_json(self.config.RB1_STANDBY_TOPIC, payload)
        return payload

    def publish_robot_select(self, robot_id: str) -> dict[str, Any]:
        """다중 로봇 선택형 UI 호환용. 로봇 고정형 AMR1에서는 보통 사용하지 않는다."""
        payload = self._base_payload()
        payload.update(
            event_type="ROBOT_SELECT",
            request_id=self.make_request_id("ROBOT"),
            robot_id=robot_id,
            select_status="REQUESTED",
        )
        self.bridge.publish_json(self.config.ROBOT_SELECT_TOPIC, payload)
        return payload

    def publish_luggage_load_confirm(self, robot_id: str) -> dict[str, Any]:
        payload = self._base_payload()
        payload.update(
            event_type="LUGGAGE_LOAD_CONFIRMED",
            request_id=self.make_request_id("LUGGAGE"),
            robot_id=robot_id,
            service_type="LUGGAGE_ASSIST",
            load_status="COMPLETED",
        )
        self.bridge.publish_json(self.config.LUGGAGE_LOAD_CONFIRM_TOPIC, payload)
        return payload

    def publish_align_request(
        self,
        *,
        robot_id: str,
        service_type: str,
        camera: str,
        detector: str,
        rotate_degrees: int,
        retry: bool,
        attempt: int,
    ) -> dict[str, Any]:
        payload = self._base_payload()
        payload.update(
            event_type="ALIGN_REQUEST",
            request_id=self.make_request_id("ALIGN"),
            robot_id=robot_id,
            service_type=service_type,
            mode="NAVIGATION" if service_type == "GUIDE" else "FOLLOWING",
            camera=camera,
            detector=detector,
            rotate_degrees=int(rotate_degrees),
            retry=bool(retry),
            attempt=int(attempt),
            status="REQUESTED",
        )
        self.bridge.publish_json(self.config.ALIGN_REQUEST_TOPIC, payload)
        return payload

    def publish_align_confirm(
        self,
        *,
        robot_id: str,
        service_type: str,
        camera: str,
        detector: str,
        attempt: int,
    ) -> dict[str, Any]:
        payload = self._base_payload()
        payload.update(
            event_type="ALIGN_CONFIRMED",
            request_id=self.make_request_id("ALIGNCONFIRM"),
            robot_id=robot_id,
            service_type=service_type,
            mode="NAVIGATION" if service_type == "GUIDE" else "FOLLOWING",
            camera=camera,
            detector=detector,
            attempt=int(attempt),
            detected=True,
            status="CONFIRMED",
        )
        self.bridge.publish_json(self.config.ALIGN_CONFIRM_TOPIC, payload)
        return payload

    def publish_service_request(
        self,
        *,
        robot_id: str,
        service_type: str,
        goal_id: str | None = None,
        destination_label: str | None = None,
    ) -> dict[str, Any]:
        """중앙 Dispatcher에 실제 서비스 배정을 요청한다."""
        payload = self._base_payload()
        request_id = self.make_request_id("SERVICE")

        if service_type == "GUIDE":
            if not goal_id:
                raise ValueError("GUIDE 서비스에는 goal_id가 필요합니다.")
            payload.update(
                event_type="SERVICE_REQUEST",
                request_id=request_id,
                robot_id=robot_id,
                service_type="GUIDE",
                mode="NAVIGATION",
                requested_mode="GUIDE",
                needs_goal=True,
                goal_id=goal_id,
                destination_label=destination_label,
                request_status="REQUESTED",
            )
        elif service_type == "LUGGAGE_ASSIST":
            payload.update(
                event_type="SERVICE_REQUEST",
                request_id=request_id,
                robot_id=robot_id,
                service_type="LUGGAGE_ASSIST",
                mode="FOLLOWING",
                requested_mode="LUGGAGE_ASSIST",
                needs_goal=False,
                goal_id=None,
                destination_label=destination_label or "승객 동행",
                request_status="REQUESTED",
            )
        else:
            raise ValueError(f"지원하지 않는 서비스입니다: {service_type}")

        self.bridge.publish_json(self.config.SERVICE_REQUEST_TOPIC, payload)
        return payload

    def publish_service_end(
        self,
        *,
        robot_id: str,
        previous_state: dict[str, Any],
        end_reason: str,
    ) -> dict[str, Any]:
        """현재 승객 서비스를 종료하고 다음 모드를 요청한다."""
        payload = self._base_payload()
        payload.update(
            event_type="SERVICE_END",
            request_id=self.make_request_id("END"),
            robot_id=robot_id,
            previous_request_id=previous_state.get("request_id"),
            previous_mission_id=previous_state.get("mission_id"),
            previous_service_type=previous_state.get("service_type") or "UNKNOWN",
            previous_mode=previous_state.get("mode") or "UNKNOWN",
            previous_goal_id=previous_state.get("goal_id"),
            previous_destination_label=previous_state.get("destination_label"),
            end_reason=end_reason,
            end_status="REQUESTED",
        )

        if end_reason == "EMERGENCY_INTERRUPTED":
            payload.update(
                emergency_override=True,
                next_mode="EMERGENCY",
                return_action="DISPATCHER_DECIDES",
                patrol_resume=False,
            )
        elif robot_id == "AMR1":
            # 로봇1 시연에서는 안내 종료 후 면세구역 순찰로 바로 복귀한다.
            payload.update(
                next_mode="PATROLLING",
                return_action="RESUME_PATROL",
                patrol_resume=True,
                patrol_zone="DUTY_FREE_ZONE",
            )
        elif robot_id == "AMR3":
            # 중앙에는 종료 사실을 JSON으로 알리고,
            # 실제 로봇3 순찰 복귀는 /service_end Bool(True)가 직접 트리거한다.
            payload.update(
                next_mode="PATROLLING",
                return_action="RESUME_PATROL",
                patrol_resume=True,
                patrol_zone="TERMINAL_ZONE",
            )
        else:
            payload.update(
                next_mode="RETURNING",
                return_action="RETURN_TO_STANDBY",
                patrol_resume=False,
            )

        if end_reason == "INACTIVITY_TIMEOUT":
            payload["inactivity_timeout_seconds"] = self.config.INACTIVITY_TIMEOUT_SECONDS

        self.bridge.publish_json(self.config.SERVICE_END_TOPIC, payload)
        return payload

    def publish_emergency_end(
        self,
        *,
        robot_id: str,
        emergency_state: dict[str, Any],
    ) -> dict[str, Any]:
        """응급 화면의 상황 종료 버튼을 중앙 Dispatcher에 전달한다."""
        emergency_payload = emergency_state.get("emergency_payload") or {}
        payload = self._base_payload()
        payload.update(
            event_type="EMERGENCY_END",
            request_id=self.make_request_id("EMERGENCYEND"),
            robot_id=robot_id,
            emergency_id=emergency_payload.get("emergency_id"),
            emergency_type=(
                emergency_state.get("emergency_type")
                or emergency_payload.get("event_type")
                or "FALL_DETECTED"
            ),
            end_reason="ON_SITE_CONFIRMATION",
            next_mode="PATROLLING",
            patrol_resume=True,
            patrol_zone=("TERMINAL_ZONE" if robot_id == "AMR3" else "DUTY_FREE_ZONE"),
            request_status="REQUESTED",
        )
        self.bridge.publish_json(self.config.EMERGENCY_END_TOPIC, payload)
        return payload
