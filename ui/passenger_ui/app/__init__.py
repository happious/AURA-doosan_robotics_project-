from __future__ import annotations

import atexit

from flask import Flask

from .config import Config
from .message_service import AuraMessageService
from .ros_bridge import AuraRosBridge
from .state import MissionStateStore


def create_app() -> Flask:
    app = Flask(__name__, template_folder="templates", static_folder="static")
    app.config.from_object(Config)

    state_store = MissionStateStore(app.config["ROBOT_ID"])
    bridge = AuraRosBridge(config=Config, state_store=state_store)
    message_service = AuraMessageService(bridge, Config)

    def handle_emergency_detected(payload: dict) -> None:
        """낙상 감지 시 기존 서비스 중단 요청과 응급 UI 상태를 한 번만 만든다."""
        newly_activated, previous_state = state_store.activate_emergency(payload)
        if not newly_activated:
            print("[AURA UI] 이미 응급상황 활성화 중 - 중복 감지 무시", flush=True)
            return

        # 실제 수행 중이던 GUIDE/FOLLOWING 미션이 있을 때만 중단 요청을 발행한다.
        if previous_state.get("active"):
            message_service.publish_service_end(
                robot_id=previous_state.get("robot_id") or app.config["ROBOT_ID"],
                previous_state=previous_state,
                end_reason="EMERGENCY_INTERRUPTED",
            )
            print("[AURA UI] 기존 승객 서비스 중단 요청 발행", flush=True)

        print("[AURA UI] 응급상황 화면 전환 상태 활성화", flush=True)

    def handle_emergency_cleared(payload: dict) -> None:
        """중앙 시스템이 상황 종료를 확인한 뒤 일반 대기 상태로 복귀한다."""
        state_store.clear_emergency(app.config["ROBOT_ID"])
        print(f"[AURA UI] 응급상황 해제: {payload}", flush=True)

    bridge.set_emergency_handlers(
        on_detected=handle_emergency_detected,
        on_cleared=handle_emergency_cleared,
    )

    app.extensions["aura_state_store"] = state_store
    app.extensions["aura_ros_bridge"] = bridge
    app.extensions["aura_message_service"] = message_service

    from .routes.api import api_bp
    from .routes.passenger import passenger_bp

    app.register_blueprint(passenger_bp)
    app.register_blueprint(api_bp)

    atexit.register(bridge.shutdown)
    return app
