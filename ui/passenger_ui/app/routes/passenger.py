from __future__ import annotations

from flask import Blueprint, current_app, redirect, render_template, request, session, url_for

passenger_bp = Blueprint("passenger", __name__)


def _state_store():
    return current_app.extensions["aura_state_store"]


def _bridge():
    return current_app.extensions["aura_ros_bridge"]


def _messages():
    return current_app.extensions["aura_message_service"]


def _robot_id() -> str:
    return current_app.config["ROBOT_ID"]


def _initialize_session() -> None:
    session["robot_id"] = _robot_id()
    session.setdefault("robot_connected", False)
    session.setdefault("pending_service", None)
    session.setdefault("goal_id", None)
    session.setdefault("destination_label", "")
    session.setdefault("destination_key", "")
    session.setdefault("retry_auto_started_attempt", 0)


def _clear_pending_service() -> None:
    session["pending_service"] = None
    session["goal_id"] = None
    session["destination_label"] = ""
    session["destination_key"] = ""
    session["retry_auto_started_attempt"] = 0


def _alignment_definition(service_type: str) -> dict:
    alignment_mode = current_app.config["ALIGNMENT_INPUT_MODE"]
    tracking_mode = alignment_mode == "tracking_int32"
    manual_mode = alignment_mode == "manual"

    if service_type == "GUIDE":
        return {
            "mode": "guide",
            "camera": "NONE" if manual_mode else "REAR",
            "camera_key": "none" if manual_mode else "rear",
            "detector": (
                "MANUAL_CONFIRM" if manual_mode
                else "TRACKING_WEB" if tracking_mode
                else "PERSON_YOLO"
            ),
            "tracking_topic": (
                current_app.config["TRACKING_WEB_TOPIC"] if tracking_mode else ""
            ),
            "rotate_degrees": 0 if manual_mode else 180,
        }
    if service_type == "LUGGAGE_ASSIST":
        return {
            "mode": "follow",
            "camera": "NONE" if manual_mode else "FRONT",
            "camera_key": "none" if manual_mode else "front",
            "detector": (
                "MANUAL_CONFIRM" if manual_mode
                else "TRACKING_RGBD" if tracking_mode
                else "LEG_YOLO"
            ),
            "tracking_topic": (
                current_app.config["TRACKING_RGBD_TOPIC"] if tracking_mode else ""
            ),
            "rotate_degrees": 0,
        }
    raise ValueError(f"지원하지 않는 정렬 서비스입니다: {service_type}")


def _start_alignment(service_type: str, *, retry: bool) -> dict:
    definition = _alignment_definition(service_type)
    current = _state_store().get()
    next_attempt = int(current.get("alignment_attempt") or 0) + 1
    alignment_input_mode = current_app.config["ALIGNMENT_INPUT_MODE"]

    if alignment_input_mode == "manual":
        # AMR1은 카메라가 없으므로 사용자가 준비를 확인한 뒤 버튼을 누른다.
        # 화면 진입 시 버튼만 활성화하고 실제 요청은 confirm_alignment에서 보낸다.
        return _state_store().prepare_manual_confirm_alignment(
            service_type=service_type,
            mode=definition["mode"],
            retry=retry,
        )

    if alignment_input_mode == "tracking_int32":
        # Int32 추적 방식에서는 회전 명령과 추적 대기 시작을 분리한다.
        # GUIDE는 사용자가 전방 정렬 완료 버튼을 누른 뒤 별도 route에서
        # /turn_around=True를 발행하고 이 함수로 들어온다.
        request_id = _messages().make_request_id("ALIGN")
    else:
        rotate_degrees = 0 if retry else definition["rotate_degrees"]
        payload = _messages().publish_align_request(
            robot_id=_robot_id(),
            service_type=service_type,
            camera=definition["camera"],
            detector=definition["detector"],
            rotate_degrees=rotate_degrees,
            retry=retry,
            attempt=next_attempt,
        )
        request_id = payload["request_id"]

    return _state_store().start_alignment(
        service_type=service_type,
        mode=definition["mode"],
        camera=definition["camera"],
        detector=definition["detector"],
        retry=retry,
        request_id=request_id,
        timeout_seconds=(
            0.0
            if service_type == "LUGGAGE_ASSIST" and alignment_input_mode == "tracking_int32"
            else current_app.config["ALIGN_DETECTION_TIMEOUT_SECONDS"]
        ),
        tracking_topic=definition["tracking_topic"],
        # Robot3 GUIDE는 시간 기반 grace를 사용하지 않는다.
        # /turn_complete=True 수신 전에는 tracking_web을 판정하지 않고,
        # 수신 후 WAITING 상태에서만 연속 프레임을 확인한다.
        tracking_grace_seconds=0.0,
        tracking_confirm_frames=(
            current_app.config["TRACKING_WEB_CONFIRM_FRAMES"]
            if service_type == "GUIDE" and alignment_input_mode == "tracking_int32"
            else 1
        ),
    )


def _prepare_guide_front_alignment(*, retry: bool) -> dict:
    """GUIDE 승객이 로봇 전방에 서서 버튼을 누르기 전 준비 상태를 만든다."""
    definition = _alignment_definition("GUIDE")
    return _state_store().prepare_manual_alignment(
        service_type="GUIDE",
        mode=definition["mode"],
        camera=definition["camera"],
        detector=definition["detector"],
        retry=retry,
        tracking_topic=definition["tracking_topic"],
    )


def _render_alignment(service_type: str, *, retry_screen: bool):
    _initialize_session()
    if session.get("pending_service") != service_type:
        return redirect(url_for("passenger.service_selection"))

    definition = _alignment_definition(service_type)
    state = _state_store().get()
    guide_turn_flow = (
        service_type == "GUIDE"
        and current_app.config["ALIGNMENT_INPUT_MODE"] == "tracking_int32"
    )
    prealign = guide_turn_flow and state.get("alignment_status") == "PREALIGN"

    manual_alignment = current_app.config["ALIGNMENT_INPUT_MODE"] == "manual"

    # UI는 영상 스트림을 표시하지 않는다.
    # 비전 컴퓨터가 발행하는 /tracking_web, /tracking_rgbd 상태값만 사용한다.

    template = (
        "04_1_align_passenger.html"
        if service_type == "GUIDE"
        else "04_2_align_passenger_retry.html"
        if retry_screen
        else "04_2_align_passenger.html"
    )
    return render_template(
        template,
        state=state,
        retry_screen=False if service_type == "GUIDE" else retry_screen,
        robot_id=_robot_id(),
        mission_type=definition["mode"],
        guide_turn_flow=guide_turn_flow,
        manual_alignment=manual_alignment,
        prealign=prealign,
        detector_label="사람" if service_type == "GUIDE" else "다리",
        alignment_input_mode=current_app.config["ALIGNMENT_INPUT_MODE"],
        tracking_topic=definition["tracking_topic"],
        destination_label=session.get("destination_label"),
        inactivity_timeout_seconds=current_app.config["INACTIVITY_TIMEOUT_SECONDS"],
    )


@passenger_bp.route("/")
def ad():
    state = _state_store().get()
    if state.get("emergency_active"):
        return redirect(url_for("passenger.emergency"))
    if state.get("active"):
        return redirect(url_for("passenger.mission_progress"))

    # 정렬/회전 진행 중에 같은 UI의 루트 화면이 새로 열리더라도 전역 상태를
    # IDLE로 초기화하지 않는다. 기존에는 다른 탭/브라우저의 GET / 요청 하나가
    # tracking_web 수신 상태를 지워 정렬 완료 화면에 멈출 수 있었다.
    if state.get("alignment_active"):
        alignment_service = str(state.get("alignment_service_type") or "")
        if session.get("pending_service") == alignment_service:
            retry = bool(state.get("alignment_retry"))
            if alignment_service == "GUIDE":
                endpoint = (
                    "passenger.align_guide_retry"
                    if retry
                    else "passenger.align_guide"
                )
            else:
                endpoint = (
                    "passenger.align_luggage_retry"
                    if retry
                    else "passenger.align_luggage"
                )
            return redirect(url_for(endpoint))

        # 다른 브라우저 세션에서는 광고 화면만 보여주되 진행 중인 로봇 상태는
        # 건드리지 않는다.
        return render_template(
            "00_ad.html",
            robot_id=_robot_id(),
            system_state_url=url_for("api.system_state"),
        )

    session.clear()
    return render_template(
        "00_ad.html",
        robot_id=_robot_id(),
        system_state_url=url_for("api.system_state"),
    )


@passenger_bp.route("/debug/home", methods=["POST"])
def debug_home():
    """디버깅용: 로봇 제어 토픽 없이 UI 상태와 세션만 최초 화면으로 초기화한다."""
    robot_id = _robot_id()
    session.clear()
    _state_store().reset(robot_id)
    print(
        f"[AURA UI DEBUG] {robot_id} UI 상태 초기화 -> 00_AD",
        flush=True,
    )
    return redirect(url_for("passenger.ad"))


@passenger_bp.route("/wake", methods=["GET", "POST"])
def wake():
    """00_AD 첫 터치 시 순찰을 멈추고 로봇 대기 화면으로 전환한다."""
    _initialize_session()
    robot_id = _robot_id()
    _clear_pending_service()
    _state_store().reset(robot_id)

    if current_app.config["CONTROL_MODE"] == "CENTRAL_JSON" and robot_id == "AMR1":
        # AMR1은 터치 즉시 멈춰야 하므로 중앙노드 JSON을 거치지 않고
        # /rb1_standby Bool(data=True)를 직접 발행하고 DDS ACK까지 확인한다.
        ok = _bridge().publish_rb1_standby()
        if not ok:
            print(
                "[AURA UI ROUTE ERROR] /rb1_standby 발행 실패 - standby 화면 전환은 계속 진행",
                flush=True,
            )
    elif robot_id == "AMR3":
        _bridge().publish_rb3_standby()
        _messages().publish_robot_select(robot_id)
    else:
        _messages().publish_robot_select(robot_id)

    session["robot_woken"] = True
    return redirect(url_for("passenger.standby"))


@passenger_bp.route("/standby")
def standby():
    _initialize_session()
    reason = request.args.get("reason")
    return render_template(
        "01_standby.html",
        robot_id=_robot_id(),
        timeout_notice=reason == "inactivity_timeout",
        emergency_resolved_notice=reason == "emergency_resolved",
    )


@passenger_bp.route("/start", methods=["GET", "POST"])
def start():
    """이미 정지된 로봇에서 서비스 선택 세션을 시작한다."""
    _initialize_session()
    robot_id = _robot_id()
    session["robot_connected"] = True
    _clear_pending_service()
    _state_store().reset(robot_id)
    return redirect(url_for("passenger.service_selection"))


@passenger_bp.route("/service-selection")
def service_selection():
    _initialize_session()
    if not session.get("robot_connected"):
        return redirect(url_for("passenger.standby"))
    return render_template(
        "02_service_selection.html",
        robot_id=_robot_id(),
        luggage_enabled=current_app.config["ENABLE_LUGGAGE_SERVICE"],
        inactivity_timeout_seconds=current_app.config["INACTIVITY_TIMEOUT_SECONDS"],
    )


@passenger_bp.route("/destination-select")
def destination_select():
    _initialize_session()
    if not session.get("robot_connected"):
        return redirect(url_for("passenger.standby"))
    return render_template(
        "03_1_destination_select.html",
        destinations=current_app.config["DESTINATIONS"],
        inactivity_timeout_seconds=current_app.config["INACTIVITY_TIMEOUT_SECONDS"],
    )


@passenger_bp.route("/guide/prepare/<dest_key>", methods=["GET", "POST"])
def prepare_guide(dest_key: str):
    _initialize_session()
    destination = current_app.config["DESTINATIONS"].get(dest_key)
    if not destination or not destination.get("enabled"):
        return redirect(url_for("passenger.destination_select"))

    session["pending_service"] = "GUIDE"
    session["goal_id"] = destination["goal_id"]
    session["destination_label"] = destination["label"]
    session["destination_key"] = dest_key
    session["retry_auto_started_attempt"] = 0

    if current_app.config["ALIGNMENT_INPUT_MODE"] == "tracking_int32":
        # 목적지 선택 직후에는 회전하지 않는다.
        # 사람이 로봇 전방에 선 뒤 UI 정렬 완료 버튼을 눌러야 회전한다.
        _prepare_guide_front_alignment(retry=False)
    else:
        _start_alignment("GUIDE", retry=False)
    return redirect(url_for("passenger.align_guide"))


@passenger_bp.route("/luggage-loading")
def luggage_loading():
    _initialize_session()
    if not current_app.config["ENABLE_LUGGAGE_SERVICE"]:
        return redirect(url_for("passenger.service_selection"))
    if not session.get("robot_connected"):
        return redirect(url_for("passenger.standby"))

    state = _state_store().get()
    waiting_tracking = bool(
        session.get("pending_service") == "LUGGAGE_ASSIST"
        and state.get("alignment_active")
        and state.get("alignment_service_type") == "LUGGAGE_ASSIST"
        and state.get("alignment_status") in {"WAITING", "DETECTED"}
    )

    return render_template(
        "03_2_luggage_loading.html",
        state=state,
        waiting_tracking=waiting_tracking,
        inactivity_timeout_seconds=current_app.config["INACTIVITY_TIMEOUT_SECONDS"],
    )


@passenger_bp.route("/luggage/prepare", methods=["GET", "POST"])
def prepare_luggage():
    _initialize_session()
    if not current_app.config["ENABLE_LUGGAGE_SERVICE"]:
        return redirect(url_for("passenger.service_selection"))
    session["pending_service"] = "LUGGAGE_ASSIST"
    session["goal_id"] = None
    session["destination_label"] = "승객 동행"
    session["destination_key"] = ""
    session["retry_auto_started_attempt"] = 0
    _messages().publish_luggage_load_confirm(_robot_id())

    if current_app.config["ALIGNMENT_INPUT_MODE"] == "tracking_int32":
        # 짐 싣기 완료 시 로봇3 비전 노드를 활성화한다.
        # /carrying=True 발행 후 /tracking_rgbd 상태를 기다린다.
        _bridge().publish_carrying()

    _start_alignment("LUGGAGE_ASSIST", retry=False)

    # 짐 들기는 별도의 카메라 정렬 화면으로 이동하지 않는다.
    # 같은 짐 적재 화면에서 /tracking_rgbd=1을 기다렸다가 자동으로 출발한다.
    return redirect(url_for("passenger.luggage_loading", tracking="1"))


@passenger_bp.route("/align/guide")
def align_guide():
    return _render_alignment("GUIDE", retry_screen=False)


@passenger_bp.route("/align/luggage")
def align_luggage():
    return _render_alignment("LUGGAGE_ASSIST", retry_screen=False)


def _retry_alignment_page(service_type: str):
    _initialize_session()
    if session.get("pending_service") != service_type:
        return redirect(url_for("passenger.service_selection"))

    state = _state_store().get()
    current_attempt = int(state.get("alignment_attempt") or 0)
    auto_started_attempt = int(session.get("retry_auto_started_attempt") or 0)

    if state.get("alignment_status") == "FAILED":
        if service_type != "GUIDE" and auto_started_attempt < current_attempt + 1:
            # LUGGAGE 또는 기존 Bool 방식은 재검출을 자동 시작한다.
            state = _start_alignment(service_type, retry=True)
            session["retry_auto_started_attempt"] = int(state.get("alignment_attempt") or 0)

    return _render_alignment(service_type, retry_screen=True)


@passenger_bp.route("/align/guide/retry")
def align_guide_retry():
    # GUIDE는 최초 1회 정렬만 사용한다. 이전 링크 호환을 위해 기본 정렬 화면으로 보낸다.
    return redirect(url_for("passenger.align_guide"))


@passenger_bp.route("/align/luggage/retry")
def align_luggage_retry():
    return _retry_alignment_page("LUGGAGE_ASSIST")


@passenger_bp.route("/align/guide/turn", methods=["POST"])
def turn_guide_to_rear_camera():
    """사람의 전방 정렬 완료 후 /turn_around=True를 1회 발행한다."""
    print("[AURA UI ROUTE] GUIDE 정렬 완료 요청 수신", flush=True)
    _initialize_session()

    if session.get("pending_service") != "GUIDE":
        print(
            "[AURA UI ROUTE ERROR] pending_service가 GUIDE가 아니어서 "
            "/turn_around를 발행하지 않았습니다.",
            flush=True,
        )
        return redirect(url_for("passenger.service_selection"))

    if current_app.config["ALIGNMENT_INPUT_MODE"] != "tracking_int32":
        print(
            "[AURA UI ROUTE ERROR] tracking_int32 모드가 아니어서 "
            "/turn_around를 발행하지 않았습니다.",
            flush=True,
        )
        return redirect(url_for("passenger.align_guide"))

    state = _state_store().get()
    if state.get("alignment_status") != "PREALIGN":
        print(
            "[AURA UI ROUTE ERROR] 정렬 상태가 PREALIGN이 아닙니다: "
            f"{state.get('alignment_status')}",
            flush=True,
        )
        endpoint = (
            "passenger.align_guide_retry"
            if state.get("alignment_retry") else "passenger.align_guide"
        )
        return redirect(url_for(endpoint))

    retry_screen = request.form.get("retry_screen") == "1"
    alignment_armed = False

    def arm_tracking_before_publish() -> None:
        nonlocal alignment_armed
        # 중요: /turn_around를 실제 publish하기 직전에 TURNING_TO_REAR 상태로 바꾼다.
        # 이 상태에서는 /tracking_web 값은 수신되더라도 판정하지 않는다.
        # Robot3의 /turn_complete=True 수신 후 WAITING부터 N프레임 연속 1을 센다.
        _start_alignment("GUIDE", retry=retry_screen)
        alignment_armed = True
        print(
            "[AURA UI STATE] GUIDE alignment PREALIGN -> TURNING_TO_REAR "
            "(/turn_around publish 직전)",
            flush=True,
        )

    published = _bridge().publish_turn_around(
        before_publish=arm_tracking_before_publish
    )

    # /turn_around는 subscriber 사전 대기 없이 1회 발행한다.
    # publish 준비 콜백 또는 실제 publish 자체가 실패한 경우에만 PREALIGN으로 복구한다.
    if not published:
        if alignment_armed:
            _prepare_guide_front_alignment(retry=retry_screen)
        print(
            "[AURA UI ROUTE ERROR] /turn_around 발행 실패 - PREALIGN 유지",
            flush=True,
        )
        endpoint = "passenger.align_guide_retry" if retry_screen else "passenger.align_guide"
        return redirect(url_for(endpoint, publish_error="1"))

    endpoint = "passenger.align_guide_retry" if retry_screen else "passenger.align_guide"
    return redirect(url_for(endpoint))


@passenger_bp.route("/align/retry/start", methods=["GET", "POST"])
def restart_alignment():
    _initialize_session()
    service_type = session.get("pending_service")
    if service_type not in {"GUIDE", "LUGGAGE_ASSIST"}:
        return redirect(url_for("passenger.service_selection"))
    if (
        service_type == "GUIDE"
        and current_app.config["ALIGNMENT_INPUT_MODE"] == "tracking_int32"
    ):
        state = _prepare_guide_front_alignment(retry=True)
    else:
        state = _start_alignment(service_type, retry=True)
        session["retry_auto_started_attempt"] = int(state.get("alignment_attempt") or 0)
    endpoint = "passenger.align_guide_retry" if service_type == "GUIDE" else "passenger.align_luggage_retry"
    return redirect(url_for(endpoint))


@passenger_bp.route("/confirm-alignment", methods=["GET", "POST"])
def confirm_alignment():
    _initialize_session()
    service_type = session.get("pending_service")
    if service_type not in {"GUIDE", "LUGGAGE_ASSIST"}:
        return redirect(url_for("passenger.service_selection"))

    if not _state_store().alignment_can_confirm(service_type):
        endpoint = "passenger.align_guide_retry" if service_type == "GUIDE" else "passenger.align_luggage_retry"
        return redirect(url_for(endpoint))

    definition = _alignment_definition(service_type)
    state = _state_store().get()
    print(
        f"[AURA UI ROUTE] {service_type} 인식 성공 확인 -> 서비스 진행 화면 전환",
        flush=True,
    )

    if current_app.config["ALIGNMENT_INPUT_MODE"] == "legacy_bool":
        _messages().publish_align_confirm(
            robot_id=_robot_id(),
            service_type=service_type,
            camera=definition["camera"],
            detector=definition["detector"],
            attempt=int(state.get("alignment_attempt") or 1),
        )

        # 기존 후방 추적 노드와의 호환을 위해 GUIDE 정렬 완료 시 재타겟팅 신호 유지.
        if service_type == "GUIDE":
            _bridge().publish_retarget_web(True)

    payload = _messages().publish_service_request(
        robot_id=_robot_id(),
        service_type=service_type,
        goal_id=session.get("goal_id"),
        destination_label=session.get("destination_label"),
    )
    _state_store().finish_alignment()
    _state_store().mark_mission_active(payload)

    mission_type = "follow" if service_type == "LUGGAGE_ASSIST" else "guide"
    return redirect(url_for("passenger.mission_progress", type=mission_type))


@passenger_bp.route("/mission-progress")
def mission_progress():
    _initialize_session()
    state = _state_store().get()
    if not state.get("active"):
        return redirect(url_for("passenger.service_selection"))

    mission_type = request.args.get("type") or (
        "follow" if state.get("service_type") == "LUGGAGE_ASSIST" else "guide"
    )
    return render_template(
        "05_mission_progress.html",
        mission_type=mission_type,
        state=state,
        robot_id=_robot_id(),
    )


@passenger_bp.route("/complete-mission", methods=["GET", "POST"])
def complete_mission():
    _initialize_session()
    if not _state_store().mark_manual_arrival():
        return redirect(url_for("passenger.service_selection"))
    return redirect(url_for("passenger.arrival"))


@passenger_bp.route("/arrival")
def arrival():
    _initialize_session()
    state = _state_store().get()
    if not state.get("arrived"):
        if state.get("active"):
            return redirect(url_for("passenger.mission_progress"))
        return redirect(url_for("passenger.service_selection"))
    return render_template("06_arrival.html", state=state, robot_id=_robot_id())


@passenger_bp.route("/additional-service")
def additional_service():
    _initialize_session()
    robot_id = _robot_id()
    _clear_pending_service()
    _state_store().reset(robot_id)
    session["robot_connected"] = True
    return redirect(url_for("passenger.service_selection"))


@passenger_bp.route("/service-end", methods=["GET", "POST"])
def service_end():
    _initialize_session()
    state = _state_store().get()
    if session.get("robot_connected"):
        if _robot_id() == "AMR1":
            _bridge().publish_rb1_service_end()
        else:
            _messages().publish_service_end(
                robot_id=_robot_id(),
                previous_state=state,
                end_reason=request.values.get("reason", "USER_REQUEST"),
            )
            if _robot_id() == "AMR3" and current_app.config["CONTROL_MODE"] == "DIRECT_TOPIC":
                _bridge().publish_direct_service_end()
    session.clear()
    _state_store().reset(_robot_id())
    return redirect(url_for("passenger.ad"))


@passenger_bp.route("/cancel-service", methods=["GET", "POST"])
def cancel_service():
    _initialize_session()
    if _robot_id() == "AMR1":
        _bridge().publish_rb1_service_end()
    else:
        _messages().publish_service_end(
            robot_id=_robot_id(),
            previous_state=_state_store().get(),
            end_reason="USER_CANCELLED",
        )
        if _robot_id() == "AMR3" and current_app.config["CONTROL_MODE"] == "DIRECT_TOPIC":
            _bridge().publish_direct_service_end()
    session.clear()
    _state_store().reset(_robot_id())
    return redirect(url_for("passenger.ad"))


@passenger_bp.route("/inactivity-timeout", methods=["POST"])
def inactivity_timeout():
    _initialize_session()
    state = _state_store().get()
    if state.get("active") and not state.get("arrived"):
        return {"ok": False, "ignored": True, "reason": "ACTIVE_MISSION"}, 409

    if session.get("robot_connected"):
        if _robot_id() == "AMR1":
            _bridge().publish_rb1_service_end()
        else:
            _messages().publish_service_end(
                robot_id=_robot_id(),
                previous_state=state,
                end_reason="INACTIVITY_TIMEOUT",
            )
            if _robot_id() == "AMR3" and current_app.config["CONTROL_MODE"] == "DIRECT_TOPIC":
                _bridge().publish_direct_service_end()
    session.clear()
    _state_store().reset(_robot_id())
    return {
        "ok": True,
        "redirect_url": url_for("passenger.ad", reason="inactivity_timeout"),
    }


@passenger_bp.route("/emergency")
def emergency():
    _initialize_session()
    state = _state_store().get()
    if not state.get("emergency_active"):
        return redirect(url_for("passenger.emergency_resolved"))
    return render_template(
        "07_emergency_alert.html",
        state=state,
        interrupted=state.get("interrupted_state") or {},
        robot_id=state.get("robot_id") or _robot_id(),
    )


@passenger_bp.route("/emergency-end", methods=["GET", "POST"])
def emergency_end():
    """현장 상황 종료 확인을 로봇에 직접 Bool(True) 명령으로 전달한다."""
    _initialize_session()
    state = _state_store().get()
    if state.get("emergency_active"):
        if _robot_id() == "AMR1":
            _bridge().publish_emergency_end()
        else:
            _messages().publish_emergency_end(
                robot_id=_robot_id(),
                emergency_state=state,
            )

    # 시연에서는 버튼을 누른 즉시 UI를 초기화한다.
    # AMR1은 /emergency_end Bool(True) 직접 명령으로 PATROLLING 복귀를 트리거한다.
    session.clear()
    _state_store().reset(_robot_id())
    return redirect(url_for("passenger.ad", reason="emergency_ended"))


@passenger_bp.route("/emergency-resolved")
def emergency_resolved():
    if _state_store().get().get("emergency_active"):
        return redirect(url_for("passenger.emergency"))
    session.clear()
    _state_store().reset(_robot_id())
    return redirect(url_for("passenger.standby", reason="emergency_resolved"))
