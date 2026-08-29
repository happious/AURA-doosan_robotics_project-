import os


class Config:
    """AURA 승객 UI 공통 설정.

    이 통합 프로젝트는 같은 화면 골조를 공유하지만 로봇별 통신 방식은 다르다.

    - AMR1: 즉시 제어 명령(/rb1_standby, /rb1_service_end, /emergency_end)은 로봇1과 Bool(True) 직접 통신 + DDS ACK
    - AMR3: 로봇 직접 명령은 Bool(True), 중앙 통신은 String(JSON)으로 분리
    """

    ROBOT_ID = os.getenv("AURA_ROBOT_ID", "AMR1").strip().upper()
    AVAILABLE_ROBOTS = ("AMR1", "AMR2", "AMR3")
    if ROBOT_ID not in AVAILABLE_ROBOTS:
        raise ValueError(
            f"지원하지 않는 AURA_ROBOT_ID입니다: {ROBOT_ID}. "
            f"사용 가능 값: {', '.join(AVAILABLE_ROBOTS)}"
        )

    # 로봇별 제어 구조
    # AMR1은 토의 결과 즉시 제어가 필요한 아래 3개 명령을 로봇1과 직접 Bool(True)로 통신한다.
    # /rb1_standby, /rb1_service_end, /emergency_end
    # 세 토픽은 RELIABLE + VOLATILE QoS로 1회 발행하고 DDS ACK를 확인한다.
    # AMR3는 Action Server가 없어서 UI가 아래 직접 명령을 Bool(True)로 발행한다.
    # /rb3_standby, /service_end, /carrying, /turn_around
    # /aura/service_request 등 중앙용 이벤트는 String(JSON)을 유지한다.
    CONTROL_MODE = "CENTRAL_JSON" if ROBOT_ID == "AMR1" else "DIRECT_TOPIC"

    HOST = os.getenv("AURA_HOST", "0.0.0.0")
    PORT = int(os.getenv("AURA_PORT", "5001"))
    DEBUG = os.getenv("AURA_DEBUG", "0").strip().lower() in {
        "1", "true", "yes", "on"
    }

    SECRET_KEY = os.getenv(
        "AURA_SECRET_KEY",
        f"aura-passenger-kiosk-{ROBOT_ID.lower()}-dev-key",
    )
    SESSION_COOKIE_NAME = os.getenv(
        "AURA_SESSION_COOKIE_NAME",
        f"aura_session_{ROBOT_ID.lower()}",
    )
    ROS_NODE_NAME = os.getenv(
        "AURA_ROS_NODE_NAME",
        f"aura_passenger_mobile_ui_{ROBOT_ID.lower()}",
    )

    # 화면 동작
    INACTIVITY_TIMEOUT_SECONDS = int(os.getenv("AURA_INACTIVITY_TIMEOUT", "20"))
    ALIGN_DETECTION_TIMEOUT_SECONDS = float(os.getenv("AURA_ALIGN_TIMEOUT", "5.0"))
    # Robot3 GUIDE는 시간 추정이 아니라 /turn_complete=True를 회전 완료 기준으로 사용한다.
    # 최초 /turn_around=True 이후 /turn_complete를 받기 전까지 tracking_web은 판정하지 않는다.
    TRACKING_WEB_CONFIRM_FRAMES = max(
        1, int(os.getenv("AURA_TRACKING_WEB_CONFIRM_FRAMES", "3"))
    )

    # AMR3 직접 Bool 명령은 구독자가 DDS graph에 나타날 때까지 잠시 기다린 뒤
    # 정확히 1회 발행한다. 무한 대기로 Flask 요청이 멈추는 것을 방지하기 위해
    # 기본 제한 시간은 5초로 둔다.
    DIRECT_COMMAND_WAIT_TIMEOUT_SECONDS = float(
        os.getenv("AURA_DIRECT_COMMAND_WAIT_TIMEOUT", "5.0")
    )
    DIRECT_COMMAND_WAIT_POLL_SECONDS = float(
        os.getenv("AURA_DIRECT_COMMAND_WAIT_POLL", "0.05")
    )
    # subscriber 확인 후 정확히 1회 발행하고, RELIABLE DDS ACK를 기다리는 시간.
    # ACK timeout이 발생해도 같은 명령을 자동 재발행하지 않는다.
    DIRECT_COMMAND_ACK_TIMEOUT_SECONDS = float(
        os.getenv("AURA_DIRECT_COMMAND_ACK_TIMEOUT", "0.5")
    )

    # 정렬 입력 방식
    # - manual: AMR1. 카메라 검출 없이 사람이 준비 후 UI 버튼으로 안내 시작
    # - legacy_bool: 기존 AMR1/AMR2 Bool 검출 호환용
    # - tracking_int32: AMR3 /tracking_web, /tracking_rgbd 방식
    default_alignment_mode = "manual" if ROBOT_ID == "AMR1" else "tracking_int32"
    ALIGNMENT_INPUT_MODE = os.getenv(
        "AURA_ALIGNMENT_INPUT_MODE",
        default_alignment_mode,
    ).strip().lower()
    if ALIGNMENT_INPUT_MODE not in {"manual", "legacy_bool", "tracking_int32"}:
        raise ValueError(
            "AURA_ALIGNMENT_INPUT_MODE는 manual, legacy_bool, tracking_int32 중 하나여야 합니다."
        )

    # AMR1 시연에서는 GUIDE만 사용한다. AMR3는 짐 들기 서비스도 사용한다.
    ENABLE_LUGGAGE_SERVICE = os.getenv(
        "AURA_ENABLE_LUGGAGE_SERVICE",
        "0" if ROBOT_ID == "AMR1" else "1",
    ).strip().lower() in {"1", "true", "yes", "on"}

    # ------------------------------------------------------------------
    # UI -> 중앙 fleet_dispatcher_node.py
    # AMR1·AMR3 공통 중앙 통신 Topic이다.
    # 모두 std_msgs/msg/String이며 msg.data는 JSON 문자열이다.
    # AMR1의 /rb1_standby 및 AMR3의 직접 로봇 동작 명령 Bool Topic과는 별개다.
    # ------------------------------------------------------------------
    # AMR1 UI -> 로봇1 직접 제어 명령.
    # 모두 std_msgs/msg/Bool(data=True)를 RELIABLE QoS로 1회 발행하고 DDS ACK를 확인한다.
    RB1_STANDBY_TOPIC = os.getenv("AURA_RB1_STANDBY_TOPIC", "/rb1_standby")
    RB1_SERVICE_END_TOPIC = os.getenv("AURA_RB1_SERVICE_END_TOPIC", "/rb1_service_end")
    EMERGENCY_END_TOPIC = os.getenv("AURA_EMERGENCY_END_TOPIC", "/emergency_end")

    ROBOT_SELECT_TOPIC = os.getenv("AURA_ROBOT_SELECT_TOPIC", "/aura/robot_select")
    SERVICE_REQUEST_TOPIC = os.getenv("AURA_SERVICE_REQUEST_TOPIC", "/aura/service_request")
    SERVICE_END_TOPIC = os.getenv("AURA_SERVICE_END_TOPIC", "/aura/service_end")
    LUGGAGE_LOAD_CONFIRM_TOPIC = os.getenv(
        "AURA_LUGGAGE_LOAD_CONFIRM_TOPIC",
        "/aura/luggage_load_confirm",
    )

    # ------------------------------------------------------------------
    # 중앙 fleet_dispatcher_node.py -> AMR1 UI
    # 모두 std_msgs/msg/String(JSON)으로 받는다.
    # raw /fall_detection의 실제 센서 타입은 중앙에서 처리하고,
    # UI에는 /aura/emergency_event JSON으로 재발행하는 구조를 권장한다.
    # ------------------------------------------------------------------
    ARRIVAL_STATUS_TOPIC = os.getenv("AURA_ARRIVAL_STATUS_TOPIC", "/aura/arrival_status")
    EMERGENCY_EVENT_TOPIC = os.getenv("AURA_EMERGENCY_EVENT_TOPIC", "/aura/emergency_event")
    ROBOT_STATUS_TOPIC = os.getenv("AURA_ROBOT_STATUS_TOPIC", "/aura/robot_status")
    MISSION_STATUS_TOPIC = os.getenv("AURA_MISSION_STATUS_TOPIC", "/aura/mission_status")

    # 기존 Action/Dispatcher 정렬 방식용 JSON Topic
    ALIGN_REQUEST_TOPIC = os.getenv("AURA_ALIGN_REQUEST_TOPIC", "/aura/align_request")
    ALIGN_STATUS_TOPIC = os.getenv("AURA_ALIGN_STATUS_TOPIC", "/aura/align_status")
    ALIGN_CONFIRM_TOPIC = os.getenv("AURA_ALIGN_CONFIRM_TOPIC", "/aura/align_confirm")

    # 기존 Bool 검출 방식
    REAR_PERSON_DETECTED_TOPIC = os.getenv(
        "AURA_REAR_PERSON_DETECTED_TOPIC",
        f"/{ROBOT_ID}/rear_person_detected",
    )
    FRONT_LEG_DETECTED_TOPIC = os.getenv(
        "AURA_FRONT_LEG_DETECTED_TOPIC",
        f"/{ROBOT_ID}/front_leg_detected",
    )
    RETARGET_WEB_TOPIC = os.getenv(
        "AURA_RETARGET_WEB_TOPIC",
        f"/{ROBOT_ID}/retarget_web",
    )

    # AMR3 UI -> 로봇3 직접 통신 Topic
    # 아래 4개 명령은 std_msgs/msg/Bool의 data=True를 1회 발행한다.
    # False는 UI에서 종료 신호로 사용하지 않는다.
    TRACKING_WEB_TOPIC = os.getenv("AURA_TRACKING_WEB_TOPIC", "/tracking_web")
    TRACKING_RGBD_TOPIC = os.getenv("AURA_TRACKING_RGBD_TOPIC", "/tracking_rgbd")
    TURN_AROUND_TOPIC = os.getenv("AURA_TURN_AROUND_TOPIC", "/turn_around")
    TURN_COMPLETE_TOPIC = os.getenv("AURA_TURN_COMPLETE_TOPIC", "/turn_complete")
    CARRYING_TOPIC = os.getenv("AURA_CARRYING_TOPIC", "/carrying")
    DIRECT_SERVICE_END_TOPIC = os.getenv("AURA_DIRECT_SERVICE_END_TOPIC", "/service_end")
    RB3_STANDBY_TOPIC = os.getenv("AURA_RB3_STANDBY_TOPIC", "/rb3_standby")

    # CCTV/웹캠 원시 응급 입력.
    # 현재 시연 구조에서는 응급상황 동작은 AMR1만 담당한다.
    # 따라서 /fall_detection Bool(data=True)는 AMR1 UI에서만 구독/처리하고,
    # AMR3 UI는 이 토픽을 구독하지 않아 응급화면으로 전환되지 않는다.
    FALL_DETECTION_EMERGENCY_ENABLED = os.getenv(
        "AURA_FALL_DETECTION_EMERGENCY_ENABLED",
        "1" if ROBOT_ID == "AMR1" else "0",
    ).strip().lower() in {"1", "true", "yes", "on"}
    FALL_DETECTION_TOPIC = os.getenv("AURA_FALL_DETECTION_TOPIC", "/fall_detection")
    EMERGENCY_CLEAR_TOPIC = os.getenv(
        "AURA_EMERGENCY_CLEAR_TOPIC",
        "/aura/emergency_clear",
    )

    # 목적지. 실제 중앙 Dispatcher의 goal 이름과 반드시 맞춰야 한다.
    # 사용자 제공 03_1_Destination_Select.html의 2x2 카드 구성은 그대로 유지한다.
    if ROBOT_ID == "AMR3":
        # AMR3: 터미널 구역 안내 목적지 2개만 사용
        # 화장품 가게 -> goal1_1, 편의점 -> goal1_2
        # 화장실은 AMR3 선택 대상이 아니다.
        DESTINATIONS = {
            "cosmetics": {
                "label": "화장품 가게",
                "goal_id": "goal1_1",
                "icon": "💄",
                "enabled": True,
                "theme": "teal",
            },
            "liquor": {
                "label": "주류 판매점",
                "goal_id": None,
                "icon": "🍷",
                "enabled": False,
                "theme": "disabled",
            },
            "convenience": {
                "label": "편의점",
                "goal_id": "goal1_2",
                "icon": "🏪",
                "enabled": True,
                "theme": "orange",
            },
            "restroom": {
                "label": "화장실",
                "goal_id": None,
                "icon": "🚻",
                "enabled": False,
                "theme": "disabled",
            },
        }
    else:
        # AMR1: 화장실만 선택 가능하고 중앙에 goal_2를 전달한다.
        DESTINATIONS = {
            "cosmetics": {
                "label": "화장품 가게",
                "goal_id": None,
                "icon": "💄",
                "enabled": False,
                "theme": "disabled",
            },
            "liquor": {
                "label": "주류 판매점",
                "goal_id": None,
                "icon": "🍷",
                "enabled": False,
                "theme": "disabled",
            },
            "convenience": {
                "label": "편의점",
                "goal_id": None,
                "icon": "🏪",
                "enabled": False,
                "theme": "disabled",
            },
            "restroom": {
                "label": "화장실",
                "goal_id": "goal_2",
                "icon": "🚻",
                "enabled": True,
                "theme": "teal",
            },
        }
