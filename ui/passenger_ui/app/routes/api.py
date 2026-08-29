from flask import Blueprint, current_app, session, url_for

api_bp = Blueprint("api", __name__)


@api_bp.route("/api/mission-state")
def mission_state():
    state = current_app.extensions["aura_state_store"].get()
    return {
        "ok": True,
        "active": bool(state.get("active")),
        "arrived": bool(state.get("arrived")),
        "awaiting_service_end": bool(state.get("awaiting_service_end")),
        "emergency_active": bool(state.get("emergency_active")),
        "mission_status": state.get("mission_status_payload"),
        "arrival_url": url_for("passenger.arrival"),
        "emergency_url": url_for("passenger.emergency"),
    }


@api_bp.route("/api/alignment-state")
def alignment_state():
    state_store = current_app.extensions["aura_state_store"]
    state = state_store.get()
    service_type = str(
        state.get("alignment_service_type") or session.get("pending_service") or ""
    )

    is_guide = service_type == "GUIDE"
    status = str(state.get("alignment_status") or "IDLE")
    return {
        "ok": True,
        "robot_id": current_app.config["ROBOT_ID"],
        "service_type": service_type,
        "mode": state.get("alignment_mode") or "",
        "camera": state.get("alignment_camera") or "",
        "detector": state.get("alignment_detector") or "",
        "active": bool(state.get("alignment_active")),
        "detected": bool(state.get("alignment_detected")),
        "status": status,
        "timed_out": status == "FAILED",
        "lost": False,
        "turning_to_rear": status == "TURNING_TO_REAR",
        "returning_to_front": False,
        "retry_required": False if is_guide else status == "FAILED",
        "tracking_value": int(state.get("alignment_tracking_value") or 0),
        "tracking_topic": state.get("alignment_tracking_topic") or "",
        "tracking_confirm_frames": int(
            state.get("alignment_tracking_confirm_frames")
            or current_app.config.get("TRACKING_WEB_CONFIRM_FRAMES", 1)
            or 1
        ),
        "tracking_one_streak": int(state.get("alignment_tracking_one_streak") or 0),
        "tracking_one_max_streak": int(state.get("alignment_tracking_one_max_streak") or 0),
        "tracking_sample_count": int(state.get("alignment_tracking_sample_count") or 0),
        "alignment_input_mode": current_app.config["ALIGNMENT_INPUT_MODE"],
        "manual_alignment": current_app.config["ALIGNMENT_INPUT_MODE"] == "manual",
        "retry": bool(state.get("alignment_retry")),
        "attempt": int(state.get("alignment_attempt") or 0),
        "retry_url": url_for("passenger.align_luggage_retry"),
        "confirm_url": url_for("passenger.confirm_alignment"),
    }


@api_bp.route("/api/system-state")
def system_state():
    state = current_app.extensions["aura_state_store"].get()
    return {
        "ok": True,
        "emergency_active": bool(state.get("emergency_active")),
        "emergency_type": state.get("emergency_type") or "",
        "emergency_time": state.get("emergency_time"),
        "robot_mode": state.get("robot_mode"),
        "battery": state.get("battery"),
        "online": state.get("online"),
        "emergency_url": url_for("passenger.emergency"),
        "emergency_resolved_url": url_for("passenger.emergency_resolved"),
    }


@api_bp.route("/ros-status")
def ros_status():
    bridge = current_app.extensions["aura_ros_bridge"]
    cfg = current_app.config
    return {
        "ros2_enabled": bool(bridge.enabled),
        "robot_id": cfg["ROBOT_ID"],
        "control_mode": cfg["CONTROL_MODE"],
        "ros_node_name": cfg["ROS_NODE_NAME"],
        "host": cfg["HOST"],
        "port": cfg["PORT"],
        "session_cookie_name": cfg["SESSION_COOKIE_NAME"],
        "alignment_input_mode": cfg["ALIGNMENT_INPUT_MODE"],
        "luggage_service_enabled": cfg["ENABLE_LUGGAGE_SERVICE"],
        "string_json_topics": {
            "ui_to_central": {
                "service_request": cfg["SERVICE_REQUEST_TOPIC"],
                "service_end": cfg["SERVICE_END_TOPIC"],
            },
            "central_to_ui": {
                "arrival_status": cfg["ARRIVAL_STATUS_TOPIC"],
                "emergency_event": cfg["EMERGENCY_EVENT_TOPIC"],
                "robot_status": cfg["ROBOT_STATUS_TOPIC"],
                "mission_status": cfg["MISSION_STATUS_TOPIC"],
            },
        },
        "emergency_bool_topics": {
            "fall_detection": cfg["FALL_DETECTION_TOPIC"],
            "emergency_clear": cfg["EMERGENCY_CLEAR_TOPIC"],
            "message_type": "std_msgs/msg/Bool",
            "trigger_value": True,
        },
        "direct_bool_topics": {
            "ui_to_robot_bool_true": {
                "rb1_standby": cfg["RB1_STANDBY_TOPIC"],
                "rb1_service_end": cfg["RB1_SERVICE_END_TOPIC"],
                "emergency_end": cfg["EMERGENCY_END_TOPIC"],
                "rb3_standby": cfg["RB3_STANDBY_TOPIC"],
                "turn_around": cfg["TURN_AROUND_TOPIC"],
                "turn_complete": cfg["TURN_COMPLETE_TOPIC"],
                "carrying": cfg["CARRYING_TOPIC"],
                "service_end": cfg["DIRECT_SERVICE_END_TOPIC"],
                "message_type": "std_msgs/msg/Bool",
                "data": True,
            },
            "robot_to_ui_int32": {
                "tracking_web": cfg["TRACKING_WEB_TOPIC"],
                "tracking_rgbd": cfg["TRACKING_RGBD_TOPIC"],
                "message_type": "std_msgs/msg/Int32",
            },
        },
        "mission_state": current_app.extensions["aura_state_store"].get(),
    }
