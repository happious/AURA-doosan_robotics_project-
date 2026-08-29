from __future__ import annotations

import os
import shlex
import sys
from pathlib import Path

from network_info import get_lan_ip


PROJECT_ROOT = Path(__file__).resolve().parent


def _ensure_ros_environment() -> None:
    """ROS2 환경이 아직 적용되지 않았다면 setup.bash를 적용해 다시 실행한다."""
    try:
        import rclpy  # noqa: F401
        return
    except Exception:
        pass

    if os.environ.get("AURA_ROS_BOOTSTRAPPED") == "1":
        print(
            "[AURA UI] ROS2 환경을 자동 적용했지만 rclpy를 불러오지 못했습니다. "
            "UI는 print-only mode로 실행될 수 있습니다.",
            flush=True,
        )
        return

    ros_setup = Path("/opt/ros/humble/setup.bash")
    workspace_setup = Path.home() / "aura_ws" / "install" / "setup.bash"

    if not ros_setup.exists():
        print(
            "[AURA UI] /opt/ros/humble/setup.bash가 없어 ROS2 자동 설정을 건너뜁니다.",
            flush=True,
        )
        return

    env = os.environ.copy()
    env["AURA_ROS_BOOTSTRAPPED"] = "1"

    source_commands = [f"source {shlex.quote(str(ros_setup))}"]
    if workspace_setup.exists():
        source_commands.append(f"source {shlex.quote(str(workspace_setup))}")

    python_executable = shlex.quote(sys.executable)
    script_path = shlex.quote(str(Path(sys.argv[0]).resolve()))
    script_args = " ".join(shlex.quote(arg) for arg in sys.argv[1:])
    run_command = f"exec {python_executable} {script_path}"
    if script_args:
        run_command += f" {script_args}"

    command = "; ".join(source_commands + [run_command])
    print("[AURA UI] ROS2 환경을 자동으로 불러온 뒤 다시 실행합니다.", flush=True)
    os.execve("/bin/bash", ["/bin/bash", "-lc", command], env)


def launch_ui(
    *,
    robot_id: str,
    port: int,
    alignment_input_mode: str = "legacy_bool",
    rear_person_detected_topic: str | None = None,
    front_leg_detected_topic: str | None = None,
    retarget_topic: str | None = None,
    tracking_web_topic: str = "/tracking_web",
    tracking_rgbd_topic: str = "/tracking_rgbd",
    turn_around_topic: str = "/turn_around",
    turn_complete_topic: str = "/turn_complete",
    carrying_topic: str = "/carrying",
    rb3_standby_topic: str = "/rb3_standby",
    direct_service_end_topic: str = "/service_end",
) -> None:
    """로봇별 환경값을 적용하고 해당 AURA 모바일 UI를 실행한다."""
    robot_id = robot_id.strip().upper()
    if robot_id not in {"AMR1", "AMR2", "AMR3"}:
        raise ValueError(f"지원하지 않는 robot_id입니다: {robot_id}")

    alignment_input_mode = alignment_input_mode.strip().lower()
    if alignment_input_mode not in {"manual", "legacy_bool", "tracking_int32"}:
        raise ValueError(
            "alignment_input_mode는 manual, legacy_bool, tracking_int32 중 하나여야 합니다."
        )

    os.chdir(PROJECT_ROOT)

    # Config 클래스 import 전에 환경변수를 먼저 설정한다.
    os.environ["AURA_ROBOT_ID"] = robot_id
    os.environ["AURA_PORT"] = str(port)
    os.environ["AURA_ALIGNMENT_INPUT_MODE"] = alignment_input_mode
    if alignment_input_mode == "tracking_int32":
        os.environ["AURA_TRACKING_WEB_TOPIC"] = tracking_web_topic
        os.environ["AURA_TRACKING_RGBD_TOPIC"] = tracking_rgbd_topic
        os.environ["AURA_TURN_AROUND_TOPIC"] = turn_around_topic
        os.environ["AURA_TURN_COMPLETE_TOPIC"] = turn_complete_topic
        os.environ["AURA_CARRYING_TOPIC"] = carrying_topic
        os.environ["AURA_RB3_STANDBY_TOPIC"] = rb3_standby_topic
        os.environ["AURA_DIRECT_SERVICE_END_TOPIC"] = direct_service_end_topic
    else:
        os.environ["AURA_REAR_PERSON_DETECTED_TOPIC"] = (
            rear_person_detected_topic or f"/{robot_id}/rear_person_detected"
        )
        os.environ["AURA_FRONT_LEG_DETECTED_TOPIC"] = (
            front_leg_detected_topic or f"/{robot_id}/front_leg_detected"
        )
        os.environ["AURA_RETARGET_WEB_TOPIC"] = (
            retarget_topic or f"/{robot_id}/retarget_web"
        )

    _ensure_ros_environment()

    from app import create_app

    app = create_app()

    if app.config["ALIGNMENT_INPUT_MODE"] == "tracking_int32":
        alignment_topics = (
            f"tracking_web={app.config['TRACKING_WEB_TOPIC']} "
            f"tracking_rgbd={app.config['TRACKING_RGBD_TOPIC']} "
            f"turn={app.config['TURN_AROUND_TOPIC']} "
            f"turn_complete={app.config['TURN_COMPLETE_TOPIC']} "
            f"carrying={app.config['CARRYING_TOPIC']} "
            f"rb3_standby={app.config['RB3_STANDBY_TOPIC']} "
            f"direct_service_end={app.config['DIRECT_SERVICE_END_TOPIC']}"
        )
    elif app.config["ALIGNMENT_INPUT_MODE"] == "manual":
        alignment_topics = "manual_alignment=no_camera"
    else:
        alignment_topics = (
            f"rear_detected={app.config['REAR_PERSON_DETECTED_TOPIC']} "
            f"front_detected={app.config['FRONT_LEG_DETECTED_TOPIC']}"
        )

    print(
        "[AURA UI] startup "
        f"robot={app.config['ROBOT_ID']} "
        f"node={app.config['ROS_NODE_NAME']} "
        f"port={app.config['PORT']} "
        f"alignment_mode={app.config['ALIGNMENT_INPUT_MODE']} "
        f"{alignment_topics}",
        flush=True,
    )

    lan_ip = get_lan_ip()
    print(
        f"[AURA UI] 같은 PC: http://127.0.0.1:{app.config['PORT']}",
        flush=True,
    )
    if lan_ip:
        print(
            f"[AURA UI] 같은 네트워크: http://{lan_ip}:{app.config['PORT']}",
            flush=True,
        )

    app.run(
        host=app.config["HOST"],
        port=app.config["PORT"],
        debug=app.config["DEBUG"],
        use_reloader=False,
        threaded=True,
    )
