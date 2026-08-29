from __future__ import annotations

import atexit
import json
import os
from pathlib import Path

from flask import Flask, Response, abort, jsonify, render_template, send_file, stream_with_context

from camera_bridge import CctvImageBridge
from database import DB_PATH, DashboardDatabase


CCTV_TOPIC = os.getenv("AURA_ADMIN_CCTV_TOPIC", "/cctv/image_raw/compressed")
POPULATION_TOPIC = os.getenv("AURA_ADMIN_POPULATION_TOPIC", "/population")
HOST = os.getenv("AURA_ADMIN_HOST", "0.0.0.0")
PORT = int(os.getenv("AURA_ADMIN_PORT", "7000"))

app = Flask(__name__)
app.config["JSON_AS_ASCII"] = False

database = DashboardDatabase(DB_PATH)
camera_bridge = CctvImageBridge(CCTV_TOPIC, population_topic=POPULATION_TOPIC)
atexit.register(camera_bridge.shutdown)


@app.get("/")
def dashboard():
    return render_template(
        "dashboard.html",
        db_path=str(database.db_path),
        cctv_topic=CCTV_TOPIC,
        population_topic=POPULATION_TOPIC,
    )


@app.get("/api/dashboard")
def dashboard_api():
    """ROS2 실시간 데이터는 DB 상태와 무관하게 항상 반환한다."""
    base_payload = {
        "ok": True,
        "db_ok": False,
        "db_error": None,
        "db_path": str(database.db_path),
        "camera": camera_bridge.get_status(),
        "population": camera_bridge.get_population_status(),
        "robots": [],
        "missions": [],
        "emergencies": [],
        "logs": [],
        "images": [],
        "hot_places": [],
        "summary": {
            "robot_count": 0,
            "emergency_count": 0,
            "completed_service_count": 0,
            "snapshot_count": 0,
        },
    }

    try:
        db_payload = database.get_dashboard()
        base_payload.update(db_payload)
        base_payload["db_ok"] = True
    except Exception as exc:
        # DB가 없어도 /population과 CCTV는 계속 정상 제공한다.
        base_payload["db_error"] = str(exc)

    return jsonify(base_payload)


@app.get("/api/camera/status")
def camera_status_api():
    return jsonify(camera_bridge.get_status())


@app.get("/api/population/stream")
def population_stream():
    """/population 변경을 브라우저에 즉시 전달하는 SSE 스트림."""

    def generate():
        last_count = -1

        # 브라우저 연결 직후 현재값을 즉시 전송한다.
        initial = camera_bridge.get_population_status()
        last_count = int(initial.get("message_count") or 0)
        yield f"event: population\ndata: {json.dumps(initial, ensure_ascii=False)}\n\n"

        while True:
            status, changed = camera_bridge.wait_for_population_update(
                last_count,
                timeout=10.0,
            )
            current_count = int(status.get("message_count") or 0)

            if changed:
                last_count = current_count
                yield (
                    "event: population\n"
                    f"data: {json.dumps(status, ensure_ascii=False)}\n\n"
                )
            else:
                # 프록시/브라우저 연결이 끊기지 않도록 heartbeat 전송
                yield ": heartbeat\n\n"

    return Response(
        stream_with_context(generate()),
        mimetype="text/event-stream",
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )


@app.get("/video/cctv")
def cctv_video():
    return Response(
        camera_bridge.mjpeg_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
        },
    )


@app.get("/event-image/<int:image_id>")
def event_image(image_id: int):
    try:
        record = database.get_event_image(image_id)
    except Exception:
        record = None
    if not record:
        abort(404)

    image_path = Path(record["image_path"]).expanduser().resolve()
    if not image_path.is_file():
        abort(404)

    return send_file(
        image_path,
        mimetype=record.get("mime_type") or "image/jpeg",
        conditional=True,
        max_age=0,
    )


if __name__ == "__main__":
    print("[AURA ADMIN] 관리자 UI 시작", flush=True)
    print(f"[AURA ADMIN] DB: {database.db_path}", flush=True)
    print(f"[AURA ADMIN] CCTV: {CCTV_TOPIC} (sensor_msgs/msg/CompressedImage)", flush=True)
    print(f"[AURA ADMIN] 인원수: {POPULATION_TOPIC} (std_msgs/msg/Int32)", flush=True)
    print(f"[AURA ADMIN] URL: http://127.0.0.1:{PORT}", flush=True)
    if camera_bridge.error:
        print(f"[AURA ADMIN] CCTV 경고: {camera_bridge.error}", flush=True)

    app.run(
        host=HOST,
        port=PORT,
        debug=False,
        threaded=True,
        use_reloader=False,
    )
