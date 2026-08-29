#!/usr/bin/env python3
"""
AURA SQLite DB Manager Node
===========================

기존 fleet_dispatcher_node.py를 수정하지 않고, 이미 발행되는 ROS2 토픽을
옆에서 구독하여 SQLite에 저장하는 독립 기록 노드입니다.

저장 대상
---------
- /robot1/status, /robot3/status
- /aura/robot_select
- /aura/service_request
- /aura/service_end
- /aura/arrival_status
- /fall_detection
- /fall_detection_point
- /hot_place
- /population
- /turn_around
- /service_cancel
- /rb1_service_end
- /tracking_web
- /tracking_rgbd
- 낙상 감지용 sensor_msgs/Image 최신 프레임 (환경변수 AURA_FALL_IMAGE_TOPIC)

실행
----
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
python3 aura_db_manager_node.py

환경변수로 DB 경로 변경 가능:
export AURA_DB_PATH=/원하는/경로/amr_system.db
"""

from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Bool, Int32, String

from fleet_interfaces.msg import RobotStatus


ROBOT_IDS = ("robot1", "robot3")

AMR_TO_ROBOT = {
    "AMR1": "robot1",
    "AMR3": "robot3",
}
ROBOT_TO_AMR = {robot_id: amr_id for amr_id, robot_id in AMR_TO_ROBOT.items()}

HOT_PLACE_SAVE_MIN_INTERVAL_SEC = 1.0
HOT_PLACE_SAVE_MIN_DISTANCE_M = 0.05

# 낙상 감지 스냅샷 설정. sensor_msgs/msg/Image 타입의 CCTV 토픽을 사용한다.
FALL_IMAGE_TOPIC = os.getenv(
    "AURA_FALL_IMAGE_TOPIC",
    "/cctv/fall_detection_image",
)
FALL_IMAGE_MAX_AGE_SEC = float(os.getenv("AURA_FALL_IMAGE_MAX_AGE_SEC", "2.0"))
FALL_IMAGE_WAIT_SEC = float(os.getenv("AURA_FALL_IMAGE_WAIT_SEC", "3.0"))


class AuraDatabase:
    """SQLite 연결, 테이블 생성, 저장 함수를 담당합니다."""

    def __init__(self, db_path: Path):
        self.db_path = db_path.expanduser().resolve()
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self.initialize()

    def connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(
            str(self.db_path),
            timeout=5.0,
            check_same_thread=False,
        )
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA journal_mode=WAL;")
        connection.execute("PRAGMA busy_timeout=5000;")
        connection.execute("PRAGMA foreign_keys=ON;")
        return connection

    def initialize(self) -> None:
        schema = """
        CREATE TABLE IF NOT EXISTS robots (
            robot_id TEXT PRIMARY KEY,
            amr_id TEXT,
            mode INTEGER,
            battery_pct REAL,
            aed_loaded INTEGER NOT NULL DEFAULT 0,
            available INTEGER NOT NULL DEFAULT 0,
            pose_x REAL,
            pose_y REAL,
            last_seen TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
        );

        CREATE TABLE IF NOT EXISTS service_requests (
            request_id TEXT PRIMARY KEY,
            robot_id TEXT,
            amr_id TEXT,
            service_type TEXT,
            goal_id TEXT,
            destination_label TEXT,
            status TEXT NOT NULL DEFAULT 'CREATED',
            end_reason TEXT,
            raw_json TEXT,
            created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            ended_at TEXT
        );

        CREATE TABLE IF NOT EXISTS missions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            request_id TEXT,
            robot_id TEXT,
            mission_type TEXT,
            status TEXT NOT NULL DEFAULT 'REQUESTED',
            goal_id TEXT,
            destination_label TEXT,
            source TEXT,
            raw_json TEXT,
            created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            started_at TEXT,
            completed_at TEXT,
            final_state TEXT,
            FOREIGN KEY (request_id) REFERENCES service_requests(request_id)
        );

        CREATE TABLE IF NOT EXISTS emergency_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            status TEXT NOT NULL DEFAULT 'DETECTED',
            point_x REAL,
            point_y REAL,
            dispatched_robot_id TEXT,
            detected_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            cleared_at TEXT
        );

        CREATE TABLE IF NOT EXISTS hot_place_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            pose_index INTEGER,
            point_x REAL NOT NULL,
            point_y REAL NOT NULL,
            pose_count INTEGER NOT NULL,
            received_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
        );

        CREATE TABLE IF NOT EXISTS event_images (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            emergency_id INTEGER,
            event_type TEXT NOT NULL,
            topic_name TEXT NOT NULL,
            image_path TEXT NOT NULL,
            mime_type TEXT NOT NULL DEFAULT 'image/jpeg',
            captured_at TEXT NOT NULL,
            created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            FOREIGN KEY (emergency_id) REFERENCES emergency_events(id)
        );

        CREATE TABLE IF NOT EXISTS system_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            event_type TEXT NOT NULL,
            level TEXT NOT NULL DEFAULT 'INFO',
            robot_id TEXT,
            message TEXT NOT NULL,
            payload_json TEXT,
            created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
        );

        CREATE INDEX IF NOT EXISTS idx_requests_robot_status
        ON service_requests(robot_id, status);

        CREATE INDEX IF NOT EXISTS idx_missions_robot_status
        ON missions(robot_id, status);

        CREATE INDEX IF NOT EXISTS idx_events_created_at
        ON system_events(created_at);

        CREATE INDEX IF NOT EXISTS idx_emergency_status
        ON emergency_events(status);

        CREATE INDEX IF NOT EXISTS idx_event_images_emergency
        ON event_images(emergency_id, captured_at);
        """

        with self._lock, self.connect() as connection:
            connection.executescript(schema)
            connection.commit()

    def execute(self, query: str, params: tuple[Any, ...] = ()) -> int:
        with self._lock, self.connect() as connection:
            cursor = connection.execute(query, params)
            connection.commit()
            return int(cursor.lastrowid or 0)

    def fetchone(self, query: str, params: tuple[Any, ...] = ()):
        with self.connect() as connection:
            return connection.execute(query, params).fetchone()

    def log_event(
        self,
        event_type: str,
        message: str,
        *,
        level: str = "INFO",
        robot_id: str | None = None,
        payload: dict[str, Any] | None = None,
    ) -> None:
        payload_json = None
        if payload is not None:
            payload_json = json.dumps(payload, ensure_ascii=False, default=str)

        self.execute(
            """
            INSERT INTO system_events (
                event_type, level, robot_id, message, payload_json
            )
            VALUES (?, ?, ?, ?, ?)
            """,
            (event_type, level, robot_id, message, payload_json),
        )

    def upsert_robot_status(self, msg: RobotStatus) -> None:
        robot_id = str(msg.robot_id)
        amr_id = ROBOT_TO_AMR.get(robot_id)
        pose_x = float(msg.pose.pose.position.x)
        pose_y = float(msg.pose.pose.position.y)

        self.execute(
            """
            INSERT INTO robots (
                robot_id, amr_id, mode, battery_pct, aed_loaded,
                available, pose_x, pose_y, last_seen, updated_at
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)
            ON CONFLICT(robot_id) DO UPDATE SET
                amr_id = excluded.amr_id,
                mode = excluded.mode,
                battery_pct = excluded.battery_pct,
                aed_loaded = excluded.aed_loaded,
                available = excluded.available,
                pose_x = excluded.pose_x,
                pose_y = excluded.pose_y,
                last_seen = CURRENT_TIMESTAMP,
                updated_at = CURRENT_TIMESTAMP
            """,
            (
                robot_id,
                amr_id,
                int(msg.mode),
                float(msg.battery_pct),
                int(bool(msg.aed_loaded)),
                int(bool(msg.available)),
                pose_x,
                pose_y,
            ),
        )

    def upsert_service_request(
        self,
        payload: dict[str, Any],
        *,
        default_status: str,
    ) -> tuple[str, str | None]:
        amr_id = payload.get("robot_id")
        robot_id = AMR_TO_ROBOT.get(amr_id, amr_id)

        request_id = payload.get("request_id")
        if not request_id:
            request_id = f"AUTO-{robot_id or 'UNKNOWN'}-{uuid.uuid4().hex[:12]}"

        raw_json = json.dumps(payload, ensure_ascii=False, default=str)

        self.execute(
            """
            INSERT INTO service_requests (
                request_id, robot_id, amr_id, service_type,
                goal_id, destination_label, status, raw_json
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ON CONFLICT(request_id) DO UPDATE SET
                robot_id = COALESCE(excluded.robot_id, service_requests.robot_id),
                amr_id = COALESCE(excluded.amr_id, service_requests.amr_id),
                service_type = COALESCE(excluded.service_type, service_requests.service_type),
                goal_id = COALESCE(excluded.goal_id, service_requests.goal_id),
                destination_label = COALESCE(
                    excluded.destination_label,
                    service_requests.destination_label
                ),
                status = excluded.status,
                raw_json = excluded.raw_json,
                updated_at = CURRENT_TIMESTAMP
            """,
            (
                str(request_id),
                robot_id,
                amr_id,
                payload.get("service_type"),
                payload.get("goal_id"),
                payload.get("destination_label"),
                default_status,
                raw_json,
            ),
        )
        return str(request_id), robot_id

    def find_active_request(self, robot_id: str | None):
        if not robot_id:
            return None

        return self.fetchone(
            """
            SELECT *
            FROM service_requests
            WHERE robot_id = ?
              AND status NOT IN ('COMPLETED', 'CANCELLED', 'FAILED', 'TIMEOUT')
            ORDER BY created_at DESC
            LIMIT 1
            """,
            (robot_id,),
        )

    def create_mission_from_request(
        self,
        request_id: str,
        robot_id: str | None,
        payload: dict[str, Any],
    ) -> int:
        service_type = payload.get("service_type") or "UNKNOWN"
        raw_json = json.dumps(payload, ensure_ascii=False, default=str)

        existing = self.fetchone(
            """
            SELECT id
            FROM missions
            WHERE request_id = ?
              AND mission_type = ?
              AND status NOT IN ('COMPLETED', 'FAILED', 'CANCELLED')
            ORDER BY id DESC
            LIMIT 1
            """,
            (request_id, service_type),
        )
        if existing:
            return int(existing["id"])

        return self.execute(
            """
            INSERT INTO missions (
                request_id, robot_id, mission_type, status,
                goal_id, destination_label, source, raw_json
            )
            VALUES (?, ?, ?, 'REQUESTED', ?, ?, 'AURA_UI', ?)
            """,
            (
                request_id,
                robot_id,
                service_type,
                payload.get("goal_id"),
                payload.get("destination_label"),
                raw_json,
            ),
        )

    def mark_request_arrived(
        self,
        payload: dict[str, Any],
        *,
        success: bool,
    ) -> None:
        amr_id = payload.get("robot_id")
        robot_id = AMR_TO_ROBOT.get(amr_id, amr_id)
        request_id = payload.get("request_id")

        if not request_id:
            active = self.find_active_request(robot_id)
            request_id = active["request_id"] if active else None

        if not request_id:
            self.log_event(
                "ARRIVAL_WITHOUT_REQUEST",
                "도착 결과를 받았지만 연결할 활성 request_id를 찾지 못했습니다.",
                level="WARN",
                robot_id=robot_id,
                payload=payload,
            )
            return

        request_status = "ARRIVED" if success else "FAILED"
        mission_status = "COMPLETED" if success else "FAILED"

        self.execute(
            """
            UPDATE service_requests
            SET status = ?,
                updated_at = CURRENT_TIMESTAMP
            WHERE request_id = ?
            """,
            (request_status, request_id),
        )

        self.execute(
            """
            UPDATE missions
            SET status = ?,
                final_state = ?,
                completed_at = CURRENT_TIMESTAMP
            WHERE request_id = ?
              AND status NOT IN ('COMPLETED', 'FAILED', 'CANCELLED')
            """,
            (mission_status, payload.get("final_state"), request_id),
        )

    def finish_service(
        self,
        payload: dict[str, Any],
    ) -> str | None:
        amr_id = payload.get("robot_id")
        robot_id = AMR_TO_ROBOT.get(amr_id, amr_id)

        # UI의 SERVICE_END 메시지는 새 END-* request_id와 함께
        # 실제 종료 대상인 previous_request_id를 보낸다.
        # 따라서 previous_request_id를 가장 먼저 사용해야 한다.
        request_id = payload.get("previous_request_id")

        # 일부 테스트/외부 클라이언트는 기존 서비스 request_id를
        # request_id 필드에 직접 보낼 수 있으므로, DB에 실제로 존재할 때만 사용한다.
        if not request_id:
            candidate_request_id = payload.get("request_id")
            if candidate_request_id:
                existing = self.fetchone(
                    """
                    SELECT request_id
                    FROM service_requests
                    WHERE request_id = ?
                    """,
                    (str(candidate_request_id),),
                )
                if existing is not None:
                    request_id = str(candidate_request_id)

        # ID가 없거나 END-* ID만 넘어온 경우에는 해당 로봇의 최신 활성 요청을 찾는다.
        if not request_id:
            active = self.find_active_request(robot_id)
            request_id = active["request_id"] if active else None

        if not request_id:
            self.log_event(
                "SERVICE_END_WITHOUT_REQUEST",
                "서비스 종료를 받았지만 연결할 활성 요청을 찾지 못했습니다.",
                level="WARN",
                robot_id=robot_id,
                payload=payload,
            )
            return None

        end_reason = payload.get("end_reason", "UNKNOWN")

        self.execute(
            """
            UPDATE service_requests
            SET status = 'COMPLETED',
                end_reason = ?,
                updated_at = CURRENT_TIMESTAMP,
                ended_at = CURRENT_TIMESTAMP
            WHERE request_id = ?
            """,
            (end_reason, request_id),
        )

        self.execute(
            """
            UPDATE missions
            SET status = CASE
                    WHEN status = 'FAILED' THEN status
                    ELSE 'COMPLETED'
                END,
                completed_at = COALESCE(completed_at, CURRENT_TIMESTAMP)
            WHERE request_id = ?
              AND status NOT IN ('COMPLETED', 'FAILED', 'CANCELLED')
            """,
            (request_id,),
        )

        return str(request_id)

    def finish_active_service_by_robot(
        self,
        robot_id: str,
        *,
        end_reason: str,
    ) -> str | None:
        """robot_id의 가장 최근 활성 서비스를 찾아 정상 종료 처리한다.

        AMR1은 /aura/service_end 대신 /rb1_service_end(Bool)을 직접 사용하므로
        이 함수로 DB의 활성 요청과 미션을 함께 종료한다.
        """
        active = self.find_active_request(robot_id)
        if active is None:
            self.log_event(
                "SERVICE_END_WITHOUT_REQUEST",
                f"{robot_id}의 활성 서비스를 찾지 못했습니다.",
                level="WARN",
                robot_id=robot_id,
                payload={"end_reason": end_reason},
            )
            return None

        amr_id = ROBOT_TO_AMR.get(robot_id, robot_id)
        return self.finish_service(
            {
                "robot_id": amr_id,
                "previous_request_id": active["request_id"],
                "end_reason": end_reason,
            }
        )

    def mark_luggage_running(self) -> None:
        self.execute(
            """
            UPDATE missions
            SET status = 'RUNNING',
                started_at = COALESCE(started_at, CURRENT_TIMESTAMP)
            WHERE id = (
                SELECT id
                FROM missions
                WHERE mission_type = 'LUGGAGE_ASSIST'
                  AND status = 'REQUESTED'
                ORDER BY id DESC
                LIMIT 1
            )
            """
        )

    def mark_cancel_requested(self) -> None:
        self.execute(
            """
            UPDATE missions
            SET status = 'CANCEL_REQUESTED'
            WHERE id = (
                SELECT id
                FROM missions
                WHERE status IN ('REQUESTED', 'RUNNING')
                ORDER BY id DESC
                LIMIT 1
            )
            """
        )

    def open_emergency(self) -> int:
        existing = self.fetchone(
            """
            SELECT id
            FROM emergency_events
            WHERE status != 'CLEARED'
            ORDER BY id DESC
            LIMIT 1
            """
        )
        if existing:
            return int(existing["id"])

        return self.execute(
            """
            INSERT INTO emergency_events (status)
            VALUES ('DETECTED')
            """
        )

    def update_open_emergency_point(self, x: float, y: float) -> None:
        emergency_id = self.open_emergency()
        self.execute(
            """
            UPDATE emergency_events
            SET point_x = ?,
                point_y = ?,
                updated_at = CURRENT_TIMESTAMP
            WHERE id = ?
            """,
            (float(x), float(y), emergency_id),
        )

    def clear_emergencies(self) -> None:
        self.execute(
            """
            UPDATE emergency_events
            SET status = 'CLEARED',
                updated_at = CURRENT_TIMESTAMP,
                cleared_at = CURRENT_TIMESTAMP
            WHERE status != 'CLEARED'
            """
        )

    def infer_emergency_dispatch_from_status(self, msg: RobotStatus) -> None:
        emergency_mode = getattr(
            RobotStatus,
            "MODE_EMERGENCY_DISPATCH",
            None,
        )
        if emergency_mode is None or int(msg.mode) != int(emergency_mode):
            return

        emergency_id = self.open_emergency()
        self.execute(
            """
            UPDATE emergency_events
            SET status = 'DISPATCHED',
                dispatched_robot_id = ?,
                updated_at = CURRENT_TIMESTAMP
            WHERE id = ?
            """,
            (str(msg.robot_id), emergency_id),
        )

    def save_event_image(
        self,
        *,
        emergency_id: int,
        event_type: str,
        topic_name: str,
        image_path: str,
        mime_type: str,
        captured_at: str,
    ) -> int:
        """이벤트 이미지 파일의 메타데이터와 경로를 DB에 저장한다."""
        return self.execute(
            """
            INSERT INTO event_images (
                emergency_id, event_type, topic_name, image_path,
                mime_type, captured_at
            )
            VALUES (?, ?, ?, ?, ?, ?)
            """,
            (
                int(emergency_id),
                event_type,
                topic_name,
                image_path,
                mime_type,
                captured_at,
            ),
        )

    def save_hot_place(
        self,
        *,
        pose_index: int,
        x: float,
        y: float,
        pose_count: int,
    ) -> None:
        self.execute(
            """
            INSERT INTO hot_place_events (
                pose_index, point_x, point_y, pose_count
            )
            VALUES (?, ?, ?, ?)
            """,
            (int(pose_index), float(x), float(y), int(pose_count)),
        )


class AuraDBManagerNode(Node):
    """AURA 토픽을 수동으로 관찰해 DB에 저장하는 독립 ROS2 노드입니다."""

    def __init__(self):
        super().__init__("aura_db_manager_node")

        base_dir = Path(__file__).resolve().parent
        db_path = Path(
            os.getenv(
                "AURA_DB_PATH",
                str(base_dir / "data" / "amr_system.db"),
            )
        )
        self.database = AuraDatabase(db_path)

        self.fall_image_topic = FALL_IMAGE_TOPIC
        self.event_image_dir = Path(
            os.getenv(
                "AURA_EVENT_IMAGE_DIR",
                str(db_path.parent / "event_images"),
            )
        ).expanduser().resolve()
        self.event_image_dir.mkdir(parents=True, exist_ok=True)

        self._image_lock = threading.Lock()
        self._cv_bridge = CvBridge()
        self._latest_fall_image: bytes | None = None
        self._latest_fall_image_format = "jpeg"
        self._latest_fall_image_received_monotonic = 0.0
        self._fall_active = False
        self._pending_snapshot_emergency_id: int | None = None
        self._pending_snapshot_deadline = 0.0

        self._last_robot_bucket: dict[str, tuple[int, int, bool, bool]] = {}
        self._last_tracking_values: dict[str, int] = {}
        self._last_population: int | None = None
        self._last_hot_place: dict[int, tuple[float, float, float]] = {}

        for robot_id in ROBOT_IDS:
            self.create_subscription(
                RobotStatus,
                f"/{robot_id}/status",
                self.on_robot_status,
                10,
            )

        self.create_subscription(
            String,
            "/aura/robot_select",
            self.on_robot_select,
            10,
        )
        self.create_subscription(
            String,
            "/aura/service_request",
            self.on_service_request,
            10,
        )
        self.create_subscription(
            String,
            "/aura/service_end",
            self.on_service_end,
            10,
        )
        self.create_subscription(
            String,
            "/aura/arrival_status",
            self.on_arrival_status,
            10,
        )

        self.create_subscription(
            Bool,
            "/fall_detection",
            self.on_fall_detection,
            10,
        )
        self.create_subscription(
            PointStamped,
            "/fall_detection_point",
            self.on_fall_detection_point,
            10,
        )

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image,
            self.fall_image_topic,
            self.on_fall_image,
            camera_qos,
        )

        self.create_subscription(
            PoseArray,
            "/hot_place",
            self.on_hot_place,
            10,
        )
        self.create_subscription(
            Int32,
            "/population",
            self.on_population,
            10,
        )

        self.create_subscription(
            Bool,
            "/turn_around",
            self.on_turn_around,
            10,
        )
        self.create_subscription(
            Bool,
            "/service_cancel",
            self.on_service_cancel,
            10,
        )
        self.create_subscription(
            Bool,
            "/rb1_service_end",
            self.on_rb1_service_end,
            10,
        )
        self.create_subscription(
            Int32,
            "/tracking_web",
            lambda msg: self.on_tracking("/tracking_web", msg),
            10,
        )
        self.create_subscription(
            Int32,
            "/tracking_rgbd",
            lambda msg: self.on_tracking("/tracking_rgbd", msg),
            10,
        )

        self.database.log_event(
            "DB_MANAGER_START",
            "aura_db_manager_node가 시작되었습니다.",
            payload={
                "db_path": str(db_path),
                "fall_image_topic": self.fall_image_topic,
                "event_image_dir": str(self.event_image_dir),
            },
        )

        self.get_logger().info(
            f"[AURA DB] 독립 DB 기록 노드 시작: {db_path}"
        )
        self.get_logger().info(
            "[AURA DB] 기존 fleet_dispatcher_node.py 수정은 필요하지 않습니다."
        )
        self.get_logger().info(
            f"[AURA DB] 낙상 스냅샷 카메라: {self.fall_image_topic}"
        )
        self.get_logger().info(
            "[AURA DB] 카메라 메시지 타입: sensor_msgs/msg/Image"
        )
        self.get_logger().info(
            f"[AURA DB] 스냅샷 저장 폴더: {self.event_image_dir}"
        )

    @staticmethod
    def parse_json(msg: String) -> dict[str, Any] | None:
        try:
            payload = json.loads(msg.data)
            return payload if isinstance(payload, dict) else None
        except (json.JSONDecodeError, TypeError):
            return None

    def on_robot_status(self, msg: RobotStatus) -> None:
        self.database.upsert_robot_status(msg)
        self.database.infer_emergency_dispatch_from_status(msg)

        battery_bucket = int(round(float(msg.battery_pct) / 10.0) * 10)
        snapshot = (
            int(msg.mode),
            battery_bucket,
            bool(msg.aed_loaded),
            bool(msg.available),
        )

        if self._last_robot_bucket.get(msg.robot_id) != snapshot:
            self._last_robot_bucket[msg.robot_id] = snapshot
            self.database.log_event(
                "ROBOT_STATUS_CHANGED",
                (
                    f"{msg.robot_id} 상태 변경: mode={msg.mode}, "
                    f"battery≈{battery_bucket}%, "
                    f"aed={bool(msg.aed_loaded)}, "
                    f"available={bool(msg.available)}"
                ),
                robot_id=msg.robot_id,
            )

    def on_robot_select(self, msg: String) -> None:
        payload = self.parse_json(msg)
        if payload is None:
            self.database.log_event(
                "JSON_PARSE_ERROR",
                "/aura/robot_select JSON 파싱 실패",
                level="ERROR",
                payload={"raw": msg.data},
            )
            return

        request_id, robot_id = self.database.upsert_service_request(
            payload,
            default_status="ROBOT_SELECTED",
        )
        self.database.log_event(
            "ROBOT_SELECTED",
            f"승객이 {robot_id}를 선택했습니다.",
            robot_id=robot_id,
            payload={"request_id": request_id, **payload},
        )

    def on_service_request(self, msg: String) -> None:
        payload = self.parse_json(msg)
        if payload is None:
            self.database.log_event(
                "JSON_PARSE_ERROR",
                "/aura/service_request JSON 파싱 실패",
                level="ERROR",
                payload={"raw": msg.data},
            )
            return

        request_id, robot_id = self.database.upsert_service_request(
            payload,
            default_status="ACTIVE",
        )
        mission_id = self.database.create_mission_from_request(
            request_id,
            robot_id,
            payload,
        )

        self.database.log_event(
            "SERVICE_REQUESTED",
            (
                f"서비스 요청 저장: request_id={request_id}, "
                f"mission_id={mission_id}, "
                f"type={payload.get('service_type')}"
            ),
            robot_id=robot_id,
            payload=payload,
        )

    def on_service_end(self, msg: String) -> None:
        payload = self.parse_json(msg)
        if payload is None:
            self.database.log_event(
                "JSON_PARSE_ERROR",
                "/aura/service_end JSON 파싱 실패",
                level="ERROR",
                payload={"raw": msg.data},
            )
            return

        amr_id = payload.get("robot_id")
        robot_id = AMR_TO_ROBOT.get(amr_id, amr_id)
        finished_request_id = self.database.finish_service(payload)
        if finished_request_id is None:
            return

        self.database.log_event(
            "SERVICE_ENDED",
            f"{robot_id} 서비스 종료 완료: request_id={finished_request_id}",
            robot_id=robot_id,
            payload={"finished_request_id": finished_request_id, **payload},
        )

    def on_arrival_status(self, msg: String) -> None:
        payload = self.parse_json(msg)
        if payload is None:
            self.database.log_event(
                "JSON_PARSE_ERROR",
                "/aura/arrival_status JSON 파싱 실패",
                level="ERROR",
                payload={"raw": msg.data},
            )
            return

        success_values = {
            "ARRIVED",
            "REACHED",
            "SUCCEEDED",
            "SUCCESS",
        }
        success = (
            str(payload.get("event_type", "")).upper() in success_values
            or str(payload.get("arrival_status", "")).upper() in success_values
        )

        self.database.mark_request_arrived(payload, success=success)
        amr_id = payload.get("robot_id")
        robot_id = AMR_TO_ROBOT.get(amr_id, amr_id)

        self.database.log_event(
            "MISSION_ARRIVAL_RESULT",
            f"{robot_id} 도착 결과: success={success}",
            robot_id=robot_id,
            payload=payload,
        )

    def on_fall_image(self, msg: Image) -> None:
        """sensor_msgs/Image를 JPEG 바이트로 변환해 최신 프레임만 메모리에 보관한다."""
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
            encode_ok, encoded = cv2.imencode(".jpg", cv_image)
            if not encode_ok:
                raise RuntimeError("cv2.imencode('.jpg') failed")
        except (CvBridgeError, Exception) as exc:
            self.database.log_event(
                "FALL_IMAGE_CONVERT_ERROR",
                f"CCTV Image → JPEG 변환 실패: {exc}",
                level="ERROR",
                payload={
                    "topic": self.fall_image_topic,
                    "encoding": getattr(msg, "encoding", None),
                    "width": getattr(msg, "width", None),
                    "height": getattr(msg, "height", None),
                },
            )
            self.get_logger().error(
                f"[AURA DB] CCTV 이미지 변환 실패: {exc}"
            )
            return

        with self._image_lock:
            self._latest_fall_image = encoded.tobytes()
            self._latest_fall_image_format = "jpeg"
            self._latest_fall_image_received_monotonic = time.monotonic()

        pending_id = self._pending_snapshot_emergency_id
        if pending_id is None or not self._fall_active:
            return

        now = time.monotonic()
        if now > self._pending_snapshot_deadline:
            self.database.log_event(
                "FALL_SNAPSHOT_TIMEOUT",
                "낙상 감지 후 제한 시간 안에 카메라 프레임을 받지 못했습니다.",
                level="WARN",
                payload={"topic": self.fall_image_topic},
            )
            self._pending_snapshot_emergency_id = None
            return

        if self._save_fall_snapshot(pending_id):
            self._pending_snapshot_emergency_id = None

    def _save_fall_snapshot(self, emergency_id: int) -> bool:
        """최신 카메라 프레임을 파일로 저장하고 event_images에 경로를 기록한다."""
        with self._image_lock:
            image_data = self._latest_fall_image
            image_format = self._latest_fall_image_format
            received_at = self._latest_fall_image_received_monotonic

        if not image_data or received_at <= 0.0:
            return False

        image_age = time.monotonic() - received_at
        if image_age > FALL_IMAGE_MAX_AGE_SEC:
            return False

        format_lower = image_format.lower()
        if "png" in format_lower:
            extension = "png"
            mime_type = "image/png"
        else:
            extension = "jpg"
            mime_type = "image/jpeg"

        captured_at = datetime.now().astimezone()
        timestamp = captured_at.strftime("%Y%m%d_%H%M%S_%f")
        filename = f"fall_{emergency_id}_{timestamp}.{extension}"
        image_path = self.event_image_dir / filename

        try:
            image_path.write_bytes(image_data)
            image_id = self.database.save_event_image(
                emergency_id=emergency_id,
                event_type="FALL_DETECTION",
                topic_name=self.fall_image_topic,
                image_path=str(image_path),
                mime_type=mime_type,
                captured_at=captured_at.isoformat(timespec="milliseconds"),
            )
        except Exception as exc:
            self.database.log_event(
                "FALL_SNAPSHOT_ERROR",
                f"낙상 스냅샷 저장 실패: {exc}",
                level="ERROR",
                payload={"topic": self.fall_image_topic},
            )
            self.get_logger().error(f"[AURA DB] 낙상 스냅샷 저장 실패: {exc}")
            return False

        self.database.log_event(
            "FALL_SNAPSHOT_SAVED",
            f"낙상 스냅샷 저장 완료: image_id={image_id}",
            level="WARN",
            payload={
                "emergency_id": emergency_id,
                "image_id": image_id,
                "image_path": str(image_path),
                "topic": self.fall_image_topic,
                "image_age_sec": round(image_age, 3),
            },
        )
        self.get_logger().warn(
            f"[AURA DB] 낙상 스냅샷 저장: {image_path}"
        )
        return True

    def on_fall_detection(self, msg: Bool) -> None:
        # 연속 True가 반복 발행돼도 한 응급 상황당 사진은 한 번만 저장한다.
        if msg.data:
            if self._fall_active:
                return

            self._fall_active = True
            emergency_id = self.database.open_emergency()
            self.database.log_event(
                "FALL_DETECTED",
                f"낙상 감지: emergency_id={emergency_id}",
                level="WARN",
            )

            if not self._save_fall_snapshot(emergency_id):
                # 감지 순간 아직 프레임이 없거나 너무 오래된 프레임이면
                # 이후 도착하는 첫 최신 프레임을 제한 시간 안에서 저장한다.
                self._pending_snapshot_emergency_id = emergency_id
                self._pending_snapshot_deadline = (
                    time.monotonic() + FALL_IMAGE_WAIT_SEC
                )
                self.database.log_event(
                    "FALL_SNAPSHOT_PENDING",
                    "최신 카메라 프레임을 기다리는 중입니다.",
                    level="WARN",
                    payload={
                        "emergency_id": emergency_id,
                        "topic": self.fall_image_topic,
                        "wait_sec": FALL_IMAGE_WAIT_SEC,
                    },
                )
            return

        if not self._fall_active:
            return

        self._fall_active = False
        self._pending_snapshot_emergency_id = None
        self.database.clear_emergencies()
        self.database.log_event(
            "EMERGENCY_CLEARED",
            "fall_detection=False 수신으로 응급 이벤트를 종료했습니다.",
        )

    def on_fall_detection_point(self, msg: PointStamped) -> None:
        self.database.update_open_emergency_point(
            msg.point.x,
            msg.point.y,
        )
        self.database.log_event(
            "FALL_POINT_UPDATED",
            (
                f"응급 위치 갱신: "
                f"({msg.point.x:.3f}, {msg.point.y:.3f})"
            ),
        )

    def on_hot_place(self, msg: PoseArray) -> None:
        now = time.monotonic()
        pose_count = len(msg.poses)

        for index, pose in enumerate(msg.poses):
            x = float(pose.position.x)
            y = float(pose.position.y)

            previous = self._last_hot_place.get(index)
            should_save = previous is None

            if previous is not None:
                prev_x, prev_y, prev_time = previous
                distance = ((x - prev_x) ** 2 + (y - prev_y) ** 2) ** 0.5
                elapsed = now - prev_time
                should_save = (
                    distance >= HOT_PLACE_SAVE_MIN_DISTANCE_M
                    and elapsed >= HOT_PLACE_SAVE_MIN_INTERVAL_SEC
                )

            if not should_save:
                continue

            self._last_hot_place[index] = (x, y, now)
            self.database.save_hot_place(
                pose_index=index,
                x=x,
                y=y,
                pose_count=pose_count,
            )

    def on_population(self, msg: Int32) -> None:
        if self._last_population == int(msg.data):
            return

        self._last_population = int(msg.data)
        self.database.log_event(
            "POPULATION_CHANGED",
            f"population={msg.data}",
            payload={"population": int(msg.data)},
        )

    def on_turn_around(self, msg: Bool) -> None:
        if not msg.data:
            return

        self.database.mark_luggage_running()
        self.database.log_event(
            "TURN_AROUND",
            "/turn_around=True 수신: 짐 보조 동작 시작으로 기록",
        )

    def on_service_cancel(self, msg: Bool) -> None:
        if not msg.data:
            return

        # /service_cancel은 robot_id가 없는 전역 Bool 토픽이다.
        # 임의의 최신 미션을 CANCEL_REQUESTED로 변경하면 다른 로봇의 미션을
        # 잘못 수정할 수 있으므로, 여기서는 신호 수신 이력만 저장한다.
        self.database.log_event(
            "SERVICE_CANCEL_SIGNAL",
            "/service_cancel=True 수신",
        )

    def on_rb1_service_end(self, msg: Bool) -> None:
        if not msg.data:
            return

        finished_request_id = self.database.finish_active_service_by_robot(
            "robot1",
            end_reason="RB1_DIRECT_SERVICE_END",
        )
        if finished_request_id is None:
            return

        self.database.log_event(
            "RB1_SERVICE_ENDED",
            f"robot1 서비스 종료 완료: request_id={finished_request_id}",
            robot_id="robot1",
            payload={
                "topic": "/rb1_service_end",
                "finished_request_id": finished_request_id,
            },
        )

    def on_tracking(self, topic_name: str, msg: Int32) -> None:
        value = int(msg.data)
        if self._last_tracking_values.get(topic_name) == value:
            return

        self._last_tracking_values[topic_name] = value
        self.database.log_event(
            "TRACKING_CHANGED",
            f"{topic_name}={value}",
            payload={"topic": topic_name, "value": value},
        )

    def destroy_node(self):
        try:
            self.database.log_event(
                "DB_MANAGER_STOP",
                "aura_db_manager_node가 종료되었습니다.",
            )
        except Exception as exc:
            self.get_logger().error(f"[AURA DB] 종료 로그 저장 실패: {exc}")

        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AuraDBManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[AURA DB] 사용자 요청으로 종료합니다.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
