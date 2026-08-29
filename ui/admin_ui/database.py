from __future__ import annotations

import os
import sqlite3
from pathlib import Path
from typing import Any


DEFAULT_DB_PATH = Path.home() / "Downloads" / "data" / "amr_system.db"
DB_PATH = Path(os.getenv("AURA_DB_PATH", str(DEFAULT_DB_PATH))).expanduser().resolve()


class DashboardDatabase:
    """AURA 관리자 UI용 읽기 전용 SQLite 조회 모듈."""

    def __init__(self, db_path: Path = DB_PATH):
        self.db_path = Path(db_path).expanduser().resolve()

    def exists(self) -> bool:
        return self.db_path.is_file()

    def connect(self) -> sqlite3.Connection:
        if not self.exists():
            raise FileNotFoundError(f"DB 파일을 찾을 수 없습니다: {self.db_path}")

        connection = sqlite3.connect(
            f"file:{self.db_path}?mode=ro",
            uri=True,
            timeout=5.0,
        )
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA busy_timeout=5000;")
        return connection

    @staticmethod
    def _table_exists(connection: sqlite3.Connection, table_name: str) -> bool:
        row = connection.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?",
            (table_name,),
        ).fetchone()
        return row is not None

    @staticmethod
    def _rows(cursor: sqlite3.Cursor) -> list[dict[str, Any]]:
        return [dict(row) for row in cursor.fetchall()]

    def get_dashboard(self) -> dict[str, Any]:
        with self.connect() as connection:
            robots = self._rows(
                connection.execute(
                    """
                    SELECT
                        robot_id,
                        amr_id,
                        mode,
                        battery_pct,
                        aed_loaded,
                        available,
                        pose_x,
                        pose_y,
                        last_seen,
                        updated_at,
                        ROUND((julianday('now') - julianday(last_seen)) * 86400, 1)
                            AS last_seen_age_sec
                    FROM robots
                    ORDER BY robot_id
                    """
                )
            ) if self._table_exists(connection, "robots") else []

            missions = self._rows(
                connection.execute(
                    """
                    SELECT
                        id,
                        request_id,
                        robot_id,
                        mission_type,
                        status,
                        goal_id,
                        destination_label,
                        source,
                        created_at,
                        started_at,
                        completed_at,
                        final_state
                    FROM missions
                    ORDER BY id DESC
                    LIMIT 20
                    """
                )
            ) if self._table_exists(connection, "missions") else []

            emergencies = self._rows(
                connection.execute(
                    """
                    SELECT
                        id,
                        status,
                        point_x,
                        point_y,
                        dispatched_robot_id,
                        detected_at,
                        updated_at,
                        cleared_at
                    FROM emergency_events
                    ORDER BY id DESC
                    LIMIT 20
                    """
                )
            ) if self._table_exists(connection, "emergency_events") else []

            logs = self._rows(
                connection.execute(
                    """
                    SELECT
                        id,
                        event_type,
                        level,
                        robot_id,
                        message,
                        created_at
                    FROM system_events
                    ORDER BY id DESC
                    LIMIT 60
                    """
                )
            ) if self._table_exists(connection, "system_events") else []

            images = self._rows(
                connection.execute(
                    """
                    SELECT
                        id,
                        emergency_id,
                        event_type,
                        topic_name,
                        image_path,
                        mime_type,
                        captured_at,
                        created_at
                    FROM event_images
                    ORDER BY id DESC
                    LIMIT 12
                    """
                )
            ) if self._table_exists(connection, "event_images") else []

            hot_places = self._rows(
                connection.execute(
                    """
                    SELECT
                        id,
                        pose_index,
                        point_x,
                        point_y,
                        pose_count,
                        received_at
                    FROM hot_place_events
                    ORDER BY id DESC
                    LIMIT 20
                    """
                )
            ) if self._table_exists(connection, "hot_place_events") else []

            emergency_count = 0
            if self._table_exists(connection, "emergency_events"):
                row = connection.execute(
                    "SELECT COUNT(*) AS count FROM emergency_events"
                ).fetchone()
                emergency_count = int(row["count"])

            completed_service_count = 0
            if self._table_exists(connection, "service_requests"):
                row = connection.execute(
                    "SELECT COUNT(*) AS count FROM service_requests WHERE status='COMPLETED'"
                ).fetchone()
                completed_service_count = int(row["count"])

        return {
            "robots": robots,
            "missions": missions,
            "emergencies": emergencies,
            "logs": logs,
            "images": images,
            "hot_places": hot_places,
            "summary": {
                "robot_count": len(robots),
                "emergency_count": emergency_count,
                "completed_service_count": completed_service_count,
                "snapshot_count": len(images),
            },
        }

    def get_event_image(self, image_id: int) -> dict[str, Any] | None:
        with self.connect() as connection:
            if not self._table_exists(connection, "event_images"):
                return None
            row = connection.execute(
                """
                SELECT id, image_path, mime_type, captured_at
                FROM event_images
                WHERE id = ?
                """,
                (int(image_id),),
            ).fetchone()
            return dict(row) if row else None
