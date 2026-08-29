from __future__ import annotations

import base64
import threading
import time
from datetime import datetime
from typing import Any


# 1x1 회색 JPEG. ROS2 카메라가 아직 없을 때 MJPEG 연결을 유지하는 용도.
_PLACEHOLDER_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAP//////////////////////////////////////////////////////////////////////////////////////2wBDAf//////////////////////////////////////////////////////////////////////////////////////wAARCAABAAEDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAX/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIQAxAAAAF//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQABBQJ//8QAFBEBAAAAAAAAAAAAAAAAAAAAAP/aAAgBAwEBPwF//8QAFBEBAAAAAAAAAAAAAAAAAAAAAP/aAAgBAgEBPwF//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQAGPwJ//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQABPyF//9oADAMBAAIAAwAAABAf/8QAFBEBAAAAAAAAAAAAAAAAAAAAAP/aAAgBAwEBPxB//8QAFBEBAAAAAAAAAAAAAAAAAAAAAP/aAAgBAgEBPxB//8QAFBABAAAAAAAAAAAAAAAAAAAAAP/aAAgBAQABPxB//9k="
)

try:
    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Int32

    ROS_CAMERA_AVAILABLE = True
    ROS_CAMERA_ERROR = None
except Exception as exc:  # 관리자 UI 자체는 DB 전용으로도 실행 가능
    rclpy = None
    CompressedImage = None
    Int32 = None
    QoSProfile = ReliabilityPolicy = HistoryPolicy = None
    ROS_CAMERA_AVAILABLE = False
    ROS_CAMERA_ERROR = exc


class CctvImageBridge:
    """CCTV 압축 영상과 공항 전체 인원수 토픽을 함께 구독한다."""

    def __init__(
        self,
        topic_name: str,
        population_topic: str = "/population",
        node_name: str = "aura_admin_monitor_bridge",
    ):
        self.topic_name = topic_name
        self.population_topic = population_topic
        self.node_name = node_name
        self.enabled = False
        self.error: str | None = None

        self._lock = threading.RLock()
        self._population_condition = threading.Condition(self._lock)
        self._latest_frame = _PLACEHOLDER_JPEG
        self._latest_received_monotonic = 0.0
        self._latest_received_at: str | None = None
        self._format: str | None = None
        self._frame_size_bytes: int | None = None
        self._frame_count = 0

        self._population_value: int | None = None
        self._population_received_monotonic = 0.0
        self._population_received_at: str | None = None
        self._population_message_count = 0

        self._node = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        self._start()

    def _start(self) -> None:
        if not ROS_CAMERA_AVAILABLE:
            self.error = f"ROS2/sensor_msgs import 실패: {ROS_CAMERA_ERROR}"
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self._node = rclpy.create_node(self.node_name)

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self._node.create_subscription(
                CompressedImage,
                self.topic_name,
                self._on_image,
                qos,
            )
            self._node.create_subscription(
                Int32,
                self.population_topic,
                self._on_population,
                qos,
            )

            self.enabled = True
            self._thread = threading.Thread(
                target=self._spin_loop,
                name="aura-admin-cctv-spin",
                daemon=True,
            )
            self._thread.start()
        except Exception as exc:
            self.error = f"CCTV bridge 초기화 실패: {exc}"
            self.enabled = False

    def _on_image(self, msg: Any) -> None:
        try:
            frame = bytes(msg.data)
            if not frame:
                raise RuntimeError("빈 CompressedImage 프레임")

            image_format = str(getattr(msg, "format", "") or "").lower()
            # 관리자 UI의 MJPEG 응답은 JPEG를 전제로 한다.
            # 일반적인 image_transport compressed 토픽은 jpeg를 사용한다.
            if image_format and "jpeg" not in image_format and "jpg" not in image_format:
                raise RuntimeError(
                    f"JPEG가 아닌 compressed 형식입니다: {image_format}"
                )
        except Exception as exc:
            self.error = f"CCTV 압축 프레임 처리 실패: {exc}"
            return

        now = datetime.now().astimezone().isoformat(timespec="seconds")
        with self._lock:
            self._latest_frame = frame
            self._latest_received_monotonic = time.monotonic()
            self._latest_received_at = now
            self._format = str(getattr(msg, "format", "") or "jpeg")
            self._frame_size_bytes = len(frame)
            self._frame_count += 1
            self.error = None


    def _on_population(self, msg: Any) -> None:
        now = datetime.now().astimezone().isoformat(timespec="seconds")
        with self._lock:
            self._population_value = max(0, int(msg.data))
            self._population_received_monotonic = time.monotonic()
            self._population_received_at = now
            self._population_message_count += 1
            self._population_condition.notify_all()

    def _spin_loop(self) -> None:
        while self.enabled and not self._stop_event.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as exc:
                self.error = f"ROS2 spin 오류: {exc}"
                time.sleep(0.2)

    def get_frame(self) -> bytes:
        with self._lock:
            return self._latest_frame

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            age = None
            if self._latest_received_monotonic > 0:
                age = round(time.monotonic() - self._latest_received_monotonic, 2)
            receiving = age is not None and age <= 3.0
            return {
                "enabled": self.enabled,
                "receiving": receiving,
                "topic": self.topic_name,
                "message_type": "sensor_msgs/msg/CompressedImage",
                "last_received_at": self._latest_received_at,
                "frame_age_sec": age,
                "frame_count": self._frame_count,
                "format": self._format,
                "frame_size_bytes": self._frame_size_bytes,
                # 기존 JS가 width/height/encoding 필드를 참조해도 깨지지 않도록 유지
                "width": None,
                "height": None,
                "encoding": self._format,
                "error": self.error,
            }


    def get_population_status(self) -> dict[str, Any]:
        with self._lock:
            age = None
            if self._population_received_monotonic > 0:
                age = round(time.monotonic() - self._population_received_monotonic, 2)
            receiving = age is not None and age <= 10.0
            return {
                "enabled": self.enabled,
                "receiving": receiving,
                "topic": self.population_topic,
                "message_type": "std_msgs/msg/Int32",
                "value": self._population_value,
                "last_received_at": self._population_received_at,
                "message_age_sec": age,
                "message_count": self._population_message_count,
                "error": self.error if not self.enabled else None,
            }

    def wait_for_population_update(
        self,
        last_message_count: int,
        timeout: float = 15.0,
    ) -> tuple[dict[str, Any], bool]:
        """새 /population 메시지가 들어올 때까지 대기한다.

        Flask SSE 응답에서 사용한다. timeout 시에도 상태를 반환해
        연결 유지용 heartbeat를 보낼 수 있게 한다.
        """
        with self._population_condition:
            if self._population_message_count == last_message_count:
                self._population_condition.wait(timeout=timeout)

            changed = self._population_message_count != last_message_count
            age = None
            if self._population_received_monotonic > 0:
                age = round(time.monotonic() - self._population_received_monotonic, 2)

            status = {
                "enabled": self.enabled,
                "receiving": age is not None and age <= 10.0,
                "topic": self.population_topic,
                "message_type": "std_msgs/msg/Int32",
                "value": self._population_value,
                "last_received_at": self._population_received_at,
                "message_age_sec": age,
                "message_count": self._population_message_count,
                "error": self.error if not self.enabled else None,
            }
            return status, changed

    def mjpeg_frames(self):
        last_frame = None
        while True:
            frame = self.get_frame()
            if frame != last_frame:
                last_frame = frame
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Cache-Control: no-cache\r\n\r\n"
                    + frame
                    + b"\r\n"
                )
            time.sleep(0.04)

    def shutdown(self) -> None:
        self._stop_event.set()
        self.enabled = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
