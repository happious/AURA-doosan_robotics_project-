from __future__ import annotations

import math
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
import torch
from packaging.version import Version
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Int32
from ultralytics import YOLO
from ultralytics import __version__ as ultralytics_version
from unidepth.models import UniDepthV2


# =============================================================================
# ROS2 Topic 설정
# =============================================================================

REAR_IMAGE_TOPIC = "/image_raw/compressed"

TRACKING_WEB_TOPIC = "/tracking_web"
TRACKING_WEB_CENTER_PIXEL_TOPIC = "/tracking_web_center_pixel"
TARGET_DEPTH_TOPIC = "/target_depth"

# 180도 회전 완료 신호. True를 받은 뒤부터 후방 사람 트래킹을 시작한다.
TURN_COMPLETE_TOPIC = "/turn_complete"

# 전방 동작 시작 신호. Int32 1을 받으면 후방 트래킹을 초기 LOCK 상태로 되돌린다.
FRONT_START_TOPIC = "/front_start"

# 웹/다른 노드에서 Bool True를 보내면 현재 타겟을 해제하고 다시 선정한다.
RETARGET_WEB_TOPIC = "/retarget_web"

# 추가: OpenCV 창의 왼쪽 YOLO/OC-SORT 화면만 compressed로 발행
ANNOTATED_IMAGE_TOPIC = "/rear_yolo_ocsort/image_raw/compressed"
ANNOTATED_FRAME_ID = "rear_yolo_ocsort"
ANNOTATED_JPEG_QUALITY = 80


# =============================================================================
# Tracking 상태값
# =============================================================================

TRACKING_IDLE = 0
TRACKING_VISIBLE = 1
TRACKING_LOST = 2

INVALID_FLOAT = 100.0


# =============================================================================
# YOLO / Tracker 설정
# =============================================================================

YOLO_MODEL_PATH = "yolo11s.pt"
YOLO_DEVICE = 0
YOLO_INPUT_SIZE = 640
YOLO_CONFIDENCE = 0.40
YOLO_IOU = 0.50
PERSON_CLASS_ID = 0

# 네가 OC-SORT를 쓴다고 했으므로 기본값은 ocsort.yaml.
# 환경에 따라 deepocsort.yaml을 쓰고 있었다면 이 값만 바꾸면 된다.
TRACKER_CONFIG = "deepocsort.yaml"

MIN_ULTRALYTICS_VERSION = "8.3.0"


# =============================================================================
# UniDepth 설정
# =============================================================================

UNIDEPTH_MODEL_ID = "lpiccinelli/unidepth-v2-vits14"
UNIDEPTH_DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"
UNIDEPTH_USE_FP16 = True
UNIDEPTH_RESOLUTION_LEVEL = 3
UNIDEPTH_EVERY_N_FRAMES = 1

DEPTH_ROI_AREA_RATIO = 0.10
DEPTH_MIN_M = 0.20
DEPTH_MAX_M = 10.0
DEPTH_MIN_SAMPLES = 20
DEPTH_HISTORY_SIZE = 3

# 최초 타겟 및 일반 retarget은 반드시 이 거리 이내의 사람만 선택한다.
INITIAL_TARGET_MAX_DISTANCE_M = 1.0

# 주행 중 기존 target_id가 사라졌을 때 마지막 유효 거리 기준 허용 오차이다.
LOST_REACQUIRE_DEPTH_TOLERANCE_M = 0.5


# =============================================================================
# 프레임 / stale 설정
# =============================================================================

MAX_FRAME_AGE_SEC = 0.50


# =============================================================================
# ReID 설정
# =============================================================================

REID_ENABLED = True
REID_MAX_LOST_SEC = 4.0
REID_EMA = 0.25

REID_H_BINS = 16
REID_S_BINS = 8
REID_MIN_CROP_AREA = 900

REID_MIN_APPEARANCE_SIMILARITY = 0.80
REID_MIN_TOTAL_SCORE = 0.80

REID_WEIGHT_APPEARANCE = 0.65
REID_WEIGHT_POSITION = 0.25
REID_WEIGHT_SCALE = 0.10

REID_POSITION_SIGMA_RATIO = 0.10
REID_SCALE_LOG_SIGMA = math.log(2.0)


# =============================================================================
# GUI 설정
# =============================================================================

WINDOW_NAME = "Rear Person Vision - YOLO11s OC-SORT + UniDepth V2"
DISPLAY_WIDTH = 960


# region GUI
ENABLE_GUI = True
SHOW_DEPTH_VIEW = True

DEPTH_VIS_MIN_M = 0.20
DEPTH_VIS_MAX_M = 6.00
DEPTH_VIS_COLORMAP = cv2.COLORMAP_TURBO


# =============================================================================
# Publish 주기
# =============================================================================

TRACKING_PUBLISH_PERIOD_SEC = 0.10
GUI_PERIOD_SEC = 0.05

# LOCK 해제 상태에서 /tracking_web이 0 또는 2이면 이 주기로
# /retarget_web=True를 자동 발행한다. /tracking_web=1이면 중단한다.
AUTO_RETARGET_PERIOD_SEC = 0.50


@dataclass(frozen=True)
class Track:
    track_id: int
    box: np.ndarray
    confidence: float


@dataclass(frozen=True)
class Observation:
    seq: int
    receive_time: float
    target_id: Optional[int]
    target_visible: bool
    confidence: float = 0.0
    bbox: Optional[tuple[int, int, int, int]] = None
    raw_distance: Optional[float] = None
    distance: Optional[float] = None
    depth_measured: bool = False
    depth_valid: bool = False
    depth_roi: Optional[tuple[int, int, int, int]] = None
    depth_view: Optional[np.ndarray] = None
    status: str = ""
    process_time_ms: float = 0.0
    unidepth_time_ms: Optional[float] = None
    reid_similarity: Optional[float] = None
    reid_score: Optional[float] = None
    reid_reacquired: bool = False


class RearPersonVisionNode(Node):
    def __init__(self, yolo_model: YOLO, depth_model: UniDepthV2):
        super().__init__("rear_person_vision_node")

        self.yolo_model = yolo_model
        self.depth_model = depth_model
        self.should_shutdown = False

        self.unidepth_device = torch.device(UNIDEPTH_DEVICE)
        self.use_fp16 = (
            UNIDEPTH_USE_FP16 and self.unidepth_device.type == "cuda"
        )

        # ---------------------------------------------------------------------
        # 최신 프레임 저장 슬롯
        # ---------------------------------------------------------------------
        self.frame_lock = threading.Lock()
        self.latest_image_msg: Optional[CompressedImage] = None
        self.latest_frame_receive_time = 0.0
        self.frame_seq = 0
        self.frame_event = threading.Event()
        self.worker_stop = threading.Event()

        # ---------------------------------------------------------------------
        # 트래킹 시작 게이트 / 타겟 ID
        # ---------------------------------------------------------------------
        # 노드 시작 직후에는 False이다. /turn_complete=True가 들어온 뒤에만
        # 현재 화면에서 가장 가까운 사람을 타겟으로 선정한다.
        self.target_lock = threading.Lock()
        self.tracking_enabled = False
        self.target_id: Optional[int] = None
        self.target_lock_time = 0.0

        # 타겟이 정상적으로 보였던 마지막 유효 거리.
        # LOST 복구 시 이 값의 ±0.5m 범위에 있는 사람을 새 타겟으로 선택한다.
        self.last_visible_target_depth: Optional[float] = None

        # 이 노드가 /retarget_web=True를 자동 발행하면 같은 노드의 subscriber도
        # 해당 메시지를 받는다. 자동 메시지가 일반 retarget처럼 상태를 초기화하지
        # 않도록 self-echo 개수를 별도로 관리한다.
        self.auto_retarget_lock = threading.Lock()
        self.pending_auto_retarget_echoes = 0
        self.last_auto_retarget_log_time = 0.0

        # ---------------------------------------------------------------------
        # ReID memory
        # ---------------------------------------------------------------------
        self.reid_lock = threading.Lock()
        self.target_appearance_feature: Optional[np.ndarray] = None
        self.target_last_bbox: Optional[tuple[int, int, int, int]] = None
        self.target_last_center: Optional[tuple[float, float]] = None
        self.target_last_area: Optional[float] = None
        self.target_last_visible_time = 0.0
        self.last_reid_similarity: Optional[float] = None
        self.last_reid_score: Optional[float] = None

        # ---------------------------------------------------------------------
        # 최신 관측값
        # ---------------------------------------------------------------------
        self.obs_lock = threading.Lock()
        self.latest_obs: Optional[Observation] = None
        self.last_observation_update_time = 0.0

        # ---------------------------------------------------------------------
        # Depth cache
        # ---------------------------------------------------------------------
        self.cached_depth_target_id: Optional[int] = None
        self.cached_raw_distance: Optional[float] = None
        self.cached_filtered_distance: Optional[float] = None
        self.cached_depth_view: Optional[np.ndarray] = None
        self.distance_history = deque(maxlen=DEPTH_HISTORY_SIZE)

        # ---------------------------------------------------------------------
        # GUI image
        # ---------------------------------------------------------------------
        self.display_lock = threading.Lock()
        self.display_image: Optional[np.ndarray] = None

        self.last_warning_time = 0.0

        # ---------------------------------------------------------------------
        # QoS 설정
        # ---------------------------------------------------------------------
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ---------------------------------------------------------------------
        # ROS2 Subscriber
        # ---------------------------------------------------------------------
        self.image_subscription = self.create_subscription(
            CompressedImage,
            REAR_IMAGE_TOPIC,
            self.image_callback,
            sensor_qos,
        )

        self.turn_complete_subscription = self.create_subscription(
            Bool,
            TURN_COMPLETE_TOPIC,
            self.turn_complete_callback,
            state_qos,
        )

        self.front_start_subscription = self.create_subscription(
            Int32,
            FRONT_START_TOPIC,
            self.front_start_callback,
            state_qos,
        )

        self.retarget_subscription = self.create_subscription(
            Bool,
            RETARGET_WEB_TOPIC,
            self.retarget_callback,
            state_qos,
        )

        # ---------------------------------------------------------------------
        # ROS2 Publisher
        # ---------------------------------------------------------------------
        self.tracking_web_publisher = self.create_publisher(
            Int32,
            TRACKING_WEB_TOPIC,
            state_qos,
        )

        self.tracking_center_pixel_publisher = self.create_publisher(
            Float32,
            TRACKING_WEB_CENTER_PIXEL_TOPIC,
            state_qos,
        )

        self.target_depth_publisher = self.create_publisher(
            Float32,
            TARGET_DEPTH_TOPIC,
            state_qos,
        )

        # LOCK 해제 상태에서 tracking 상태가 0 또는 2이면 자동으로 True를 발행한다.
        # 같은 토픽을 이 노드도 구독하므로 auto-generated self echo는 콜백에서 구분한다.
        self.retarget_web_publisher = self.create_publisher(
            Bool,
            RETARGET_WEB_TOPIC,
            state_qos,
        )

        self.annotated_image_publisher = self.create_publisher(
            CompressedImage,
            ANNOTATED_IMAGE_TOPIC,
            sensor_qos,
        )

        # ---------------------------------------------------------------------
        # ROS2 Timer
        # ---------------------------------------------------------------------
        self.create_timer(
            TRACKING_PUBLISH_PERIOD_SEC,
            self.publish_tracking_topics,
        )

        self.create_timer(
            GUI_PERIOD_SEC,
            self.gui_callback,
        )

        self.create_timer(
            AUTO_RETARGET_PERIOD_SEC,
            self.auto_retarget_timer_callback,
        )

        # ---------------------------------------------------------------------
        # OpenCV window
        # ---------------------------------------------------------------------
        if ENABLE_GUI:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        # ---------------------------------------------------------------------
        # Worker thread
        # ---------------------------------------------------------------------
        self.worker = threading.Thread(
            target=self.worker_loop,
            daemon=True,
        )
        self.worker.start()

        self.get_logger().info("Rear person vision ROS2 node started")
        self.get_logger().info(f"sub image: {REAR_IMAGE_TOPIC}")
        self.get_logger().info(
            f"sub tracking start: {TURN_COMPLETE_TOPIC}"
        )
        self.get_logger().info(f"sub tracking lock: {FRONT_START_TOPIC}")
        self.get_logger().info(f"sub retarget: {RETARGET_WEB_TOPIC}")
        self.get_logger().info(f"pub tracking: {TRACKING_WEB_TOPIC}")
        self.get_logger().info(f"pub center: {TRACKING_WEB_CENTER_PIXEL_TOPIC}")
        self.get_logger().info(f"pub depth: {TARGET_DEPTH_TOPIC}")
        self.get_logger().info(f"pub auto retarget: {RETARGET_WEB_TOPIC}")
        self.get_logger().info(f"pub annotated image: {ANNOTATED_IMAGE_TOPIC}")

    # =========================================================================
    # 기본 유틸
    # =========================================================================

    @staticmethod
    def draw_text(
        image: np.ndarray,
        text: str,
        xy: tuple[int, int],
        color: tuple[int, int, int] = (255, 255, 255),
        scale: float = 0.5,
    ) -> None:
        cv2.putText(
            image,
            text,
            xy,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            text,
            xy,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            2,
            cv2.LINE_AA,
        )

    @staticmethod
    def decode_compressed_image(msg: CompressedImage) -> Optional[np.ndarray]:
        image = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8),
            cv2.IMREAD_COLOR,
        )
        return image

    @staticmethod
    def clip_box(
        box: np.ndarray | tuple[float, float, float, float],
        image_width: int,
        image_height: int,
    ) -> tuple[int, int, int, int]:
        x1, y1, x2, y2 = np.asarray(box, dtype=np.float32)[:4]

        x1 = int(np.clip(round(float(x1)), 0, image_width - 1))
        y1 = int(np.clip(round(float(y1)), 0, image_height - 1))
        x2 = int(np.clip(round(float(x2)), x1 + 1, image_width))
        y2 = int(np.clip(round(float(y2)), y1 + 1, image_height))

        return x1, y1, x2, y2

    @staticmethod
    def box_center(
        box: tuple[int, int, int, int],
    ) -> tuple[float, float]:
        x1, y1, x2, y2 = box
        return (x1 + x2) * 0.5, (y1 + y2) * 0.5

    @staticmethod
    def box_area(
        box: tuple[int, int, int, int],
    ) -> float:
        x1, y1, x2, y2 = box
        return float(max(1, x2 - x1) * max(1, y2 - y1))

    @staticmethod
    def cosine_similarity(
        a: Optional[np.ndarray],
        b: Optional[np.ndarray],
    ) -> float:
        if a is None or b is None:
            return 0.0

        denom = float(np.linalg.norm(a) * np.linalg.norm(b))

        if denom <= 1e-8:
            return 0.0

        return float(np.clip(np.dot(a, b) / denom, 0.0, 1.0))

    # =========================================================================
    # ROS2 callbacks
    # =========================================================================

    def image_callback(self, msg: CompressedImage) -> None:
        with self.frame_lock:
            self.frame_seq += 1
            self.latest_image_msg = msg
            self.latest_frame_receive_time = time.monotonic()
            self.frame_event.set()

    def turn_complete_callback(self, msg: Bool) -> None:
        # False는 상태 변경에 사용하지 않는다. True를 한 번 받은 뒤부터
        # 트래킹은 계속 활성 상태를 유지한다.
        if not msg.data:
            return

        with self.target_lock:
            if self.tracking_enabled:
                return

            self.tracking_enabled = True

        self.get_logger().info(
            "turn_complete=True received: nearest-person tracking enabled"
        )

        # 활성화 시점에 이전 캐시가 남지 않도록 초기화한다.
        # 다음 영상 프레임에서 모든 사람의 depth를 비교해 타겟을 선정한다.
        self.reset_target(source="turn_complete")

    def front_start_callback(self, msg: Int32) -> None:
        # /front_start는 전방 노드가 제어를 시작했다는 신호로 사용한다.
        # 값이 1이면 현재 타겟과 모든 추적 메모리를 제거하고,
        # 노드 시작 직후와 동일한 LOCK 상태로 되돌린다.
        if msg.data != 1:
            return

        with self.target_lock:
            self.tracking_enabled = False

        self.get_logger().info(
            "front_start=1 received: rear tracking locked and reset"
        )

        # reset_target()은 target_id, 거리 캐시, ReID 메모리,
        # 최신 관측값을 모두 초기화하고 IDLE 토픽을 즉시 발행한다.
        self.reset_target(source="front_start=1")

    def retarget_callback(self, msg: Bool) -> None:
        """
        외부에서 들어온 /retarget_web=True는 일반 retarget으로 처리한다.

        이 노드가 자동으로 발행한 /retarget_web=True는 self-echo이므로,
        현재 target_id와 마지막 Depth를 지우지 않고 무시한다.
        실제 자동 재선정은 process_frame()에서 매 프레임 수행한다.
        """
        if not msg.data:
            return

        auto_generated = False

        with self.auto_retarget_lock:
            if self.pending_auto_retarget_echoes > 0:
                self.pending_auto_retarget_echoes -= 1
                auto_generated = True

        if auto_generated:
            return

        with self.target_lock:
            tracking_enabled = self.tracking_enabled

        if not tracking_enabled:
            self.get_logger().info(
                "Retarget ignored: waiting for /turn_complete=True"
            )
            return

        self.get_logger().info(
            "Manual retarget requested: selecting a person within 1.0m"
        )
        self.reset_target(source="topic")

    def auto_retarget_timer_callback(self) -> None:
        """
        LOCK 해제 상태에서 tracking 상태가 0 또는 2이면
        /retarget_web=True를 주기적으로 발행한다.

        - 상태 0: 1m 이내 최초/일반 타겟 선정을 계속 시도한다.
        - 상태 2: 기존 ID를 먼저 기다리고, 없으면 마지막 Depth ±0.5m로 복구한다.
        - 상태 1: 자동 발행을 즉시 중단한다.
        """
        with self.target_lock:
            tracking_enabled = self.tracking_enabled

        if not tracking_enabled:
            return

        tracking_status, _, _ = self.make_tracking_snapshot()

        if tracking_status == TRACKING_VISIBLE:
            return

        with self.auto_retarget_lock:
            self.pending_auto_retarget_echoes += 1

        msg = Bool()
        msg.data = True
        self.retarget_web_publisher.publish(msg)

        now = time.monotonic()
        if now - self.last_auto_retarget_log_time >= 2.0:
            self.last_auto_retarget_log_time = now
            self.get_logger().info(
                f"Auto retarget published: /tracking_web={tracking_status}"
            )

    # =========================================================================
    # Worker
    # =========================================================================

    def worker_loop(self) -> None:
        while not self.worker_stop.is_set():
            if not self.frame_event.wait(timeout=0.1):
                continue

            with self.frame_lock:
                msg = self.latest_image_msg
                seq = self.frame_seq
                receive_time = self.latest_frame_receive_time

                self.latest_image_msg = None
                self.frame_event.clear()

            if msg is None:
                continue

            try:
                self.process_frame(
                    seq=seq,
                    msg=msg,
                    receive_time=receive_time,
                )

            except Exception as exc:
                now = time.monotonic()

                if now - self.last_warning_time > 2.0:
                    self.last_warning_time = now
                    self.get_logger().warn(
                        f"frame processing failed: "
                        f"{type(exc).__name__}: {exc}"
                    )

    # =========================================================================
    # YOLO / OC-SORT
    # =========================================================================

    def extract_tracks(self, result) -> list[Track]:
        boxes = result.boxes

        if boxes is None or len(boxes) == 0 or boxes.id is None:
            return []

        xyxy = boxes.xyxy.detach().cpu().numpy()
        ids = boxes.id.detach().cpu().numpy().astype(int)
        confs = boxes.conf.detach().cpu().numpy()

        tracks: list[Track] = []

        for box, track_id, conf in zip(xyxy, ids, confs):
            tracks.append(
                Track(
                    track_id=int(track_id),
                    box=box.astype(np.float32),
                    confidence=float(conf),
                )
            )

        return tracks

    def select_nearest_track_by_depth(
        self,
        tracks: list[Track],
        depth_map: np.ndarray,
        image_width: int,
        image_height: int,
        max_distance: Optional[float] = None,
    ) -> tuple[
        Optional[Track],
        Optional[float],
        Optional[tuple[int, int, int, int]],
    ]:
        """
        유효한 사람 중 가장 가까운 ID를 선택한다.

        max_distance가 주어지면 해당 거리 이내의 사람만 후보로 인정한다.
        최초 타겟과 일반 retarget에서는 1.0m가 전달된다.
        """
        nearest_track: Optional[Track] = None
        nearest_distance: Optional[float] = None
        nearest_roi: Optional[tuple[int, int, int, int]] = None

        for track in tracks:
            box = self.clip_box(
                track.box,
                image_width,
                image_height,
            )

            roi = self.make_depth_roi(
                box,
                image_width,
                image_height,
            )

            distance = self.compute_roi_distance(depth_map, roi)

            if distance is None:
                continue

            if max_distance is not None and distance > max_distance:
                continue

            if nearest_distance is None or distance < nearest_distance:
                nearest_track = track
                nearest_distance = distance
                nearest_roi = roi

        return nearest_track, nearest_distance, nearest_roi

    def select_track_by_reference_depth(
        self,
        tracks: list[Track],
        depth_map: np.ndarray,
        image_width: int,
        image_height: int,
        reference_depth: float,
        tolerance: float,
    ) -> tuple[
        Optional[Track],
        Optional[float],
        Optional[tuple[int, int, int, int]],
    ]:
        """
        주행 중 LOST 복구용 타겟 선택.

        기존 target_id가 현재 프레임에 없을 때만 호출한다.
        마지막 유효 Depth의 ±tolerance 범위에 있는 사람만 후보로 두고,
        마지막 Depth와 차이가 가장 작은 사람을 새 타겟으로 선택한다.

        이 복구 모드에는 1m 제한을 적용하지 않는다.
        """
        min_depth = max(DEPTH_MIN_M, reference_depth - tolerance)
        max_depth = min(DEPTH_MAX_M, reference_depth + tolerance)

        best_track: Optional[Track] = None
        best_distance: Optional[float] = None
        best_roi: Optional[tuple[int, int, int, int]] = None
        best_difference: Optional[float] = None

        for track in tracks:
            box = self.clip_box(
                track.box,
                image_width,
                image_height,
            )

            roi = self.make_depth_roi(
                box,
                image_width,
                image_height,
            )

            distance = self.compute_roi_distance(depth_map, roi)

            if distance is None:
                continue

            if distance < min_depth or distance > max_depth:
                continue

            difference = abs(distance - reference_depth)

            if (
                best_difference is None
                or difference < best_difference
                or (
                    math.isclose(difference, best_difference, abs_tol=1e-6)
                    and best_distance is not None
                    and distance < best_distance
                )
            ):
                best_track = track
                best_distance = distance
                best_roi = roi
                best_difference = difference

        return best_track, best_distance, best_roi

    # =========================================================================
    # ReID
    # =========================================================================

    def extract_appearance_feature(
        self,
        image: np.ndarray,
        box: tuple[int, int, int, int],
    ) -> Optional[np.ndarray]:
        image_height, image_width = image.shape[:2]

        x1, y1, x2, y2 = self.clip_box(
            box,
            image_width,
            image_height,
        )

        box_width = x2 - x1
        box_height = y2 - y1

        if box_width * box_height < REID_MIN_CROP_AREA:
            return None

        margin_x = round(box_width * 0.15)
        margin_y = round(box_height * 0.05)

        crop = image[
            y1 + margin_y:y2 - margin_y,
            x1 + margin_x:x2 - margin_x,
        ]

        if crop.size == 0:
            return None

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        split_y = max(1, hsv.shape[0] // 2)
        parts = (hsv[:split_y], hsv[split_y:])

        features = []

        for part in parts:
            hist = cv2.calcHist(
                [part],
                [0, 1],
                None,
                [REID_H_BINS, REID_S_BINS],
                [0, 180, 0, 256],
            ).astype(np.float32)

            norm = float(np.linalg.norm(hist))

            if norm > 1e-8:
                hist = hist / norm

            features.append(hist.reshape(-1))

        feature = np.concatenate(features).astype(np.float32)
        norm = float(np.linalg.norm(feature))

        if norm <= 1e-8:
            return None

        return feature / norm

    def update_reid_memory(
        self,
        image: np.ndarray,
        box: tuple[int, int, int, int],
    ) -> None:
        if not REID_ENABLED:
            return

        feature = self.extract_appearance_feature(image, box)

        if feature is None:
            return

        with self.reid_lock:
            if self.target_appearance_feature is None:
                self.target_appearance_feature = feature

            else:
                updated = (
                    (1.0 - REID_EMA) * self.target_appearance_feature
                    + REID_EMA * feature
                )

                self.target_appearance_feature = (
                    updated / max(float(np.linalg.norm(updated)), 1e-8)
                ).astype(np.float32)

            self.target_last_bbox = box
            self.target_last_center = self.box_center(box)
            self.target_last_area = self.box_area(box)
            self.target_last_visible_time = time.monotonic()

    def reset_reid_memory(self) -> None:
        with self.reid_lock:
            self.target_appearance_feature = None
            self.target_last_bbox = None
            self.target_last_center = None
            self.target_last_area = None
            self.target_last_visible_time = 0.0
            self.last_reid_similarity = None
            self.last_reid_score = None

    def try_reidentify_target(
        self,
        image: np.ndarray,
        tracks: list[Track],
        image_width: int,
        image_height: int,
    ) -> tuple[Optional[Track], Optional[float], Optional[float]]:
        if not REID_ENABLED or not tracks:
            return None, None, None

        with self.reid_lock:
            reference_feature = (
                None
                if self.target_appearance_feature is None
                else self.target_appearance_feature.copy()
            )
            last_center = self.target_last_center
            last_area = self.target_last_area
            last_visible_time = self.target_last_visible_time

        if reference_feature is None or last_center is None or last_area is None:
            return None, None, None

        if time.monotonic() - last_visible_time > REID_MAX_LOST_SEC:
            return None, None, None

        position_sigma = max(
            1.0,
            math.hypot(image_width, image_height) * REID_POSITION_SIGMA_RATIO,
        )

        best_track: Optional[Track] = None
        best_similarity: Optional[float] = None
        best_score: Optional[float] = None

        for track in tracks:
            box = self.clip_box(
                track.box,
                image_width,
                image_height,
            )

            feature = self.extract_appearance_feature(image, box)
            similarity = self.cosine_similarity(reference_feature, feature)

            if similarity < REID_MIN_APPEARANCE_SIMILARITY:
                continue

            center_x, center_y = self.box_center(box)

            position_distance = math.hypot(
                center_x - last_center[0],
                center_y - last_center[1],
            )

            position_score = math.exp(
                -((position_distance / position_sigma) ** 2)
            )

            area_ratio = self.box_area(box) / max(last_area, 1.0)

            scale_score = math.exp(
                -(
                    (
                        abs(math.log(max(area_ratio, 1e-6)))
                        / REID_SCALE_LOG_SIGMA
                    )
                    ** 2
                )
            )

            total_score = (
                REID_WEIGHT_APPEARANCE * similarity
                + REID_WEIGHT_POSITION * position_score
                + REID_WEIGHT_SCALE * scale_score
            )

            if best_score is None or total_score > best_score:
                best_track = track
                best_similarity = similarity
                best_score = total_score

        if best_track is None or best_score is None:
            return None, best_similarity, best_score

        if best_score < REID_MIN_TOTAL_SCORE:
            return None, best_similarity, best_score

        return best_track, best_similarity, best_score

    # =========================================================================
    # Depth
    # =========================================================================

    def make_depth_roi(
        self,
        box: tuple[int, int, int, int],
        image_width: int,
        image_height: int,
    ) -> tuple[int, int, int, int]:
        x1, y1, x2, y2 = map(float, box)

        box_width = max(1.0, x2 - x1)
        box_height = max(1.0, y2 - y1)

        side_ratio = math.sqrt(DEPTH_ROI_AREA_RATIO)

        roi_width = max(1, round(box_width * side_ratio))
        roi_height = max(1, round(box_height * side_ratio))

        center_x = (x1 + x2) * 0.5
        center_y = (y1 + y2) * 0.5

        roi_x1 = int(round(center_x - roi_width / 2))
        roi_y1 = int(round(center_y - roi_height / 2))

        roi_x1 = max(0, min(roi_x1, image_width - 1))
        roi_y1 = max(0, min(roi_y1, image_height - 1))

        roi_x2 = max(roi_x1 + 1, min(roi_x1 + roi_width, image_width))
        roi_y2 = max(roi_y1 + 1, min(roi_y1 + roi_height, image_height))

        return roi_x1, roi_y1, roi_x2, roi_y2

    def infer_depth(self, image: np.ndarray) -> tuple[np.ndarray, float]:
        image_height, image_width = image.shape[:2]

        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        tensor = torch.from_numpy(
            np.ascontiguousarray(rgb)
        ).permute(2, 0, 1).contiguous()

        tensor = tensor.to(self.unidepth_device)

        if self.unidepth_device.type == "cuda":
            torch.cuda.synchronize(self.unidepth_device)

        start_time = time.perf_counter()

        with torch.inference_mode(), torch.autocast(
            self.unidepth_device.type,
            dtype=torch.float16,
            enabled=self.use_fp16,
        ):
            prediction = self.depth_model.infer(tensor)

        if self.unidepth_device.type == "cuda":
            torch.cuda.synchronize(self.unidepth_device)

        depth = prediction["depth"].detach().float().squeeze().cpu().numpy()

        if depth.shape != (image_height, image_width):
            depth = cv2.resize(
                depth,
                (image_width, image_height),
                interpolation=cv2.INTER_LINEAR,
            )

        elapsed_ms = (time.perf_counter() - start_time) * 1000.0

        return depth, elapsed_ms

    def compute_roi_distance(
        self,
        depth: np.ndarray,
        roi: tuple[int, int, int, int],
    ) -> Optional[float]:
        x1, y1, x2, y2 = roi

        values = depth[y1:y2, x1:x2].reshape(-1)

        values = values[
            np.isfinite(values)
            & (values >= DEPTH_MIN_M)
            & (values <= DEPTH_MAX_M)
        ]

        if values.size < DEPTH_MIN_SAMPLES:
            return None

        low, high = np.percentile(values, [5.0, 95.0])
        values = values[(values >= low) & (values <= high)]

        if values.size < DEPTH_MIN_SAMPLES:
            return None

        return float(np.median(values))

    def filter_distance(self, raw_distance: float) -> float:
        self.distance_history.append(float(raw_distance))
        return float(np.median(np.asarray(self.distance_history)))

    def reset_distance_filter(self) -> None:
        self.distance_history.clear()

        self.cached_depth_target_id = None
        self.cached_raw_distance = None
        self.cached_filtered_distance = None
        self.cached_depth_view = None

    def make_depth_colormap(self, depth: np.ndarray) -> np.ndarray:
        valid = (
            np.isfinite(depth)
            & (depth >= DEPTH_VIS_MIN_M)
            & (depth <= DEPTH_VIS_MAX_M)
        )

        normalized = (
            np.clip(depth, DEPTH_VIS_MIN_M, DEPTH_VIS_MAX_M)
            - DEPTH_VIS_MIN_M
        ) / max(DEPTH_VIS_MAX_M - DEPTH_VIS_MIN_M, 1e-6)

        view = cv2.applyColorMap(
            np.clip((1.0 - normalized) * 255, 0, 255).astype(np.uint8),
            DEPTH_VIS_COLORMAP,
        )

        view[~valid] = (35, 35, 35)

        return view

    # =========================================================================
    # Main frame processing
    # =========================================================================

    def process_frame(
        self,
        seq: int,
        msg: CompressedImage,
        receive_time: float,
    ) -> None:
        if time.monotonic() - receive_time > MAX_FRAME_AGE_SEC:
            return

        process_start_time = time.perf_counter()

        image = self.decode_compressed_image(msg)

        if image is None:
            return

        image_height, image_width = image.shape[:2]

        result = self.yolo_model.track(
            source=image,
            persist=True,
            tracker=TRACKER_CONFIG,
            classes=[PERSON_CLASS_ID],
            conf=YOLO_CONFIDENCE,
            iou=YOLO_IOU,
            imgsz=YOLO_INPUT_SIZE,
            device=YOLO_DEVICE,
            verbose=False,
        )[0]

        tracks = self.extract_tracks(result)

        with self.target_lock:
            tracking_enabled = self.tracking_enabled
            current_target_id = self.target_id
            last_visible_depth = self.last_visible_target_depth

        # LOCK 상태에서는 검출 화면만 유지하고 타겟 선정/복구는 수행하지 않는다.
        if not tracking_enabled:
            obs = Observation(
                seq=seq,
                receive_time=receive_time,
                target_id=None,
                target_visible=False,
                depth_view=self.cached_depth_view,
                status="tracking locked: waiting /turn_complete=True",
                process_time_ms=(time.perf_counter() - process_start_time)
                * 1000.0,
            )

            self.store_observation(obs)
            self.draw_and_publish_display(image, tracks, obs)
            return

        selection_raw_distance: Optional[float] = None
        selection_depth_roi: Optional[tuple[int, int, int, int]] = None
        selection_unidepth_time_ms: Optional[float] = None
        selected_on_this_frame = False
        selection_mode: Optional[str] = None

        # ------------------------------------------------------------------
        # 1) 최초 타겟 / 일반 retarget
        # ------------------------------------------------------------------
        # target_id가 없으면 반드시 1m 이내의 사람만 후보로 사용한다.
        if current_target_id is None:
            if not tracks:
                obs = Observation(
                    seq=seq,
                    receive_time=receive_time,
                    target_id=None,
                    target_visible=False,
                    depth_view=self.cached_depth_view,
                    status="tracking active: waiting person within 1.0m",
                    process_time_ms=(time.perf_counter() - process_start_time)
                    * 1000.0,
                )

                self.store_observation(obs)
                self.draw_and_publish_display(image, tracks, obs)
                return

            depth_map, selection_unidepth_time_ms = self.infer_depth(image)
            depth_view = self.make_depth_colormap(depth_map)

            (
                selected_track,
                selection_raw_distance,
                selection_depth_roi,
            ) = self.select_nearest_track_by_depth(
                tracks,
                depth_map,
                image_width,
                image_height,
                max_distance=INITIAL_TARGET_MAX_DISTANCE_M,
            )

            if selected_track is None or selection_raw_distance is None:
                self.cached_depth_view = depth_view

                obs = Observation(
                    seq=seq,
                    receive_time=receive_time,
                    target_id=None,
                    target_visible=False,
                    depth_view=self.cached_depth_view,
                    status="tracking active: no person within 1.0m",
                    process_time_ms=(time.perf_counter() - process_start_time)
                    * 1000.0,
                    unidepth_time_ms=selection_unidepth_time_ms,
                )

                self.store_observation(obs)
                self.draw_and_publish_display(image, tracks, obs)
                return

            self.reset_distance_filter()
            self.cached_depth_view = depth_view
            self.reset_reid_memory()

            with self.target_lock:
                if self.tracking_enabled and self.target_id is None:
                    self.target_id = selected_track.track_id
                    self.target_lock_time = time.monotonic()

                current_target_id = self.target_id

            if current_target_id != selected_track.track_id:
                return

            selected_on_this_frame = True
            selection_mode = "initial_1m"

            self.get_logger().info(
                f"TARGET LOCKED WITHIN 1.0m: "
                f"ID {current_target_id}, "
                f"distance={selection_raw_distance:.3f}m"
            )

        # ------------------------------------------------------------------
        # 2) 동일 ID 유지
        # ------------------------------------------------------------------
        # 주행 중에는 같은 target_id가 다시 보이면 다른 후보보다 무조건 우선한다.
        target_track = next(
            (
                track
                for track in tracks
                if track.track_id == current_target_id
            ),
            None,
        )

        reid_similarity = None
        reid_score = None
        reid_reacquired = False

        # ------------------------------------------------------------------
        # 3) 기존 ID가 없을 때 마지막 Depth ±0.5m로 LOST 복구
        # ------------------------------------------------------------------
        if target_track is None and current_target_id is not None:
            if tracks and last_visible_depth is not None:
                depth_map, selection_unidepth_time_ms = self.infer_depth(image)
                depth_view = self.make_depth_colormap(depth_map)

                (
                    recovery_track,
                    selection_raw_distance,
                    selection_depth_roi,
                ) = self.select_track_by_reference_depth(
                    tracks,
                    depth_map,
                    image_width,
                    image_height,
                    reference_depth=last_visible_depth,
                    tolerance=LOST_REACQUIRE_DEPTH_TOLERANCE_M,
                )

                if (
                    recovery_track is not None
                    and selection_raw_distance is not None
                ):
                    previous_target_id = current_target_id

                    self.reset_distance_filter()
                    self.cached_depth_view = depth_view
                    self.reset_reid_memory()

                    with self.target_lock:
                        if (
                            self.tracking_enabled
                            and self.target_id == previous_target_id
                        ):
                            self.target_id = recovery_track.track_id
                            self.target_lock_time = time.monotonic()

                        current_target_id = self.target_id

                    if current_target_id != recovery_track.track_id:
                        return

                    target_track = recovery_track
                    selected_on_this_frame = True
                    selection_mode = "lost_depth_recovery"

                    self.get_logger().info(
                        f"TARGET RECOVERED BY DEPTH: "
                        f"ID {previous_target_id} -> {current_target_id}, "
                        f"last={last_visible_depth:.3f}m, "
                        f"new={selection_raw_distance:.3f}m, "
                        f"range=±{LOST_REACQUIRE_DEPTH_TOLERANCE_M:.1f}m"
                    )
                else:
                    self.cached_depth_view = depth_view

            if target_track is None:
                if last_visible_depth is None:
                    status = (
                        f"target ID {current_target_id} not visible; "
                        "waiting same ID (no last valid depth)"
                    )
                else:
                    status = (
                        f"target ID {current_target_id} not visible; "
                        f"waiting same ID or "
                        f"{last_visible_depth:.2f}±"
                        f"{LOST_REACQUIRE_DEPTH_TOLERANCE_M:.1f}m"
                    )

                obs = Observation(
                    seq=seq,
                    receive_time=receive_time,
                    target_id=current_target_id,
                    target_visible=False,
                    depth_view=self.cached_depth_view,
                    status=status,
                    process_time_ms=(time.perf_counter() - process_start_time)
                    * 1000.0,
                    unidepth_time_ms=selection_unidepth_time_ms,
                )

                self.store_observation(obs)
                self.draw_and_publish_display(image, tracks, obs)
                return

        target_box = self.clip_box(
            target_track.box,
            image_width,
            image_height,
        )

        depth_roi = (
            selection_depth_roi
            if selected_on_this_frame and selection_depth_roi is not None
            else self.make_depth_roi(
                target_box,
                image_width,
                image_height,
            )
        )

        should_measure_depth = (
            selected_on_this_frame
            or (seq - 1) % UNIDEPTH_EVERY_N_FRAMES == 0
            or self.cached_depth_target_id != current_target_id
            or self.cached_filtered_distance is None
        )

        raw_distance = self.cached_raw_distance
        filtered_distance = self.cached_filtered_distance

        depth_measured = False
        depth_valid = (
            self.cached_depth_target_id == current_target_id
            and self.cached_filtered_distance is not None
        )

        unidepth_time_ms = None
        status = "cached depth"

        if selected_on_this_frame:
            raw_distance = selection_raw_distance
            unidepth_time_ms = selection_unidepth_time_ms
            depth_measured = True

        elif should_measure_depth:
            depth_map, unidepth_time_ms = self.infer_depth(image)
            self.cached_depth_view = self.make_depth_colormap(depth_map)

            raw_distance = self.compute_roi_distance(
                depth_map,
                depth_roi,
            )

            depth_measured = True

        if depth_measured:
            if raw_distance is None:
                depth_valid = False
                status = "invalid depth ROI"

            else:
                if self.cached_depth_target_id != current_target_id:
                    self.distance_history.clear()

                filtered_distance = self.filter_distance(raw_distance)

                self.cached_depth_target_id = current_target_id
                self.cached_raw_distance = raw_distance
                self.cached_filtered_distance = filtered_distance

                depth_valid = True

                if selection_mode == "initial_1m":
                    status = "target selected within 1.0m"
                elif selection_mode == "lost_depth_recovery":
                    status = "target recovered by last depth ±0.5m"
                else:
                    status = "depth measured"

                # 사라지기 직전 복구 기준으로 사용할 마지막 정상 Depth를 저장한다.
                with self.target_lock:
                    if (
                        self.tracking_enabled
                        and self.target_id == current_target_id
                    ):
                        self.last_visible_target_depth = float(
                            filtered_distance
                        )

        self.update_reid_memory(image, target_box)

        obs = Observation(
            seq=seq,
            receive_time=receive_time,
            target_id=current_target_id,
            target_visible=True,
            confidence=target_track.confidence,
            bbox=target_box,
            raw_distance=raw_distance,
            distance=filtered_distance,
            depth_measured=depth_measured,
            depth_valid=depth_valid,
            depth_roi=depth_roi,
            depth_view=self.cached_depth_view,
            status=status,
            process_time_ms=(time.perf_counter() - process_start_time)
            * 1000.0,
            unidepth_time_ms=unidepth_time_ms,
            reid_similarity=reid_similarity,
            reid_score=reid_score,
            reid_reacquired=reid_reacquired,
        )

        self.store_observation(obs)
        self.draw_and_publish_display(image, tracks, obs)

    def store_observation(self, obs: Observation) -> None:
        with self.obs_lock:
            self.latest_obs = obs
            self.last_observation_update_time = time.monotonic()

    # =========================================================================
    # ROS2 topic publish
    # =========================================================================

    def make_tracking_snapshot(self) -> tuple[int, float, float]:
        with self.target_lock:
            tracking_enabled = self.tracking_enabled
            current_target_id = self.target_id

        if not tracking_enabled or current_target_id is None:
            return TRACKING_IDLE, INVALID_FLOAT, INVALID_FLOAT

        with self.obs_lock:
            obs = self.latest_obs
            obs_age = time.monotonic() - self.last_observation_update_time

        if (
            obs is None
            or obs_age > MAX_FRAME_AGE_SEC
            or obs.target_id != current_target_id
            or not obs.target_visible
            or obs.bbox is None
        ):
            return TRACKING_LOST, INVALID_FLOAT, INVALID_FLOAT

        x1, _, x2, _ = obs.bbox
        center_x = (x1 + x2) * 0.5

        depth_value = (
            obs.distance
            if obs.depth_valid and obs.distance is not None
            else INVALID_FLOAT
        )

        return TRACKING_VISIBLE, float(center_x), float(depth_value)

    def publish_tracking_topics(self) -> None:
        tracking_status, center_pixel, target_depth = self.make_tracking_snapshot()

        tracking_msg = Int32()
        center_msg = Float32()
        depth_msg = Float32()

        tracking_msg.data = int(tracking_status)
        center_msg.data = float(center_pixel)
        depth_msg.data = float(target_depth)

        self.tracking_web_publisher.publish(tracking_msg)
        self.tracking_center_pixel_publisher.publish(center_msg)
        self.target_depth_publisher.publish(depth_msg)

    def publish_annotated_image(self, image: np.ndarray) -> None:
        if image is None or image.size == 0:
            return

        jpeg_quality = max(1, min(100, int(ANNOTATED_JPEG_QUALITY)))

        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
        )

        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ANNOTATED_FRAME_ID
        msg.format = "jpeg"
        msg.data = encoded.tobytes()

        self.annotated_image_publisher.publish(msg)

    # =========================================================================
    # Display
    # =========================================================================

    def draw_and_publish_display(
        self,
        image: np.ndarray,
        tracks: list[Track],
        obs: Observation,
    ) -> None:
        image_height, image_width = image.shape[:2]

        with self.target_lock:
            tracking_enabled = self.tracking_enabled
            current_target_id = self.target_id

        for track in tracks:
            x1, y1, x2, y2 = self.clip_box(
                track.box,
                image_width,
                image_height,
            )

            is_target = track.track_id == current_target_id

            color = (0, 0, 255) if is_target else (0, 200, 0)
            thickness = 3 if is_target else 2

            cv2.rectangle(
                image,
                (x1, y1),
                (x2, y2),
                color,
                thickness,
            )

            label = (
                f"TARGET ID {track.track_id} {track.confidence:.2f}"
                if is_target
                else f"ID {track.track_id} {track.confidence:.2f}"
            )

            self.draw_text(
                image,
                label,
                (x1, max(22, y1 - 8)),
                color,
                0.52,
            )

        if obs.depth_roi is not None:
            rx1, ry1, rx2, ry2 = obs.depth_roi

            cv2.rectangle(
                image,
                (rx1, ry1),
                (rx2, ry2),
                (255, 255, 0),
                2,
            )

        self.draw_text(
            image,
            "YOLO11s / OC-SORT + UniDepth",
            (20, 30),
            (0, 255, 255),
            0.55,
        )

        self.draw_text(
            image,
            (
                "Tracking: ACTIVE"
                if tracking_enabled
                else "Tracking: LOCKED - wait /turn_complete=True"
            ),
            (20, 58),
            (0, 255, 0) if tracking_enabled else (0, 165, 255),
            0.50,
        )

        self.draw_text(
            image,
            f"Target ID: {current_target_id if current_target_id is not None else 'NONE'}",
            (20, 86),
            (0, 0, 255) if current_target_id is not None else (180, 180, 180),
            0.50,
        )

        if obs.distance is not None and obs.depth_valid:
            distance_text = f"Distance: {obs.distance:.3f}m"
            distance_color = (255, 255, 0)

        else:
            distance_text = "Distance: invalid"
            distance_color = (0, 0, 255)

        self.draw_text(
            image,
            distance_text,
            (20, 114),
            distance_color,
            0.50,
        )

        if obs.reid_similarity is not None or obs.reid_score is not None:
            self.draw_text(
                image,
                (
                    f"ReID sim="
                    f"{obs.reid_similarity if obs.reid_similarity is not None else -1:.2f} "
                    f"score="
                    f"{obs.reid_score if obs.reid_score is not None else -1:.2f}"
                ),
                (20, 142),
                (255, 180, 0),
                0.48,
            )

        timing_text = f"process={obs.process_time_ms:.1f}ms"

        if obs.unidepth_time_ms is not None:
            timing_text += f" | UniDepth={obs.unidepth_time_ms:.1f}ms"

        self.draw_text(
            image,
            f"{timing_text} | {obs.status}",
            (20, image_height - 45),
            (220, 220, 220),
            0.45,
        )

        self.draw_text(
            image,
            "start: /turn_complete=True | lock: /front_start=1 | r or /retarget_web=True: retarget | q/ESC: quit",
            (20, image_height - 18),
            (220, 220, 220),
            0.45,
        )

        # 핵심:
        # 여기의 image는 아직 depth panel과 합치기 전이다.
        # 따라서 compressed 토픽에는 YOLO/OC-SORT 화면만 나간다.
        self.publish_annotated_image(image)

        if not ENABLE_GUI:
            return

        display = image

        if SHOW_DEPTH_VIEW:
            depth_panel = self.make_depth_panel(
                obs,
                image_width,
                image_height,
            )
            display = np.hstack((image, depth_panel))

        with self.display_lock:
            self.display_image = display

    def make_depth_panel(
        self,
        obs: Observation,
        image_width: int,
        image_height: int,
    ) -> np.ndarray:
        if obs.depth_view is None:
            panel = np.zeros(
                (image_height, image_width, 3),
                dtype=np.uint8,
            )

            self.draw_text(
                panel,
                "UniDepth: waiting",
                (20, 35),
                (0, 255, 255),
                0.65,
            )

            return panel

        if obs.depth_view.shape[:2] != (image_height, image_width):
            panel = cv2.resize(
                obs.depth_view,
                (image_width, image_height),
            )

        else:
            panel = obs.depth_view.copy()

        self.draw_text(
            panel,
            f"UniDepth view: {DEPTH_VIS_MIN_M:.1f}-{DEPTH_VIS_MAX_M:.1f}m",
            (20, 35),
            (255, 255, 255),
            0.55,
        )

        if obs.bbox is not None:
            x1, y1, x2, y2 = obs.bbox

            cv2.rectangle(
                panel,
                (x1, y1),
                (x2, y2),
                (255, 255, 255),
                1,
            )

        if obs.depth_roi is not None:
            rx1, ry1, rx2, ry2 = obs.depth_roi

            cv2.rectangle(
                panel,
                (rx1, ry1),
                (rx2, ry2),
                (255, 255, 255),
                2,
            )

        if obs.distance is not None and obs.depth_valid:
            self.draw_text(
                panel,
                f"ROI depth: {obs.distance:.3f}m",
                (20, 70),
                (255, 255, 0),
                0.55,
            )

        return panel

    def gui_callback(self) -> None:
        if not ENABLE_GUI:
            return

        with self.display_lock:
            frame = (
                None
                if self.display_image is None
                else self.display_image.copy()
            )

        if frame is not None:
            if DISPLAY_WIDTH > 0 and frame.shape[1] != DISPLAY_WIDTH:
                scale = DISPLAY_WIDTH / float(frame.shape[1])

                frame = cv2.resize(
                    frame,
                    (
                        DISPLAY_WIDTH,
                        max(1, round(frame.shape[0] * scale)),
                    ),
                    interpolation=cv2.INTER_AREA,
                )

            cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), ord("Q"), 27):
            self.should_shutdown = True

        elif key in (ord("r"), ord("R")):
            with self.target_lock:
                tracking_enabled = self.tracking_enabled

            if tracking_enabled:
                self.reset_target(source="keyboard")
            else:
                self.get_logger().info(
                    "Keyboard retarget ignored: waiting for "
                    "/turn_complete=True"
                )

    # =========================================================================
    # Reset / Shutdown
    # =========================================================================

    def reset_target(self, source: str = "manual") -> None:
        with self.target_lock:
            previous_target = self.target_id
            self.target_id = None
            self.target_lock_time = 0.0
            self.last_visible_target_depth = None

        self.reset_distance_filter()
        self.reset_reid_memory()

        with self.obs_lock:
            self.latest_obs = None
            self.last_observation_update_time = 0.0

        self.get_logger().info(
            f"Target reset by {source}: previous ID={previous_target}"
        )

        self.publish_tracking_topics()

    def shutdown(self) -> None:
        self.worker_stop.set()
        self.frame_event.set()

        if self.worker.is_alive():
            self.worker.join(timeout=1.0)

        if ENABLE_GUI:
            cv2.destroyAllWindows()


# =============================================================================
# Configuration validation
# =============================================================================

def validate_configuration() -> None:
    if Version(ultralytics_version) < Version(MIN_ULTRALYTICS_VERSION):
        raise RuntimeError(
            f"Ultralytics>={MIN_ULTRALYTICS_VERSION} is required. "
            f"Current={ultralytics_version}"
        )

    if UNIDEPTH_EVERY_N_FRAMES < 1:
        raise ValueError(
            "UNIDEPTH_EVERY_N_FRAMES must be >= 1"
        )

    if not 0.0 < DEPTH_ROI_AREA_RATIO <= 1.0:
        raise ValueError(
            "DEPTH_ROI_AREA_RATIO must be in (0, 1]"
        )

    if DEPTH_MIN_M <= 0.0:
        raise ValueError(
            "DEPTH_MIN_M must be > 0"
        )

    if DEPTH_MAX_M <= DEPTH_MIN_M:
        raise ValueError(
            "DEPTH_MAX_M must be greater than DEPTH_MIN_M"
        )

    if DEPTH_HISTORY_SIZE < 1:
        raise ValueError(
            "DEPTH_HISTORY_SIZE must be >= 1"
        )

    if not DEPTH_MIN_M <= INITIAL_TARGET_MAX_DISTANCE_M <= DEPTH_MAX_M:
        raise ValueError(
            "INITIAL_TARGET_MAX_DISTANCE_M must be inside the valid depth range"
        )

    if LOST_REACQUIRE_DEPTH_TOLERANCE_M <= 0.0:
        raise ValueError(
            "LOST_REACQUIRE_DEPTH_TOLERANCE_M must be > 0"
        )

    if AUTO_RETARGET_PERIOD_SEC <= 0.0:
        raise ValueError(
            "AUTO_RETARGET_PERIOD_SEC must be > 0"
        )

    if not 0 <= UNIDEPTH_RESOLUTION_LEVEL <= 9:
        raise ValueError(
            "UNIDEPTH_RESOLUTION_LEVEL must be in [0, 9]"
        )

    if not 1 <= ANNOTATED_JPEG_QUALITY <= 100:
        raise ValueError(
            "ANNOTATED_JPEG_QUALITY must be in [1, 100]"
        )


# =============================================================================
# Model loading
# =============================================================================

def load_models() -> tuple[YOLO, UniDepthV2]:
    print(f"[INFO] Loading YOLO model: {YOLO_MODEL_PATH}")

    yolo_model = YOLO(YOLO_MODEL_PATH)

    warmup_image = np.zeros(
        (YOLO_INPUT_SIZE, YOLO_INPUT_SIZE, 3),
        dtype=np.uint8,
    )

    yolo_model.predict(
        source=warmup_image,
        imgsz=YOLO_INPUT_SIZE,
        device=YOLO_DEVICE,
        classes=[PERSON_CLASS_ID],
        verbose=False,
    )

    print(f"[INFO] Loading UniDepth V2: {UNIDEPTH_MODEL_ID}")

    depth_model = UniDepthV2.from_pretrained(UNIDEPTH_MODEL_ID)
    depth_model = depth_model.to(torch.device(UNIDEPTH_DEVICE)).eval()
    depth_model.resolution_level = UNIDEPTH_RESOLUTION_LEVEL

    if torch.cuda.is_available():
        torch.backends.cudnn.benchmark = True

    return yolo_model, depth_model


# =============================================================================
# main
# =============================================================================

def main() -> None:
    try:
        validate_configuration()
        yolo_model, depth_model = load_models()

    except Exception as exc:
        print(
            f"[ERROR] Initialization failed: "
            f"{type(exc).__name__}: {exc}"
        )
        sys.exit(1)

    rclpy.init()

    node = RearPersonVisionNode(
        yolo_model=yolo_model,
        depth_model=depth_model,
    )

    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested by Ctrl+C")

    finally:
        node.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()