#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Int32
from ultralytics import YOLO


# ============================================================
# 사용자 설정
# ============================================================

MODEL_PATH = "yolo11x-pose.pt"
CAMERA_INDEX = 2
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

YOLO_CONF = 0.7
KEYPOINT_CONF = 0.40
IMGSZ = 640
TRACKER = "deepocsort.yaml"

WINDOW_NAME = "CCTV Fall + Crowd Hot Place"
SHOW_DEBUG = True

# ROS2 토픽
POPULATION_TOPIC = "/population"
HOT_PLACE_TOPIC = "/hot_place"
FALL_DETECTION_TOPIC = "/fall_detection"
FALL_DETECTION_POINT_TOPIC = "/fall_detection_point"

# hot_place / fall_detection 최종 발행 전 연속 유지 시간
EVENT_CONFIRM_SEC = 2.0

# 이벤트 이미지 토픽
HOT_PLACE_IMAGE_TOPIC = "/cctv/hot_place_image"
FALL_DETECTION_IMAGE_TOPIC = "/cctv/fall_detection_image"
RAW_IMAGE_TOPIC = "/cctv/image_raw/compressed"
RAW_IMAGE_FPS = 10.0

# hot_place 조건
MIN_PEOPLE_IN_CLUSTER = 2
CLUSTER_DIST_M = 1.0

PERSON_POINT_MODE = "bottom"
FALL_POINT_MODE = "bottom"

FONT_SCALE_SMALL = 0.45
FONT_SCALE_STATUS = 0.55
FONT_THICKNESS = 1

INCOMPLETE_BOX_COLOR = (255, 0, 0)
NORMAL_BOX_COLOR = (0, 255, 0)
FALL_BOX_COLOR = (0, 0, 255)
INCOMPLETE_AXIS_COLOR = (255, 0, 0)
NORMAL_AXIS_COLOR = (0, 255, 0)
FALL_AXIS_COLOR = (0, 0, 255)
MAP_POINT_COLOR = (0, 0, 255)
HOT_PLACE_COLOR = (255, 0, 255)


# Homography 보정용 픽셀 좌표: MAP_POINTS와 1:1 대응하므로 변경하지 않는다.
IMAGE_POINTS = np.array([
    [276, 149],
    [248, 197],
    [173, 191],
    [80, 272],
    [538, 262],
    [451, 149],
], dtype=np.float32)

# 사람 검출 허용 영역. 바운딩박스 하단 중앙점이 이 다각형 내부/경계에
# 있을 때만 population, hot_place, fall_detection 및 화면 표시에 사용한다.
DETECTION_AREA_POINTS = np.array([
    [450, 140],
    [270, 135],
    [246, 197],
    [17, 186],
    [1, 201],
    [3, 289],
    [566, 274],
    #[455, 131],
], dtype=np.float32)

MAP_POINTS = np.array([
    [-0.2117, 0.7590],
    [-2.1342, 0.93209],
    [-2.4062, 2.1197],
    [-4.4131, 2.2154],
    [-4.4505, -1.9947],
    [-0.4579, -2.3891],
], dtype=np.float32)


MIN_TORSO_TILT_ANGLE_DEG = 75.0
MIN_LEG_TILT_ANGLE_DEG = 65.0
MIN_ANGULAR_SPEED_DEG_S = 60.0
# MIN_TORSO_TILT_ANGLE_DEG = 30.0
# MIN_LEG_TILT_ANGLE_DEG = 30.0
# MIN_ANGULAR_SPEED_DEG_S = 60.0

ANGLE_EMA_ALPHA = 0.70
SPEED_MEMORY_SEC = 0.15
FALL_CONFIRM_SEC = 0.05

RECOVERY_TORSO_ANGLE_DEG = 30.0
RECOVERY_LEG_ANGLE_DEG = 30.0
RECOVERY_CONFIRM_SEC = 1.00
STATE_TIMEOUT_SEC = 1.00


LEFT_SHOULDER = 5
RIGHT_SHOULDER = 6
LEFT_HIP = 11
RIGHT_HIP = 12
LEFT_ANKLE = 15
RIGHT_ANKLE = 16

TORSO_KEYPOINTS = [
    LEFT_SHOULDER,
    RIGHT_SHOULDER,
    LEFT_HIP,
    RIGHT_HIP,
]

COCO_SKELETON = [
    (5, 7), (7, 9),
    (6, 8), (8, 10),
    (5, 6),
    (5, 11), (6, 12),
    (11, 12),
    (11, 13), (13, 15),
    (12, 14), (14, 16),
    (0, 1), (0, 2),
    (1, 3), (2, 4),
]

YOLO_POSE_PALETTE_RGB = np.array([
    [255, 128, 0],
    [255, 153, 51],
    [255, 178, 102],
    [230, 230, 0],
    [255, 153, 255],
    [153, 204, 255],
    [255, 102, 255],
    [255, 51, 255],
    [102, 178, 255],
    [51, 153, 255],
    [255, 153, 153],
    [255, 102, 102],
    [255, 51, 51],
    [153, 255, 153],
    [102, 255, 102],
    [51, 255, 51],
    [0, 255, 0],
    [0, 0, 255],
    [255, 0, 0],
    [255, 255, 255],
], dtype=np.uint8)

YOLO_POSE_PALETTE_BGR = YOLO_POSE_PALETTE_RGB[:, ::-1]

KPT_COLOR_INDEXES = [
    16, 16, 16, 16, 16,
    0, 0, 0, 0, 0, 0,
    9, 9, 9, 9, 9, 9,
]

LIMB_COLOR_INDEXES = [
    0, 0,
    0, 0,
    7,
    7, 7,
    7,
    9, 9,
    9, 9,
    16, 16,
    16, 16,
]

KEYPOINT_RADIUS = 3
SKELETON_THICKNESS = 2


@dataclass
class PersonState:
    filtered_torso_angle: Optional[float] = None
    filtered_leg_angle: Optional[float] = None
    last_torso_angle: Optional[float] = None
    last_time: Optional[float] = None
    torso_angular_speed: float = 0.0
    rapid_until: float = 0.0
    fall_candidate_since: Optional[float] = None
    fall_detected: bool = False
    recovery_since: Optional[float] = None
    last_seen: float = 0.0


def midpoint(point1: np.ndarray, point2: np.ndarray) -> np.ndarray:
    return (point1 + point2) / 2.0


def calculate_vertical_tilt_deg(
    upper_point: np.ndarray,
    lower_point: np.ndarray,
) -> float:
    dx = float(lower_point[0] - upper_point[0])
    dy = float(lower_point[1] - upper_point[1])

    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return 0.0

    return math.degrees(math.atan2(abs(dx), abs(dy)))


def get_leg_axis_points(
    person_xy: np.ndarray,
    person_conf: np.ndarray,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:

    left_leg_valid = (
        person_conf[LEFT_HIP] >= KEYPOINT_CONF
        and person_conf[LEFT_ANKLE] >= KEYPOINT_CONF
    )
    right_leg_valid = (
        person_conf[RIGHT_HIP] >= KEYPOINT_CONF
        and person_conf[RIGHT_ANKLE] >= KEYPOINT_CONF
    )

    if left_leg_valid and right_leg_valid:
        hip_center = midpoint(person_xy[LEFT_HIP], person_xy[RIGHT_HIP])
        ankle_center = midpoint(person_xy[LEFT_ANKLE], person_xy[RIGHT_ANKLE])
        return hip_center, ankle_center

    if left_leg_valid:
        return person_xy[LEFT_HIP], person_xy[LEFT_ANKLE]

    if right_leg_valid:
        return person_xy[RIGHT_HIP], person_xy[RIGHT_ANKLE]

    return None


def update_person_state(
    state: PersonState,
    raw_torso_angle: float,
    raw_leg_angle: float,
    now: float,
) -> None:

    if state.filtered_torso_angle is None:
        filtered_torso_angle = raw_torso_angle
        filtered_leg_angle = raw_leg_angle
        angular_speed = 0.0
    else:
        filtered_torso_angle = (
            ANGLE_EMA_ALPHA * raw_torso_angle
            + (1.0 - ANGLE_EMA_ALPHA) * state.filtered_torso_angle
        )

        if state.filtered_leg_angle is None:
            filtered_leg_angle = raw_leg_angle
        else:
            filtered_leg_angle = (
                ANGLE_EMA_ALPHA * raw_leg_angle
                + (1.0 - ANGLE_EMA_ALPHA) * state.filtered_leg_angle
            )

        dt = now - state.last_time if state.last_time is not None else 0.0

        if dt > 1e-4 and state.last_torso_angle is not None:
            angular_speed = abs(filtered_torso_angle - state.last_torso_angle) / dt
        else:
            angular_speed = 0.0

    state.filtered_torso_angle = filtered_torso_angle
    state.filtered_leg_angle = filtered_leg_angle
    state.torso_angular_speed = angular_speed
    state.last_torso_angle = filtered_torso_angle
    state.last_time = now
    state.last_seen = now

    if angular_speed >= MIN_ANGULAR_SPEED_DEG_S:
        state.rapid_until = now + SPEED_MEMORY_SEC

    torso_angle_condition = filtered_torso_angle >= MIN_TORSO_TILT_ANGLE_DEG
    leg_angle_condition = filtered_leg_angle >= MIN_LEG_TILT_ANGLE_DEG
    speed_condition = now <= state.rapid_until

    fall_condition = torso_angle_condition and leg_angle_condition and speed_condition

    if fall_condition:
        if state.fall_candidate_since is None:
            state.fall_candidate_since = now

        if now - state.fall_candidate_since >= FALL_CONFIRM_SEC:
            state.fall_detected = True
            state.recovery_since = None
    else:
        state.fall_candidate_since = None

    if state.fall_detected:
        recovered_posture = (
            filtered_torso_angle <= RECOVERY_TORSO_ANGLE_DEG
            and filtered_leg_angle <= RECOVERY_LEG_ANGLE_DEG
        )

        if recovered_posture:
            if state.recovery_since is None:
                state.recovery_since = now
            elif now - state.recovery_since >= RECOVERY_CONFIRM_SEC:
                state.fall_detected = False
                state.recovery_since = None
                state.rapid_until = 0.0
                state.fall_candidate_since = None
        else:
            state.recovery_since = None


def bbox_point(box: np.ndarray, mode: str = "bottom") -> Tuple[int, int]:
    x1, y1, x2, y2 = box[:4]
    u = int((x1 + x2) / 2.0)

    if mode == "center":
        v = int((y1 + y2) / 2.0)
    else:
        v = int(y2)

    return u, v


def is_point_inside_detection_area(u: int, v: int) -> bool:
    """바운딩박스 하단 중앙점이 검출 영역 내부/경계에 있는지 확인한다."""
    polygon = DETECTION_AREA_POINTS.reshape((-1, 1, 2))
    return cv2.pointPolygonTest(polygon, (float(u), float(v)), False) >= 0


def palette_color(index: int) -> Tuple[int, int, int]:
    color = YOLO_POSE_PALETTE_BGR[int(index) % len(YOLO_POSE_PALETTE_BGR)]
    return int(color[0]), int(color[1]), int(color[2])


def draw_pose_skeleton(
    frame: np.ndarray,
    person_xy: np.ndarray,
    person_conf: np.ndarray,
) -> None:

    if person_xy is None or person_conf is None:
        return

    keypoint_count = min(len(person_xy), len(person_conf))

    for limb_index, (start_idx, end_idx) in enumerate(COCO_SKELETON):
        if start_idx >= keypoint_count or end_idx >= keypoint_count:
            continue

        if (
            person_conf[start_idx] < KEYPOINT_CONF
            or person_conf[end_idx] < KEYPOINT_CONF
        ):
            continue

        start_point = tuple(person_xy[start_idx].astype(int))
        end_point = tuple(person_xy[end_idx].astype(int))
        limb_color = palette_color(LIMB_COLOR_INDEXES[limb_index])

        cv2.line(
            frame,
            start_point,
            end_point,
            limb_color,
            SKELETON_THICKNESS,
            cv2.LINE_AA,
        )

    for keypoint_index in range(keypoint_count):
        if person_conf[keypoint_index] < KEYPOINT_CONF:
            continue

        point = tuple(person_xy[keypoint_index].astype(int))
        keypoint_color = palette_color(KPT_COLOR_INDEXES[keypoint_index])

        cv2.circle(
            frame,
            point,
            KEYPOINT_RADIUS,
            keypoint_color,
            -1,
            cv2.LINE_AA,
        )


class CCTVFallCrowdNode(Node):
    def __init__(self) -> None:
        super().__init__("cctv_fall_crowd_node")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.population_pub = self.create_publisher(Int32, POPULATION_TOPIC, qos)
        self.hot_place_pub = self.create_publisher(PoseArray, HOT_PLACE_TOPIC, qos)
        self.fall_pub = self.create_publisher(Bool, FALL_DETECTION_TOPIC, qos)
        self.fall_point_pub = self.create_publisher(
            PointStamped,
            FALL_DETECTION_POINT_TOPIC,
            qos,
        )

        self.hot_place_image_pub = self.create_publisher(
            Image,
            HOT_PLACE_IMAGE_TOPIC,
            qos,
        )
        self.fall_detection_image_pub = self.create_publisher(
            Image,
            FALL_DETECTION_IMAGE_TOPIC,
            qos,
        )
        self.raw_image_pub = self.create_publisher(
            CompressedImage,
            RAW_IMAGE_TOPIC,
            qos,
        )
        self.raw_image_publish_period = 1.0 / RAW_IMAGE_FPS
        self.last_raw_image_publish_time = 0.0

        self.hot_place_event_active = False
        self.fall_event_active = False

        # 원시 검출이 연속 EVENT_CONFIRM_SEC 동안 유지됐는지 확인하는 타이머
        self.hot_place_candidate_since: Optional[float] = None
        self.fall_candidate_since: Optional[float] = None

        self.H, _ = cv2.findHomography(IMAGE_POINTS, MAP_POINTS)
        self.H_inv, _ = cv2.findHomography(MAP_POINTS, IMAGE_POINTS)

        if self.H is None or self.H_inv is None:
            raise RuntimeError("Homography 계산 실패. IMAGE_POINTS와 MAP_POINTS를 확인하세요.")

        self.model = YOLO(MODEL_PATH)
        self.person_states: Dict[int, PersonState] = {}

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if not self.cap.isOpened():
            raise RuntimeError(f"웹캠 또는 영상을 열 수 없습니다. CAMERA_INDEX={CAMERA_INDEX}")

        self.timer = self.create_timer(0.03, self.timer_callback)

        self.get_logger().info("CCTV fall + crowd node started.")
        self.get_logger().info(f"Publish: {POPULATION_TOPIC}")
        self.get_logger().info(f"Publish: {HOT_PLACE_TOPIC} (geometry_msgs/msg/PoseArray)")
        self.get_logger().info(f"Publish: {FALL_DETECTION_TOPIC}")
        self.get_logger().info(f"Publish: {FALL_DETECTION_POINT_TOPIC}")
        self.get_logger().info(f"Publish: {HOT_PLACE_IMAGE_TOPIC} (event image)")
        self.get_logger().info(f"Publish: {FALL_DETECTION_IMAGE_TOPIC} (event image)")
        self.get_logger().info(f"Publish: {RAW_IMAGE_TOPIC} ({RAW_IMAGE_FPS:.1f} FPS JPEG compressed webcam image)")

    def publish_event_image(self, publisher, frame: np.ndarray) -> None:
        if frame is None or frame.size == 0:
            self.get_logger().warn("이벤트 이미지 발행 실패: 빈 프레임")
            return

        if not frame.flags["C_CONTIGUOUS"]:
            frame = np.ascontiguousarray(frame)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "cctv_camera"
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(frame.strides[0])
        msg.data = frame.tobytes()

        publisher.publish(msg)


    def publish_raw_image_10fps(self, frame: np.ndarray, now: float) -> None:
        """바운딩박스 등이 없는 원본 웹캠 영상을 JPEG 압축 형태로 10 FPS 발행한다."""
        if now - self.last_raw_image_publish_time < self.raw_image_publish_period:
            return

        success, encoded = cv2.imencode(".jpg", frame)
        if not success:
            self.get_logger().warn("원본 영상 JPEG 압축 실패")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "cctv_camera"
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.raw_image_pub.publish(msg)

        self.last_raw_image_publish_time = now

    def pixel_to_map(self, u: int, v: int) -> Tuple[float, float]:
        pixel = np.array([[[float(u), float(v)]]], dtype=np.float32)
        map_point = cv2.perspectiveTransform(pixel, self.H)

        x = float(map_point[0][0][0])
        y = float(map_point[0][0][1])
        return x, y

    def map_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        map_point = np.array([[[float(x), float(y)]]], dtype=np.float32)
        pixel = cv2.perspectiveTransform(map_point, self.H_inv)

        u = int(pixel[0][0][0])
        v = int(pixel[0][0][1])
        return u, v

    @staticmethod
    def distance(p1: List[float], p2: List[float]) -> float:
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def get_clusters(self, points: List[List[float]]) -> List[List[List[float]]]:
        n = len(points)
        visited = [False] * n
        clusters = []

        for i in range(n):
            if visited[i]:
                continue

            queue = [i]
            visited[i] = True
            cluster = []

            while queue:
                current = queue.pop(0)
                cluster.append(points[current])

                for j in range(n):
                    if visited[j]:
                        continue

                    if self.distance(points[current], points[j]) <= CLUSTER_DIST_M:
                        visited[j] = True
                        queue.append(j)

            clusters.append(cluster)

        return clusters

    def get_hot_places(self, person_map_points: List[List[float]]) -> List[Tuple[float, float, int]]:
        if len(person_map_points) < MIN_PEOPLE_IN_CLUSTER:
            return []

        clusters = self.get_clusters(person_map_points)
        valid_clusters = [
            cluster
            for cluster in clusters
            if len(cluster) >= MIN_PEOPLE_IN_CLUSTER
        ]

        hot_places: List[Tuple[float, float, int]] = []

        # 유효한 모든 군집 중심을 계산한다. 큰 군집부터 발행한다.
        for cluster in sorted(valid_clusters, key=len, reverse=True):
            hot_x = sum(point[0] for point in cluster) / len(cluster)
            hot_y = sum(point[1] for point in cluster) / len(cluster)
            hot_places.append((hot_x, hot_y, len(cluster)))

        return hot_places

    def publish_population(self, count: int) -> None:
        msg = Int32()
        msg.data = int(count)
        self.population_pub.publish(msg)

    def publish_fall_detection(self, detected: bool) -> None:
        msg = Bool()
        msg.data = bool(detected)
        self.fall_pub.publish(msg)

    def publish_point(self, publisher, topic_x: float, topic_y: float) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = float(topic_x)
        msg.point.y = float(topic_y)
        msg.point.z = 0.0
        publisher.publish(msg)

    def publish_hot_places(
        self,
        hot_results: List[Tuple[float, float, int]],
    ) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        for hot_x, hot_y, _ in hot_results:
            pose = Pose()
            pose.position.x = float(hot_x)
            pose.position.y = float(hot_y)
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        # hot_results가 비어 있으면 빈 PoseArray를 발행한다.
        # 수신 노드는 매번 현재 배열로 교체하면 이전 좌표가 남지 않는다.
        self.hot_place_pub.publish(msg)

    def publish_fall_detection_point(self, x: float, y: float) -> None:
        self.publish_point(self.fall_point_pub, x, y)

    def update_event_image_publication(
        self,
        hot_detected: bool,
        fall_detected: bool,
        annotated_frame: np.ndarray,
    ) -> None:
        if hot_detected and not self.hot_place_event_active:
            self.publish_event_image(self.hot_place_image_pub, annotated_frame)
            self.hot_place_event_active = True
            self.get_logger().info("hot_place 발생: /cctv/hot_place_image 1회 발행")
        elif not hot_detected:
            self.hot_place_event_active = False

        if fall_detected and not self.fall_event_active:
            self.publish_event_image(self.fall_detection_image_pub, annotated_frame)
            self.fall_event_active = True
            self.get_logger().info("fall_detection 발생: /cctv/fall_detection_image 1회 발행")
        elif not fall_detected:
            self.fall_event_active = False

    def update_confirmed_detection(
        self,
        detected_now: bool,
        now: float,
        candidate_attr: str,
    ) -> bool:
        """
        detected_now가 EVENT_CONFIRM_SEC 동안 끊기지 않고 유지된 경우에만 True를 반환한다.

        검출이 한 프레임이라도 False가 되면 후보 시작 시간을 초기화하므로,
        이후 다시 검출되면 2초를 처음부터 다시 측정한다.
        """
        candidate_since = getattr(self, candidate_attr)

        if not detected_now:
            setattr(self, candidate_attr, None)
            return False

        if candidate_since is None:
            setattr(self, candidate_attr, now)
            return False

        return (now - candidate_since) >= EVENT_CONFIRM_SEC

    def remove_stale_states(self, now: float) -> None:
        stale_track_ids = [
            track_id
            for track_id, state in self.person_states.items()
            if now - state.last_seen > STATE_TIMEOUT_SEC
        ]

        for track_id in stale_track_ids:
            del self.person_states[track_id]

    def draw_person_label(
        self,
        frame: np.ndarray,
        box: np.ndarray,
        text: str,
        color: Tuple[int, int, int],
    ) -> None:
        x1, y1, x2, y2 = box.astype(int)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            frame,
            text,
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            FONT_SCALE_STATUS,
            color,
            FONT_THICKNESS,
            cv2.LINE_AA,
        )

    def draw_debug_header(
        self,
        frame: np.ndarray,
        population: int,
        fall_detected: bool,
        hot_results: List[Tuple[float, float, int]],
        fall_point: Optional[Tuple[float, float]],
    ) -> None:
        fall_text = "True" if fall_detected else "False"
        cv2.putText(
            frame,
            f"population={population}  fall_detection={fall_text}",
            (15, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            FONT_SCALE_STATUS,
            FALL_BOX_COLOR if fall_detected else (255, 255, 255),
            FONT_THICKNESS,
            cv2.LINE_AA,
        )

        if hot_results:
            hot_summary = " | ".join(
                f"H{index + 1}=({hot_x:.2f},{hot_y:.2f}) n={hot_count}"
                for index, (hot_x, hot_y, hot_count) in enumerate(hot_results)
            )
            text = f"hot_places={len(hot_results)}  {hot_summary}"
        else:
            text = "hot_places=0"

        cv2.putText(
            frame,
            text,
            (15, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            FONT_SCALE_SMALL,
            (255, 255, 255),
            FONT_THICKNESS,
            cv2.LINE_AA,
        )

        if fall_point is not None:
            fx, fy = fall_point
            cv2.putText(
                frame,
                f"fall_detection_point=({fx:.2f}, {fy:.2f})",
                (15, 69),
                cv2.FONT_HERSHEY_SIMPLEX,
                FONT_SCALE_SMALL,
                FALL_BOX_COLOR,
                FONT_THICKNESS,
                cv2.LINE_AA,
            )

    def timer_callback(self) -> None:
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("웹캠 또는 영상 프레임을 읽지 못했습니다.")
            return

        now = time.monotonic()

        # 원본 웹캠 영상은 추론/바운딩박스 표시 전에 10 FPS로 발행한다.
        self.publish_raw_image_10fps(frame, now)

        debug = frame.copy()

        results = self.model.track(
            source=frame,
            persist=True,
            tracker=TRACKER,
            classes=[0],
            conf=YOLO_CONF,
            imgsz=IMGSZ,
            verbose=False,
        )
        result = results[0]

        boxes = np.empty((0, 4), dtype=np.float32)
        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes.xyxy.cpu().numpy()

        # 사람 검출 허용 영역을 화면에 표시한다.
        if SHOW_DEBUG:
            cv2.polylines(
                debug,
                [DETECTION_AREA_POINTS.astype(np.int32).reshape((-1, 1, 2))],
                True,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # 바운딩박스 하단 중앙점이 감지 영역 안에 있는 검출만 사용한다.
        inside_detection_indices: List[int] = []
        person_map_points: List[List[float]] = []

        for index, box in enumerate(boxes):
            u, v = bbox_point(box, PERSON_POINT_MODE)

            if not is_point_inside_detection_area(u, v):
                continue

            inside_detection_indices.append(index)
            map_x, map_y = self.pixel_to_map(u, v)
            person_map_points.append([map_x, map_y])

            if SHOW_DEBUG:
                cv2.circle(debug, (u, v), 4, MAP_POINT_COLOR, -1)
                cv2.putText(
                    debug,
                    f"({map_x:.2f},{map_y:.2f})",
                    (u + 4, max(15, v - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    FONT_SCALE_SMALL,
                    MAP_POINT_COLOR,
                    FONT_THICKNESS,
                    cv2.LINE_AA,
                )

        population = len(inside_detection_indices)
        hot_results = self.get_hot_places(person_map_points)

        keypoints_xy = None
        keypoints_conf = None
        track_ids: List[Optional[int]] = []

        has_pose_result = (
            result.boxes is not None
            and result.keypoints is not None
            and len(boxes) > 0
        )

        if has_pose_result:
            keypoints_xy = result.keypoints.xy.cpu().numpy()

            if result.keypoints.conf is not None:
                keypoints_conf = result.keypoints.conf.cpu().numpy()
            else:
                keypoints_conf = np.ones(keypoints_xy.shape[:2], dtype=np.float32)

            if result.boxes.id is not None:
                track_ids = result.boxes.id.int().cpu().tolist()
            else:
                track_ids = [None] * len(boxes)

        current_fall_candidates: List[Tuple[float, float, float, int, int, np.ndarray]] = []

        if has_pose_result and keypoints_xy is not None and keypoints_conf is not None:
            detection_count = min(len(boxes), len(track_ids), len(keypoints_xy), len(keypoints_conf))

            for index in inside_detection_indices:
                if index >= detection_count:
                    continue

                track_id = track_ids[index]
                box = boxes[index]

                if track_id is None:
                    if SHOW_DEBUG:
                        draw_pose_skeleton(
                            debug,
                            keypoints_xy[index],
                            keypoints_conf[index],
                        )
                        self.draw_person_label(debug, box, "ID:None", INCOMPLETE_BOX_COLOR)
                    continue

                track_id = int(track_id)

                existing_state = self.person_states.get(track_id)
                if existing_state is not None:
                    existing_state.last_seen = now

                person_xy = keypoints_xy[index]
                person_conf = keypoints_conf[index]

                if SHOW_DEBUG:
                    draw_pose_skeleton(debug, person_xy, person_conf)

                torso_valid = all(
                    person_conf[keypoint_index] >= KEYPOINT_CONF
                    for keypoint_index in TORSO_KEYPOINTS
                )

                shoulder_center = None
                hip_center = None

                if torso_valid:
                    shoulder_center = midpoint(
                        person_xy[LEFT_SHOULDER],
                        person_xy[RIGHT_SHOULDER],
                    )
                    hip_center = midpoint(
                        person_xy[LEFT_HIP],
                        person_xy[RIGHT_HIP],
                    )

                leg_axis = get_leg_axis_points(person_xy, person_conf)
                has_torso_axis = shoulder_center is not None and hip_center is not None
                has_leg_axis = leg_axis is not None
                has_two_fall_axes = has_torso_axis and has_leg_axis

                if not has_two_fall_axes:
                    if SHOW_DEBUG:
                        if has_torso_axis:
                            cv2.line(
                                debug,
                                tuple(shoulder_center.astype(int)),
                                tuple(hip_center.astype(int)),
                                INCOMPLETE_AXIS_COLOR,
                                2,
                                cv2.LINE_AA,
                            )

                        if has_leg_axis:
                            leg_upper_point, leg_lower_point = leg_axis
                            cv2.line(
                                debug,
                                tuple(leg_upper_point.astype(int)),
                                tuple(leg_lower_point.astype(int)),
                                INCOMPLETE_AXIS_COLOR,
                                2,
                                cv2.LINE_AA,
                            )

                        self.draw_person_label(
                            debug,
                            box,
                            f"ID:{track_id} CHECK",
                            INCOMPLETE_BOX_COLOR,
                        )
                    continue

                leg_upper_point, leg_lower_point = leg_axis

                raw_torso_angle = calculate_vertical_tilt_deg(shoulder_center, hip_center)
                raw_leg_angle = calculate_vertical_tilt_deg(leg_upper_point, leg_lower_point)

                state = self.person_states.setdefault(track_id, PersonState(last_seen=now))
                update_person_state(state, raw_torso_angle, raw_leg_angle, now)

                is_fall = state.fall_detected

                if is_fall:
                    bbox_color = FALL_BOX_COLOR
                    axis_color = FALL_AXIS_COLOR
                    status = "FALL"
                else:
                    bbox_color = NORMAL_BOX_COLOR
                    axis_color = NORMAL_AXIS_COLOR
                    status = "NORMAL"

                if SHOW_DEBUG:
                    cv2.line(
                        debug,
                        tuple(shoulder_center.astype(int)),
                        tuple(hip_center.astype(int)),
                        axis_color,
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.line(
                        debug,
                        tuple(leg_upper_point.astype(int)),
                        tuple(leg_lower_point.astype(int)),
                        axis_color,
                        2,
                        cv2.LINE_AA,
                    )

                    label = (
                        f"ID:{track_id} {status} "
                        f"T:{state.filtered_torso_angle:.0f} "
                        f"L:{state.filtered_leg_angle:.0f}"
                    )
                    self.draw_person_label(debug, box, label, bbox_color)

                if is_fall:
                    fall_u, fall_v = bbox_point(box, FALL_POINT_MODE)
                    fall_map_x, fall_map_y = self.pixel_to_map(fall_u, fall_v)
                    x1, y1, x2, y2 = box[:4]
                    area = float((x2 - x1) * (y2 - y1))
                    current_fall_candidates.append(
                        (area, fall_map_x, fall_map_y, fall_u, fall_v, box)
                    )

                    if SHOW_DEBUG:
                        cv2.circle(debug, (fall_u, fall_v), 8, MAP_POINT_COLOR, -1)
                        cv2.putText(
                            debug,
                            "fall point",
                            (fall_u + 7, fall_v + 16),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            FONT_SCALE_SMALL,
                            (0, 0, 255),
                            FONT_THICKNESS,
                            cv2.LINE_AA,
                        )

        # 현재 프레임에서 감지 영역 밖에 있는 트랙은 상태를 즉시 삭제한다.
        # 그래야 영역 밖 사람의 이전 FALL 상태가 /fall_detection에 남지 않는다.
        inside_track_ids = {
            int(track_ids[index])
            for index in inside_detection_indices
            if index < len(track_ids) and track_ids[index] is not None
        }
        outside_or_missing_track_ids = [
            track_id
            for track_id in self.person_states
            if track_id not in inside_track_ids
        ]
        for track_id in outside_or_missing_track_ids:
            del self.person_states[track_id]

        self.remove_stale_states(now)

        any_fall_detected = any(state.fall_detected for state in self.person_states.values())

        selected_fall_point: Optional[Tuple[float, float]] = None

        if current_fall_candidates:
            _, fall_x, fall_y, _, _, _ = max(
                current_fall_candidates,
                key=lambda item: item[0],
            )
            selected_fall_point = (fall_x, fall_y)

        # ------------------------------------------------------------
        # 최종 ROS 발행 조건
        # raw hot/fall 검출이 연속 2초 이상 유지된 뒤에만 True/좌표를 발행한다.
        # 도중에 검출이 끊기면 후보 타이머는 즉시 초기화된다.
        # ------------------------------------------------------------
        raw_hot_detected = bool(hot_results)
        raw_fall_detected = any_fall_detected and selected_fall_point is not None

        confirmed_hot_detected = self.update_confirmed_detection(
            detected_now=raw_hot_detected,
            now=now,
            candidate_attr="hot_place_candidate_since",
        )
        confirmed_fall_detected = self.update_confirmed_detection(
            detected_now=raw_fall_detected,
            now=now,
            candidate_attr="fall_candidate_since",
        )

        confirmed_hot_results = hot_results if confirmed_hot_detected else []
        confirmed_fall_point = (
            selected_fall_point if confirmed_fall_detected else None
        )

        self.publish_population(population)
        self.publish_fall_detection(confirmed_fall_detected)

        # 2초 확인 전에는 poses=[]인 빈 PoseArray를 발행한다.
        # 2초 확인 후에는 현재 프레임에서 계산된 hot place 좌표를 발행한다.
        self.publish_hot_places(confirmed_hot_results)

        # fall_detection_point 역시 fall 검출이 2초 유지된 뒤에만 발행한다.
        if confirmed_fall_point is not None:
            fall_x, fall_y = confirmed_fall_point
            self.publish_fall_detection_point(fall_x, fall_y)

        for hot_index, (hot_x, hot_y, hot_count) in enumerate(confirmed_hot_results):
            hot_u, hot_v = self.map_to_pixel(hot_x, hot_y)

            cv2.circle(debug, (hot_u, hot_v), 10, HOT_PLACE_COLOR, -1)
            cv2.putText(
                debug,
                f"HOT {hot_index + 1} ({hot_x:.2f},{hot_y:.2f}) n={hot_count}",
                (hot_u + 10, max(15, hot_v - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                FONT_SCALE_SMALL,
                HOT_PLACE_COLOR,
                FONT_THICKNESS,
                cv2.LINE_AA,
            )

        self.draw_debug_header(
            debug,
            population,
            confirmed_fall_detected,
            confirmed_hot_results,
            confirmed_fall_point,
        )

        # 이벤트 이미지도 2초 유지가 확정된 시점에 1회만 발행한다.
        self.update_event_image_publication(
            hot_detected=confirmed_hot_detected,
            fall_detected=confirmed_fall_detected,
            annotated_frame=debug,
        )

        if SHOW_DEBUG:
            cv2.imshow(WINDOW_NAME, debug)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q") or key == 27:
                rclpy.shutdown()

    def destroy_node(self) -> None:
        if self.cap is not None:
            self.cap.release()

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[CCTVFallCrowdNode] = None

    try:
        node = CCTVFallCrowdNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_population(0)
            node.publish_fall_detection(False)
            node.publish_hot_places([])
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()