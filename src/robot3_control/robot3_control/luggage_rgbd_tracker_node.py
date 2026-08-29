from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Int32
from ultralytics import YOLO


# =============================================================================
# 코드 리뷰 핵심 설계 요약
# =============================================================================
# 이 파일은 TurtleBot4 전방 RGB-D 카메라에서 사람을 추적하기 위한 ROS2 노드이다.
#
# 설계를 가장 잘 보여주는 핵심 흐름은 다음 4단계이다.
#   1) RGB 압축 영상과 Depth 압축 영상을 ApproximateTimeSynchronizer로 시간 동기화한다.
#   2) YOLO + DeepOCSORT track()으로 사람 bbox와 track_id를 얻는다.
#   3) /carrying=True를 받기 전에는 YOLO tracking을 수행하지 않는다.
#   4) 잠금 해제 후 1m 이내 사람만 대상으로 최초 타깃을 선택하며 tracking=1까지 반복한다.
#   5) 주행 중에는 기존 ID를 우선 유지하고, LOST 시 마지막 유효 Depth ±0.5m로 새 ID를 복구한다.
#   6) /service_end=True를 받으면 타깃/Depth 기록을 지우고 다시 TRACKING LOCKED 상태로 돌아간다.
#
# 코드 리뷰에서는 process_pair()를 가장 집중적으로 설명하면 좋다.
# 이유:
#   - RGB/Depth 디코딩, YOLO 추적, 타깃 상태 분기, depth 계산, 토픽 발행이 모두 모이는 함수이다.
#   - “이 함수는 한 쌍의 RGB-D 프레임에서 타깃의 가시 상태와 거리 정보를 산출해야 하기 때문에
#      입력 검증 → 추론 → 타깃 선택/유지 → depth 산출 → 상태 발행 순서로 설계했다”고 설명할 수 있다.
#
# 모델 선택 근거:
#   - YOLO11s는 YOLO11 계열 중 n보다 표현력이 높고, m/l/x보다 연산량이 낮은 중간형 모델이다.
#   - 실시간 TurtleBot4 추적에서는 정확도만 높은 대형 모델보다 지연 시간이 작고 ID 유지가 가능한 모델이 유리하다.
#   - DeepOCSORT는 단순 프레임 단위 검출보다 track_id를 유지할 수 있고, bbox 움직임/appearance 기반 추적에
#     더 적합하므로 “같은 사람을 계속 따라간다”는 요구에 맞다.
#   - 단, 코드만으로는 YOLO11n/s/m 또는 ByteTrack/BoT-SORT와 정량 비교 실험을 했다고 단정할 수 없다.
#     발표에서는 FPS, ID switch, occlusion recovery, depth 안정성을 기준으로 비교했다고 설명할 근거가 필요하다.
#
# 주요 파라미터 검토 기준:
#   - yolo_conf=0.40: 너무 낮으면 오검출 증가, 너무 높으면 사람 누락 증가. 실시간 추적에서는 누락보다 오검출이
#     track_id 혼선을 만들 수 있어 중간값으로 둔다.
#   - yolo_iou=0.50: NMS 중복 제거의 일반적인 균형값이다. 사람이 겹치는 환경에서는 너무 낮으면 박스가 과하게
#     제거될 수 있고, 너무 높으면 중복 박스가 남을 수 있다.
#   - sync_slop_sec=0.08: RGB와 Depth 타임스탬프 차이를 약 80ms까지 허용한다. 너무 작으면 동기화 실패,
#     너무 크면 RGB bbox와 Depth가 다른 시점의 사람이 되어 거리 오차가 커진다.
#   - depth_lowest_percent=5.0: bbox 전체 평균은 배경/바닥 픽셀에 오염되기 쉽기 때문에 가까운 일부 픽셀만 사용한다.
#     다만 너무 작으면 노이즈에 민감하고, 너무 크면 배경이 섞인다.
#   - depth_history_size=3: depth 순간 튐을 median으로 줄이되, 추적 반응성이 지나치게 느려지지 않도록 짧게 둔다.
#
# 조건/케이스 검증 포인트:
#   - 사람 없음: /tracking_rgbd=0, center/depth=-1.0 발행
#   - 타깃 보임: /tracking_rgbd=1, center_x와 depth 발행
#   - 타깃 사라짐: /tracking_rgbd=2, center/depth=-1.0 발행
#   - RGB 디코딩 실패: 상태 0 발행
#   - Depth 디코딩 실패: 타깃 중심은 발행 가능하지만 depth는 -1.0 유지
#   - 오래된 프레임: 처리하지 않고 폐기
#   위 조건들은 코드상 분기되어 있으나, 실제 테스트 완료 여부는 로그/실험표가 있어야 확정할 수 있다.
# =============================================================================


@dataclass(frozen=True)
class Track:
    """
    YOLO tracker에서 얻은 사람 1명의 추적 결과를 담는 불변 데이터 클래스.

    설계 의도:
        이 클래스는 track_id, bbox, confidence를 하나의 단위로 묶기 위해 사용한다.
        YOLO 결과를 그대로 여러 곳에서 다루면 boxes.xyxy, boxes.id, boxes.conf 접근이 반복되고
        코드가 복잡해지기 때문에, 후속 로직에서는 Track 객체만 사용하도록 단순화했다.

    frozen=True를 사용한 이유:
        추적 결과는 한 프레임에서 계산된 관측값이므로 중간 로직에서 임의로 바뀌면 안 된다.
        따라서 불변 객체로 만들어 디버깅 시 상태 변화를 줄이고, “한 프레임의 검출 결과”라는 의미를 명확히 했다.
    """

    # DeepOCSORT가 부여한 객체 ID이다.
    # 이 ID를 기준으로 “처음 선택한 사람”과 “현재 보이는 사람”이 같은지 판단한다.
    track_id: int

    # bbox 좌표는 [x1, y1, x2, y2] 형식이다.
    # RGB 이미지 기준 좌표이며, depth 이미지에 적용할 때는 scale_bbox()로 해상도 변환이 필요하다.
    box: np.ndarray

    # YOLO가 해당 bbox를 사람이라고 판단한 신뢰도이다.
    # 화면 표시와 디버깅에 사용하며, 현재 로직에서는 타깃 선택 점수로 직접 사용하지 않는다.
    confidence: float

    # pose 모델이 출력한 keypoint 좌표이다.
    # COCO pose 기준 양발 ankle keypoint는 left_ankle=15, right_ankle=16이다.
    keypoints_xy: Optional[np.ndarray] = None

    # 각 keypoint의 신뢰도이다.
    # 발목 keypoint가 낮은 신뢰도로 추정되면 해당 발의 depth는 사용하지 않는다.
    keypoints_conf: Optional[np.ndarray] = None


class RgbdPersonTrackingNode(Node):
    """
    RGB-D 기반 사람 추적 ROS2 노드.

    클래스 설계 의도:
        이 클래스는 ROS2 통신, RGB-D 동기화, YOLO 추적, depth 계산, GUI 표시를 하나의 노드로 묶는다.
        전방 카메라에서 사람을 추적해 다른 노드가 사용할 수 있도록 다음 3개 토픽을 발행한다.

        - /tracking_rgbd
            0: 아직 타깃이 없거나 추적하지 않음
            1: 타깃이 현재 RGB 영상에서 보임
            2: 타깃 ID가 사라졌지만 추적 상태는 유지 중

        - /tracking_rgbd_center_pixel
            타깃 bbox의 x축 중심 픽셀 좌표.
            후속 제어 노드는 이 값을 영상 중심과 비교해 좌/우 회전 제어에 사용할 수 있다.

        - /target_depth_rgbd
            타깃 bbox 내부의 추정 거리값.
            유효하지 않을 때는 -1.0을 발행해 후속 노드가 invalid 상태를 쉽게 구분하게 했다.

    설계상 중요한 선택:
        1) ROS 콜백에서 YOLO 추론을 직접 하지 않고 별도 worker thread에서 처리한다.
           YOLO 추론은 상대적으로 오래 걸리므로 콜백에서 직접 수행하면 메시지 수신이 밀릴 수 있다.

        2) latest_pair 하나만 유지한다.
           처리 속도보다 카메라 입력이 빠른 경우 모든 프레임을 처리하면 지연이 누적된다.
           이 코드는 오래된 프레임보다 최신 프레임 반응성이 중요하므로 가장 최근 RGB-D 쌍만 처리한다.

        3) /carrying=True를 받기 전에는 YOLO tracking을 수행하지 않는다.
           잠금 상태에서는 상태 0과 invalid 값을 발행하고, 영상 수신과 GUI 표시만 유지한다.

        4) 잠금 해제 후 최초 타깃은 현재 보이는 모든 사람의 발목 주변 depth를 비교해 선택한다.
           가장 작은 유효 depth를 가진 track_id를 한 번 고정하고, 이후에는 기존 ID 유지 로직을 그대로 사용한다.

        5) /service_end=True를 받으면 추적을 다시 잠그고 초기 상태로 복귀한다.
           추론 중 종료 메시지가 들어와도 세션 번호를 검사해 이전 결과가 재발행되지 않게 한다.
    """

    def __init__(
        self,
        yolo_model: YOLO,
        rgb_topic: str = "/robot3/oakd/rgb/image_raw/compressed",
        depth_topic: str = "/robot3/oakd/stereo/image_raw/compressedDepth",
        tracking_topic: str = "/tracking_rgbd",
        center_topic: str = "/tracking_rgbd_center_pixel",
        target_depth_topic: str = "/target_depth_rgbd",
        unlock_topic: str = "/carrying",
        service_end_topic: str = "/service_end",
        tracking_none: int = 0,
        tracking_visible: int = 1,
        tracking_lost: int = 2,
        invalid_float: float = 100.0,
        tracker_config: str = "deepocsort.yaml",
        yolo_device: int | str = 0,
        yolo_imgsz: int = 640,
        yolo_conf: float = 0.40,
        yolo_iou: float = 0.50,
        person_class_id: int = 0,
        sync_queue_size: int = 10,
        sync_slop_sec: float = 0.08,
        max_frame_age_sec: float = 0.45,
        depth_lowest_percent: float = 5.0,
        depth_min_m: float = 0.15,
        depth_max_m: float = 5.0,
        depth_min_valid_pixels: int = 30,
        foot_keypoint_indices: tuple[int, int] = (15, 16),
        foot_keypoint_conf: float = 0.25,
        foot_depth_radius_px: int = 8,
        depth_history_size: int = 3,
        initial_target_max_depth_m: float = 0.9,
        lost_recovery_depth_tolerance_m: float = 0.3,
        enable_gui: bool = True,
        window_name: str = "RGB-D Person Tracking",
        display_width: int = 960,
    ):
        """
        노드 초기화 함수.

        설계 의도:
            이 함수는 ROS2 노드가 동작하기 위해 필요한 모든 의존성을 초기화한다.
            특히 토픽 이름, 상태 코드, YOLO 파라미터, Depth 필터 파라미터를 생성자 인자로 빼두었다.
            이렇게 하면 코드 내부를 수정하지 않고도 launch 파일 또는 main()에서 실험 조건을 바꾸기 쉽다.

        파라미터 설계 근거:
            - tracking_none/visible/lost는 후속 제어 노드가 상태를 단순한 정수로 판단하게 하기 위한 상태 머신 값이다.
            - invalid_float=-1.0은 실제 depth가 음수가 될 수 없기 때문에 invalid sentinel로 안전하다.
            - tracker_config="deepocsort.yaml"은 단순 검출이 아니라 ID 유지가 필요하기 때문에 tracker를 명시한다.
            - max_frame_age_sec=0.45는 지나치게 오래된 RGB-D 쌍이 제어에 반영되는 것을 막기 위한 안전장치이다.
            - depth_min/max는 OAK-D depth의 유효 거리 범위를 제한해 0, inf, 원거리 배경값을 제거한다.

        검증 관점:
            파라미터의 최종값은 카메라 FPS, GPU 추론 시간, 실내 조명, 사람 간 겹침 정도에 따라 달라진다.
            따라서 발표 시에는 “현재 값은 안정 동작을 위한 초기값이고, 실제 환경에서 누락률/오검출률/지연을 보며
            튜닝했다”는 식으로 실험 결과와 연결하는 것이 좋다.
        """
        super().__init__("rgbd_person_tracking_node")

        # YOLO 모델 객체를 외부에서 주입받는다.
        # 모델 로딩을 노드 내부가 아니라 load_yolo_model()에서 수행하면,
        # 모델 warm-up과 ROS 초기화를 분리할 수 있어 초기 실행 지연을 관리하기 쉽다.
        self.yolo_model = yolo_model

        # 입력 RGB/Depth 토픽명이다.
        # TurtleBot4 OAK-D의 compressed RGB와 compressedDepth 토픽을 기본값으로 둔다.
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic

        # 추적 상태값이다.
        # 숫자를 하드코딩하지 않고 멤버 변수로 저장해 상태 의미를 한 곳에서 관리한다.
        self.tracking_none = tracking_none
        self.tracking_visible = tracking_visible
        self.tracking_lost = tracking_lost

        # center/depth가 유효하지 않을 때 사용할 값이다.
        # depth는 물리적으로 음수가 아니므로 -1.0을 invalid sentinel로 쓰기 적합하다.
        self.invalid_float = invalid_float

        # YOLO tracking 설정이다.
        # deepocsort.yaml은 occlusion과 ID 유지가 중요한 사람 추적에서 단순 detect보다 유리한 선택이다.
        self.tracker_config = tracker_config
        self.yolo_device = yolo_device
        self.yolo_imgsz = yolo_imgsz
        self.yolo_conf = yolo_conf
        self.yolo_iou = yolo_iou

        # COCO 기준 person class는 0번이다.
        # classes=[0]으로 제한하면 자동차/의자/가방 같은 불필요한 객체 추론 결과를 제거해 후속 로직이 단순해진다.
        self.person_class_id = person_class_id

        # 프레임 유효 시간 제한이다.
        # 로봇 제어에서는 늦게 도착한 과거 프레임을 처리하는 것보다 버리는 것이 안전하다.
        self.max_frame_age_sec = max_frame_age_sec

        # Depth 산출 관련 파라미터이다.
        # bbox 전체 평균 대신 가까운 일부 픽셀 평균을 쓰는 이유는 bbox 안에 배경/바닥 픽셀이 섞일 수 있기 때문이다.
        self.depth_lowest_percent = depth_lowest_percent
        self.depth_min_m = depth_min_m
        self.depth_max_m = depth_max_m
        self.depth_min_valid_pixels = depth_min_valid_pixels

        # pose 모델의 COCO keypoint 기준 양발 ankle index이다.
        # left_ankle=15, right_ankle=16 중 유효한 keypoint 주변 depth를 사용한다.
        self.foot_keypoint_indices = foot_keypoint_indices

        # 발 keypoint confidence가 너무 낮으면 잘못 찍힌 발 위치일 수 있으므로 depth 계산에서 제외한다.
        self.foot_keypoint_conf = foot_keypoint_conf

        # 발 keypoint 주변에서 depth를 평균낼 정사각형 패치 반경이다.
        # 너무 작으면 depth hole에 민감하고, 너무 크면 바닥/배경이 섞인다.
        self.foot_depth_radius_px = foot_depth_radius_px

        # 최근 depth 값을 짧게 저장해 median filter를 적용한다.
        # depth_history_size=3은 순간 튐은 줄이고, 사람 움직임에 대한 반응성은 유지하는 절충값이다.
        self.depth_history_size = depth_history_size

        # 최초 타깃은 이 거리 이내의 사람만 선택한다.
        # 이 제한은 최초 타깃 획득 전까지만 적용하며, 주행 중 LOST 복구에는 적용하지 않는다.
        self.initial_target_max_depth_m = initial_target_max_depth_m

        # 주행 중 기존 ID를 잃었을 때 마지막 유효 발행 Depth와 비교할 허용 오차이다.
        self.lost_recovery_depth_tolerance_m = lost_recovery_depth_tolerance_m

        # GUI 표시 관련 설정이다.
        # 실제 로봇 주행에서는 enable_gui=False로 두면 OpenCV 창 출력 비용을 줄일 수 있다.
        self.enable_gui = enable_gui
        self.window_name = window_name
        self.display_width = display_width

        # q 또는 ESC 입력 시 main loop가 종료되도록 하는 플래그이다.
        self.should_shutdown = False

        # 추적 잠금 상태와 타깃 상태는 ROS 콜백 스레드와 영상 처리 worker가 함께 접근한다.
        # /service_end가 YOLO 추론 도중 들어오더라도 이전 추적 결과가 다시 발행되지 않도록 보호한다.
        self.tracking_state_lock = threading.Lock()

        # 추적 세션 번호이다. /carrying으로 시작하거나 /service_end로 종료할 때마다 증가한다.
        # worker는 자신이 시작한 세션 번호가 아직 유효한지 확인한 뒤 결과를 발행한다.
        self.tracking_generation = 0

        # /service_end 이후 다음 추적 시작 시 DeepOCSORT 내부 상태를 초기화하기 위한 플래그이다.
        # tracker 객체는 worker thread에서만 건드려 추론 도중 동시 접근을 방지한다.
        self.tracker_reset_requested = False

        # /carrying=True를 받기 전까지 추적을 잠그는 플래그이다.
        # False 상태에서는 YOLO track()을 호출하지 않고 상태 0만 발행한다.
        self.tracking_unlocked = False

        # 현재 추적 중인 타깃 ID이다.
        # None이면 아직 타깃이 선택되지 않은 상태이고, 잠금 해제 후 가장 가까운 사람을 선택한다.
        self.target_id: Optional[int] = None

        # 최초 타깃이 실제로 보이는 상태(/tracking_rgbd=1)까지 도달했는지 나타낸다.
        # False인 동안에는 매 프레임 1m 이내 사람만 대상으로 최초 타깃 선택을 반복한다.
        self.initial_target_acquired = False

        # /target_depth_rgbd로 마지막에 발행한 유효한 타깃 거리이다.
        # 주행 중 LOST 시 새 ID 후보를 이 값의 ±0.5m 범위에서 찾는다.
        self.last_valid_target_depth: Optional[float] = None

        # depth median filter를 위한 최근 depth 저장 리스트이다.
        self.depth_history: list[float] = []

        # 동일 경고가 너무 자주 출력되는 것을 막기 위한 timestamp 저장소이다.
        self.last_warning_time: dict[str, float] = {}

        # RGB-D 동기화 콜백과 worker thread가 같은 latest_pair에 접근하므로 lock이 필요하다.
        self.pair_lock = threading.Lock()

        # 처리할 최신 RGB-D 쌍이다.
        # 구조: (sequence number, rgb_msg, depth_msg, receive_time)
        # 모든 프레임을 큐에 쌓지 않고 최신 쌍만 유지해 지연 누적을 막는다.
        self.latest_pair: Optional[tuple[int, CompressedImage, CompressedImage, float]] = None

        # 새 RGB-D 쌍이 들어올 때마다 증가하는 번호이다.
        # 현재 코드에서는 디버깅/확장용 의미가 크며, 프레임 순서 추적에 사용할 수 있다.
        self.pair_seq = 0

        # 동기화된 새 프레임이 들어왔음을 worker thread에 알리는 이벤트이다.
        self.pair_event = threading.Event()

        # worker thread 종료 요청 이벤트이다.
        self.worker_stop = threading.Event()

        # GUI 이미지 접근 보호용 lock이다.
        # worker가 이미지를 갱신하고 ROS timer가 imshow를 수행하므로 동시 접근을 막아야 한다.
        self.display_lock = threading.Lock()

        # GUI에 표시할 최신 이미지이다.
        self.display_image: Optional[np.ndarray] = None

        # 센서 입력 QoS 설정이다.
        # BEST_EFFORT를 사용한 이유:
        #   카메라 스트림은 실시간성이 중요하고, 손실된 과거 프레임을 재전송받는 것보다 최신 프레임을 받는 것이 낫다.
        # KEEP_LAST depth=5를 사용한 이유:
        #   짧은 버퍼만 유지해 순간적인 수신 지연은 흡수하되, 오래된 영상이 쌓이지 않게 한다.
        # VOLATILE을 사용한 이유:
        #   카메라 데이터는 과거 값을 새 구독자에게 다시 전달할 필요가 없다.
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # 발행 QoS 설정이다.
        # RELIABLE을 사용한 이유:
        #   추적 상태, 중심 픽셀, depth는 후속 제어 노드가 사용하는 제어 입력이므로 손실을 줄이는 것이 중요하다.
        # VOLATILE을 사용한 이유:
        #   현재 상태 토픽은 계속 새로 발행되므로, 늦게 붙은 구독자에게 오래된 값을 저장 전달할 필요가 작다.
        pub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # 추적 상태 발행자이다.
        # 후속 노드는 이 값을 통해 “타깃 없음/보임/사라짐”을 즉시 구분한다.
        self.tracking_pub = self.create_publisher(Int32, tracking_topic, pub_qos)

        # 타깃 bbox 중심 x좌표 발행자이다.
        # 로봇의 중앙 정렬 제어에서는 영상 중앙과 이 값을 비교해 angular.z를 계산할 수 있다.
        self.center_pub = self.create_publisher(Float32, center_topic, pub_qos)

        # 타깃 depth 발행자이다.
        # 로봇이 사람과의 거리 유지 또는 정지 조건을 판단하는 데 사용된다.
        self.depth_pub = self.create_publisher(Float32, target_depth_topic, pub_qos)

        # 추적 잠금 해제 토픽 구독자이다.
        # Bool(data=True)가 들어오면 한 번 잠금을 해제하며, False 메시지는 무시한다.
        self.unlock_sub = self.create_subscription(
            Bool,
            unlock_topic,
            self.carrying_callback,
            pub_qos,
        )

        # 추적 종료 및 초기 LOCK 상태 복귀 토픽 구독자이다.
        # /service_end=True가 들어오면 타깃과 Depth 기록을 제거하고 추적을 다시 잠근다.
        self.service_end_sub = self.create_subscription(
            Bool,
            service_end_topic,
            self.service_end_callback,
            pub_qos,
        )

        # message_filters의 Subscriber를 사용한 이유:
        #   RGB와 Depth를 각각 독립 콜백으로 받으면 서로 다른 시점의 프레임이 섞일 수 있다.
        #   동기화 추적에서는 같은 시점의 RGB bbox와 Depth 픽셀을 맞추는 것이 중요하다.
        self.rgb_sub = Subscriber(self, CompressedImage, rgb_topic, qos_profile=sensor_qos)
        self.depth_sub = Subscriber(self, CompressedImage, depth_topic, qos_profile=sensor_qos)


        # region RGB-D 동기화
        # ApproximateTimeSynchronizer를 사용한 이유:
        #   실제 카메라에서 RGB와 Depth 타임스탬프가 완전히 같지 않을 수 있다.
        #   ExactTimeSynchronizer는 프레임 드롭이 커질 수 있으므로 약간의 시간 오차를 허용한다.
        #
        # sync_queue_size=10:
        #   너무 작으면 매칭 가능한 프레임을 찾기 전에 버려질 수 있고,
        #   너무 크면 오래된 프레임이 매칭되어 레이턴시가 증가할 수 있다.
        #
        # sync_slop_sec=0.08:
        #   약 80ms 이내의 RGB/Depth 차이를 허용한다.
        #   실험에서는 카메라 FPS와 실제 토픽 timestamp 차이를 보고 조정해야 한다.

        """
        rgb/depth 동기화 작업
        """
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=sync_queue_size,
            slop=sync_slop_sec,
            allow_headerless=False,
        )

        # 동기화된 RGB-D 쌍이 들어오면 synced_callback()을 호출한다.
        # 이 콜백에서는 무거운 추론을 수행하지 않고 latest_pair만 갱신한다.
        self.sync.registerCallback(self.synced_callback)

        # GUI timer는 ROS spin과 같은 흐름에서 OpenCV 창을 갱신한다.
        # 0.05초는 약 20Hz로, 추론 FPS보다 높거나 비슷하게 화면을 갱신하기 위한 값이다.
        self.gui_timer = self.create_timer(0.05, self.gui_callback)

        # region 비전AI 추론 별도 스레드
        # YOLO/Depth 처리는 별도 worker thread에서 수행한다.
        # 설계상 핵심은 ROS 수신 콜백을 가볍게 유지하고, 추론 지연이 메시지 동기화에 직접 영향을 주지 않게 하는 것이다.
        """
        이미지 추론 시간으로 인한 토픽 메시지 지연을 방지하기 위해 스레드사용
        """
        self.worker = threading.Thread(target=self.processing_worker, daemon=True)
        self.worker.start()

        # 시작 로그는 실제 실행 시 어떤 토픽을 쓰는지 확인하기 위한 디버깅 정보이다.
        self.get_logger().info("RGB-D person tracking node started")
        self.get_logger().info(f"RGB   : {rgb_topic}")
        self.get_logger().info(f"Depth : {depth_topic}")
        self.get_logger().info(f"Sub   : {unlock_topic} (Bool True -> tracking unlock)")
        self.get_logger().info(f"Sub   : {service_end_topic} (Bool True -> return to LOCKED)")
        self.get_logger().info(f"Pub   : {tracking_topic}, {center_topic}, {target_depth_topic}")
        self.get_logger().info("Tracking is LOCKED until /carrying=True")
        self.get_logger().info(
            f"Initial target limit: <= {self.initial_target_max_depth_m:.2f}m"
        )
        self.get_logger().info(
            "LOST recovery: keep previous ID first; otherwise select within "
            f"last valid depth ±{self.lost_recovery_depth_tolerance_m:.2f}m"
        )

    @staticmethod
    def decode_rgb(msg: CompressedImage) -> Optional[np.ndarray]:
        """
        compressed RGB 메시지를 OpenCV BGR 이미지로 디코딩한다.

        설계 의도:
            ROS의 CompressedImage는 bytes 형태이므로 YOLO에 넣기 전에 np.ndarray 이미지로 바꿔야 한다.
            이 함수는 RGB 디코딩만 담당하게 분리해 process_pair()의 핵심 흐름을 단순하게 만든다.

        반환:
            - 성공: OpenCV BGR 이미지
            - 실패: None

        실패 케이스를 None으로 처리하는 이유:
            예외를 던지면 worker thread 전체가 불안정해질 수 있으므로,
            상위 로직에서 상태 0을 발행하고 다음 프레임으로 넘어가게 했다.
        """
        # 메시지가 없거나 데이터가 비어 있으면 디코딩할 수 없으므로 None을 반환한다.
        if msg is None or not msg.data:
            return None

        # bytes buffer를 uint8 배열로 해석한 뒤 cv2.imdecode로 이미지로 복원한다.
        # OpenCV는 기본적으로 BGR 순서를 사용하며, Ultralytics YOLO는 ndarray 입력을 처리할 수 있다.
        return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    @staticmethod
    def decode_depth_to_meters(msg: CompressedImage) -> Optional[np.ndarray]:
        """
        compressedDepth 메시지를 meter 단위 depth map으로 변환한다.

        설계 의도:
            Depth 토픽은 카메라 설정에 따라 uint16 PNG(mm), float32 PNG(m), RVL 등 여러 형식이 가능하다.
            이 함수는 현재 코드가 처리 가능한 PNG 기반 depth만 meter 단위로 정규화한다.

        중요 설계:
            - RVL 또는 32FC1 compressedDepth는 여기서 직접 복원하지 않는다.
              해당 포맷은 별도 transport/decode 로직이 필요할 수 있으므로 None으로 처리한다.
            - uint16 depth는 일반적으로 mm 단위이므로 0.001을 곱해 meter로 변환한다.
            - 비정상값 NaN/Inf는 0.0으로 바꿔 이후 유효 범위 필터에서 제거되게 한다.

        테스트해야 할 케이스:
            1) uint16 PNG depth
            2) float32 depth
            3) RVL compressedDepth
            4) 비어 있는 메시지
            5) depth_raw가 3채널 또는 uint8로 잘못 들어오는 경우
        """
        # 메시지가 없거나 데이터가 비어 있으면 depth를 계산할 수 없다.
        if msg is None or not msg.data:
            return None

        # format 문자열을 소문자로 바꿔 transport 형식을 확인한다.
        fmt = (msg.format or "").lower()

        # RVL 또는 32FC1 형식은 현재 함수에서 안전하게 복원하지 않는다.
        # 잘못 복원한 depth를 발행하는 것보다 invalid depth로 처리하는 것이 제어 안전성 측면에서 낫다.
        if "rvl" in fmt or "32fc1" in fmt:
            return None

        # compressedDepth 메시지는 앞쪽에 metadata가 붙고 중간부터 PNG가 시작될 수 있다.
        # PNG signature를 찾아 그 위치부터 실제 PNG 데이터로 취급한다.
        data = bytes(msg.data)
        png_start = data.find(b"\x89PNG\r\n\x1a\n")
        if png_start >= 0:
            data = data[png_start:]

        # depth 이미지는 원본 bit depth를 유지해야 하므로 IMREAD_UNCHANGED를 사용한다.
        depth_raw = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)

        # depth 디코딩 실패, 3채널 이미지, uint8 이미지는 metric depth로 보기 어렵다.
        # 이 경우 None을 반환해 상위 로직이 depth invalid로 처리하게 한다.
        if depth_raw is None or depth_raw.ndim == 3 or depth_raw.dtype == np.uint8:
            return None

        # uint16 depth는 일반적으로 mm 단위로 들어오므로 meter로 변환한다.
        if depth_raw.dtype == np.uint16:
            depth_m = depth_raw.astype(np.float32) * 0.001

        # float32 depth는 이미 meter 단위일 가능성이 높으므로 복사 없이 float32로 사용한다.
        elif depth_raw.dtype == np.float32:
            depth_m = depth_raw.astype(np.float32, copy=False)

        # 기타 타입은 float32로 변환한다.
        # median이 100보다 크면 mm 단위일 가능성이 있으므로 meter로 보정한다.
        else:
            depth_m = depth_raw.astype(np.float32)
            if depth_m.size and np.nanmedian(depth_m) > 100.0:
                depth_m *= 0.001

        # NaN/Inf는 이후 범위 필터에서 제외될 수 있도록 0.0으로 정리한다.
        depth_m[~np.isfinite(depth_m)] = 0.0

        # meter 단위 depth map을 반환한다.
        return depth_m

    @staticmethod
    def clamp_bbox(box: np.ndarray, width: int, height: int) -> tuple[int, int, int, int]:
        """
        YOLO bbox 좌표를 이미지 경계 안으로 제한한다.

        설계 의도:
            YOLO가 반환한 bbox는 부동소수점 좌표이고, 일부 좌표가 이미지 경계를 넘어갈 수 있다.
            depth crop 또는 rectangle draw에서 인덱스 오류가 나지 않도록 안전한 int 좌표로 변환한다.

        중요 로직:
            x2는 최소 x1+1, y2는 최소 y1+1이 되게 한다.
            이렇게 해야 bbox 폭/높이가 0이 되어 crop이 비는 문제를 막을 수 있다.
        """
        # YOLO bbox는 float 좌표이므로 이미지 indexing과 drawing을 위해 int로 변환한다.
        x1, y1, x2, y2 = box.astype(int)

        # 왼쪽 위 좌표는 이미지 내부 [0, width-1], [0, height-1]로 제한한다.
        x1 = int(np.clip(x1, 0, width - 1))
        y1 = int(np.clip(y1, 0, height - 1))

        # 오른쪽 아래 좌표는 최소한 x1+1, y1+1보다 크도록 보장한다.
        # OpenCV crop에서 x2, y2는 끝 index로 사용되므로 width, height까지 허용한다.
        x2 = int(np.clip(x2, x1 + 1, width))
        y2 = int(np.clip(y2, y1 + 1, height))

        # 안전하게 보정된 bbox를 반환한다.
        return x1, y1, x2, y2

    @staticmethod
    def scale_bbox(
        bbox: tuple[int, int, int, int],
        src_w: int,
        src_h: int,
        dst_w: int,
        dst_h: int,
    ) -> tuple[int, int, int, int]:
        """
        RGB 이미지 기준 bbox를 Depth 이미지 해상도 기준 bbox로 변환한다.

        설계 의도:
            RGB와 Depth 토픽은 같은 장면을 보더라도 해상도가 다를 수 있다.
            YOLO bbox는 RGB 기준이므로, depth map에서 같은 영역을 crop하려면 좌표 스케일 변환이 필요하다.

        한계:
            이 함수는 단순 비율 스케일만 수행한다.
            RGB와 Depth가 정확히 align_depth 되어 있지 않거나 optical frame이 다르면 오차가 생길 수 있다.
            TurtleBot4/OAK-D 설정에서 depth alignment가 켜져 있다는 전제가 있어야 정확하다.
        """
        # bbox를 각 좌표로 분해한다.
        x1, y1, x2, y2 = bbox

        # 원본 크기가 0에 가까운 경우 division error를 막기 위해 최소 1.0으로 제한한다.
        sx = dst_w / max(float(src_w), 1.0)
        sy = dst_h / max(float(src_h), 1.0)

        # 좌표를 depth 해상도 비율로 변환하고, depth 이미지 경계 안으로 제한한다.
        dx1 = int(np.clip(round(x1 * sx), 0, dst_w - 1))
        dy1 = int(np.clip(round(y1 * sy), 0, dst_h - 1))

        # 오른쪽 아래 좌표도 최소 1픽셀 이상 영역이 생기도록 보정한다.
        dx2 = int(np.clip(round(x2 * sx), dx1 + 1, dst_w))
        dy2 = int(np.clip(round(y2 * sy), dy1 + 1, dst_h))

        # depth 이미지 기준 bbox를 반환한다.
        return dx1, dy1, dx2, dy2

    @staticmethod
    def extract_tracks(result) -> list[Track]:
        """
        Ultralytics pose tracking 결과에서 Track 리스트를 추출한다.

        설계 의도:
            기존 bbox + track_id 기반 추적 로직은 유지하되, YOLO11s-pose가 출력한 keypoint를
            Track 객체에 함께 저장한다. 이렇게 하면 타깃 유지, 중심 픽셀 계산은 기존 bbox 흐름을 그대로 쓰고,
            depth 산출 위치만 사람 bbox 내부가 아니라 양발 keypoint 주변으로 바꿀 수 있다.

        중요 조건:
            boxes.id가 None이면 tracker가 ID를 부여하지 못한 상태이다.
            이 경우 기존 로직과 동일하게 타깃 유지가 불가능하므로 빈 리스트를 반환한다.
        """
        # YOLO result에서 bbox 컨테이너를 가져온다.
        boxes = result.boxes

        # bbox가 없거나 ID가 없으면 추적 가능한 객체가 없다고 판단한다.
        if boxes is None or len(boxes) == 0 or boxes.id is None:
            return []

        # torch tensor를 CPU numpy 배열로 변환한다.
        # ROS/OpenCV 후처리 로직은 numpy 기반이므로 여기서 자료형을 맞춘다.
        xyxy = boxes.xyxy.detach().cpu().numpy()
        ids = boxes.id.detach().cpu().numpy().astype(int)
        confs = boxes.conf.detach().cpu().numpy()

        # pose 모델에서는 result.keypoints에 사람별 keypoint 좌표와 confidence가 들어 있다.
        # 모델이나 tracker 상태에 따라 keypoints가 없을 수 있으므로 Optional로 처리한다.
        keypoints_xy = None
        keypoints_conf = None
        keypoints = getattr(result, "keypoints", None)
        if keypoints is not None and getattr(keypoints, "xy", None) is not None:
            keypoints_xy = keypoints.xy.detach().cpu().numpy().astype(np.float32)
            if getattr(keypoints, "conf", None) is not None:
                keypoints_conf = keypoints.conf.detach().cpu().numpy().astype(np.float32)

        # 각 bbox, id, confidence, keypoint를 Track 객체로 묶어 반환한다.
        tracks: list[Track] = []
        for index, (box, track_id, conf) in enumerate(zip(xyxy, ids, confs)):
            kp_xy = keypoints_xy[index] if keypoints_xy is not None and index < len(keypoints_xy) else None
            kp_conf = keypoints_conf[index] if keypoints_conf is not None and index < len(keypoints_conf) else None
            tracks.append(
                Track(
                    int(track_id),
                    box.astype(np.float32),
                    float(conf),
                    kp_xy,
                    kp_conf,
                )
            )

        return tracks

    def get_track_depth(
        self,
        track: Track,
        depth_m: Optional[np.ndarray],
        rgb_width: int,
        rgb_height: int,
    ) -> Optional[float]:
        """한 사람의 발목 keypoint 주변에서 유효한 raw Depth를 계산한다."""
        if depth_m is None:
            return None

        return self.foot_keypoint_mean_depth(
            depth_m,
            track,
            rgb_width,
            rgb_height,
            foot_keypoint_indices=self.foot_keypoint_indices,
            foot_keypoint_conf=self.foot_keypoint_conf,
            foot_depth_radius_px=self.foot_depth_radius_px,
            depth_min_m=self.depth_min_m,
            depth_max_m=self.depth_max_m,
            depth_min_valid_pixels=self.depth_min_valid_pixels,
        )

    def select_nearest_depth_track(
        self,
        tracks: list[Track],
        depth_m: Optional[np.ndarray],
        rgb_width: int,
        rgb_height: int,
        max_depth_m: Optional[float] = None,
    ) -> tuple[Optional[Track], Optional[float]]:
        """
        현재 프레임에서 유효 Depth가 가장 작은 사람을 선택한다.

        max_depth_m이 지정되면 해당 거리 이하인 사람만 후보로 사용한다.
        최초 타깃 선택에서는 max_depth_m=1.0을 전달하고, 일반 선택에서는 생략할 수 있다.
        """
        if not tracks or depth_m is None:
            return None, None

        nearest_track: Optional[Track] = None
        nearest_depth: Optional[float] = None

        for track in tracks:
            candidate_depth = self.get_track_depth(
                track, depth_m, rgb_width, rgb_height
            )

            if candidate_depth is None:
                continue

            if max_depth_m is not None and candidate_depth > max_depth_m:
                continue

            if nearest_depth is None or candidate_depth < nearest_depth:
                nearest_track = track
                nearest_depth = float(candidate_depth)

        return nearest_track, nearest_depth

    def select_depth_reference_track(
        self,
        tracks: list[Track],
        depth_m: Optional[np.ndarray],
        rgb_width: int,
        rgb_height: int,
        reference_depth_m: Optional[float],
        tolerance_m: float,
    ) -> tuple[Optional[Track], Optional[float]]:
        """
        마지막 유효 Depth와 가장 유사한 사람을 LOST 복구 타깃으로 선택한다.

        후보 조건:
            abs(candidate_depth - reference_depth_m) <= tolerance_m

        여러 후보가 있으면 reference_depth_m과의 절대 차이가 가장 작은 사람을 선택한다.
        이 함수에는 최초 타깃용 1m 제한을 적용하지 않는다.
        """
        if (
            not tracks
            or depth_m is None
            or reference_depth_m is None
            or not np.isfinite(reference_depth_m)
        ):
            return None, None

        selected_track: Optional[Track] = None
        selected_depth: Optional[float] = None
        selected_error: Optional[float] = None

        for track in tracks:
            candidate_depth = self.get_track_depth(
                track, depth_m, rgb_width, rgb_height
            )
            if candidate_depth is None:
                continue

            depth_error = abs(float(candidate_depth) - float(reference_depth_m))
            if depth_error > tolerance_m:
                continue

            if (
                selected_error is None
                or depth_error < selected_error
                or (
                    np.isclose(depth_error, selected_error)
                    and selected_depth is not None
                    and candidate_depth < selected_depth
                )
            ):
                selected_track = track
                selected_depth = float(candidate_depth)
                selected_error = float(depth_error)

        return selected_track, selected_depth

    @staticmethod
    def lowest_percent_mean_depth(
        depth_m: np.ndarray,
        bbox: tuple[int, int, int, int],
        depth_lowest_percent: float = 5.0,
        depth_min_m: float = 0.15,
        depth_max_m: float = 10.0,
        depth_min_valid_pixels: int = 30,
    ) -> Optional[float]:
        """
        bbox 내부 depth 중 가까운 하위 percent 값들의 평균을 계산한다.

        설계 의도:
            사람 bbox 안에는 실제 사람뿐 아니라 배경, 바닥, 빈 공간이 섞일 수 있다.
            bbox 전체 평균을 사용하면 뒤쪽 배경 depth가 섞여 사람이 실제보다 멀게 계산될 수 있다.
            따라서 가장 가까운 일부 depth만 평균내어 “사람 표면에 가까운 거리”를 추정한다.

        왜 최소값 1개가 아니라 하위 5% 평균인가:
            단일 최소값은 노이즈, 구멍, 잘못된 depth 픽셀에 매우 취약하다.
            하위 5% 평균은 가까운 표면을 반영하면서도 단일 픽셀 노이즈 영향을 줄이는 절충안이다.

        파라미터 의미:
            - depth_lowest_percent=5.0:
                bbox 내부에서 가장 가까운 5% 픽셀만 평균낸다.
            - depth_min_m=0.15:
                OAK-D 근거리 비정상값 또는 0에 가까운 invalid 값을 제거한다.
            - depth_max_m=10.0:
                실내 추종에서 너무 먼 배경값을 제거한다.
            - depth_min_valid_pixels=30:
                유효 픽셀이 너무 적을 때 depth를 신뢰하지 않도록 한다.

        검증 포인트:
            5%, 10%, median, bbox 중앙 영역 평균을 비교해 MAE/분산/제어 안정성을 확인하면
            이 수치가 최선이라는 근거를 만들 수 있다.
        """
        # bbox 좌표를 분해한다.
        x1, y1, x2, y2 = bbox

        # bbox 영역의 depth 값을 1차원 배열로 펼친다.
        values = depth_m[y1:y2, x1:x2].reshape(-1)

        # NaN/Inf, 너무 가까운 값, 너무 먼 값을 제거한다.
        # 이 필터링은 depth sensor의 invalid 값과 배경 오염을 줄이기 위한 핵심 전처리이다.
        values = values[
            np.isfinite(values)
            & (values >= depth_min_m)
            & (values <= depth_max_m)
        ]

        # 유효 픽셀이 너무 적으면 해당 bbox의 depth를 신뢰하지 않는다.
        # 이 경우 상위 로직은 depth=-1.0을 발행한다.
        if values.size < depth_min_valid_pixels:
            return None

        # 사용할 픽셀 개수를 계산한다.
        # 최소 1개 이상, 최대 values.size 이하로 제한한다.
        k = max(1, min(int(np.ceil(values.size * depth_lowest_percent / 100.0)), values.size))

        # partition으로 전체 정렬보다 빠르게 하위 k개 값을 얻을 수 있다.
        # 실시간 노드에서는 불필요한 full sort를 피하는 것이 유리하다.

        return float(np.mean(np.partition(values, k - 1)[:k]))

    @staticmethod
    def foot_keypoint_mean_depth(
        depth_m: np.ndarray,
        track: Track,
        rgb_width: int,
        rgb_height: int,
        foot_keypoint_indices: tuple[int, int] = (15, 16),
        foot_keypoint_conf: float = 0.25,
        foot_depth_radius_px: int = 8,
        depth_min_m: float = 0.15,
        depth_max_m: float = 5.0,
        depth_min_valid_pixels: int = 30,
    ) -> Optional[float]:
        """
        타깃 사람의 양발 keypoint 중 유효한 keypoint 주변 depth 평균을 계산한다.

        설계 의도:
            기존 코드는 bbox 내부 가까운 depth를 사용했기 때문에 사람 주변의 배경, 바닥, 다른 사람 depth가
            섞일 수 있었다. 이 함수는 YOLO11s-pose가 제공하는 발목 keypoint를 기준으로 작은 주변 패치만 보므로
            실제 제어 기준이 되는 사람의 발 위치 근방 depth를 더 직접적으로 사용할 수 있다.

        양발 중 하나를 선택하는 방식:
            - left_ankle=15, right_ankle=16을 각각 검사한다.
            - confidence가 낮거나 이미지 밖이거나 유효 depth 픽셀이 부족하면 제외한다.
            - 두 발 모두 유효하면 더 가까운 depth를 선택한다.
              접근 제어에서는 가까운 발 기준이 더 보수적이고 충돌 안전성이 높기 때문이다.
        """
        # pose keypoint가 없으면 발 위치 기반 depth를 계산할 수 없다.
        if track.keypoints_xy is None:
            return None

        # depth 이미지 크기를 가져온다.
        depth_height, depth_width = depth_m.shape[:2]

        # RGB 좌표를 depth 좌표로 바꾸기 위한 스케일이다.
        sx = depth_width / max(float(rgb_width), 1.0)
        sy = depth_height / max(float(rgb_height), 1.0)

        # 양발 후보에서 계산된 depth들을 저장한다.
        candidate_depths: list[float] = []

        # left_ankle, right_ankle을 순회한다.
        for kp_index in foot_keypoint_indices:
            # keypoint index가 모델 출력 범위를 벗어나면 무시한다.
            if kp_index < 0 or kp_index >= len(track.keypoints_xy):
                continue

            # keypoint confidence가 있으면 낮은 신뢰도의 발목 좌표를 제외한다.
            if track.keypoints_conf is not None:
                if kp_index >= len(track.keypoints_conf):
                    continue
                if float(track.keypoints_conf[kp_index]) < foot_keypoint_conf:
                    continue

            # RGB 기준 keypoint 좌표를 가져온다.
            x_rgb, y_rgb = track.keypoints_xy[kp_index]

            # 일부 pose 결과는 미검출 keypoint를 (0, 0) 근처로 둘 수 있으므로 제외한다.
            if not np.isfinite(x_rgb) or not np.isfinite(y_rgb) or x_rgb <= 1.0 or y_rgb <= 1.0:
                continue

            # RGB 기준 발목 좌표를 depth 이미지 좌표로 변환한다.
            x_depth = int(np.clip(round(float(x_rgb) * sx), 0, depth_width - 1))
            y_depth = int(np.clip(round(float(y_rgb) * sy), 0, depth_height - 1))

            # keypoint 주변 정사각형 패치 범위를 만든다.
            x1 = max(0, x_depth - foot_depth_radius_px)
            y1 = max(0, y_depth - foot_depth_radius_px)
            x2 = min(depth_width, x_depth + foot_depth_radius_px + 1)
            y2 = min(depth_height, y_depth + foot_depth_radius_px + 1)

            # 패치 내부 depth를 가져와 1차원으로 펼친다.
            values = depth_m[y1:y2, x1:x2].reshape(-1)

            # NaN/Inf, 너무 가까운 invalid 값, 너무 먼 배경값을 제거한다.
            values = values[
                np.isfinite(values)
                & (values >= depth_min_m)
                & (values <= depth_max_m)
            ]

            # 유효 픽셀이 너무 적으면 해당 발 keypoint 주변 depth를 신뢰하지 않는다.
            if values.size < depth_min_valid_pixels:
                continue

            # 작은 패치 내부 평균 depth를 후보로 사용한다.
            candidate_depths.append(float(np.mean(values)))

        # 양발 모두 유효하지 않으면 invalid depth로 처리한다.
        if not candidate_depths:
            return None

        # 두 발 모두 유효하면 더 가까운 발 depth를 선택한다.
        return float(min(candidate_depths))

    def reset_depth_filter(self) -> None:
        """
        Depth median filter 상태를 초기화한다.

        설계 의도:
            타깃이 바뀌었는데 이전 타깃의 depth history가 남아 있으면 새 타깃의 거리값이 왜곡된다.
            따라서 타깃 lock/reset 시에는 depth history를 반드시 비운다.
        """
        # 최근 depth 기록을 모두 삭제한다.
        self.depth_history.clear()

    def filter_depth(self, depth_m: float):
        """
        최근 depth 값에 median filter를 적용한다.

        설계 의도:
            Depth 카메라는 bbox 내부 픽셀 노이즈, 반사, 사람 움직임으로 순간적인 튐이 발생할 수 있다.
            최근 3개 값의 median을 사용하면 단발성 outlier를 줄이면서 반응 속도도 유지할 수 있다.

        왜 moving average가 아니라 median인가:
            평균은 큰 outlier 하나에도 값이 끌린다.
            median은 짧은 window에서도 튀는 값 제거에 강하다.
        """
        # history size가 1 이하이면 필터를 적용하지 않고 원본 값을 그대로 반환한다.
        if self.depth_history_size <= 1:
            return float(depth_m)

        # 새 depth 값을 history에 추가한다.
        self.depth_history.append(float(depth_m))

        # history 길이를 depth_history_size로 제한한다.
        # 오래된 값이 계속 남으면 현재 거리 변화에 늦게 반응하므로 짧게 유지한다.
        self.depth_history = self.depth_history[-self.depth_history_size:]

        # 최근 depth 값들의 median을 최종 depth로 사용한다.
        return float(np.median(np.asarray(self.depth_history, dtype=np.float32)))

    def carrying_callback(self, msg: Bool) -> None:
        """
        /carrying=True를 받으면 LOCK을 해제하고 새로운 최초 타깃 획득 과정을 시작한다.

        이미 UNLOCKED 상태이면 중복 True 메시지는 무시한다.
        최초 타깃은 /tracking_rgbd=1이 될 때까지 매 프레임 1m 이내에서 다시 찾는다.
        """
        if not msg.data:
            return

        with self.tracking_state_lock:
            if self.tracking_unlocked:
                return

            self.tracking_unlocked = True
            self.tracking_generation += 1
            self.target_id = None
            self.initial_target_acquired = False
            self.last_valid_target_depth = None
            self.reset_depth_filter()
            self.publish_state(self.tracking_none, self.invalid_float, self.invalid_float)

        self.get_logger().info(
            "TRACKING UNLOCKED: /carrying=True received. "
            f"Searching initial target within {self.initial_target_max_depth_m:.2f}m."
        )

    def service_end_callback(self, msg: Bool) -> None:
        """
        /service_end=True를 받으면 실행 직후와 같은 TRACKING LOCKED 상태로 복귀한다.

        초기 타깃 획득 여부, 마지막 유효 Depth, 타깃 ID, Depth 필터 기록을 모두 제거한다.
        추론 도중 메시지가 들어와도 tracking_generation 검증으로 이전 결과 발행을 차단한다.
        """
        if not msg.data:
            return

        with self.tracking_state_lock:
            previous_target = self.target_id
            was_unlocked = self.tracking_unlocked

            self.tracking_unlocked = False
            self.tracking_generation += 1
            self.target_id = None
            self.initial_target_acquired = False
            self.last_valid_target_depth = None
            self.reset_depth_filter()
            self.tracker_reset_requested = True

            self.publish_state(self.tracking_none, self.invalid_float, self.invalid_float)

        self.get_logger().info(
            "TRACKING LOCKED: /service_end=True received. "
            f"previous_target={previous_target}, was_unlocked={was_unlocked}"
        )

    def is_tracking_session_active(self, generation: int) -> bool:
        """worker가 처리 중인 추적 세션이 아직 유효한지 확인한다."""
        with self.tracking_state_lock:
            return self.tracking_unlocked and self.tracking_generation == generation

    def publish_state_if_active(
        self,
        generation: int,
        tracking_state: int,
        center_pixel: float,
        target_depth: float,
    ) -> bool:
        """
        동일한 추적 세션이 유지 중일 때만 결과를 발행한다.

        /service_end가 추론 도중 들어오면 generation이 변경되므로 이전 프레임의
        tracking=1 또는 tracking=2 결과가 LOCK 상태 뒤에 다시 발행되는 것을 막는다.
        """
        with self.tracking_state_lock:
            if not self.tracking_unlocked or self.tracking_generation != generation:
                return False

            self.publish_state(tracking_state, center_pixel, target_depth)
            return True

    def reset_yolo_tracker_state(self) -> None:
        """가능한 경우 Ultralytics predictor가 보유한 tracker 내부 상태를 초기화한다."""
        predictor = getattr(self.yolo_model, "predictor", None)
        trackers = getattr(predictor, "trackers", None)

        if not trackers:
            return

        reset_count = 0
        for tracker in trackers:
            reset_method = getattr(tracker, "reset", None)
            if callable(reset_method):
                reset_method()
                reset_count += 1

        if reset_count > 0:
            self.get_logger().info(f"Tracker state reset: {reset_count} tracker(s)")

    def synced_callback(self, rgb_msg: CompressedImage, depth_msg: CompressedImage) -> None:
        """
        RGB와 Depth가 시간 동기화되어 들어왔을 때 호출되는 콜백.

        설계 의도:
            이 콜백은 최대한 가볍게 설계했다.
            YOLO 추론과 depth 계산을 여기서 직접 수행하지 않고 latest_pair만 갱신한다.
            이렇게 하면 ROS 메시지 수신이 추론 시간 때문에 막히는 문제를 줄일 수 있다.

        latest_pair만 유지하는 이유:
            실시간 로봇 제어에서는 모든 프레임을 처리하는 것보다 최신 프레임에 반응하는 것이 중요하다.
            프레임을 queue에 계속 쌓으면 지연이 누적되어 로봇이 과거 위치를 따라갈 수 있다.
        """
        # worker thread와 공유하는 latest_pair를 수정하므로 lock으로 보호한다.
        with self.pair_lock:
            # 새 프레임 쌍 번호를 증가시킨다.(디버깅용)
            self.pair_seq += 1

            # 최신 RGB-D 쌍과 수신 시간을 저장한다.
            # time.monotonic()은 시스템 시간 변경의 영향을 받지 않아 age 계산에 적합하다.
            self.latest_pair = (self.pair_seq, rgb_msg, depth_msg, time.monotonic())

            # worker thread에게 처리할 새 프레임이 생겼음을 알린다.
            self.pair_event.set()


   
    def processing_worker(self) -> None:
        """
        RGB-D 프레임 처리 전용 worker thread.

        설계 의도:
            이 함수는 무거운 YOLO 추론과 depth 계산을 ROS 콜백 밖에서 수행하기 위해 만든다.
            카메라 콜백은 계속 최신 프레임만 갱신하고, worker는 가능한 속도로 최신 프레임을 처리한다.

        예외 처리 설계:
            추론 중 예외가 발생해도 노드 전체가 죽지 않도록 try/except로 감싼다.
            실패 시에는 tracking_none과 invalid 값을 발행해 후속 제어 노드가 안전 상태로 판단하게 한다.
        """
        # 종료 이벤트가 설정될 때까지 반복한다.
        while not self.worker_stop.is_set():
            # 새 RGB-D 쌍이 들어올 때까지 최대 0.1초 대기한다.
            # timeout을 둔 이유는 종료 이벤트를 주기적으로 확인하기 위해서이다.
            if not self.pair_event.wait(timeout=0.1):
                continue

            # latest_pair를 가져오고 비운다.
            # lock을 사용해 synced_callback()과 동시에 접근하는 상황을 방지한다.
            """
            최신 이미지 사용
            """
            with self.pair_lock:
                pair = self.latest_pair
                self.latest_pair = None
                self.pair_event.clear()

            # 처리할 pair가 없으면 다음 루프로 넘어간다.
            if pair is None:
                continue

            try:
                # 실제 핵심 처리 함수로 RGB-D 쌍을 전달한다.
                self.process_pair(*pair)

            except Exception as exc:
                # 예외가 반복 출력되는 것을 막기 위해 warn_throttled()를 사용한다.
                self.warn_throttled("worker", f"process_pair failed: {type(exc).__name__}: {exc}")

                # 처리 실패 시 후속 제어 노드가 타깃 없음으로 판단하도록 invalid 상태를 발행한다.
                self.publish_state(self.tracking_none, self.invalid_float, self.invalid_float)


    # region 메인프로세스
    def process_pair(
        self,
        seq: int,
        rgb_msg: CompressedImage,
        depth_msg: CompressedImage,
        receive_time: float,
    ) -> None:
        """
        동기화된 RGB-D 한 쌍에서 타깃 상태, 중심 픽셀, Depth를 계산해 발행한다.

        타깃 상태 머신:
            1) LOCKED: /carrying=True 대기
            2) 최초 획득 전: 매 프레임 1m 이내 사람만 검색
            3) 최초 획득 후: 기존 target_id를 최우선 유지
            4) 주행 중 LOST: 기존 ID가 없을 때만 마지막 유효 Depth ±0.5m 후보로 재타겟팅
        """
        start = time.perf_counter()

        # 제어에 반영하기에는 너무 오래된 프레임은 폐기한다.
        if time.monotonic() - receive_time > self.max_frame_age_sec:
            return

        rgb = self.decode_rgb(rgb_msg)
        if rgb is None:
            self.warn_throttled("rgb_decode", "RGB decode failed")
            self.publish_state(self.tracking_none, self.invalid_float, self.invalid_float)
            return

        # LOCK 상태에서는 YOLO/Depth 처리를 하지 않는다.
        with self.tracking_state_lock:
            if not self.tracking_unlocked:
                state = self.tracking_none
                center = self.invalid_float
                depth = self.invalid_float
                status = "LOCKED - waiting /carrying=True"
                self.publish_state(state, center, depth)
                session_generation = None
            else:
                session_generation = self.tracking_generation

        if session_generation is None:
            self.draw_display(rgb, [], state, center, depth, status, start)
            return

        # /service_end 이후 첫 추적 세션에서 DeepOCSORT 내부 상태를 worker thread가 초기화한다.
        with self.tracking_state_lock:
            reset_tracker = (
                self.tracking_unlocked
                and self.tracking_generation == session_generation
                and self.tracker_reset_requested
            )
            if reset_tracker:
                self.tracker_reset_requested = False

        if reset_tracker:
            self.reset_yolo_tracker_state()

        depth_m = self.decode_depth_to_meters(depth_msg)
        if depth_m is None:
            self.warn_throttled(
                "depth_decode",
                "Depth decode failed or topic is not metric depth",
            )

        rgb_h, rgb_w = rgb.shape[:2]

        result = self.yolo_model.track(
            source=rgb,
            persist=True,
            tracker=self.tracker_config,
            classes=[self.person_class_id],
            conf=self.yolo_conf,
            iou=self.yolo_iou,
            imgsz=self.yolo_imgsz,
            device=self.yolo_device,
            verbose=False,
        )[0]

        # 추론 중 /service_end가 들어왔다면 이 프레임은 폐기한다.
        if not self.is_tracking_session_active(session_generation):
            return

        tracks = self.extract_tracks(result)

        with self.tracking_state_lock:
            if not self.tracking_unlocked or self.tracking_generation != session_generation:
                return
            current_target_id = self.target_id
            initial_target_acquired = self.initial_target_acquired
            last_valid_target_depth = self.last_valid_target_depth

        target = next(
            (track for track in tracks if track.track_id == current_target_id),
            None,
        )

        # -----------------------------------------------------------------
        # A. 최초 타깃 획득 전: /tracking_rgbd=1이 될 때까지 1m 이내 검색 반복
        # -----------------------------------------------------------------
        if not initial_target_acquired:
            # 이전 프레임의 임시 target_id가 사라졌다면 그 ID를 고집하지 않는다.
            had_provisional_target = current_target_id is not None
            if target is None:
                selected, selected_depth = self.select_nearest_depth_track(
                    tracks,
                    depth_m,
                    rgb_w,
                    rgb_h,
                    max_depth_m=self.initial_target_max_depth_m,
                )

                if selected is None or selected_depth is None:
                    with self.tracking_state_lock:
                        if (
                            not self.tracking_unlocked
                            or self.tracking_generation != session_generation
                        ):
                            return
                        self.target_id = None
                        self.reset_depth_filter()

                    state = (
                        self.tracking_lost
                        if had_provisional_target
                        else self.tracking_none
                    )
                    center = self.invalid_float
                    depth = self.invalid_float
                    status = (
                        f"initial retarget: waiting person <= "
                        f"{self.initial_target_max_depth_m:.2f}m"
                    )
                    if not self.publish_state_if_active(
                        session_generation, state, center, depth
                    ):
                        return
                    self.draw_display(
                        rgb, tracks, state, center, depth, status, start
                    )
                    return

                with self.tracking_state_lock:
                    if (
                        not self.tracking_unlocked
                        or self.tracking_generation != session_generation
                    ):
                        return
                    self.target_id = selected.track_id
                    self.reset_depth_filter()
                    current_target_id = self.target_id

                target = selected
                self.get_logger().info(
                    f"INITIAL TARGET CANDIDATE: ID {current_target_id}, "
                    f"depth={selected_depth:.3f}m "
                    f"(limit={self.initial_target_max_depth_m:.3f}m)"
                )

        # -----------------------------------------------------------------
        # B. 최초 타깃 획득 후: 기존 ID 우선, 없을 때만 Depth 기반 LOST 복구
        # -----------------------------------------------------------------
        elif target is None:
            recovered, recovered_depth = self.select_depth_reference_track(
                tracks,
                depth_m,
                rgb_w,
                rgb_h,
                reference_depth_m=last_valid_target_depth,
                tolerance_m=self.lost_recovery_depth_tolerance_m,
            )

            if recovered is not None and recovered_depth is not None:
                previous_target_id = current_target_id
                with self.tracking_state_lock:
                    if (
                        not self.tracking_unlocked
                        or self.tracking_generation != session_generation
                    ):
                        return
                    self.target_id = recovered.track_id
                    self.reset_depth_filter()
                    current_target_id = self.target_id

                target = recovered
                self.get_logger().info(
                    f"TARGET RECOVERED: old ID {previous_target_id} -> "
                    f"new ID {current_target_id}, depth={recovered_depth:.3f}m, "
                    f"reference={last_valid_target_depth:.3f}m, "
                    f"tolerance=±{self.lost_recovery_depth_tolerance_m:.3f}m"
                )
            else:
                state = self.tracking_lost
                center = self.invalid_float
                depth = self.invalid_float
                if last_valid_target_depth is None:
                    status = (
                        f"target ID {current_target_id} lost; "
                        "no last valid depth for recovery"
                    )
                else:
                    status = (
                        f"target ID {current_target_id} lost; waiting depth "
                        f"{last_valid_target_depth:.2f}±"
                        f"{self.lost_recovery_depth_tolerance_m:.2f}m"
                    )

                if not self.publish_state_if_active(
                    session_generation, state, center, depth
                ):
                    return
                self.draw_display(
                    rgb, tracks, state, center, depth, status, start
                )
                return

        # 안전상 target이 여전히 없으면 상태를 발행하고 다음 프레임에서 다시 시도한다.
        if target is None:
            state = (
                self.tracking_none
                if not initial_target_acquired
                else self.tracking_lost
            )
            center = self.invalid_float
            depth = self.invalid_float
            status = "target unavailable - retrying"
            if not self.publish_state_if_active(
                session_generation, state, center, depth
            ):
                return
            self.draw_display(rgb, tracks, state, center, depth, status, start)
            return

        # -----------------------------------------------------------------
        # C. 타깃이 보임: bbox 중심과 발목 Depth 계산
        # -----------------------------------------------------------------
        rgb_bbox = self.clamp_bbox(target.box, rgb_w, rgb_h)
        x1, _, x2, _ = rgb_bbox
        center = 0.5 * float(x1 + x2)

        depth = self.invalid_float
        raw_depth: Optional[float] = None
        status = "target visible, invalid depth"

        if depth_m is not None:
            raw_depth = self.get_track_depth(
                target, depth_m, rgb_w, rgb_h
            )

        # 최초 타깃은 실제 가시 상태 1로 확정되는 순간에도 1m 조건을 보장한다.
        if (
            not initial_target_acquired
            and (
                raw_depth is None
                or raw_depth > self.initial_target_max_depth_m
            )
        ):
            with self.tracking_state_lock:
                if (
                    not self.tracking_unlocked
                    or self.tracking_generation != session_generation
                ):
                    return
                self.target_id = None
                self.reset_depth_filter()

            state = self.tracking_none
            center = self.invalid_float
            depth = self.invalid_float
            status = (
                f"initial candidate rejected; waiting person <= "
                f"{self.initial_target_max_depth_m:.2f}m"
            )
            if not self.publish_state_if_active(
                session_generation, state, center, depth
            ):
                return
            self.draw_display(rgb, tracks, state, center, depth, status, start)
            return

        if raw_depth is not None:
            with self.tracking_state_lock:
                if (
                    not self.tracking_unlocked
                    or self.tracking_generation != session_generation
                ):
                    return
                depth = self.filter_depth(raw_depth)

                # 마지막 유효 Depth는 실제 /target_depth_rgbd로 발행할 값을 저장한다.
                self.last_valid_target_depth = float(depth)

            status = "target visible, foot keypoint depth mean"

        # 최초로 tracking=1에 도달한 시점에만 최초 획득 완료 상태로 전환한다.
        with self.tracking_state_lock:
            if (
                not self.tracking_unlocked
                or self.tracking_generation != session_generation
            ):
                return

            first_acquisition_completed = not self.initial_target_acquired
            self.initial_target_acquired = True
            active_target_id = self.target_id

        if not self.publish_state_if_active(
            session_generation,
            self.tracking_visible,
            center,
            depth,
        ):
            return

        if first_acquisition_completed:
            self.get_logger().info(
                f"INITIAL TARGET ACQUIRED: ID {active_target_id}, "
                f"depth={depth:.3f}m, tracking_rgbd=1"
            )

        self.draw_display(
            rgb,
            tracks,
            self.tracking_visible,
            center,
            depth,
            status,
            start,
        )

    def publish_state(self, tracking_state: int, center_pixel: float, target_depth: float) -> None:
        """
        추적 상태, 중심 픽셀, depth를 ROS2 토픽으로 발행한다.

        설계 의도:
            후속 제어 노드가 복잡한 vision 내부 로직을 몰라도 되도록,
            필요한 최소 제어 입력만 3개 토픽으로 분리해 발행한다.

        왜 세 토픽으로 나누었는가:
            - tracking_state는 상태 머신 판단용이다.
            - center_pixel은 회전 중앙 정렬용이다.
            - target_depth는 접근/정지 거리 판단용이다.

        대안:
            커스텀 메시지 하나로 묶으면 timestamp 일관성은 좋아진다.
            하지만 현재 구조는 std_msgs만 사용하므로 구현과 테스트가 단순하다.
        """
        # Int32 메시지를 생성하고 상태값을 넣는다.
        state_msg = Int32()
        state_msg.data = int(tracking_state)

        # tracking 상태를 발행한다.
        self.tracking_pub.publish(state_msg)

        # Float32 메시지를 생성하고 bbox 중심 x좌표를 넣는다.
        center_msg = Float32()
        center_msg.data = float(center_pixel)

        # 중심 픽셀을 발행한다.
        self.center_pub.publish(center_msg)

        # Float32 메시지를 생성하고 타깃 depth를 넣는다.
        depth_msg = Float32()
        depth_msg.data = float(target_depth)

        # 타깃 depth를 발행한다.
        self.depth_pub.publish(depth_msg)

    def warn_throttled(self, key: str, message: str, period: float = 2.0) -> None:
        """
        동일 경고를 일정 주기 이상 간격으로만 출력한다.

        설계 의도:
            카메라 디코딩 실패나 depth 실패가 매 프레임 발생하면 로그가 과도하게 쌓인다.
            로그 폭주를 막으면서도 문제 상황을 주기적으로 확인할 수 있게 한다.

        period=2.0의 의미:
            같은 종류의 경고는 최소 2초 간격으로 출력한다.
            너무 짧으면 로그가 많고, 너무 길면 문제 인지가 늦어진다.
        """
        # 현재 monotonic 시간을 가져온다.
        now = time.monotonic()

        # 같은 key의 마지막 경고 시각과 비교해 period 이상 지났을 때만 출력한다.
        if now - self.last_warning_time.get(key, 0.0) >= period:
            # 마지막 경고 시각을 갱신한다.
            self.last_warning_time[key] = now

            # ROS warning 로그를 출력한다.
            self.get_logger().warn(message)

    @staticmethod
    def draw_text(
        image: np.ndarray,
        text: str,
        xy: tuple[int, int],
        color: tuple[int, int, int] = (255, 255, 255),
        scale: float = 0.55,
    ) -> None:
        """
        OpenCV 이미지에 가독성 좋은 텍스트를 그린다.

        설계 의도:
            배경색이 복잡한 RGB 영상 위에 바로 흰 글씨만 쓰면 잘 보이지 않을 수 있다.
            그래서 먼저 검은색 두꺼운 글씨를 그리고, 그 위에 실제 색상 글씨를 한 번 더 그려 외곽선 효과를 만든다.
        """
        # 검은색 굵은 텍스트를 먼저 그려 outline 역할을 하게 한다.
        cv2.putText(image, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)

        # 실제 색상 텍스트를 그 위에 얇게 그린다.
        cv2.putText(image, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)

    def draw_pose_keypoints(
        self,
        image: np.ndarray,
        track: Track,
        is_target: bool,
    ) -> None:
        """
        YOLO pose 모델이 추정한 사람 keypoint와 skeleton을 GUI에 그린다.

        설계 의도:
            pose 모델로 바꿔도 OpenCV 화면에는 자동으로 keypoint가 표시되지 않는다.
            Ultralytics result.plot()을 쓰면 keypoint는 쉽게 보이지만, 현재 코드의 target_id 색상,
            tracking 상태 표시, depth 표시를 세밀하게 제어하기 어렵다.
            그래서 Track 객체에 저장한 keypoint를 직접 그려 기존 GUI 로직을 유지했다.

        시각화 기준:
            - 일반 keypoint는 작은 원으로 표시한다.
            - 양발 ankle keypoint(left_ankle=15, right_ankle=16)는 더 큰 원과 사각형으로 강조한다.
            - confidence가 foot_keypoint_conf보다 낮은 keypoint는 화면에 그리지 않는다.
              낮은 신뢰도의 점을 보여주면 실제 발 위치로 오해할 수 있기 때문이다.
        """
        # pose 결과가 없으면 그릴 keypoint가 없다.
        if track.keypoints_xy is None:
            return

        # COCO pose skeleton 연결 관계이다.
        # 0:nose, 5/6:shoulder, 7/8:elbow, 9/10:wrist, 11/12:hip, 13/14:knee, 15/16:ankle
        skeleton_edges = (
            (5, 6),
            (5, 7), (7, 9),
            (6, 8), (8, 10),
            (5, 11), (6, 12),
            (11, 12),
            (11, 13), (13, 15),
            (12, 14), (14, 16),
        )

        # 타깃과 비타깃의 keypoint 색상을 분리해 타깃 사람을 더 쉽게 확인한다.
        keypoint_color = (0, 255, 255) if is_target else (180, 180, 180)
        skeleton_color = (255, 0, 255) if is_target else (90, 90, 90)
        foot_color = (0, 0, 255) if is_target else (0, 200, 255)

        # 이미지 크기는 좌표 clamp에 사용한다.
        h, w = image.shape[:2]

        def valid_keypoint(kp_index: int) -> Optional[tuple[int, int]]:
            """
            한 keypoint가 화면에 그릴 수 있는 유효 좌표인지 확인한다.

            confidence가 낮거나, 좌표가 NaN/Inf이거나, 미검출 기본값처럼 (0, 0)에 가까우면 제외한다.
            """
            # keypoint index가 출력 범위를 벗어나면 무효이다.
            if kp_index < 0 or kp_index >= len(track.keypoints_xy):
                return None

            # confidence 배열이 있으면 낮은 신뢰도의 keypoint는 제외한다.
            if track.keypoints_conf is not None:
                if kp_index >= len(track.keypoints_conf):
                    return None
                if float(track.keypoints_conf[kp_index]) < self.foot_keypoint_conf:
                    return None

            # keypoint 좌표를 꺼낸다.
            x, y = track.keypoints_xy[kp_index]

            # NaN/Inf와 미검출 좌표를 제외한다.
            if not np.isfinite(x) or not np.isfinite(y) or x <= 1.0 or y <= 1.0:
                return None

            # 이미지 경계 안으로 좌표를 제한한다.
            px = int(np.clip(round(float(x)), 0, w - 1))
            py = int(np.clip(round(float(y)), 0, h - 1))
            return px, py

        # 먼저 skeleton 선을 그린다.
        # 선을 먼저 그리고 점을 나중에 그리면 keypoint 원이 선 위에 올라와 더 잘 보인다.
        for a, b in skeleton_edges:
            pa = valid_keypoint(a)
            pb = valid_keypoint(b)
            if pa is not None and pb is not None:
                cv2.line(image, pa, pb, skeleton_color, 2 if is_target else 1, cv2.LINE_AA)

        # 모든 keypoint를 원으로 표시한다.
        for kp_index in range(len(track.keypoints_xy)):
            point = valid_keypoint(kp_index)
            if point is None:
                continue

            # 양발 ankle은 depth 계산 기준점이므로 더 크게 강조한다.
            is_foot = kp_index in self.foot_keypoint_indices
            radius = 6 if is_foot else 4
            color = foot_color if is_foot else keypoint_color
            cv2.circle(image, point, radius, color, -1, cv2.LINE_AA)
            cv2.circle(image, point, radius + 1, (0, 0, 0), 1, cv2.LINE_AA)

            # 타깃의 발목 keypoint에는 L/R 라벨과 depth 패치 범위를 같이 표시한다.
            if is_target and is_foot:
                label = "L ankle" if kp_index == 15 else "R ankle"
                self.draw_text(image, label, (point[0] + 6, max(18, point[1] - 6)), foot_color, 0.42)

                # 실제 depth 계산에 쓰는 주변 패치 반경을 RGB 화면 기준으로 표시한다.
                # RGB와 Depth 해상도가 다를 수 있지만, 사용자가 keypoint 주변을 확인하는 디버깅 용도이다.
                r = int(self.foot_depth_radius_px)
                cv2.rectangle(
                    image,
                    (max(0, point[0] - r), max(0, point[1] - r)),
                    (min(w - 1, point[0] + r), min(h - 1, point[1] + r)),
                    foot_color,
                    1,
                    cv2.LINE_AA,
                )

    def draw_display(
        self,
        rgb: np.ndarray,
        tracks: list[Track],
        state: int,
        center: float,
        depth: float,
        status: str,
        start_time: float,
    ) -> None:
        """
        추적 결과를 OpenCV GUI 이미지로 시각화한다.

        설계 의도:
            로봇 제어 코드는 토픽만으로 동작할 수 있지만, 개발/디버깅 단계에서는
            어떤 사람이 타깃인지, track_id가 유지되는지, depth가 유효한지 직접 확인해야 한다.
            이 함수는 추적 상태를 한 화면에 표시하기 위해 존재한다.

        중요 시각화:
            - 타깃 bbox는 빨간색으로 표시한다.
            - 비타깃 사람은 초록색으로 표시한다.
            - 상태값, center_x, depth, 처리 시간, 조작키를 표시한다.
        """
        # GUI가 꺼져 있으면 이미지 복사/그리기 비용을 쓰지 않고 바로 반환한다.
        if not self.enable_gui:
            return

        # 원본 RGB 이미지를 보존하기 위해 copy해서 그린다.
        image = rgb.copy()

        # 이미지 크기를 얻는다.
        h, w = image.shape[:2]

        # 현재 프레임의 모든 track을 순회하며 bbox를 그린다.
        for track in tracks:
            # bbox를 이미지 내부 좌표로 보정한다.
            x1, y1, x2, y2 = self.clamp_bbox(track.box, w, h)

            # 현재 track이 타깃인지 확인한다.
            is_target = track.track_id == self.target_id

            # 타깃은 빨간색, 비타깃은 초록색으로 구분한다.
            color = (0, 0, 255) if is_target else (0, 200, 0)

            # 타깃 bbox는 더 두껍게 그려 시각적으로 강조한다.
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 3 if is_target else 2)

            # bbox 위에 track_id와 confidence를 표시한다.
            label = f"TARGET ID {track.track_id}" if is_target else f"ID {track.track_id}"
            self.draw_text(image, f"{label} {track.confidence:.2f}", (x1, max(22, y1 - 8)), color, 0.52)

            # pose 모델이 추정한 keypoint와 skeleton을 직접 그린다.
            # 이 호출이 없으면 YOLO11s-pose를 사용해도 OpenCV GUI에는 bbox만 보인다.
            self.draw_pose_keypoints(image, track, is_target)

        # 현재 노드가 어떤 방식으로 추적 중인지 상단에 표시한다.
        self.draw_text(image, "YOLO11s-pose RGB tracking + foot keypoint depth", (20, 30), (0, 255, 255), 0.55)

        # 추적 잠금 상태를 표시한다.
        lock_text = "UNLOCKED" if self.tracking_unlocked else "LOCKED"
        lock_color = (0, 255, 0) if self.tracking_unlocked else (0, 0, 255)
        self.draw_text(image, f"Tracking lock: {lock_text}", (20, 58), lock_color)

        # 현재 target_id를 표시한다.
        self.draw_text(image, f"Target ID: {self.target_id if self.target_id is not None else 'NONE'}", (20, 86))

        # ROS로 발행되는 tracking 상태값을 그대로 표시한다.
        self.draw_text(image, f"tracking_rgbd: {state}", (20, 114))

        # center_x가 유효하면 px 단위로 표시하고, invalid이면 invalid로 표시한다.
        self.draw_text(
            image,
            f"center_x: {center:.1f}px" if center >= 0 else "center_x: invalid",
            (20, 142),
            (255, 255, 0),
        )

        # depth가 유효하면 meter 단위로 표시하고, invalid이면 빨간색으로 invalid를 표시한다.
        self.draw_text(
            image,
            f"depth: {depth:.3f}m" if depth >= 0 else "depth: invalid",
            (20, 170),
            (255, 255, 0) if depth >= 0 else (0, 0, 255),
        )

        # process_pair() 시작부터 현재까지 걸린 시간을 ms로 계산한다.
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0

        # 처리 시간과 상태 문자열을 하단에 표시한다.
        self.draw_text(image, f"{elapsed_ms:.1f}ms | {status}", (20, h - 45), (220, 220, 220), 0.45)

        # 키보드 조작 안내를 표시한다.
        self.draw_text(image, "r: retarget | q or ESC: quit", (20, h - 18), (220, 220, 220), 0.45)

        # display_width가 지정되어 있으면 GUI 표시 크기를 조정한다.
        # 추론 입력 크기와 화면 표시 크기를 분리해, 큰 모니터/작은 모니터 모두에서 확인하기 쉽게 한다.
        if self.display_width > 0 and image.shape[1] != self.display_width:
            scale = self.display_width / float(image.shape[1])
            image = cv2.resize(
                image,
                (self.display_width, max(1, int(round(image.shape[0] * scale)))),
                cv2.INTER_AREA,
            )

        # GUI timer와 worker thread가 공유하는 display_image를 lock으로 보호하며 갱신한다.
        with self.display_lock:
            self.display_image = image

    def gui_callback(self) -> None:
        """
        OpenCV GUI를 주기적으로 갱신하고 키 입력을 처리한다.

        설계 의도:
            OpenCV imshow/waitKey는 주기적으로 호출되어야 창이 정상 갱신된다.
            ROS timer를 사용해 worker thread와 분리된 주기로 GUI만 담당하게 했다.

        키 입력:
            - q, Q, ESC: 노드 종료 요청
            - r, R: 현재 target_id 초기화 후 다시 중앙 기준으로 타깃 선택
        """
        # GUI가 비활성화되어 있으면 아무 작업도 하지 않는다.
        if not self.enable_gui:
            return

        # worker가 갱신한 display_image를 복사해온다.
        # lock 밖에서 imshow를 수행해 lock 점유 시간을 줄인다.
        with self.display_lock:
            frame = None if self.display_image is None else self.display_image.copy()

        # 표시할 프레임이 있으면 창에 출력한다.
        if frame is not None:
            cv2.imshow(self.window_name, frame)

        # waitKey(1)은 OpenCV 이벤트 처리와 키 입력 확인을 동시에 수행한다.
        key = cv2.waitKey(1) & 0xFF

        # q/Q/ESC 입력 시 main loop가 종료되도록 플래그를 세운다.
        if key in (ord("q"), ord("Q"), 27):
            self.should_shutdown = True

        # r/R 입력 시 타깃을 재선택할 수 있도록 초기화한다.
        elif key in (ord("r"), ord("R")):
            self.reset_target()

    def reset_target(self) -> None:
        """
        수동 재타겟팅 시 최초 타깃 획득 단계로 되돌린다.

        다음 프레임부터 다시 1m 이내 사람만 대상으로 /tracking_rgbd=1이 될 때까지 선택을 반복한다.
        """
        with self.tracking_state_lock:
            if not self.tracking_unlocked:
                self.get_logger().info("Target reset ignored: tracking is LOCKED")
                return

            previous = self.target_id
            self.target_id = None
            self.initial_target_acquired = False
            self.last_valid_target_depth = None
            self.reset_depth_filter()
            self.publish_state(
                self.tracking_none,
                self.invalid_float,
                self.invalid_float,
            )

        self.get_logger().info(
            f"Target reset: previous ID={previous}; "
            f"searching within {self.initial_target_max_depth_m:.2f}m"
        )

    def shutdown(self) -> None:
        """
        노드 종료 시 worker thread와 OpenCV 창을 정리한다.

        설계 의도:
            worker thread가 살아 있는 상태로 프로그램이 종료되면 리소스가 정상 해제되지 않을 수 있다.
            따라서 종료 이벤트를 세우고, 대기 중인 worker를 깨운 뒤 join한다.
        """
        # worker thread에 종료 요청을 보낸다.
        self.worker_stop.set()

        # worker가 pair_event.wait()에서 대기 중일 수 있으므로 이벤트를 세워 깨운다.
        self.pair_event.set()

        # worker가 살아 있으면 최대 1초간 종료를 기다린다.
        if self.worker.is_alive():
            self.worker.join(timeout=1.0)

        # GUI를 사용했다면 OpenCV 창을 닫는다.
        if self.enable_gui:
            cv2.destroyAllWindows()


def load_yolo_model(
    model_path: str = "yolo11s-pose.pt",
    yolo_imgsz: int = 640,
    yolo_device: int | str = 0,
    person_class_id: int = 0,
):
    """
    YOLO 모델을 로드하고 warm-up 추론을 수행한다.

    설계 의도:
        첫 추론은 모델 로딩, CUDA context 생성, kernel 초기화 등으로 지연이 크게 발생할 수 있다.
        실제 ROS 프레임 처리 전에 dummy image로 한 번 predict를 수행해 첫 프레임 지연을 줄인다.

    모델 선택 설명:
        - yolo11s-pose.pt는 YOLO11 계열의 pose 모델 중 speed/accuracy 균형형 모델이다.
        - yolo11n은 더 빠르지만 멀리 있는 사람이나 occlusion 상황에서 검출 안정성이 낮아질 수 있다.
        - yolo11m/l/x는 정확도는 높을 수 있지만 TurtleBot4 실시간 추적에서는 지연과 GPU 사용량이 커질 수 있다.
        - 따라서 실시간 RGB-D 추적 노드에서는 yolo11s-pose가 타협점으로 적합하다.

    검증 필요:
        코드에는 yolo11n/s/m 비교 실험 결과가 포함되어 있지 않다.
        발표에서는 같은 카메라 영상에서 FPS, 검출 누락률, ID switch 수, 추적 복구 시간을 비교하면
        모델 선택의 근거가 명확해진다.
    """
    # 어떤 모델 파일을 로드하는지 콘솔에 표시한다.
    print(f"[INFO] Loading YOLO model: {model_path}")

    # Ultralytics YOLO 모델 객체를 생성한다.
    model = YOLO(model_path)

    # dummy image로 warm-up 추론을 수행한다.
    # 실제 카메라 첫 프레임에서 발생할 수 있는 초기 지연을 미리 흡수하기 위한 로직이다.
    model.predict(
        source=np.zeros((yolo_imgsz, yolo_imgsz, 3), dtype=np.uint8),
        imgsz=yolo_imgsz,
        device=yolo_device,
        classes=[person_class_id],
        verbose=False,
    )

    # warm-up이 끝난 모델을 반환한다.
    return model


def main() -> None:
    """
    프로그램 진입점.

    설계 의도:
        main()은 모델 로드, ROS 초기화, 노드 생성, spin loop, 종료 정리를 담당한다.
        실제 추적 로직은 RgbdPersonTrackingNode 안에 두고, main()은 실행 생명주기만 관리하게 분리했다.

    실행 흐름:
        1) YOLO 모델 로드 및 warm-up
        2) rclpy 초기화
        3) RgbdPersonTrackingNode 생성
        4) spin_once로 ROS 이벤트 처리
        5) q/ESC 또는 Ctrl+C 시 shutdown 정리
    """
    # YOLO 모델을 먼저 로드한다.
    # 모델 로딩 실패가 있으면 ROS 노드 생성 전에 문제를 확인할 수 있다.
    yolo_model = load_yolo_model()

    # ROS2 Python 클라이언트를 초기화한다.
    rclpy.init()

    # RGB-D 사람 추적 노드를 생성한다.
    node = RgbdPersonTrackingNode(yolo_model)

    try:
        # ROS가 정상 상태이고 GUI 종료 요청이 없을 때까지 반복한다.
        while rclpy.ok() and not node.should_shutdown:
            # spin_once를 사용한 이유:
            #   OpenCV GUI와 worker thread 종료 플래그를 함께 확인하기 위해 직접 loop를 관리한다.
            # timeout_sec=0.05는 약 20Hz 주기로 ROS callback을 처리하면서 GUI 종료 플래그에도 반응하기 위한 값이다.
            rclpy.spin_once(node, timeout_sec=0.05)

    except KeyboardInterrupt:
        # Ctrl+C로 종료 요청이 들어오면 로그만 남기고 finally에서 정리한다.
        node.get_logger().info("Shutdown requested by Ctrl+C")

    finally:
        # worker thread와 OpenCV 창을 정리한다.
        node.shutdown()

        # ROS 노드 객체를 파괴한다.
        node.destroy_node()

        # rclpy가 아직 살아 있으면 shutdown한다.
        if rclpy.ok():
            rclpy.shutdown()


# 이 파일을 직접 실행할 때만 main()을 호출한다.
# 다른 파일에서 import할 때 자동 실행되지 않게 하기 위한 Python 표준 패턴이다.
if __name__ == "__main__":
    main()