import math
import time
from typing import Optional

import rclpy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32, Int32, String


class GuardedNavToGoal(Node):
    """
    사람 추적 상태, 거리, 중앙정렬 조건을 모두 만족할 때만
    Nav2 목적지 이동을 수행한다.

    초기 상태에서는 SYSTEM_LOCKED=True로 시작하여 일반 이동 제어권을
    사용하지 않는다. 단, /turn_around=True에 의한 180도 회전은
    SYSTEM_LOCKED 상태에서도 예외적으로 수행할 수 있다.
    180도 회전 완료 후 발행되는 /turn_complete=True를 수신하면
    SYSTEM_LOCKED를 해제하고 후방 목적지 이동 기능을 활성화한다.
    /front_start=1 또는 /service_end=True를 수신하면 다시 잠근다.
    또한 /start_front=0과 /service_end=True가 동시에 성립하면
    초기 잠금 상태로 복귀하고 /start_patrol=True를 한 번 발행한다.

    이동 허용 조건
    ----------------
    1. /robot3/mission_cmd로 유효한 목적지를 받아야 한다.
    2. /tracking_web == 1이어야 한다.
    3. 0.0 < /target_depth <= 1.0이어야 한다.
    4. /tracking_web_center_pixel이 중앙 30% 영역에 있어야 한다.
    5. 세 추적 토픽이 최근까지 계속 수신되고 있어야 한다.

    목적지 도착 후
    -------------
    current_mission을 None으로 초기화하고 완전히 정지한다.
    최신 목적지의 Nav2 결과가 성공이면 /mission_complete=True를 한 번 발행한다.
    이후 다음 /robot3/mission_cmd를 기다린다.

    180도 회전
    ---------
    /turn_around=True를 받으면 현재 Nav2 Goal을 취소하고,
    /robot3/odom 누적 yaw로 180도 회전을 확인한다.
    정상 완료 시 /turn_complete에 Bool(data=True)를 한 번 발행한다.
    완료 또는 중단 후 내부 회전 상태를 False로 초기화하므로,
    다음 /turn_around=True 명령으로 다시 회전할 수 있다.
    """

    # ================================================================
    # 토픽 및 Action 이름
    # ================================================================

    MISSION_TOPIC = "/robot3/mission_cmd"
    MISSION_COMPLETE_TOPIC = "/robot3/mission_complete"

    TRACKING_TOPIC = "/tracking_web"
    CENTER_PIXEL_TOPIC = "/tracking_web_center_pixel"
    TARGET_DEPTH_TOPIC = "/target_depth"

    TURN_AROUND_TOPIC = "/turn_around"
    TURN_COMPLETE_TOPIC = "/turn_complete"
    FRONT_START_TOPIC = "/front_start"
    SERVICE_END_TOPIC = "/service_end"
    START_FRONT_TOPIC = "/start_front"
    START_PATROL_TOPIC = "/start_patrol"
    ODOM_TOPIC = "/robot3/odom"

    CMD_VEL_TOPIC = "/robot3/cmd_vel"
    NAV2_ACTION_NAME = "/robot3/navigate_to_pose"

    # ================================================================
    # 카메라 및 중앙정렬 범위
    # ================================================================

    CAMERA_WIDTH = 640.0
    CAMERA_HEIGHT = 480.0

    CAMERA_CENTER_X = CAMERA_WIDTH / 2.0  # 320

    # 화면 중심을 기준으로 좌우 각각 화면 폭의 30%까지 허용한다.
    # 따라서 전체 중앙 허용 영역의 폭은 화면 폭의 60%이다.
    CENTER_ZONE_RATIO = 0.50

    CENTER_HALF_WIDTH = (
        CAMERA_WIDTH * CENTER_ZONE_RATIO / 2.0
    )

    # CAMERA_WIDTH=640일 때 128 ~ 512
    CENTER_MIN_X = CAMERA_CENTER_X - CENTER_HALF_WIDTH
    CENTER_MAX_X = CAMERA_CENTER_X + CENTER_HALF_WIDTH

    # ================================================================
    # 추적 및 거리 조건
    # ================================================================

    REQUIRED_TRACKING_STATE = 1

    MIN_TARGET_DEPTH = 0.0
    MAX_TARGET_DEPTH = 1.0

    # 각 토픽이 0.35초 동안 갱신되지 않으면 비정상 처리
    TOPIC_TIMEOUT_SEC = 0.35

    # ================================================================
    # 제어 주기
    # ================================================================

    CONTROL_PERIOD_SEC = 0.05  # 20Hz
    STATUS_LOG_PERIOD_SEC = 1.0

    # ================================================================
    # 중앙정렬 P 제어 설정
    # ================================================================

    ALIGN_KP = 0.50

    MIN_ANGULAR_SPEED = 0.35
    MAX_ANGULAR_SPEED = 0.45

    # 일반적인 비반전 카메라 기준:
    # 객체가 오른쪽이면 angular.z 음수로 오른쪽 회전
    # 객체가 왼쪽이면 angular.z 양수로 왼쪽 회전
    ALIGN_DIRECTION_SIGN = -1.0

    NAV_RETRY_DELAY_SEC = 1.0

    # ================================================================
    # 180도 회전 제어 설정
    # ================================================================

    # +1.0: 반시계 방향(왼쪽), -1.0: 시계 방향(오른쪽)
    TURN_DIRECTION = 1.0

    TURN_TARGET_ANGLE_RAD = math.pi
    TURN_TOLERANCE_RAD = math.radians(3.0)

    TURN_KP = 0.85
    TURN_MIN_ANGULAR_SPEED = 0.14
    TURN_MAX_ANGULAR_SPEED = 0.45

    ODOM_TIMEOUT_SEC = 0.50
    TURN_MAX_DURATION_SEC = 20.0

    # /turn_complete DDS Reliable ACK 설정
    # 각 발행마다 모든 매칭된 RELIABLE 구독자의 DDS ACK를 기다린다.
    TURN_COMPLETE_ACK_TIMEOUT_SEC = 1.0
    TURN_COMPLETE_MAX_PUBLISH_ATTEMPTS = 3

    def __init__(self):
        super().__init__("guarded_nav_to_goal")

        # ============================================================
        # 목적지 좌표
        # ============================================================

        self.goal_poses = {
            "detect_pose": {
                "x": -2.168346575735853,
                "y": 0.34778344933508026,

            },
            "goal1_1": {
                "x": -2.924,
                "y": 2.946,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.4544777,
                'qw': 0.890758,
            },
            "goal1_2": {
                "x": -3.97043,
                "y": 3.462406,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.862973,
                'qw': 0.5052491,
            },
            "goal2": {
                "x": -4.248990535736084,
                "y": -1.706390142440796,
                'qx': 0.0,
                'qy': 0.0,
                'qz': -0.7576175018036345,
                'qw': 0.6526987980384366,
            },
            "pre_dock": {
                "x": -0.43677089190051244,
                "y": 0.018926375739573814,
            },
        }

        # ============================================================
        # 전체 기능 잠금 상태
        # ============================================================

        # 초기 상태부터 일반 이동 기능은 잠금 상태로 시작한다.
        # SYSTEM_LOCKED 상태에서는 목적지 이동, 사람 추적, 중앙정렬에는
        # 관여하지 않지만 /turn_around=True에 의한 180도 회전은 허용한다.
        # /turn_complete=True를 수신하면 일반 이동 기능의 잠금을 해제하고,
        # /front_start=1 또는 /service_end=True를 수신하면 다시 잠근다.
        self.system_locked = True

        # /start_front와 /service_end의 최신 상태를 저장한다.
        # 두 조건이 동시에 성립하는 순간에만 /start_patrol=True를
        # 한 번 발행하기 위해 래치 상태도 함께 관리한다.
        self.start_front_value: Optional[int] = None
        self.service_end_value = False
        self.patrol_restart_condition_active = False

        # ============================================================
        # 최근 추적 데이터
        # ============================================================

        self.tracking_web: Optional[int] = None
        self.tracking_web_center_pixel: Optional[float] = None
        self.target_depth: Optional[float] = None

        # ============================================================
        # 각 토픽의 마지막 수신 시간
        # ============================================================

        self.tracking_web_received_time: Optional[float] = None
        self.center_pixel_received_time: Optional[float] = None
        self.target_depth_received_time: Optional[float] = None

        # ============================================================
        # 180도 회전 상태
        # ============================================================

        self.current_odom_yaw: Optional[float] = None
        self.odom_received_time: Optional[float] = None

        # True 명령을 수신했지만 아직 회전을 시작하지 못한 상태
        self.turn_request_pending = False

        # 실제 angular.z를 발행하며 회전 중인 상태
        self.turning = False

        # /turn_around=True 요청을 처리 중인지 나타내는 상태값이다.
        # 회전 완료 또는 중단 시 다시 False로 초기화되므로,
        # 별도의 /turn_around=False 없이 다음 True 명령을 받을 수 있다.
        self.turn_command_active = False

        self.turn_last_yaw: Optional[float] = None
        self.turn_accumulated_angle = 0.0
        self.turn_started_time: Optional[float] = None

        # ============================================================
        # 현재 목적지
        # ============================================================

        # None이면 목적지 명령을 기다리는 상태
        self.current_mission: Optional[str] = None

        # 목적지 명령이 새로 들어올 때마다 증가한다.
        # 과거 Action 결과와 최신 목적지를 구분하기 위해 사용한다.
        self.mission_revision = 0

        # ============================================================
        # Nav2 상태
        # ============================================================

        self.nav_goal_handle = None
        self.nav_goal_name: Optional[str] = None
        self.nav_goal_revision: Optional[int] = None

        self.goal_send_pending = False
        self.cancel_requested = False

        self.nav_retry_not_before = 0.0

        # ============================================================
        # 로그 상태
        # ============================================================

        self.control_state = "STARTING"
        self.last_status_log_time = 0.0
        self.last_nav_server_warning_time = 0.0

        # ============================================================
        # QoS
        # ============================================================

        tracking_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        mission_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ============================================================
        # Subscriber
        # ============================================================

        self.create_subscription(
            String,
            self.MISSION_TOPIC,
            self.mission_callback,
            mission_qos,
        )

        self.create_subscription(
            Int32,
            self.TRACKING_TOPIC,
            self.tracking_web_callback,
            tracking_qos,
        )

        self.create_subscription(
            Float32,
            self.CENTER_PIXEL_TOPIC,
            self.center_pixel_callback,
            tracking_qos,
        )

        self.create_subscription(
            Float32,
            self.TARGET_DEPTH_TOPIC,
            self.target_depth_callback,
            tracking_qos,
        )

        self.create_subscription(
            Bool,
            self.TURN_AROUND_TOPIC,
            self.turn_around_callback,
            mission_qos,
        )

        # back_move가 직접 발행한 메시지이든 외부 노드가 발행한 메시지이든
        # /turn_complete=True를 수신하면 일반 이동 기능의 Lock을 해제한다.
        self.create_subscription(
            Bool,
            self.TURN_COMPLETE_TOPIC,
            self.turn_complete_callback,
            mission_qos,
        )

        self.create_subscription(
            Int32,
            self.FRONT_START_TOPIC,
            self.front_start_callback,
            mission_qos,
        )

        # 서비스 종료 신호를 받으면 일반 후방 이동 기능을 다시 잠근다.
        self.create_subscription(
            Bool,
            self.SERVICE_END_TOPIC,
            self.service_end_callback,
            mission_qos,
        )

        # /start_front의 최신 값을 저장하고 /service_end와 조합해
        # 순찰 재시작 조건을 판정한다.
        self.create_subscription(
            Int32,
            self.START_FRONT_TOPIC,
            self.start_front_callback,
            mission_qos,
        )

        self.create_subscription(
            Odometry,
            self.ODOM_TOPIC,
            self.odom_callback,
            tracking_qos,
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.CMD_VEL_TOPIC,
            10,
        )

        # 180도 회전이 정상 완료된 경우 Bool(True)를 RELIABLE QoS로
        # 발행하고, wait_for_all_acked()로 DDS ACK 완료를 확인한다.
        turn_complete_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.turn_complete_publisher = self.create_publisher(
            Bool,
            self.TURN_COMPLETE_TOPIC,
            turn_complete_qos,
        )

        # /start_front=0과 /service_end=True가 동시에 성립하면
        # 순찰 노드 시작 신호를 한 번 발행한다.
        self.start_patrol_publisher = self.create_publisher(
            Bool,
            self.START_PATROL_TOPIC,
            10,
        )

        # Nav2가 최신 목적지에 정상 도착했을 때 Bool(True)를 한 번 발행한다.
        # 기본 ROS2 구독자와 호환되도록 RELIABLE + VOLATILE QoS를 사용한다.
        mission_complete_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.mission_complete_publisher = self.create_publisher(
            Bool,
            self.MISSION_COMPLETE_TOPIC,
            mission_complete_qos,
        )

        # ============================================================
        # Nav2 Action Client
        # ============================================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.NAV2_ACTION_NAME,
        )

        # ============================================================
        # 실시간 제어 타이머
        # ============================================================

        self.control_timer = self.create_timer(
            self.CONTROL_PERIOD_SEC,
            self.control_loop,
        )

        # 초기 SYSTEM_LOCKED 상태에서는 다른 노드의 제어를 방해하지 않도록
        # 시작 시 /robot3/cmd_vel 정지 명령을 발행하지 않는다.
        if not self.system_locked:
            self.publish_stop()

        self.get_logger().info("Guarded Nav2 노드 시작")
        self.get_logger().info(
            f"목적지 명령: {self.MISSION_TOPIC}"
        )
        self.get_logger().info(
            f"목적지 도착 완료 발행: {self.MISSION_COMPLETE_TOPIC}=True"
        )
        self.get_logger().info(
            f"추적 상태: {self.TRACKING_TOPIC}"
        )
        self.get_logger().info(
            f"타겟 중심: {self.CENTER_PIXEL_TOPIC}"
        )
        self.get_logger().info(
            f"타겟 거리: {self.TARGET_DEPTH_TOPIC}"
        )
        self.get_logger().info(
            f"180도 회전 명령: {self.TURN_AROUND_TOPIC}"
        )
        self.get_logger().info(
            f"180도 회전 완료: {self.TURN_COMPLETE_TOPIC}=True"
        )
        self.get_logger().info(
            f"일반 이동 Lock 해제: {self.TURN_COMPLETE_TOPIC}=True"
        )
        self.get_logger().info(
            f"초기 일반 이동 잠금: system_locked={self.system_locked}"
        )
        self.get_logger().info(
            f"일반 이동 잠금 명령: {self.FRONT_START_TOPIC}=1"
        )
        self.get_logger().info(
            f"서비스 종료 잠금 명령: {self.SERVICE_END_TOPIC}=True"
        )
        self.get_logger().info(
            f"순찰 복귀 조건: {self.START_FRONT_TOPIC}=0 AND "
            f"{self.SERVICE_END_TOPIC}=True"
        )
        self.get_logger().info(
            f"순찰 시작 발행: {self.START_PATROL_TOPIC}=True"
        )
        self.get_logger().info(
            "SYSTEM_LOCKED 상태에서도 /turn_around=True 회전은 허용"
        )
        self.get_logger().info(
            f"회전각 측정 odom: {self.ODOM_TOPIC}"
        )
        self.get_logger().info(
            f"Nav2 Action: {self.NAV2_ACTION_NAME}"
        )
        self.get_logger().info(
            f"cmd_vel: {self.CMD_VEL_TOPIC}"
        )
        self.get_logger().info(
            f"중앙 허용 범위: "
            f"{self.CENTER_MIN_X:.1f} ~ "
            f"{self.CENTER_MAX_X:.1f}"
        )
        self.get_logger().info(
            f"이동 허용 거리: "
            f"{self.MIN_TARGET_DEPTH:.1f} < depth "
            f"<= {self.MAX_TARGET_DEPTH:.1f}m"
        )

    # ================================================================
    # 토픽 콜백
    # ================================================================

    def mission_callback(self, msg: String):
        """
        목적지 문자열을 수신한다.

        목적지를 수신하더라도 즉시 이동하지 않는다.
        control_loop에서 추적, 거리, 중앙정렬 상태를 확인한 후
        모든 조건이 정상일 때만 Nav2 Goal을 전송한다.
        """
        if self.system_locked:
            self.get_logger().warning(
                "SYSTEM_LOCKED: 목적지 명령 무시"
            )
            return

        mission_name = msg.data.strip()

        if mission_name not in self.goal_poses:
            self.get_logger().warning(
                f"등록되지 않은 목적지 명령: '{mission_name}'"
            )
            return

        # 동일한 목적지를 이미 처리 중이면 중복 전송을 무시한다.
        if mission_name == self.current_mission:
            self.get_logger().info(
                f"이미 처리 중인 목적지: {mission_name}"
            )
            return

        previous_mission = self.current_mission

        self.current_mission = mission_name
        self.mission_revision += 1

        goal = self.goal_poses[mission_name]

        self.get_logger().info(
            f"목적지 수신: {mission_name}, "
            f"x={goal['x']:.4f}, "
            f"y={goal['y']:.4f}"
        )

        # 이전 목적지가 실행 중이면 취소한다.
        if (
            previous_mission is not None
            and self.nav_goal_handle is not None
        ):
            self.request_nav_cancel(
                reason=(
                    f"목적지 변경: "
                    f"{previous_mission} -> {mission_name}"
                )
            )

            self.publish_stop()

    def tracking_web_callback(self, msg: Int32):
        """현재 타겟 추적 상태를 저장한다."""
        if self.system_locked:
            return

        self.tracking_web = int(msg.data)
        self.tracking_web_received_time = time.monotonic()

    def center_pixel_callback(self, msg: Float32):
        """타겟 바운딩 박스의 x축 중심 좌표를 저장한다."""
        if self.system_locked:
            return

        self.tracking_web_center_pixel = float(msg.data)
        self.center_pixel_received_time = time.monotonic()

    def target_depth_callback(self, msg: Float32):
        """현재 타겟까지의 거리값을 저장한다."""
        if self.system_locked:
            return

        self.target_depth = float(msg.data)
        self.target_depth_received_time = time.monotonic()

    def front_start_callback(self, msg: Int32):
        """
        /front_start에서 Int32(data=1)을 받으면 일반 이동 기능을 잠근다.

        180도 회전은 SYSTEM_LOCKED 상태에서도 예외적으로 허용된다.

        잠금 전환 동작
        -------------
        1. 이 노드가 실행 중이던 Nav2 Goal을 한 번 취소한다.
        2. 현재 목적지를 제거한다.
        3. 제어권 인계 전에 /robot3/cmd_vel 정지 명령을 한 번 발행한다.
        4. 이후 일반 목적지 이동, 추적, 중앙정렬에는 관여하지 않는다.
        5. 단, /turn_around=True를 받으면 180도 회전은 수행한다.

        즉, 잠금 상태에서는 평상시 일반 이동 제어권을 포기하지만,
        명시적인 180도 회전 요청 동안에는 일시적으로 cmd_vel을 발행한다.

        data=0은 잠금을 해제하지 않는다. 잠금 해제는
        /turn_complete=True 메시지로만 수행한다.
        """
        if int(msg.data) != 1:
            return

        if self.system_locked:
            return

        self.system_locked = True

        # 비동기 Goal 결과가 잠금 전에 생성된 목적지와 섞이지 않도록
        # revision을 증가시키고 현재 목적지를 제거한다.
        self.mission_revision += 1
        self.current_mission = None
        self.nav_retry_not_before = 0.0

        # SYSTEM_LOCKED 상태에서도 180도 회전은 허용하므로,
        # 진행 중인 회전 요청과 회전 상태는 폐기하지 않는다.

        self.get_logger().warning(
            f"{self.FRONT_START_TOPIC}=1 수신: "
            "일반 이동 기능 SYSTEM_LOCKED"
        )

        # 제어권 인계 시점에 이 노드가 보유하던 일반 이동만 정리한다.
        # 이후에는 명시적인 180도 회전 요청이 있을 때만 cmd_vel을 발행한다.
        self.request_nav_cancel(
            reason=f"{self.FRONT_START_TOPIC}=1 제어권 인계"
        )
        self.publish_stop()

        self.set_control_state(
            "SYSTEM_LOCKED",
            "/front_start=1, 일반 이동 제어권 포기(180도 회전 예외)",
        )

    def service_end_callback(self, msg: Bool):
        """
        /service_end의 최신 Bool 값을 저장한다.

        True이면 기존 동작과 동일하게 일반 후방 이동 기능을 잠근다.
        추가로 /start_front의 최신 값이 0이면 초기 잠금 상태로
        완전히 복귀한 뒤 /start_patrol=True를 한 번 발행한다.

        False가 들어오면 순찰 재시작 조합 조건을 해제하여,
        다음 True 이벤트에서 다시 한 번 발행할 수 있게 한다.
        """
        self.service_end_value = bool(msg.data)

        # 두 토픽의 도착 순서와 관계없이 최신 값 조합으로 판정한다.
        self.check_patrol_restart_condition()

        if not msg.data:
            return

        if self.system_locked:
            self.get_logger().info(
                f"{self.SERVICE_END_TOPIC}=True 수신: "
                "이미 SYSTEM_LOCKED 상태"
            )
            return

        self.system_locked = True

        # 잠금 전의 비동기 Nav2 결과가 이후 상태와 섞이지 않도록
        # revision을 증가시키고 현재 목적지를 제거한다.
        self.mission_revision += 1
        self.current_mission = None
        self.nav_retry_not_before = 0.0

        self.get_logger().warning(
            f"{self.SERVICE_END_TOPIC}=True 수신: "
            "일반 이동 기능 SYSTEM_LOCKED"
        )

        # 진행 중이던 일반 이동을 정리하고 제어권을 포기한다.
        self.request_nav_cancel(
            reason=f"{self.SERVICE_END_TOPIC}=True 서비스 종료"
        )
        self.publish_stop()

        self.set_control_state(
            "SYSTEM_LOCKED",
            "/service_end=True, 일반 이동 제어권 포기(180도 회전 예외)",
        )

    def start_front_callback(self, msg: Int32):
        """
        /start_front의 최신 Int32 값을 저장한다.

        값이 0이고 이미 /service_end=True가 수신된 상태라면
        초기 잠금 상태로 복귀하고 /start_patrol=True를 발행한다.
        두 토픽의 수신 순서는 상관없다.
        """
        self.start_front_value = int(msg.data)

        self.get_logger().info(
            f"{self.START_FRONT_TOPIC}={self.start_front_value} 수신"
        )

        self.check_patrol_restart_condition()

    def check_patrol_restart_condition(self):
        """
        /start_front=0 AND /service_end=True 조건을 검사한다.

        조건이 False에서 True로 바뀌는 순간에만 초기화 및
        /start_patrol=True 발행을 수행한다. 같은 값이 반복 수신되어도
        중복 발행하지 않으며, 어느 한 조건이 해제된 뒤 다시 성립하면
        다음 사이클에서 다시 한 번 발행할 수 있다.
        """
        condition_met = (
            self.start_front_value == 0
            and self.service_end_value is True
        )

        if not condition_met:
            self.patrol_restart_condition_active = False
            return

        if self.patrol_restart_condition_active:
            return

        self.patrol_restart_condition_active = True

        self.reset_to_initial_locked_state(
            reason=(
                f"{self.START_FRONT_TOPIC}=0 AND "
                f"{self.SERVICE_END_TOPIC}=True"
            )
        )

        start_patrol_msg = Bool()
        start_patrol_msg.data = True
        self.start_patrol_publisher.publish(start_patrol_msg)

        self.get_logger().warning(
            f"{self.START_FRONT_TOPIC}=0, "
            f"{self.SERVICE_END_TOPIC}=True: "
            "초기 SYSTEM_LOCKED 상태 복귀 및 "
            f"{self.START_PATROL_TOPIC}=True 발행"
        )

    def reset_to_initial_locked_state(self, reason: str):
        """
        현재 작업 상태를 정리하고 노드 시작 직후와 같은 잠금 상태로 복귀한다.

        순찰 노드로 제어권을 넘기기 전에 이 노드가 보유한 Nav2 Goal과
        직접 속도 제어 상태를 제거하며, 잠금 중에는 이후 cmd_vel을
        반복 발행하지 않는다.
        """
        self.system_locked = True

        self.mission_revision += 1
        self.current_mission = None
        self.nav_retry_not_before = 0.0

        self.tracking_web = None
        self.tracking_web_center_pixel = None
        self.target_depth = None

        self.tracking_web_received_time = None
        self.center_pixel_received_time = None
        self.target_depth_received_time = None

        # 초기 상태 복귀이므로 진행 중이던 180도 회전도 완료 처리하지 않고
        # 중단한다. /turn_complete=True도 발행하지 않는다.
        self.turn_request_pending = False
        self.turning = False
        self.turn_command_active = False
        self.turn_last_yaw = None
        self.turn_accumulated_angle = 0.0
        self.turn_started_time = None

        self.request_nav_cancel(
            reason=f"초기 잠금 상태 복귀: {reason}"
        )
        self.publish_stop()

        self.set_control_state(
            "SYSTEM_LOCKED",
            f"{reason}, 초기 상태 복귀 및 후방 제어권 포기",
        )

    def turn_complete_callback(self, msg: Bool):
        """
        /turn_complete=True를 받으면 일반 목적지 이동 Lock을 해제한다.

        이 토픽은 180도 회전이 정상 완료됐을 때 back_move가 직접 발행하며,
        동일 토픽을 이 노드가 다시 수신하여 후방 이동 기능을 활성화한다.
        False 메시지는 상태 변경 명령으로 사용하지 않고 무시한다.
        """
        if not msg.data:
            return

        if not self.system_locked:
            self.get_logger().info(
                f"{self.TURN_COMPLETE_TOPIC}=True 수신: "
                "이미 SYSTEM_UNLOCKED 상태"
            )
            return

        self.system_locked = False

        # 잠금 상태에서는 추적 콜백을 무시하므로, 해제 이후 들어오는
        # 새로운 tracking/depth/center 데이터만 이동 판단에 사용한다.
        self.tracking_web = None
        self.tracking_web_center_pixel = None
        self.target_depth = None

        self.tracking_web_received_time = None
        self.center_pixel_received_time = None
        self.target_depth_received_time = None

        self.nav_retry_not_before = 0.0

        self.get_logger().warning(
            f"{self.TURN_COMPLETE_TOPIC}=True 수신: "
            "일반 이동 기능 SYSTEM_UNLOCKED"
        )

        self.set_control_state(
            "WAIT_MISSION",
            "/turn_complete=True, 후방 목적지 이동 활성화",
        )

    def turn_around_callback(self, msg: Bool):
        """
        /turn_around=True를 받으면 180도 회전을 한 번 요청한다.

        회전 요청이 대기 중이거나 실제 회전 중일 때 들어오는 추가 True는
        중복 실행을 막기 위해 무시한다. 회전이 완료되거나 중단되면
        turn_command_active를 다시 False로 초기화하므로,
        다음 /turn_around=True 명령으로 다시 회전할 수 있다.

        따라서 /turn_around=False를 별도로 발행할 필요가 없다.
        """
        # SYSTEM_LOCKED 상태에서도 180도 회전 명령은 예외적으로 허용한다.
        # False 메시지는 동작 명령이 아니므로 무시한다.
        if not msg.data:
            return

        # 이미 한 번의 회전 요청을 처리 중이면 중복 요청을 무시한다.
        if self.turn_command_active:
            self.get_logger().info(
                "/turn_around=True 중복 수신: 현재 회전 요청 처리 중"
            )
            return

        # 이번 True 명령을 접수하고, 완료 또는 중단 전까지 잠근다.
        self.turn_command_active = True
        self.turn_request_pending = True

        self.get_logger().warning(
            "/turn_around=True 수신: Nav2 정지 후 180도 회전 시작"
        )

        # 회전 명령은 일반 이동 조건보다 우선한다.
        self.request_nav_cancel(
            reason="/turn_around=True 수신"
        )
        self.publish_stop()

    def odom_callback(self, msg: Odometry):
        """
        Odometry quaternion에서 yaw를 계산하고 회전 누적각을 갱신한다.

        SYSTEM_LOCKED 상태에서도 180도 회전을 수행해야 하므로 odom은
        계속 수신한다. 다만 실제 누적각 계산은 turning=True일 때만 수행한다.
        """
        orientation = msg.pose.pose.orientation

        yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        self.current_odom_yaw = yaw
        self.odom_received_time = time.monotonic()

        if not self.turning:
            return

        if self.turn_last_yaw is None:
            self.turn_last_yaw = yaw
            return

        delta_yaw = self.normalize_angle(
            yaw - self.turn_last_yaw
        )

        # 지정한 회전 방향 성분만 누적한다.
        directed_delta = self.TURN_DIRECTION * delta_yaw

        self.turn_accumulated_angle = max(
            0.0,
            self.turn_accumulated_angle + directed_delta,
        )

        self.turn_last_yaw = yaw

    # ================================================================
    # 메인 제어 루프
    # ================================================================

    def control_loop(self):
        """
        20Hz로 추적 상태, 중앙좌표, 거리, 목적지 상태를 계속 검사한다.

        중앙정렬은 target_depth 조건보다 우선한다. 따라서 타겟이
        이동 허용 거리 밖에 있거나 depth 토픽이 아직 정상 범위가 아니어도,
        tracking_web과 center_pixel이 정상이라면 먼저 제자리 중앙정렬한다.
        Nav2 목적지 이동은 중앙정렬 완료 후 거리 조건까지 만족할 때만 수행한다.
        """
        now = time.monotonic()

        # /turn_around 요청과 회전 동작은 SYSTEM_LOCKED 여부와 관계없이
        # 목적지, tracking, depth, 중앙정렬 조건보다 항상 우선한다.
        if self.turn_request_pending or self.turning:
            self.handle_turn_around(now)
            return

        # 잠금 상태에서는 일반 이동 제어권만 포기한다.
        # 단, 위에서 처리한 180도 회전은 잠금 상태에서도 허용한다.
        if self.system_locked:
            return

        tracking_fresh = self.is_topic_fresh(
            self.tracking_web_received_time,
            now,
        )

        depth_fresh = self.is_topic_fresh(
            self.target_depth_received_time,
            now,
        )

        center_fresh = self.is_topic_fresh(
            self.center_pixel_received_time,
            now,
        )

        tracking_ok = (
            tracking_fresh
            and self.tracking_web == self.REQUIRED_TRACKING_STATE
        )

        depth_ok = (
            depth_fresh
            and self.target_depth is not None
            and math.isfinite(self.target_depth)
            and self.MIN_TARGET_DEPTH < self.target_depth
            and self.target_depth <= self.MAX_TARGET_DEPTH
        )

        center_data_ok = (
            center_fresh
            and self.tracking_web_center_pixel is not None
            and math.isfinite(
                self.tracking_web_center_pixel
            )
            and 0.0 <= self.tracking_web_center_pixel
            and self.tracking_web_center_pixel <= self.CAMERA_WIDTH
        )

        centered = (
            center_data_ok
            and self.CENTER_MIN_X
            <= self.tracking_web_center_pixel
            <= self.CENTER_MAX_X
        )

        self.print_status_periodically(
            now=now,
            tracking_fresh=tracking_fresh,
            depth_fresh=depth_fresh,
            center_fresh=center_fresh,
            tracking_ok=tracking_ok,
            depth_ok=depth_ok,
            centered=centered,
        )

        # ------------------------------------------------------------
        # 목적지가 없으면 다음 목적지 명령을 기다린다.
        # 현재 구조에서는 목적지 명령이 있어야 중앙정렬 및 이동을 수행한다.
        # ------------------------------------------------------------

        if self.current_mission is None:
            self.stop_navigation_and_robot(
                state="WAIT_MISSION",
                reason="다음 목적지 명령 대기",
            )
            return

        # ------------------------------------------------------------
        # tracking_web 실시간 검사
        # ------------------------------------------------------------

        if not tracking_fresh:
            self.stop_navigation_and_robot(
                state="TRACKING_TIMEOUT",
                reason="/tracking_web 수신 타임아웃",
            )
            return

        if self.tracking_web != self.REQUIRED_TRACKING_STATE:
            self.stop_navigation_and_robot(
                state="TRACKING_INVALID",
                reason=(
                    f"/tracking_web={self.tracking_web}, "
                    f"필요값={self.REQUIRED_TRACKING_STATE}"
                ),
            )
            return

        # ------------------------------------------------------------
        # center_pixel 실시간 검사
        # 중앙정렬은 target_depth 검사보다 먼저 수행한다.
        # ------------------------------------------------------------

        if not center_fresh:
            self.stop_navigation_and_robot(
                state="CENTER_TIMEOUT",
                reason=(
                    "/tracking_web_center_pixel "
                    "수신 타임아웃"
                ),
            )
            return

        if not center_data_ok:
            self.stop_navigation_and_robot(
                state="CENTER_INVALID",
                reason=(
                    "/tracking_web_center_pixel 값이 "
                    "유효한 영상 좌표가 아님"
                ),
            )
            return

        # ------------------------------------------------------------
        # 중앙에서 벗어난 경우
        # depth가 허용 범위 밖이어도 중앙정렬은 수행한다.
        # ------------------------------------------------------------

        if not centered:
            # Nav2가 이동 중이라면 먼저 Goal을 취소한다.
            if (
                self.nav_goal_handle is not None
                or self.goal_send_pending
            ):
                self.request_nav_cancel(
                    reason="타겟 중앙 이탈"
                )

                self.publish_stop()

                self.set_control_state(
                    "CANCELING_FOR_ALIGNMENT",
                    (
                        f"Nav2 취소 후 중앙정렬, "
                        f"center="
                        f"{self.tracking_web_center_pixel:.1f}, "
                        f"depth={self.target_depth}"
                    ),
                )
                return

            # 취소 결과를 기다리는 동안에는 회전하지 않는다.
            if self.cancel_requested:
                self.publish_stop()

                self.set_control_state(
                    "WAIT_NAV_CANCEL",
                    "Nav2 Goal 취소 완료 대기",
                )
                return

            # Nav2가 완전히 종료된 후 angular.z만 사용한다.
            # 이 지점에서는 target_depth의 범위나 freshness를 검사하지 않는다.
            self.align_target_to_center()
            return

        # ------------------------------------------------------------
        # 중앙정렬 완료 후 target_depth 실시간 검사
        # 거리 조건은 Nav2 목적지 이동 허용 여부에만 사용한다.
        # ------------------------------------------------------------

        if not depth_fresh:
            self.stop_navigation_and_robot(
                state="DEPTH_TIMEOUT",
                reason=(
                    "/target_depth 수신 타임아웃, "
                    "중앙정렬은 완료됐지만 Nav2 이동은 정지"
                ),
            )
            return

        if (
            self.target_depth is None
            or not math.isfinite(self.target_depth)
        ):
            self.stop_navigation_and_robot(
                state="DEPTH_INVALID",
                reason=(
                    "/target_depth 값이 유효하지 않음, "
                    "중앙정렬은 완료됐지만 Nav2 이동은 정지"
                ),
            )
            return

        if not (
            self.MIN_TARGET_DEPTH
            < self.target_depth
            <= self.MAX_TARGET_DEPTH
        ):
            self.stop_navigation_and_robot(
                state="DEPTH_OUT_OF_RANGE",
                reason=(
                    f"/target_depth={self.target_depth:.3f}m, "
                    f"허용값=0.0 초과 "
                    f"{self.MAX_TARGET_DEPTH:.1f} 이하, "
                    "중앙정렬 완료 후 거리 조건 대기"
                ),
            )
            return

        # ------------------------------------------------------------
        # 중앙정렬과 거리 조건이 모두 정상인 상태
        # ------------------------------------------------------------

        if self.nav_goal_handle is not None:
            # Nav2 이동 중에는 cmd_vel을 직접 발행하지 않는다.
            self.set_control_state(
                "NAVIGATING",
                (
                    f"{self.current_mission} 이동 중, "
                    f"depth={self.target_depth:.3f}m, "
                    f"center="
                    f"{self.tracking_web_center_pixel:.1f}"
                ),
            )
            return

        if self.goal_send_pending:
            self.publish_stop()

            self.set_control_state(
                "GOAL_SEND_PENDING",
                f"{self.current_mission} Goal 응답 대기",
            )
            return

        if self.cancel_requested:
            self.publish_stop()

            self.set_control_state(
                "WAIT_NAV_CANCEL",
                "Nav2 Goal 취소 결과 대기",
            )
            return

        if now < self.nav_retry_not_before:
            self.publish_stop()

            self.set_control_state(
                "WAIT_NAV_RETRY",
                "Nav2 Goal 재전송 대기",
            )
            return

        # 중앙정렬 때 발생했던 회전 명령을 0으로 만든 후
        # Nav2 Goal을 전송한다.
        self.publish_stop()
        self.send_current_nav_goal()

    # ================================================================
    # 180도 회전
    # ================================================================

    def handle_turn_around(self, now: float):
        """
        /turn_around 요청을 최우선으로 처리한다.

        Nav2 Goal이 남아 있으면 먼저 취소하고, Action이 완전히 종료된 뒤
        /robot3/cmd_vel의 angular.z만 발행한다. 완료 판정은
        /robot3/odom에서 계산한 누적 yaw 변화량을 사용한다.
        """
        if self.turn_request_pending and not self.turning:
            # Goal 전송 응답을 기다리는 중이면 응답이 도착할 때까지 정지한다.
            if self.goal_send_pending:
                self.publish_stop()
                self.set_control_state(
                    "WAIT_TURN_GOAL_RESPONSE",
                    "Nav2 Goal 전송 응답 후 취소 대기",
                )
                return

            # 활성 Nav2 Goal이 있으면 먼저 취소한다.
            if self.nav_goal_handle is not None:
                self.request_nav_cancel(
                    reason="180도 회전 요청"
                )
                self.publish_stop()
                self.set_control_state(
                    "CANCELING_FOR_TURN",
                    "Nav2 Goal 취소 후 180도 회전",
                )
                return

            # 취소 결과가 완전히 처리될 때까지 직접 회전하지 않는다.
            if self.cancel_requested:
                self.publish_stop()
                self.set_control_state(
                    "WAIT_NAV_CANCEL_FOR_TURN",
                    "Nav2 Goal 취소 완료 대기",
                )
                return

            # 실제 회전각 판정을 위해 최신 odometry가 필요하다.
            if not self.is_topic_fresh(
                self.odom_received_time,
                now,
                timeout_sec=self.ODOM_TIMEOUT_SEC,
            ):
                self.publish_stop()
                self.set_control_state(
                    "WAIT_ODOM_FOR_TURN",
                    f"{self.ODOM_TOPIC} 수신 대기",
                )
                return

            if self.current_odom_yaw is None:
                self.publish_stop()
                self.set_control_state(
                    "WAIT_ODOM_FOR_TURN",
                    f"{self.ODOM_TOPIC} yaw 대기",
                )
                return

            self.start_turn_around(now)

        if not self.turning:
            return

        if self.turn_started_time is not None:
            elapsed = now - self.turn_started_time

            if elapsed > self.TURN_MAX_DURATION_SEC:
                self.abort_turn_around(
                    "180도 회전 제한시간 초과"
                )
                return

        if not self.is_topic_fresh(
            self.odom_received_time,
            now,
            timeout_sec=self.ODOM_TIMEOUT_SEC,
        ):
            self.publish_stop()
            self.set_control_state(
                "TURN_ODOM_TIMEOUT",
                f"{self.ODOM_TOPIC} 갱신 중단으로 회전 정지",
            )
            return

        remaining_angle = (
            self.TURN_TARGET_ANGLE_RAD
            - self.turn_accumulated_angle
        )

        if remaining_angle <= self.TURN_TOLERANCE_RAD:
            self.finish_turn_around()
            return

        angular_speed = self.clamp(
            self.TURN_KP * remaining_angle,
            self.TURN_MIN_ANGULAR_SPEED,
            self.TURN_MAX_ANGULAR_SPEED,
        )

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = (
            self.TURN_DIRECTION * angular_speed
        )

        self.cmd_vel_publisher.publish(twist)

        self.set_control_state(
            "TURNING_180",
            (
                f"누적={math.degrees(self.turn_accumulated_angle):.1f}도, "
                f"남음={math.degrees(remaining_angle):.1f}도, "
                f"angular.z={twist.angular.z:.3f}"
            ),
        )

        if (
            now - self.last_status_log_time
            >= self.STATUS_LOG_PERIOD_SEC
        ):
            self.last_status_log_time = now

            self.get_logger().info(
                "[TURN MONITOR] "
                f"누적={math.degrees(self.turn_accumulated_angle):.1f}도, "
                f"남음={math.degrees(remaining_angle):.1f}도, "
                f"angular.z={twist.angular.z:.3f}"
            )

    def start_turn_around(self, now: float):
        """최신 odometry yaw를 시작점으로 180도 회전을 시작한다."""
        self.turn_request_pending = False
        self.turning = True

        self.turn_last_yaw = self.current_odom_yaw
        self.turn_accumulated_angle = 0.0
        self.turn_started_time = now

        self.publish_stop()

        direction_text = (
            "반시계 방향"
            if self.TURN_DIRECTION > 0.0
            else "시계 방향"
        )

        self.get_logger().warning(
            f"180도 회전 시작: {direction_text}"
        )

        self.set_control_state(
            "TURNING_180",
            f"180도 {direction_text} 회전 시작",
        )

    def finish_turn_around(self):
        """
        180도 회전을 정지하고 /turn_complete=True를 발행한다.

        발행 후 모든 매칭된 RELIABLE 구독자의 DDS ACK를 기다린다.
        제한시간 안에 ACK가 완료되지 않으면 같은 Bool(True)를 다시 발행하며,
        최대 TURN_COMPLETE_MAX_PUBLISH_ATTEMPTS회까지 시도한다.
        """
        completed_angle_deg = math.degrees(
            self.turn_accumulated_angle
        )

        # 회전 속도를 먼저 0으로 만든 뒤 완료 신호를 발행한다.
        self.publish_stop()

        self.turning = False
        self.turn_request_pending = False
        self.turn_command_active = False
        self.turn_last_yaw = None
        self.turn_started_time = None

        ack_received = self.publish_turn_complete_with_dds_ack()

        if ack_received:
            self.get_logger().info(
                f"180도 회전 완료: 누적={completed_angle_deg:.1f}도, "
                f"{self.TURN_COMPLETE_TOPIC}=True DDS ACK 완료"
            )

            self.set_control_state(
                "TURN_COMPLETED",
                "180도 회전 완료 및 /turn_complete=True DDS ACK 완료",
            )
        else:
            self.get_logger().error(
                f"180도 회전 완료: 누적={completed_angle_deg:.1f}도, "
                f"{self.TURN_COMPLETE_TOPIC}=True 발행은 수행했지만 "
                "DDS ACK 확인 실패"
            )

            self.set_control_state(
                "TURN_COMPLETE_ACK_TIMEOUT",
                "/turn_complete=True DDS ACK 제한시간 초과",
            )

        self.turn_accumulated_angle = 0.0

    def publish_turn_complete_with_dds_ack(self) -> bool:
        """
        /turn_complete=True를 RELIABLE QoS로 발행하고 DDS ACK를 기다린다.

        wait_for_all_acked()의 True는 현재 publisher와 매칭된 모든
        RELIABLE DataReader가 발행 데이터를 DDS 계층에서 확인했다는 뜻이다.
        애플리케이션 콜백의 실행 완료를 의미하지는 않는다.
        """
        complete_msg = Bool()
        complete_msg.data = True

        timeout = Duration(
            seconds=self.TURN_COMPLETE_ACK_TIMEOUT_SEC
        )

        for attempt in range(
            1,
            self.TURN_COMPLETE_MAX_PUBLISH_ATTEMPTS + 1,
        ):
            subscription_count = (
                self.turn_complete_publisher.get_subscription_count()
            )

            self.turn_complete_publisher.publish(complete_msg)

            self.get_logger().info(
                f"{self.TURN_COMPLETE_TOPIC}=True 발행 "
                f"({attempt}/{self.TURN_COMPLETE_MAX_PUBLISH_ATTEMPTS}), "
                f"매칭 구독자={subscription_count}, DDS ACK 대기"
            )

            try:
                acked = (
                    self.turn_complete_publisher.wait_for_all_acked(
                        timeout
                    )
                )
            except Exception as error:
                self.get_logger().error(
                    f"{self.TURN_COMPLETE_TOPIC} DDS ACK 대기 오류: "
                    f"{error}"
                )
                acked = False

            if acked:
                self.get_logger().info(
                    f"{self.TURN_COMPLETE_TOPIC}=True "
                    f"DDS ACK 확인 완료 ({attempt}회차)"
                )
                return True

            self.get_logger().warning(
                f"{self.TURN_COMPLETE_TOPIC}=True "
                f"DDS ACK 타임아웃 ({attempt}/"
                f"{self.TURN_COMPLETE_MAX_PUBLISH_ATTEMPTS})"
            )

        return False

    def abort_turn_around(self, reason: str):
        """안전상 회전을 중단하고 내부 회전 상태를 초기화한다."""
        self.publish_stop()

        self.turning = False
        self.turn_request_pending = False
        self.turn_command_active = False
        self.turn_last_yaw = None
        self.turn_started_time = None
        self.turn_accumulated_angle = 0.0

        self.get_logger().error(
            f"180도 회전 중단: {reason}"
        )

        self.set_control_state(
            "TURN_AROUND_ABORTED",
            reason,
        )

    # ================================================================
    # 중앙정렬
    # ================================================================

    def align_target_to_center(self):
        """
        타겟 중심이 CENTER_MIN_X~CENTER_MAX_X 범위 밖이면 제자리 회전한다.

        linear.x는 항상 0이다.
        화면 중심과의 픽셀 오차에 비례해 angular.z를 계산한다.
        """
        if self.tracking_web_center_pixel is None:
            self.publish_stop()
            return

        pixel_error = (
            self.tracking_web_center_pixel
            - self.CAMERA_CENTER_X
        )

        normalized_error = pixel_error / (
            self.CAMERA_WIDTH / 2.0
        )

        angular_speed = (
            self.ALIGN_DIRECTION_SIGN
            * self.ALIGN_KP
            * normalized_error
        )

        angular_speed = self.clamp(
            angular_speed,
            -self.MAX_ANGULAR_SPEED,
            self.MAX_ANGULAR_SPEED,
        )

        # 모터의 최소 구동 속도를 보장한다.
        if abs(angular_speed) < self.MIN_ANGULAR_SPEED:
            angular_speed = math.copysign(
                self.MIN_ANGULAR_SPEED,
                angular_speed,
            )

        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed

        self.cmd_vel_publisher.publish(twist)

        direction = (
            "왼쪽 회전"
            if angular_speed > 0.0
            else "오른쪽 회전"
        )

        self.set_control_state(
            "ALIGNING",
            (
                f"{direction}, "
                f"center="
                f"{self.tracking_web_center_pixel:.1f}, "
                f"error={pixel_error:.1f}, "
                f"angular.z={angular_speed:.3f}"
            ),
        )

    # ================================================================
    # Nav2 Goal 전송
    # ================================================================

    def send_current_nav_goal(self):
        """현재 목적지를 Nav2 NavigateToPose Action으로 전송한다."""
        if self.system_locked:
            self.publish_stop()
            return

        if self.current_mission is None:
            return

        if self.current_mission not in self.goal_poses:
            self.get_logger().error(
                f"목적지 좌표 없음: {self.current_mission}"
            )
            self.current_mission = None
            self.publish_stop()
            return

        if not self.nav_client.server_is_ready():
            now = time.monotonic()

            if (
                now - self.last_nav_server_warning_time
                >= self.STATUS_LOG_PERIOD_SEC
            ):
                self.get_logger().warning(
                    f"Nav2 Action Server 대기 중: "
                    f"{self.NAV2_ACTION_NAME}"
                )

                self.last_nav_server_warning_time = now

            self.set_control_state(
                "WAIT_NAV_SERVER",
                "Nav2 Action Server 연결 대기",
            )
            return

        goal_data = self.goal_poses[self.current_mission]

        goal_x = float(goal_data["x"])
        goal_y = float(goal_data["y"])

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0

        # qx, qy, qz, qw가 있으면 RViz에서 얻은 quaternion을 직접 사용한다.
        # quaternion이 없으면 yaw 값을 quaternion으로 변환한다.
        quaternion_keys = ("qx", "qy", "qz", "qw")

        if all(key in goal_data for key in quaternion_keys):
            goal_qx = float(goal_data["qx"])
            goal_qy = float(goal_data["qy"])
            goal_qz = float(goal_data["qz"])
            goal_qw = float(goal_data["qw"])
            orientation_text = (
                f"q=({goal_qx:.3f}, {goal_qy:.3f}, "
                f"{goal_qz:.3f}, {goal_qw:.3f})"
            )
        else:
            goal_yaw = float(goal_data.get("yaw", 0.0))
            goal_qx = 0.0
            goal_qy = 0.0
            goal_qz = math.sin(goal_yaw / 2.0)
            goal_qw = math.cos(goal_yaw / 2.0)
            orientation_text = f"yaw={goal_yaw:.3f}"

        goal_msg.pose.pose.orientation.x = goal_qx
        goal_msg.pose.pose.orientation.y = goal_qy
        goal_msg.pose.pose.orientation.z = goal_qz
        goal_msg.pose.pose.orientation.w = goal_qw

        mission_name = self.current_mission
        mission_revision = self.mission_revision

        self.goal_send_pending = True

        send_future = self.nav_client.send_goal_async(
            goal_msg
        )

        send_future.add_done_callback(
            lambda future: self.goal_response_callback(
                future,
                mission_name,
                mission_revision,
            )
        )

        self.set_control_state(
            "SENDING_GOAL",
            (
                f"{mission_name} Goal 전송, "
                f"x={goal_x:.4f}, "
                f"y={goal_y:.4f}, "
                f"{orientation_text}"
            ),
        )

    def goal_response_callback(
        self,
        future,
        mission_name: str,
        mission_revision: int,
    ):
        """Nav2가 Goal을 수락했는지 처리한다."""
        self.goal_send_pending = False

        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f"Nav2 Goal 전송 실패: {error}"
            )

            self.nav_retry_not_before = (
                time.monotonic()
                + self.NAV_RETRY_DELAY_SEC
            )
            return

        if not goal_handle.accepted:
            self.get_logger().warning(
                f"Nav2 Goal 거부: {mission_name}"
            )

            self.nav_retry_not_before = (
                time.monotonic()
                + self.NAV_RETRY_DELAY_SEC
            )
            return

        self.nav_goal_handle = goal_handle
        self.nav_goal_name = mission_name
        self.nav_goal_revision = mission_revision
        self.cancel_requested = False

        self.get_logger().info(
            f"Nav2 Goal 수락: {mission_name}"
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda result: self.goal_result_callback(
                result,
                mission_name,
                mission_revision,
            )
        )

        # Goal을 전송하고 수락받는 사이에 추적 조건이 깨졌거나
        # 목적지가 바뀌었을 수 있으므로 다시 검사한다.
        if (
            self.system_locked
            or mission_revision != self.mission_revision
            or mission_name != self.current_mission
            or self.turn_request_pending
            or self.turning
            or not self.navigation_conditions_are_valid()
        ):
            self.request_nav_cancel(
                reason="Goal 수락 시점에 이동 조건 불만족"
            )

            self.publish_stop()

    # ================================================================
    # Nav2 취소
    # ================================================================

    def request_nav_cancel(self, reason: str):
        """활성 Nav2 Goal 취소를 한 번만 요청한다."""
        if self.nav_goal_handle is None:
            return

        if self.cancel_requested:
            return

        self.cancel_requested = True

        self.get_logger().warning(
            f"Nav2 Goal 취소 요청: {reason}"
        )

        cancel_future = (
            self.nav_goal_handle.cancel_goal_async()
        )

        cancel_future.add_done_callback(
            self.cancel_response_callback
        )

    def cancel_response_callback(self, future):
        """Nav2 취소 요청 응답을 처리한다."""
        try:
            cancel_response = future.result()

        except Exception as error:
            self.get_logger().error(
                f"Nav2 Goal 취소 요청 실패: {error}"
            )

            self.cancel_requested = False
            return

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(
                "Nav2 Goal 취소 처리 중"
            )
        else:
            self.get_logger().warning(
                "Nav2에서 취소할 활성 Goal을 반환하지 않음"
            )

    # ================================================================
    # Nav2 결과
    # ================================================================

    def goal_result_callback(
        self,
        future,
        mission_name: str,
        mission_revision: int,
    ):
        """
        Nav2 Goal의 성공, 취소, 중단 결과를 처리한다.

        성공한 경우 current_mission을 None으로 초기화하여
        다음 목적지 명령 대기 상태로 돌아간다.
        """
        try:
            wrapped_result = future.result()
            status = wrapped_result.status

        except Exception as error:
            self.get_logger().error(
                f"Nav2 결과 수신 실패: {error}"
            )

            self.clear_nav_goal_state(
                mission_revision
            )

            self.nav_retry_not_before = (
                time.monotonic()
                + self.NAV_RETRY_DELAY_SEC
            )
            return

        self.clear_nav_goal_state(
            mission_revision
        )

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"목적지 도착 완료: {mission_name}"
            )

            # 현재 수행 중인 최신 목적지에 대한 성공 결과인 경우에만
            # 목적지를 비우고 다음 명령 대기 상태로 전환한다.
            if (
                mission_revision == self.mission_revision
                and mission_name == self.current_mission
            ):
                self.current_mission = None
                self.publish_stop()
                self.publish_mission_complete(mission_name)

                self.set_control_state(
                    "WAIT_MISSION",
                    (
                        f"{mission_name} 도착 완료, "
                        f"{self.MISSION_COMPLETE_TOPIC}=True 발행, "
                        "다음 목적지 명령 대기"
                    ),
                )

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(
                f"Nav2 Goal 취소 완료: {mission_name}"
            )

            # 추적 조건 때문에 취소된 경우 목적지는 유지한다.
            # 조건이 다시 정상화되면 동일 목적지로 재출발한다.

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warning(
                f"Nav2 Goal 중단: {mission_name}"
            )

            self.nav_retry_not_before = (
                time.monotonic()
                + self.NAV_RETRY_DELAY_SEC
            )

        else:
            self.get_logger().warning(
                f"Nav2 Goal 종료: "
                f"mission={mission_name}, "
                f"status={status}"
            )

            self.nav_retry_not_before = (
                time.monotonic()
                + self.NAV_RETRY_DELAY_SEC
            )

    def publish_mission_complete(self, mission_name: str):
        """최신 Nav2 목적지가 정상 도착했음을 Bool(True)로 한 번 알린다."""
        complete_msg = Bool()
        complete_msg.data = True
        self.mission_complete_publisher.publish(complete_msg)

        self.get_logger().info(
            f"목적지 도착 신호 발행: mission={mission_name}, "
            f"{self.MISSION_COMPLETE_TOPIC}=True"
        )

    def clear_nav_goal_state(
        self,
        mission_revision: int,
    ):
        """
        결과를 반환한 Goal이 현재 활성 Goal과 동일할 때만
        Nav2 상태를 초기화한다.
        """
        if self.nav_goal_revision != mission_revision:
            return

        self.nav_goal_handle = None
        self.nav_goal_name = None
        self.nav_goal_revision = None

        self.cancel_requested = False
        self.goal_send_pending = False

    # ================================================================
    # 정지
    # ================================================================

    def stop_navigation_and_robot(
        self,
        state: str,
        reason: str,
    ):
        """
        활성 Nav2 Goal이 있다면 취소하고 로봇을 정지한다.

        tracking_web 또는 target_depth가 비정상이면
        중앙정렬도 수행하지 않고 완전 정지한다.
        """
        self.request_nav_cancel(reason=reason)
        self.publish_stop()
        self.set_control_state(state, reason)

    def publish_stop(self):
        """선속도와 각속도를 모두 0으로 발행한다."""
        stop_msg = Twist()

        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0

        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(stop_msg)

    # ================================================================
    # 이동 조건 재검사
    # ================================================================

    def navigation_conditions_are_valid(self) -> bool:
        """
        현재 시점에서 Nav2 이동 조건이 모두 정상인지 검사한다.

        비동기 Goal 수락 응답이 도착했을 때 조건을 다시 확인하기 위해
        사용한다.
        """
        now = time.monotonic()

        if self.system_locked:
            return False

        if self.turn_request_pending or self.turning:
            return False

        if self.current_mission is None:
            return False

        if not self.is_topic_fresh(
            self.tracking_web_received_time,
            now,
        ):
            return False

        if self.tracking_web != self.REQUIRED_TRACKING_STATE:
            return False

        if not self.is_topic_fresh(
            self.target_depth_received_time,
            now,
        ):
            return False

        if (
            self.target_depth is None
            or not math.isfinite(self.target_depth)
        ):
            return False

        if not (
            self.MIN_TARGET_DEPTH
            < self.target_depth
            <= self.MAX_TARGET_DEPTH
        ):
            return False

        if not self.is_topic_fresh(
            self.center_pixel_received_time,
            now,
        ):
            return False

        if (
            self.tracking_web_center_pixel is None
            or not math.isfinite(
                self.tracking_web_center_pixel
            )
        ):
            return False

        if not (
            0.0
            <= self.tracking_web_center_pixel
            <= self.CAMERA_WIDTH
        ):
            return False

        if not (
            self.CENTER_MIN_X
            <= self.tracking_web_center_pixel
            <= self.CENTER_MAX_X
        ):
            return False

        return True

    def is_topic_fresh(
        self,
        received_time: Optional[float],
        now: float,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        """토픽이 지정한 제한 시간 이내에 수신됐는지 검사한다."""
        if received_time is None:
            return False

        timeout = (
            self.TOPIC_TIMEOUT_SEC
            if timeout_sec is None
            else timeout_sec
        )

        return now - received_time <= timeout

    # ================================================================
    # 로그
    # ================================================================

    def set_control_state(
        self,
        new_state: str,
        message: str,
    ):
        """상태가 변경됐을 때만 로그를 출력한다."""
        if new_state == self.control_state:
            return

        previous_state = self.control_state
        self.control_state = new_state

        self.get_logger().info(
            f"[{previous_state} -> {new_state}] "
            f"{message}"
        )

    def print_status_periodically(
        self,
        now: float,
        tracking_fresh: bool,
        depth_fresh: bool,
        center_fresh: bool,
        tracking_ok: bool,
        depth_ok: bool,
        centered: bool,
    ):
        """1초마다 현재 입력값과 상태를 출력한다."""
        if (
            now - self.last_status_log_time
            < self.STATUS_LOG_PERIOD_SEC
        ):
            return

        self.last_status_log_time = now

        tracking_age = self.topic_age(
            self.tracking_web_received_time,
            now,
        )

        depth_age = self.topic_age(
            self.target_depth_received_time,
            now,
        )

        center_age = self.topic_age(
            self.center_pixel_received_time,
            now,
        )

        tracking_text = (
            "None"
            if self.tracking_web is None
            else str(self.tracking_web)
        )

        depth_text = (
            "None"
            if self.target_depth is None
            else f"{self.target_depth:.3f}"
        )

        center_text = (
            "None"
            if self.tracking_web_center_pixel is None
            else f"{self.tracking_web_center_pixel:.1f}"
        )

        self.get_logger().info(
            "[MONITOR] "
            f"mission={self.current_mission}, "
            f"state={self.control_state}, "
            f"system_locked={self.system_locked}, "
            f"tracking={tracking_text}"
            f"(fresh={tracking_fresh}, age={tracking_age}), "
            f"depth={depth_text}"
            f"(fresh={depth_fresh}, age={depth_age}), "
            f"center={center_text}"
            f"(fresh={center_fresh}, age={center_age}), "
            f"tracking_ok={tracking_ok}, "
            f"depth_ok={depth_ok}, "
            f"centered={centered}, "
            f"nav_active="
            f"{self.nav_goal_handle is not None}, "
            f"goal_pending={self.goal_send_pending}, "
            f"canceling={self.cancel_requested}, "
            f"turn_active={self.turn_command_active}, "
            f"turn_pending={self.turn_request_pending}, "
            f"turning={self.turning}, "
            f"turn_progress="
            f"{math.degrees(self.turn_accumulated_angle):.1f}deg"
        )

    @staticmethod
    def topic_age(
        received_time: Optional[float],
        now: float,
    ) -> str:
        """토픽이 마지막으로 들어온 후 경과 시간을 반환한다."""
        if received_time is None:
            return "None"

        return f"{now - received_time:.3f}s"

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """각도를 -pi 이상 pi 미만 범위로 정규화한다."""
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    @staticmethod
    def quaternion_to_yaw(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> float:
        """Quaternion을 평면 이동 로봇의 yaw로 변환한다."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    @staticmethod
    def clamp(
        value: float,
        minimum: float,
        maximum: float,
    ) -> float:
        """값을 최솟값과 최댓값 사이로 제한한다."""
        return max(
            minimum,
            min(value, maximum),
        )

    # ================================================================
    # 종료 처리
    # ================================================================

    def shutdown(self):
        """노드 종료 시 현재 수행 중인 제어만 안전하게 정리한다."""
        if self.system_locked:
            # 잠금 상태에서도 180도 회전은 가능하므로, 종료 시 실제 회전
            # 명령을 발행 중인 경우에만 정지 명령을 보낸다.
            if (
                self.turn_request_pending
                or self.turning
                or self.turn_command_active
            ):
                self.publish_stop()

            self.get_logger().info(
                "노드 종료: SYSTEM_LOCKED 일반 이동 제어에는 관여하지 않음"
            )
            return

        self.get_logger().info(
            "노드 종료: Nav2 취소 및 로봇 정지"
        )

        self.request_nav_cancel(
            reason="노드 종료"
        )

        for _ in range(5):
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)

    node = GuardedNavToGoal()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()