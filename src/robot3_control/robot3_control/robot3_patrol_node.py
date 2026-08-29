import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool


class WaypointPatrolNode(Node):
    """
    POINT_1~POINT_6 순찰 노드.

    추가된 /hot_place 처리 순서:
    1. 첫 번째 non-empty PoseArray를 받으면 해당 배열을 스냅샷으로 저장한다.
    2. 진행 중인 순찰 Nav2 골을 취소한다.
    3. 첫 메시지에 포함된 좌표 중 현재 로봇과 가장 가까운 좌표를 선택한다.
    4. 선택 좌표에 정확히 진입하지 않고 hot_place_standoff_m만큼 앞에 정지한다.
    5. 이후 /hot_place 메시지는 좌표를 갱신하지 않고 수신 시각만 갱신한다.
    6. 마지막 수신 후 hot_place_timeout_sec 동안 메시지가 없으면 핫플레이스 골을
       취소하고 POINT_1부터 순찰을 다시 시작한다.
    """

    GOAL_KIND_PATROL = "PATROL"
    GOAL_KIND_HOT_PLACE = "HOT_PLACE"

    def __init__(self):
        super().__init__("waypoint_patrol_node")

        # 상태 변경 콜백을 순차 실행해 골 취소/전환 경합을 방지한다.
        self.cb_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("robot_pose_topic", "amcl_pose")
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("retry_delay_sec", 1.0)
        self.declare_parameter("hot_place_timeout_sec", 3.0)
        self.declare_parameter("hot_place_check_period_sec", 0.1)
        self.declare_parameter("hot_place_standoff_m", 0.8)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.robot_pose_topic = str(
            self.get_parameter("robot_pose_topic").value
        )
        self.start_delay_sec = float(
            self.get_parameter("start_delay_sec").value
        )
        self.retry_delay_sec = float(
            self.get_parameter("retry_delay_sec").value
        )
        self.hot_place_timeout_sec = max(
            0.1,
            float(self.get_parameter("hot_place_timeout_sec").value),
        )
        self.hot_place_check_period_sec = max(
            0.05,
            float(
                self.get_parameter("hot_place_check_period_sec").value
            ),
        )
        self.hot_place_standoff_m = max(
            0.0,
            float(self.get_parameter("hot_place_standoff_m").value),
        )

        # POINT_1 → POINT_2 → ... → POINT_6 → POINT_1 무한 반복
        self.patrol_points: List[Tuple[float, float]] = [
            (-0.5, 0.38),      # POINT_1
            (-2.3, 0.38),      # POINT_2
            (-2.56, -0.41),    # POINT_3
            (-2.56, -1.82),    # POINT_4
            (-4.1, -1.82),     # POINT_5
            (-0.5, -1.82),     # POINT_6
        ]
        self.patrol_index = 0

        # /rb3_standby에 의한 외부 정지 상태.
        self.standby = False

        # POINT_1부터 재시작하기 위해 기존 골 종료를 기다리는 상태.
        self.restart_pending = False

        # 현재/요청 중인 Nav2 골 상태.
        self.active_goal_handle = None
        self.active_goal_name: Optional[str] = None
        self.active_goal_kind: Optional[str] = None

        self.pending_goal_name: Optional[str] = None
        self.pending_goal_kind: Optional[str] = None

        self.goal_request_pending = False
        self.cancel_request_pending = False

        # /rb3_standby=True 처리 완료 ACK 대기 상태.
        self.standby_ack_pending = False

        # 가장 최근 AMCL 로봇 위치(map 기준).
        self.robot_x: Optional[float] = None
        self.robot_y: Optional[float] = None

        # /hot_place 상태.
        self.hot_place_active = False
        self.hot_place_first_points: List[Tuple[float, float]] = []
        self.hot_place_selected_point: Optional[Tuple[float, float]] = None
        self.hot_place_approach_goal: Optional[
            Tuple[float, float, float]
        ] = None
        self.hot_place_goal_send_required = False
        self.last_hot_place_rx_ns: Optional[int] = None
        self.hot_place_retry_not_before_ns = 0

        # 타이머.
        self.start_timer = None
        self.retry_timer = None
        self.hot_place_timer = None

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
            callback_group=self.cb_group,
        )

        self.standby_sub = self.create_subscription(
            Bool,
            "/rb3_standby",
            self.rb3_standby_callback,
            10,
            callback_group=self.cb_group,
        )

        self.start_patrol_sub = self.create_subscription(
            Bool,
            "/start_patrol",
            self.start_patrol_callback,
            10,
            callback_group=self.cb_group,
        )

        self.hot_place_sub = self.create_subscription(
            PoseArray,
            "/hot_place",
            self.hot_place_callback,
            qos_profile_sensor_data,
            callback_group=self.cb_group,
        )

        # 상대 토픽명 "amcl_pose"는 노드 namespace가 /robot3이면
        # 자동으로 /robot3/amcl_pose를 구독한다.
        self.robot_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.robot_pose_topic,
            self.robot_pose_callback,
            qos_profile_sensor_data,
            callback_group=self.cb_group,
        )

        self.standby_done_pub = self.create_publisher(
            Bool,
            "/rb3_standby_done",
            10,
        )

        # 기존 코드와 동일하게 노드 실행 후 자동 순찰 시작.
        self.start_timer = self.create_timer(
            self.start_delay_sec,
            self.start_patrol_once,
            callback_group=self.cb_group,
        )

        # 핫플레이스 타임아웃 및 골 전환 관리 타이머.
        self.hot_place_timer = self.create_timer(
            self.hot_place_check_period_sec,
            self.hot_place_state_timer_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info("waypoint_patrol_node started")
        self.get_logger().info(
            "patrol order: 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> repeat"
        )
        self.get_logger().info("sub: /rb3_standby (std_msgs/Bool)")
        self.get_logger().info("sub: /start_patrol (std_msgs/Bool)")
        self.get_logger().info(
            "sub: /hot_place (geometry_msgs/PoseArray)"
        )
        self.get_logger().info(
            f"sub: {self.robot_pose_topic} "
            "(geometry_msgs/PoseWithCovarianceStamped)"
        )
        self.get_logger().info(
            "pub: /rb3_standby_done (std_msgs/Bool)"
        )
        self.get_logger().info(
            f"hot place timeout={self.hot_place_timeout_sec:.2f}s, "
            f"standoff={self.hot_place_standoff_m:.2f}m"
        )

        for index, (x, y) in enumerate(self.patrol_points, start=1):
            self.get_logger().info(
                f"POINT_{index}: x={x:.3f}, y={y:.3f}"
            )

    # ------------------------------------------------------------------
    # 로봇 위치 및 핫플레이스 처리
    # ------------------------------------------------------------------

    def robot_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)

        # 첫 hot_place 수신 당시 AMCL 위치가 아직 없었던 경우,
        # 최초 위치 수신 직후 저장된 첫 메시지 기준으로 목표를 계산한다.
        if (
            self.hot_place_active
            and self.hot_place_selected_point is None
        ):
            self.prepare_hot_place_goal()

    def hot_place_callback(self, msg: PoseArray):
        now_ns = self.get_clock().now().nanoseconds

        # 이미 핫플레이스 모드이면 좌표는 절대 갱신하지 않고
        # 마지막 수신 시각만 갱신한다.
        if self.hot_place_active:
            self.last_hot_place_rx_ns = now_ns
            return

        if len(msg.poses) == 0:
            self.get_logger().warn(
                "/hot_place received with an empty poses array; ignored"
            )
            return

        message_frame = msg.header.frame_id.strip()
        if message_frame and message_frame != self.frame_id:
            self.get_logger().error(
                "/hot_place frame mismatch: "
                f"received='{message_frame}', expected='{self.frame_id}'"
            )
            return

        # 첫 번째 메시지의 좌표만 스냅샷으로 저장한다.
        self.hot_place_first_points = [
            (float(pose.position.x), float(pose.position.y))
            for pose in msg.poses
        ]
        self.hot_place_active = True
        self.last_hot_place_rx_ns = now_ns
        self.hot_place_selected_point = None
        self.hot_place_approach_goal = None
        self.hot_place_goal_send_required = False
        self.hot_place_retry_not_before_ns = 0

        # 핫플레이스가 순찰 재시작 요청보다 우선한다.
        self.restart_pending = False
        self.cancel_navigation_timers()

        self.get_logger().warn(
            "/hot_place first message received: "
            f"stop patrol and freeze {len(self.hot_place_first_points)} "
            "candidate coordinates"
        )

        for index, (x, y) in enumerate(
            self.hot_place_first_points,
            start=1,
        ):
            self.get_logger().info(
                f"HOT_PLACE_CANDIDATE_{index}: x={x:.3f}, y={y:.3f}"
            )

        self.prepare_hot_place_goal()

        # 진행 중인 순찰 골 또는 승인 대기 중인 순찰 골을 먼저 정리한다.
        if self.active_goal_handle is not None:
            self.get_logger().info(
                f"cancel current goal for hot place: "
                f"{self.active_goal_name}"
            )
            self.cancel_active_goal()
        elif self.goal_request_pending:
            self.get_logger().info(
                "goal request is pending; cancel immediately after "
                "acceptance for hot place transition"
            )
        else:
            self.process_hot_place_state()

    def prepare_hot_place_goal(self) -> bool:
        """첫 PoseArray에서 로봇과 가장 가까운 점과 접근 목표를 계산한다."""
        if not self.hot_place_active:
            return False

        if self.hot_place_selected_point is not None:
            return True

        if not self.hot_place_first_points:
            return False

        if self.robot_x is None or self.robot_y is None:
            return False

        robot_x = self.robot_x
        robot_y = self.robot_y

        selected_index, selected_point = min(
            enumerate(self.hot_place_first_points),
            key=lambda item: math.hypot(
                item[1][0] - robot_x,
                item[1][1] - robot_y,
            ),
        )

        hot_x, hot_y = selected_point
        distance = math.hypot(hot_x - robot_x, hot_y - robot_y)

        if distance > 1e-6:
            direction_x = (hot_x - robot_x) / distance
            direction_y = (hot_y - robot_y) / distance

            # 선택 좌표까지 완전히 들어가지 않고 설정 거리만큼 앞에서 정지.
            travel_distance = max(
                0.0,
                distance - self.hot_place_standoff_m,
            )
            goal_x = robot_x + direction_x * travel_distance
            goal_y = robot_y + direction_y * travel_distance
            goal_yaw = math.atan2(hot_y - goal_y, hot_x - goal_x)
        else:
            # 이미 선택 좌표에 있는 경우 현재 위치를 유지한다.
            goal_x = robot_x
            goal_y = robot_y
            goal_yaw = 0.0

        self.hot_place_selected_point = (hot_x, hot_y)
        self.hot_place_approach_goal = (
            goal_x,
            goal_y,
            goal_yaw,
        )
        self.hot_place_goal_send_required = True

        self.get_logger().warn(
            "nearest hot place selected from first message: "
            f"index={selected_index + 1}, "
            f"robot=({robot_x:.3f}, {robot_y:.3f}), "
            f"hot=({hot_x:.3f}, {hot_y:.3f}), "
            f"distance={distance:.3f}m"
        )
        self.get_logger().warn(
            "hot place approach goal: "
            f"x={goal_x:.3f}, y={goal_y:.3f}, "
            f"yaw={goal_yaw:.3f}, "
            f"standoff={self.hot_place_standoff_m:.3f}m"
        )
        return True

    def hot_place_state_timer_callback(self):
        if not self.hot_place_active:
            return

        now_ns = self.get_clock().now().nanoseconds

        if self.last_hot_place_rx_ns is not None:
            elapsed_sec = (
                now_ns - self.last_hot_place_rx_ns
            ) / 1_000_000_000.0

            if elapsed_sec >= self.hot_place_timeout_sec:
                self.finish_hot_place_mode(elapsed_sec)
                return

        self.process_hot_place_state()

    def process_hot_place_state(self):
        """기존 골이 완전히 정리된 뒤 핫플레이스 골을 한 번 전송한다."""
        if not self.hot_place_active:
            return

        # 외부 standby가 활성화되면 핫플레이스 메시지는 감시하되
        # Nav2 골은 보내지 않는다.
        if self.standby:
            return

        if not self.prepare_hot_place_goal():
            return

        if not self.hot_place_goal_send_required:
            return

        if self.goal_request_pending:
            return

        if self.active_goal_handle is not None:
            return

        if self.cancel_request_pending:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.hot_place_retry_not_before_ns:
            return

        self.send_hot_place_goal()

    def send_hot_place_goal(self):
        if not self.hot_place_active:
            return

        if self.hot_place_approach_goal is None:
            return

        goal_x, goal_y, goal_yaw = self.hot_place_approach_goal

        sent = self.send_navigation_goal(
            x=goal_x,
            y=goal_y,
            yaw=goal_yaw,
            goal_name="HOT_PLACE_APPROACH",
            goal_kind=self.GOAL_KIND_HOT_PLACE,
        )

        if sent:
            # 요청 자체는 한 번만 보낸다. 거절/실패 시 콜백에서 재시도를 예약한다.
            self.hot_place_goal_send_required = False

    def finish_hot_place_mode(self, elapsed_sec: float):
        """3초 이상 메시지가 없으면 핫플레이스 모드를 종료한다."""
        if not self.hot_place_active:
            return

        self.get_logger().warn(
            f"/hot_place timeout: no message for {elapsed_sec:.2f}s; "
            "cancel hot place navigation and restart patrol from POINT_1"
        )

        self.hot_place_active = False
        self.hot_place_first_points = []
        self.hot_place_selected_point = None
        self.hot_place_approach_goal = None
        self.hot_place_goal_send_required = False
        self.last_hot_place_rx_ns = None
        self.hot_place_retry_not_before_ns = 0

        self.cancel_navigation_timers()
        self.patrol_index = 0

        # 외부 standby 상태에서는 자동 재시작하지 않는다.
        if self.standby:
            self.restart_pending = False

            if self.active_goal_handle is not None:
                self.cancel_active_goal()

            self.get_logger().info(
                "hot place mode ended, but external standby remains active"
            )
            return

        self.restart_pending = True

        if self.active_goal_handle is not None:
            self.cancel_active_goal()
            return

        if self.goal_request_pending:
            self.get_logger().info(
                "hot place goal request is pending; cancel after acceptance "
                "and then restart patrol"
            )
            return

        self.finish_restart()

    # ------------------------------------------------------------------
    # 외부 정지/재시작 처리
    # ------------------------------------------------------------------

    def rb3_standby_callback(self, msg: Bool):
        """
        /rb3_standby=True:
        - 이 노드가 보낸 현재 Nav2 골 취소
        - 순찰 시작/재시도 타이머 취소
        - 이후 순찰 및 핫플레이스 골 전송 차단
        - 골 완전 종료 후 /rb3_standby_done=True 발행
        """
        if not msg.data:
            if self.standby:
                self.get_logger().info(
                    "/rb3_standby=False received; patrol remains stopped"
                )
            return

        first_request = not self.standby

        self.standby = True
        self.restart_pending = False
        self.standby_ack_pending = True
        self.cancel_navigation_timers()

        if first_request:
            self.get_logger().warn(
                "/rb3_standby=True received: "
                "cancel current goal and stop this node"
            )
        else:
            self.get_logger().info(
                "/rb3_standby=True received again: keep stopped"
            )

        if self.active_goal_handle is not None:
            self.cancel_active_goal()
        elif self.goal_request_pending:
            self.get_logger().info(
                "goal request pending; cancel immediately after acceptance"
            )
        else:
            self.get_logger().info("no active navigation goal")
            self.try_publish_standby_done()

    def try_publish_standby_done(self):
        if not self.standby:
            return

        if not self.standby_ack_pending:
            return

        if self.goal_request_pending:
            return

        if self.active_goal_handle is not None:
            return

        if self.cancel_request_pending:
            return

        msg = Bool()
        msg.data = True
        self.standby_done_pub.publish(msg)
        self.standby_ack_pending = False

        self.get_logger().warn(
            "/rb3_standby_done=True published: "
            "navigation goal fully stopped"
        )

    def start_patrol_callback(self, msg: Bool):
        """
        /start_patrol=True:
        - 외부 standby 해제
        - 기존 골 취소
        - POINT_1부터 순찰 재시작

        활성 /hot_place가 있으면 핫플레이스가 우선하므로 요청을 무시한다.
        """
        if not msg.data:
            return

        if self.hot_place_active:
            self.get_logger().warn(
                "/start_patrol=True ignored while /hot_place is active"
            )
            return

        self.get_logger().warn(
            "/start_patrol=True received: restart from POINT_1"
        )

        self.cancel_navigation_timers()

        self.standby = False
        self.standby_ack_pending = False
        self.patrol_index = 0
        self.restart_pending = True

        if self.active_goal_handle is not None:
            self.get_logger().info(
                f"cancel current goal before restart: "
                f"{self.active_goal_name}"
            )
            self.cancel_active_goal()
            return

        if self.goal_request_pending:
            self.get_logger().info(
                "goal request pending; cancel after acceptance and "
                "restart from POINT_1"
            )
            return

        self.finish_restart()

    def finish_restart(self):
        """기존 골 정리가 끝난 후 POINT_1 골 전송을 예약한다."""
        if not self.restart_pending:
            return

        if self.standby:
            self.restart_pending = False
            return

        if self.hot_place_active:
            return

        if self.active_goal_handle is not None:
            return

        if self.goal_request_pending:
            return

        if self.cancel_request_pending:
            return

        self.restart_pending = False
        self.patrol_index = 0

        self.get_logger().info("restart patrol from POINT_1")
        self.schedule_patrol_retry(0.1)

    # ------------------------------------------------------------------
    # Nav2 골 취소 및 공통 콜백
    # ------------------------------------------------------------------

    def cancel_active_goal(self):
        """현재 이 노드가 보낸 NavigateToPose 골만 취소한다."""
        if self.active_goal_handle is None:
            return

        if self.cancel_request_pending:
            return

        self.cancel_request_pending = True

        self.get_logger().info(
            f"request goal cancellation: {self.active_goal_name}"
        )

        cancel_future = self.active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        self.cancel_request_pending = False

        try:
            cancel_response = future.result()
        except Exception as error:
            self.get_logger().error(
                f"goal cancel response error: {error}"
            )
            return

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info(
                "navigation goal cancellation accepted; wait for result"
            )
        else:
            self.get_logger().warn(
                "navigation goal cancellation was not accepted or the "
                "goal had already finished"
            )

        self.try_publish_standby_done()
        self.process_hot_place_state()
        self.finish_restart()

    def send_navigation_goal(
        self,
        x: float,
        y: float,
        yaw: float,
        goal_name: str,
        goal_kind: str,
    ) -> bool:
        if self.active_goal_handle is not None:
            return False

        if self.goal_request_pending:
            return False

        if self.cancel_request_pending:
            return False

        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                "navigate_to_pose action server unavailable"
            )

            if goal_kind == self.GOAL_KIND_PATROL:
                self.schedule_patrol_retry(self.retry_delay_sec)
            else:
                self.hot_place_retry_not_before_ns = (
                    self.get_clock().now().nanoseconds
                    + int(self.retry_delay_sec * 1_000_000_000)
                )
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(x, y, yaw)

        self.pending_goal_name = goal_name
        self.pending_goal_kind = goal_kind
        self.goal_request_pending = True

        self.get_logger().info(
            f"send goal [{goal_name}] kind={goal_kind}, "
            f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
        )

        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        requested_name = self.pending_goal_name
        requested_kind = self.pending_goal_kind

        self.goal_request_pending = False
        self.pending_goal_name = None
        self.pending_goal_kind = None

        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(
                f"goal response error [{requested_name}]: {error}"
            )
            self.handle_goal_request_failure(
                requested_name,
                requested_kind,
                rejected=False,
            )
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f"goal rejected: {requested_name}"
            )
            self.handle_goal_request_failure(
                requested_name,
                requested_kind,
                rejected=True,
            )
            return

        self.active_goal_handle = goal_handle
        self.active_goal_name = requested_name
        self.active_goal_kind = requested_kind

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

        # 골 승인 대기 중 상태가 바뀐 경우 승인 직후 즉시 취소한다.
        cancel_after_accept = False
        cancel_reason = ""

        if self.standby:
            cancel_after_accept = True
            cancel_reason = "standby request"
        elif (
            requested_kind == self.GOAL_KIND_PATROL
            and self.hot_place_active
        ):
            cancel_after_accept = True
            cancel_reason = "hot place transition"
        elif (
            requested_kind == self.GOAL_KIND_HOT_PLACE
            and not self.hot_place_active
        ):
            cancel_after_accept = True
            cancel_reason = "hot place timeout"
        elif self.restart_pending:
            cancel_after_accept = True
            cancel_reason = "patrol restart"

        if cancel_after_accept:
            self.get_logger().info(
                f"goal accepted after {cancel_reason}; cancel immediately"
            )
            self.cancel_active_goal()

    def handle_goal_request_failure(
        self,
        goal_name: Optional[str],
        goal_kind: Optional[str],
        rejected: bool,
    ):
        if self.standby:
            self.try_publish_standby_done()
            return

        if self.restart_pending:
            self.finish_restart()
            return

        if self.hot_place_active:
            if goal_kind == self.GOAL_KIND_HOT_PLACE:
                self.hot_place_goal_send_required = True
                self.hot_place_retry_not_before_ns = (
                    self.get_clock().now().nanoseconds
                    + int(self.retry_delay_sec * 1_000_000_000)
                )
            self.process_hot_place_state()
            return

        if goal_kind == self.GOAL_KIND_PATROL:
            if rejected:
                self.move_to_next_point()
            else:
                self.schedule_patrol_retry(self.retry_delay_sec)

    def feedback_callback(self, feedback_msg):
        if self.standby or self.restart_pending:
            return

        distance = feedback_msg.feedback.distance_remaining
        goal_name = self.active_goal_name or self.pending_goal_name

        self.get_logger().info(
            f"{goal_name} remaining={distance:.2f}m"
        )

    def result_callback(self, future):
        goal_name = self.active_goal_name
        goal_kind = self.active_goal_kind

        self.active_goal_handle = None
        self.active_goal_name = None
        self.active_goal_kind = None
        self.cancel_request_pending = False

        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:
            self.get_logger().error(
                f"result error [{goal_name}]: {error}"
            )
            status = GoalStatus.STATUS_UNKNOWN

        self.get_logger().info(
            f"goal result [{goal_name}] kind={goal_kind} "
            f"-> {self.goal_status_to_text(status)}"
        )

        if self.standby:
            self.get_logger().info(
                "standby state: no next navigation goal will be sent"
            )
            self.try_publish_standby_done()
            return

        if self.restart_pending:
            self.finish_restart()
            return

        if self.hot_place_active:
            if goal_kind == self.GOAL_KIND_HOT_PLACE:
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.hot_place_goal_send_required = False
                    self.get_logger().warn(
                        "HOT_PLACE_APPROACH arrived; wait for "
                        "/hot_place timeout"
                    )
                else:
                    self.hot_place_goal_send_required = True
                    self.hot_place_retry_not_before_ns = (
                        self.get_clock().now().nanoseconds
                        + int(self.retry_delay_sec * 1_000_000_000)
                    )
                    self.get_logger().warn(
                        "hot place goal did not succeed; retry while "
                        "/hot_place remains active"
                    )

            # 순찰 골 취소 결과가 도착한 경우에도 여기서 핫플레이스 골 전송.
            self.process_hot_place_state()
            return

        # 핫플레이스가 종료된 뒤 도착한 HOT_PLACE 결과는 다음 순찰점으로
        # 넘기지 않고 POINT_1 재시작 로직에서 처리한다.
        if goal_kind == self.GOAL_KIND_HOT_PLACE:
            self.finish_restart()
            return

        if goal_kind == self.GOAL_KIND_PATROL:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"{goal_name} arrived")
            else:
                self.get_logger().warn(
                    f"{goal_name} failed, skip to next point"
                )

            self.move_to_next_point()

    # ------------------------------------------------------------------
    # 순찰 처리
    # ------------------------------------------------------------------

    def start_patrol_once(self):
        if self.start_timer is not None:
            self.start_timer.cancel()
            self.start_timer = None

        if self.standby:
            self.get_logger().info(
                "patrol start blocked: standby state"
            )
            return

        if self.hot_place_active:
            self.get_logger().info(
                "patrol start blocked: hot place active"
            )
            return

        self.get_logger().info("start infinite waypoint patrol")
        self.send_current_patrol_goal()

    def send_current_patrol_goal(self):
        if self.standby:
            self.get_logger().info(
                "patrol goal send blocked: standby state"
            )
            return

        if self.hot_place_active:
            self.get_logger().info(
                "patrol goal send blocked: hot place active"
            )
            return

        if self.restart_pending:
            return

        x, y = self.patrol_points[self.patrol_index]
        yaw = self.compute_patrol_goal_yaw(self.patrol_index)

        self.send_navigation_goal(
            x=x,
            y=y,
            yaw=yaw,
            goal_name=f"POINT_{self.patrol_index + 1}",
            goal_kind=self.GOAL_KIND_PATROL,
        )

    def move_to_next_point(self):
        if self.standby:
            self.get_logger().info(
                "next point transition blocked: standby state"
            )
            return

        if self.hot_place_active:
            self.get_logger().info(
                "next point transition blocked: hot place active"
            )
            return

        if self.restart_pending:
            return

        self.patrol_index = (
            self.patrol_index + 1
        ) % len(self.patrol_points)

        self.get_logger().info(
            f"next goal: POINT_{self.patrol_index + 1}"
        )

        self.schedule_patrol_retry(0.2)

    def schedule_patrol_retry(self, delay_sec: float):
        if self.standby or self.hot_place_active:
            return

        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None

        self.retry_timer = self.create_timer(
            max(0.01, float(delay_sec)),
            self.patrol_retry_callback,
            callback_group=self.cb_group,
        )

    def patrol_retry_callback(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None

        if self.standby or self.hot_place_active:
            return

        self.send_current_patrol_goal()

    # ------------------------------------------------------------------
    # 공통 유틸리티
    # ------------------------------------------------------------------

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def compute_patrol_goal_yaw(self, current_index: int) -> float:
        current_x, current_y = self.patrol_points[current_index]
        next_index = (current_index + 1) % len(self.patrol_points)
        next_x, next_y = self.patrol_points[next_index]

        return math.atan2(
            next_y - current_y,
            next_x - current_x,
        )

    def cancel_navigation_timers(self):
        if self.start_timer is not None:
            self.start_timer.cancel()
            self.start_timer = None

        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None

    def cancel_all_timers(self):
        self.cancel_navigation_timers()

        if self.hot_place_timer is not None:
            self.hot_place_timer.cancel()
            self.hot_place_timer = None

    @staticmethod
    def goal_status_to_text(status: int) -> str:
        return {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(status, str(status))


def main(args=None):
    rclpy.init(args=args)

    node = WaypointPatrolNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("patrol stopped")
    finally:
        node.cancel_all_timers()
        executor.remove_node(node)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
