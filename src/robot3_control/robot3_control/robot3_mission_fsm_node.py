"""
robot_mission_fsm
==================
로봇팀이 만든 tb2_mission_fsm_test.py(선점/취소/응급 세부 흐름)를 그대로 살리고,
중앙 노드(fleet_dispatcher_node) 인터페이스(fleet_interfaces)에 맞게 감싼 버전.

원본에서 바뀐 것:
  - test_cmd(String) 구독, /fall_detection 자동 반응  ->  execute_mission Action Server
  - /fall_person_point 직접 구독으로 좌표 수신          ->  Dispatcher가 goal.dest_x/dest_y로 좌표 전달
  - status(String 자유 텍스트)                          ->  status(RobotStatus.msg)
  - aed_loaded 하드코딩                                  ->  실행 파라미터(aed_loaded)로 관리

원본에서 그대로 유지한 것 (핵심 가치라 통째로 재사용):
  - RobotMode 상태 전이 (CLEARING_PEOPLE -> RGBD_WAITING -> FINAL_APPROACH -> EMERGENCY_WORKING)
  - preempt_to_goal / cancel_current_goal (기존 goal 취소 후 pending goal 실행)
  - 삐뽀삐뽀(beep), RGB-D 최종 접근, 응급 작업 타이머 흐름
  - /rgbd_fall_person_point 구독 (응급 중 최종 접근 좌표는 여전히 센서에서 직접 받음)

실행 예시:
  ros2 run robot_mission_pkg robot_mission_fsm --ros-args -r __ns:=/robot3 -p aed_loaded:=true
  ros2 run robot_mission_pkg robot_mission_fsm --ros-args -r __ns:=/robot1 -p aed_loaded:=false
"""

import threading
import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration

from fleet_interfaces.msg import RobotStatus
from fleet_interfaces.action import ExecuteMission
from fleet_interfaces.srv import EmergencyStop


class RobotMode(str, Enum):
    IDLE = "IDLE"
    GUIDE_RUNNING = "GUIDE_RUNNING"
    LUGGAGE_RUNNING = "LUGGAGE_RUNNING"
    HOTPLACE_RUNNING = "HOTPLACE_RUNNING"

    EMERGENCY_DISPATCH = "EMERGENCY_DISPATCH"
    CLEARING_PEOPLE = "CLEARING_PEOPLE"
    RGBD_WAITING = "RGBD_WAITING"
    FINAL_APPROACH = "FINAL_APPROACH"
    EMERGENCY_WORKING = "EMERGENCY_WORKING"

    RETURNING = "RETURNING"
    DOCKED = "DOCKED"
    ESTOP = "ESTOP"
    ERROR = "ERROR"


EMERGENCY_SUBMODES = {
    RobotMode.EMERGENCY_DISPATCH,
    RobotMode.CLEARING_PEOPLE,
    RobotMode.RGBD_WAITING,
    RobotMode.FINAL_APPROACH,
    RobotMode.EMERGENCY_WORKING,
}

MODE_TO_ENUM = {
    RobotMode.IDLE: RobotStatus.MODE_IDLE,
    RobotMode.GUIDE_RUNNING: RobotStatus.MODE_GUIDE,
    RobotMode.LUGGAGE_RUNNING: RobotStatus.MODE_LUGGAGE_ASSIST,
    # RobotStatus.msg에 MODE_HOTPLACE_DISPATCH=6 상수 추가됨 (fleet_interfaces 재빌드 필요).
    RobotMode.HOTPLACE_RUNNING: RobotStatus.MODE_HOTPLACE_DISPATCH,
    RobotMode.EMERGENCY_DISPATCH: RobotStatus.MODE_EMERGENCY_DISPATCH,
    RobotMode.CLEARING_PEOPLE: RobotStatus.MODE_EMERGENCY_DISPATCH,
    RobotMode.RGBD_WAITING: RobotStatus.MODE_EMERGENCY_DISPATCH,
    RobotMode.FINAL_APPROACH: RobotStatus.MODE_EMERGENCY_DISPATCH,
    RobotMode.EMERGENCY_WORKING: RobotStatus.MODE_EMERGENCY_DISPATCH,
    RobotMode.RETURNING: RobotStatus.MODE_IDLE,
    RobotMode.DOCKED: RobotStatus.MODE_IDLE,
    RobotMode.ESTOP: RobotStatus.MODE_ERROR,
    RobotMode.ERROR: RobotStatus.MODE_ERROR,
}

EMERGENCY_PRIORITY = 10

# =====================================================
# GUIDE / LUGGAGE_ASSIST 위임용 goal 이름 테이블
# =====================================================
# 2_back_move.py(GuardedNavToGoal)의 self.goal_poses와 반드시 동기화해야 한다.
# fleet_dispatcher는 (dest_x, dest_y) 좌표만 넘겨주므로, 여기서 좌표를
# 2_back_move.py가 이해하는 "목적지 이름" 문자열로 역매핑해서 mission_cmd로 보낸다.
# 새 목적지가 goal_poses에 추가되면 이 테이블에도 동일한 (x, y)를 추가할 것.
FOLLOW_GOAL_NAME_TABLE = {
    "detect_pose": (-1.168346575735853, 0.34778344933508026),
    'goal1_1': (-2.92409,2.94599 ),
    'goal1_2': (-3.97043, 3.462406),
    'goal_2': (-4.248990535736084,-1.706390142440796,),
    "pre_dock": (-0.43677089190051244, 0.018926375739573814),
}

# 좌표 매칭 허용 오차(m). 실수로 인한 미세 오차를 흡수한다.
FOLLOW_GOAL_MATCH_TOLERANCE_M = 0.05


def resolve_follow_goal_name(x: float, y: float):
    """(x, y)에 가장 가까운 목적지 이름을 찾는다.

    허용 오차(FOLLOW_GOAL_MATCH_TOLERANCE_M) 내에 일치하는 항목이 없으면 None을 반환한다.
    """
    best_name = None
    best_dist = None
    for name, (gx, gy) in FOLLOW_GOAL_NAME_TABLE.items():
        dist = math.hypot(gx - x, gy - y)
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_name = name
    if best_dist is None or best_dist > FOLLOW_GOAL_MATCH_TOLERANCE_M:
        return None
    return best_name


class RobotMissionFsm(Node):
    def __init__(self):
        super().__init__("robot_mission_fsm")
        self.cb_group = ReentrantCallbackGroup()

        # =========================
        # 파라미터
        # =========================
        self.declare_parameter("aed_loaded", False)
        self.declare_parameter("hotplace_patrol_resume_delay_sec", 5.0)
        self.declare_parameter("patrol_stop_ack_timeout_sec", 10.0)

        self.aed_loaded = self.get_parameter("aed_loaded").get_parameter_value().bool_value
        self.hotplace_patrol_resume_delay_sec = float(
            self.get_parameter("hotplace_patrol_resume_delay_sec").value
        )
        self.patrol_stop_ack_timeout_sec = float(
            self.get_parameter("patrol_stop_ack_timeout_sec").value
        )
        self.robot_id = self.get_namespace().strip("/") or "robot_unknown"

        self.frame_id = "map"
        self.clearing_wait_sec = 4.0
        self.rgbd_wait_timeout_sec = 10.0
        self.battery_pct = 100.0  # TODO(팀원): 실제 배터리 값 연결

        # 도킹 좌표 (팀 환경에 맞춰 launch 파라미터로 빼는 것을 권장)
        self.dock_pose_data = {
            "x": -0.17385751948123415, "y": -1.7934206990527808, "z": 0.0,
            "qx": 0.0, "qy": 0.0, "qz": -0.014605659366637753, "qw": 0.9998933316681664,
        }

        # =========================
        # FSM 상태 변수 (원본 그대로)
        # =========================
        self.mode = RobotMode.IDLE
        self.current_priority = 0

        self.active_goal_handle = None
        self.active_goal_name = None
        self.pending_goal_name = None
        self.pending_pose = None

        self.last_nav_status = "NONE"
        self.last_result_code = "NONE"
        self.latest_rgbd_point = None

        self.goal2_timer = None
        self.clearing_timer = None
        self.rgbd_wait_timer = None
        self.emergency_timer = None
        self.patrol_stop_ack_timer = None
        self.hotplace_resume_timer = None

        # HOTPLACE_DISPATCH 전용 상태
        self.waiting_patrol_stop_ack = False
        self.pending_hotplace_pose = None
        self.last_hotplace_request_time = None

        # Action Server <-> 내부 FSM 콜백 브리지용 (threading.Event 사용 - asyncio 루프 불필요)
        self._action_goal_handle = None      # 현재 처리 중인 ExecuteMission goal_handle
        self._mission_done_event = threading.Event()
        self._mission_result = None          # (success: bool, final_state: str)

        # GUIDE/LUGGAGE_ASSIST는 FSM 자체 Nav2 대신 2_back_move.py/4_front_move.py
        # (mission_cmd/turn_around/service_cancel 기반 기존 코드)에 위임한다.
        # "NAV": FSM이 자체 nav_client로 직접 이동 관리 (EMERGENCY 계열)
        # "FOLLOW": mission_cmd를 발행해 기존 추종 파이프라인에 위임 (GUIDE/LUGGAGE_ASSIST)
        self._active_flow = None
        self._follow_cancel_sent = False

        # 새 goal이 들어왔을 때 기존 임무(NAV 또는 FOLLOW)를 "취소 요청 -> 완전히
        # 끝날 때까지 대기 -> 그 다음에야 새 임무 시작" 순서로 강제해서, 서로 다른
        # execute_cb 스레드가 동시에 Nav2/FOLLOW 파이프라인을 건드리는 레이스
        # 컨디션을 막는다. FOLLOW는 외부 노드(4_front_move.py)가 mission_complete를
        # 발행할 때까지 시간이 걸릴 수 있어 NAV보다 타임아웃을 넉넉히 둔다.
        self._mission_lock = threading.Lock()
        self._mission_preempt_timeout_sec = 30.0

        # =========================
        # ROS Interface
        # =========================
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose",
                                        callback_group=self.cb_group)

        self._mission_server = ActionServer(
            self, ExecuteMission, "execute_mission",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group)

        self.create_service(EmergencyStop, "emergency_stop", self.on_emergency_stop,
                             callback_group=self.cb_group)

        # 응급 중 최종 접근 좌표는 여전히 센서(RGB-D)에서 직접 받음 (원본 유지)
        self.rgbd_point_sub = self.create_subscription(
            PointStamped, "/rgbd_fall_person_point", self.rgbd_point_callback, 10,
            callback_group=self.cb_group)

        # =========================
        # GUIDE/LUGGAGE_ASSIST 위임(follow 파이프라인 연동)
        # =========================
        # 네임스페이스(__ns:=/robot3) 하위 상대 토픽 -> 실제로는 /robot3/mission_cmd가 된다.
        # (2_back_move.py의 MISSION_TOPIC = "/robot3/mission_cmd"와 동일해야 함)
        self.mission_cmd_pub = self.create_publisher(String, "mission_cmd", 10)

        # /turn_around, /service_cancel, /turn_around_done, /robot3/mission_complete는
        # 2_back_move.py/4_front_move.py에서 절대 경로("/...")로 하드코딩되어 있으므로
        # 여기서도 동일하게 절대 경로로 맞춘다 (네임스페이스 자동 적용 대상 아님).
        self.service_cancel_pub = self.create_publisher(Bool, "/service_cancel", 10)
        self.mission_complete_sub = self.create_subscription(
            Bool, "/robot3/mission_complete", self.mission_complete_callback, 10,
            callback_group=self.cb_group)

        # HOTPLACE_DISPATCH와 패트롤 노드 간 정지/재시작 핸드셰이크
        self.rb3_standby_pub = self.create_publisher(Bool, "/rb3_standby", 10)
        self.start_patrol_pub = self.create_publisher(Bool, "/start_patrol", 10)
        self.rb3_standby_done_sub = self.create_subscription(
            Bool,
            "/rb3_standby_done",
            self.rb3_standby_done_callback,
            10,
            callback_group=self.cb_group,
        )

        self.status_pub = self.create_publisher(RobotStatus, "status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.audio_pub = self.create_publisher(AudioNoteVector, "cmd_audio", 10)

        self.create_timer(1.0, self.publish_status, callback_group=self.cb_group)
        self.create_timer(0.1, self.estop_zero_timer, callback_group=self.cb_group)

        self.get_logger().info(
            f"[robot_mission_fsm] {self.robot_id} 시작 (aed_loaded={self.aed_loaded})"
        )

    # =====================================================
    # Pose 생성 (원본 유지)
    # =====================================================
    def make_pose_from_data(self, data):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(data["x"])
        pose.pose.position.y = float(data["y"])
        pose.pose.position.z = float(data["z"])
        pose.pose.orientation.x = float(data["qx"])
        pose.pose.orientation.y = float(data["qy"])
        pose.pose.orientation.z = float(data["qz"])
        pose.pose.orientation.w = float(data["qw"])
        return pose

    def make_pose_from_xy(self, x: float, y: float, yaw: float = 0.0):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def make_pose_from_point(self, point_msg):
        return self.make_pose_from_xy(point_msg.point.x, point_msg.point.y)

    def dock_pose(self):
        return self.make_pose_from_data(self.dock_pose_data)

    # =====================================================
    # Action Server: goal_cb / cancel_cb / execute_cb
    # =====================================================
    def goal_cb(self, goal_request):
        if goal_request.mode == "EMERGENCY_DISPATCH":
            if self.mode in EMERGENCY_SUBMODES:
                self.get_logger().warn("[goal_cb] 이미 응급 대응 중, 중복 무시")
                return GoalResponse.REJECT
            self.get_logger().warn("[goal_cb] EMERGENCY_DISPATCH -> 무조건 수락, 기존 임무 선점")
            return GoalResponse.ACCEPT

        if self.mode in EMERGENCY_SUBMODES or self.mode == RobotMode.ESTOP:
            self.get_logger().warn("[goal_cb] 응급/정지 중, 다른 요청 거부")
            return GoalResponse.REJECT

        # 실행 중인 HOTPLACE_DISPATCH에는 같은 우선순위의 최신 좌표도 수락한다.
        # 새 요청이 들어오면 기존 핫플레이스 골을 선점하고 최신 좌표로 갱신한다.
        if (
            goal_request.mode == "HOTPLACE_DISPATCH"
            and self.mode == RobotMode.HOTPLACE_RUNNING
        ):
            self.last_hotplace_request_time = time.monotonic()
            self._cancel_timer("hotplace_resume_timer")
            self.get_logger().info(
                "[goal_cb] 최신 HOTPLACE_DISPATCH 수락: 기존 핫플레이스 임무 갱신"
            )
            return GoalResponse.ACCEPT

        busy = (
            self.active_goal_handle is not None
            or self.waiting_patrol_stop_ack
            or self.mode in (RobotMode.GUIDE_RUNNING, RobotMode.LUGGAGE_RUNNING)
        )
        if busy and goal_request.priority <= self.current_priority:
            self.get_logger().info("[goal_cb] 우선순위 낮음, 거부")
            return GoalResponse.REJECT

        if goal_request.mode == "HOTPLACE_DISPATCH":
            self.last_hotplace_request_time = time.monotonic()
            self._cancel_timer("hotplace_resume_timer")

        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info("[cancel_cb] 취소 요청 수신")
        self.cancel_all_timers()
        if self.active_goal_handle is not None:
            self.cancel_current_goal()
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        """threading.Event를 폴링하는 방식 (asyncio 루프가 없는 rclpy 환경에서 안전하게 동작).
        이 함수는 MultiThreadedExecutor의 한 스레드를 점유하며 블로킹 대기하지만,
        ReentrantCallbackGroup + 여러 스레드 덕분에 다른 콜백(nav_result_callback 등)은
        별도 스레드에서 계속 실행되어 _finish_mission()을 정상적으로 호출할 수 있다.

        기존 임무 -> 새 임무 전환 안전장치
        --------------------------------
        goal_cb는 priority 비교로 accept/reject만 결정할 뿐, 기존 임무(NAV 또는
        FOLLOW)를 실제로 끊지는 않는다. 여기서 "취소 요청 -> 기존 execute_cb가
        완전히 끝날 때까지 대기(_mission_lock) -> 그 다음에야 새 임무 시작" 순서를
        강제해서, 서로 다른 goal의 execute_cb가 동시에 Nav2/FOLLOW 파이프라인을
        건드리는 상황을 막는다."""
        req = goal_handle.request

        # 핫플레이스 새 요청 수신 자체가 5초 무수신 대기를 중단시킨다.
        if req.mode == "HOTPLACE_DISPATCH":
            self.last_hotplace_request_time = time.monotonic()
            self._cancel_timer("hotplace_resume_timer")

        if self._active_flow == "FOLLOW":
            self.get_logger().warn(
                f"[execute_cb] 기존 FOLLOW 임무 취소 요청 후 종료 대기 (new mode={req.mode})"
            )
            self.request_follow_cancel()
        elif self.active_goal_handle is not None:
            self.get_logger().warn(
                f"[execute_cb] 기존 NAV 임무 취소 요청 후 종료 대기 (new mode={req.mode})"
            )
            self.cancel_current_goal()
        elif self.waiting_patrol_stop_ack:
            self.get_logger().warn(
                f"[execute_cb] 패트롤 정지 ACK 대기 중인 기존 HOTPLACE 임무 선점 "
                f"(new mode={req.mode})"
            )
            self.clear_pending_hotplace_start()
            self._finish_mission(False, "PREEMPTED")

        if not self._mission_lock.acquire(timeout=self._mission_preempt_timeout_sec):
            self.get_logger().error(
                f"[execute_cb] 기존 임무가 {self._mission_preempt_timeout_sec}초 내에 "
                f"끝나지 않음 - 새 goal(mode={req.mode}) 거부"
            )
            goal_handle.abort()
            return ExecuteMission.Result(success=False, final_state="ERROR")

        try:
            self._action_goal_handle = goal_handle
            self.current_priority = req.priority
            self._mission_done_event = threading.Event()
            self._mission_result = None

            feedback = ExecuteMission.Feedback()
            feedback.current_mode = req.mode
            feedback.progress_pct = 0.0
            goal_handle.publish_feedback(feedback)

            if req.mode == "EMERGENCY_DISPATCH":
                self._active_flow = "NAV"
                self.preempt_to_emergency(req.dest_x, req.dest_y)
            elif req.mode == "GUIDE":
                # GUIDE는 중앙 노드가 전달한 좌표를 2_back_move.py가 이해하는
                # 목적지 이름으로 변환한 뒤 /robot3/mission_cmd로 위임한다.
                # 실제 Nav2 이동과 사람 추적 기반 주행은 2_back_move.py가 담당하고,
                # 도착 완료는 /robot3/mission_complete로 받아 Action을 종료한다.
                goal_name = resolve_follow_goal_name(req.dest_x, req.dest_y)
                if goal_name is None:
                    self.get_logger().error(
                        f"[execute_cb] GUIDE: dest=({req.dest_x:.3f},{req.dest_y:.3f})에 "
                        "일치하는 목적지가 FOLLOW_GOAL_NAME_TABLE에 없음 - 거부"
                    )
                    goal_handle.abort()
                    return ExecuteMission.Result(success=False, final_state="ERROR")

                self.mode = RobotMode.GUIDE_RUNNING
                self.cancel_all_timers()
                self._active_flow = "FOLLOW"
                self._follow_cancel_sent = False
                self.send_follow_mission_cmd(goal_name)
            elif req.mode == "LUGGAGE_ASSIST":
                # LUGGAGE_ASSIST는 특정 목적지 개념이 없다 (그 자리에서 바로 추종 시작).
                # dest_x/dest_y가 FOLLOW_GOAL_NAME_TABLE과 안 맞아도 실패시키지 않는다 -
                # 성공/종료 판단은 오직 /robot3/mission_complete 수신 여부로만 한다.
                goal_name = resolve_follow_goal_name(req.dest_x, req.dest_y)
                if goal_name is not None:
                    # 좌표가 우연히 특정 목적지와 일치하면(예: 픽업 지점 경유가 필요한 경우)
                    # 참고용으로 mission_cmd도 함께 보내준다. 매칭 안 되어도 무시하고 계속 진행.
                    self.get_logger().info(
                        f"[execute_cb] LUGGAGE_ASSIST: goal_name={goal_name} 매칭됨 -> mission_cmd도 전송"
                    )
                else:
                    self.get_logger().info(
                        "[execute_cb] LUGGAGE_ASSIST: dest 좌표가 FOLLOW_GOAL_NAME_TABLE과 "
                        "안 맞음 - mission_cmd 생략, /robot3/mission_complete 수신만으로 종료 판단"
                    )

                self.mode = RobotMode.LUGGAGE_RUNNING
                self.cancel_all_timers()
                self._active_flow = "FOLLOW"
                self._follow_cancel_sent = False
                if goal_name is not None:
                    self.send_follow_mission_cmd(goal_name)
            elif req.mode == "HOTPLACE_DISPATCH":
                # 패트롤이 Nav2 골을 사용 중이므로 먼저 /rb3_standby=True를 보내고,
                # 패트롤 노드가 /rb3_standby_done=True로 취소 완료를 알린 뒤에만
                # HOTPLACE_DISPATCH Nav2 골을 전송한다.
                self._active_flow = "NAV"
                self.mode = RobotMode.HOTPLACE_RUNNING
                self.cancel_all_timers()
                self.request_patrol_stop_then_hotplace(
                    self.make_pose_from_xy(req.dest_x, req.dest_y)
                )
            else:
                self.get_logger().warn(f"알 수 없는 mode: {req.mode}")
                goal_handle.abort()
                return ExecuteMission.Result(success=False, final_state="ERROR")

            # 내부 FSM(콜백 체인)이 끝날 때까지 폴링 대기 (0.2초 간격, 취소 요청도 함께 체크)
            while not self._mission_done_event.wait(timeout=0.2):
                if goal_handle.is_cancel_requested:
                    if self._active_flow == "FOLLOW":
                        self.request_follow_cancel()
                        # service_cancel=True를 받은 4_front_move.py가 복귀/도킹 후
                        # mission_complete를 발행할 때까지 계속 대기한다.
                    else:
                        self.cancel_all_timers()
                        if self.waiting_patrol_stop_ack:
                            self.clear_pending_hotplace_start()
                            self.mode = RobotMode.IDLE
                            self.publish_start_patrol()
                            self._finish_mission(False, "CANCELED")
                        elif self.active_goal_handle is not None:
                            self.cancel_current_goal()
                        # cancel_current_goal의 결과 콜백이 결국 _finish_mission을 호출할 때까지 계속 대기

            success, final_state = self._mission_result or (False, "ERROR")

            if goal_handle.is_cancel_requested and not success:
                goal_handle.canceled()
                self._action_goal_handle = None
                self.current_priority = 0
                self._active_flow = None
                return ExecuteMission.Result(success=False, final_state="PREEMPTED")

            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            self._action_goal_handle = None
            self.current_priority = 0
            self._active_flow = None
            return ExecuteMission.Result(success=success, final_state=final_state)
        finally:
            self._mission_lock.release()

    def _finish_mission(self, success: bool, final_state: str):
        """내부 FSM이 최종 상태(성공/실패)에 도달했을 때 호출 -> execute_cb를 깨움."""
        self._mission_result = (success, final_state)
        if self._mission_done_event is not None:
            self._mission_done_event.set()

    # =====================================================
    # HOTPLACE_DISPATCH <-> 패트롤 정지/재시작 연동
    # =====================================================
    def request_patrol_stop_then_hotplace(self, pose):
        """패트롤 정지 완료 ACK를 받은 뒤 핫플레이스 Nav2 골을 전송한다."""
        self.clear_pending_hotplace_start()
        self.pending_hotplace_pose = pose
        self.waiting_patrol_stop_ack = True

        msg = Bool()
        msg.data = True
        self.rb3_standby_pub.publish(msg)
        self.get_logger().warn(
            "[HOTPLACE] /rb3_standby=True 발행, 패트롤 골 취소 완료 ACK 대기"
        )

        self.patrol_stop_ack_timer = self.create_timer(
            self.patrol_stop_ack_timeout_sec,
            self.patrol_stop_ack_timeout_callback,
            callback_group=self.cb_group,
        )

    def rb3_standby_done_callback(self, msg: Bool):
        if not msg.data or not self.waiting_patrol_stop_ack:
            return

        pose = self.pending_hotplace_pose
        self.waiting_patrol_stop_ack = False
        self.pending_hotplace_pose = None
        self._cancel_timer("patrol_stop_ack_timer")

        if self.mode != RobotMode.HOTPLACE_RUNNING or pose is None:
            self.get_logger().warn(
                "[HOTPLACE] 늦게 도착한 패트롤 정지 ACK 무시"
            )
            return

        self.get_logger().warn(
            "[HOTPLACE] 패트롤 골 취소 완료 확인, 핫플레이스 Nav2 골 전송"
        )
        self.send_nav_goal("HOTPLACE_DISPATCH", pose)

    def patrol_stop_ack_timeout_callback(self):
        self._cancel_timer("patrol_stop_ack_timer")
        if not self.waiting_patrol_stop_ack:
            return

        self.get_logger().error(
            f"[HOTPLACE] {self.patrol_stop_ack_timeout_sec:.1f}초 동안 "
            "/rb3_standby_done ACK를 받지 못함"
        )
        self.clear_pending_hotplace_start()
        self.mode = RobotMode.ERROR
        self.publish_start_patrol()
        self._finish_mission(False, "PATROL_STOP_TIMEOUT")

    def clear_pending_hotplace_start(self):
        self._cancel_timer("patrol_stop_ack_timer")
        self.waiting_patrol_stop_ack = False
        self.pending_hotplace_pose = None

    def schedule_patrol_resume_after_hotplace(self):
        self._cancel_timer("hotplace_resume_timer")
        self.hotplace_resume_timer = self.create_timer(
            self.hotplace_patrol_resume_delay_sec,
            self.hotplace_resume_timer_callback,
            callback_group=self.cb_group,
        )
        self.get_logger().info(
            f"[HOTPLACE] 추가 HOTPLACE_DISPATCH를 "
            f"{self.hotplace_patrol_resume_delay_sec:.1f}초 대기"
        )

    def hotplace_resume_timer_callback(self):
        self._cancel_timer("hotplace_resume_timer")

        # 타이머 만료 직전에 새 핫플레이스 요청이 들어온 경합을 방지한다.
        if self.last_hotplace_request_time is not None:
            elapsed = time.monotonic() - self.last_hotplace_request_time
            if elapsed < self.hotplace_patrol_resume_delay_sec:
                remaining = self.hotplace_patrol_resume_delay_sec - elapsed
                self.hotplace_resume_timer = self.create_timer(
                    max(0.05, remaining),
                    self.hotplace_resume_timer_callback,
                    callback_group=self.cb_group,
                )
                return

        if (
            self.mode != RobotMode.IDLE
            or self.active_goal_handle is not None
            or self.waiting_patrol_stop_ack
        ):
            self.get_logger().info(
                "[HOTPLACE] 다른 임무가 활성화되어 패트롤 재시작 생략"
            )
            return

        self.publish_start_patrol()

    def publish_start_patrol(self):
        msg = Bool()
        msg.data = True
        self.start_patrol_pub.publish(msg)
        self.get_logger().warn(
            "[HOTPLACE] /start_patrol=True 발행: POINT_1부터 패트롤 재시작"
        )

    # =====================================================
    # GUIDE/LUGGAGE_ASSIST -> 기존 추종 파이프라인(2_back_move.py/4_front_move.py) 위임
    # =====================================================
    def send_follow_mission_cmd(self, goal_name: str):
        """중앙노드에서 받은 목적지를 mission_cmd로 발행해 기존 코드에 위임한다."""
        msg = String()
        msg.data = goal_name
        self.mission_cmd_pub.publish(msg)
        self.get_logger().info(f"[FOLLOW] mission_cmd 발행: {goal_name}")

    def request_follow_cancel(self):
        """선점/취소 요청을 service_cancel로 변환해 기존 코드가 복귀하도록 한다."""
        if self._follow_cancel_sent:
            return
        self._follow_cancel_sent = True
        msg = Bool()
        msg.data = True
        self.service_cancel_pub.publish(msg)
        self.get_logger().warn("[FOLLOW] service_cancel=True 발행 (선점/취소)")

    def mission_complete_callback(self, msg: Bool):
        """위임된 GUIDE/LUGGAGE_ASSIST의 완료 신호를 처리한다.

        GUIDE는 2_back_move.py가 목적지 도착 후 발행한 완료 신호를 사용하고,
        LUGGAGE_ASSIST는 4_front_move.py가 서비스 종료 후 발행한 완료 신호를 사용한다.
        """
        if self.mode not in (
            RobotMode.GUIDE_RUNNING,
            RobotMode.LUGGAGE_RUNNING,
        ):
            # FOLLOW 위임 중이 아닐 때 온 신호는 무시 (예: 수동 테스트, 잔여 메시지)
            return

        success = bool(msg.data)
        self.mode = RobotMode.IDLE if success else RobotMode.ERROR
        self.get_logger().info(f"[FOLLOW] mission_complete 수신: success={success}")
        self._finish_mission(success, "IDLE" if success else "ERROR")

    # =====================================================
    # 응급 플로우 (원본 로직 그대로, 트리거만 Action goal로 교체)
    # =====================================================
    def preempt_to_emergency(self, x: float, y: float):
        self.cancel_all_timers()
        self.mode = RobotMode.EMERGENCY_DISPATCH

        emergency_pose = self.make_pose_from_xy(x, y)
        self.pending_goal_name = "EMERGENCY"
        self.pending_pose = emergency_pose

        if self.active_goal_handle is not None:
            self.get_logger().warn("Emergency dispatch: 기존 goal 선점")
            self.cancel_current_goal()
        else:
            self.send_pending_goal()

    def send_pending_goal(self):
        if self.pending_goal_name is None:
            return
        name, pose = self.pending_goal_name, self.pending_pose
        self.pending_goal_name = None
        self.pending_pose = None
        self.send_nav_goal(name, pose)

    def start_clearing_people(self):
        self.mode = RobotMode.CLEARING_PEOPLE
        self.get_logger().warn("Emergency point arrived. Beep beep!")
        self.beep()
        self.clearing_timer = self.create_timer(self.clearing_wait_sec, self.clearing_done)

    def clearing_done(self):
        self._cancel_timer("clearing_timer")
        if self.mode != RobotMode.CLEARING_PEOPLE:
            return
        self.start_rgbd_waiting()

    def start_rgbd_waiting(self):
        if self.latest_rgbd_point is not None:
            self.send_final_approach()
            return
        self.mode = RobotMode.RGBD_WAITING
        self.get_logger().warn("Waiting RGB-D fall person point...")
        self.rgbd_wait_timer = self.create_timer(self.rgbd_wait_timeout_sec, self.rgbd_wait_timeout)

    def rgbd_point_callback(self, msg):
        self.latest_rgbd_point = msg
        self.get_logger().info(f"rgbd_fall_person_point: x={msg.point.x:.3f}, y={msg.point.y:.3f}")
        if self.mode == RobotMode.RGBD_WAITING:
            self.send_final_approach()

    def send_final_approach(self):
        if self.latest_rgbd_point is None:
            return
        self._cancel_timer("rgbd_wait_timer")
        self.mode = RobotMode.FINAL_APPROACH
        self.get_logger().warn("Final approach to fallen person")
        self.send_nav_goal("FALLEN_PERSON", self.make_pose_from_point(self.latest_rgbd_point))

    def rgbd_wait_timeout(self):
        self._cancel_timer("rgbd_wait_timer")
        if self.mode != RobotMode.RGBD_WAITING:
            return
        self.get_logger().warn("RGB-D point timeout. Start emergency work here.")
        self.start_emergency_work()

    def start_emergency_work(self):
        self.mode = RobotMode.EMERGENCY_WORKING
        self.get_logger().info("Emergency work start")
        self.emergency_timer = self.create_timer(5.0, self.emergency_done)

    def emergency_done(self):
        self._cancel_timer("emergency_timer")
        if self.mode != RobotMode.EMERGENCY_WORKING:
            return
        self.get_logger().info("Emergency work done. Return dock.")
        self.mode = RobotMode.RETURNING
        self.send_nav_goal("DOCK", self.dock_pose())

    def beep(self):
        # 구급차 사이렌: 고음/저음을 0.5초씩 번갈아 재생, 총 5초(10음)
        note = lambda freq: AudioNote(frequency=freq, max_runtime=Duration(sec=0, nanosec=500000000))
        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [note(f) for f in [960, 600] * 5]
        self.audio_pub.publish(msg)

    # =====================================================
    # Nav2 Action (원본 유지)
    # =====================================================
    def send_nav_goal(self, goal_name, pose):
        if self.mode == RobotMode.ESTOP:
            return
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 action server not available")
            self.mode = RobotMode.ERROR
            self._finish_mission(False, "ERROR")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.active_goal_name = goal_name
        self.last_nav_status = "SENDING"

        self.get_logger().info(
            f"Send goal [{goal_name}] x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}")

        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.nav_feedback_callback)
        future.add_done_callback(lambda f: self.nav_goal_response_callback(f, goal_name))

    def nav_goal_response_callback(self, future, goal_name):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal response error: {e}")
            self.mode = RobotMode.ERROR
            self._finish_mission(False, "ERROR")
            return

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected: {goal_name}")
            self.active_goal_handle = None
            self.active_goal_name = None
            self.mode = RobotMode.ERROR
            self._finish_mission(False, "ERROR")
            return

        self.active_goal_handle = goal_handle
        self.last_nav_status = "RUNNING"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.nav_result_callback(f, goal_name))

    def nav_feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.last_nav_status = f"RUNNING {dist:.2f}m"
        if self._action_goal_handle is not None:
            fb = ExecuteMission.Feedback()
            fb.current_mode = self.mode.value
            fb.progress_pct = max(0.0, 100.0 - dist * 10.0)  # 대략치, 필요시 총거리 기반으로 개선
            self._action_goal_handle.publish_feedback(fb)

    def nav_result_callback(self, future, goal_name):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Result error: {e}")
            self.mode = RobotMode.ERROR
            self._finish_mission(False, "ERROR")
            return

        status = result.status
        self.last_result_code = self.goal_status_to_text(status)
        self.get_logger().info(f"Goal result: {goal_name}, {self.last_result_code}")

        self.active_goal_handle = None
        self.active_goal_name = None

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Goal not succeeded: {goal_name}, {self.last_result_code}")
            if self.pending_goal_name is not None:
                self.get_logger().warn(f"Send pending goal after {self.last_result_code}")
                self.send_pending_goal()
                return
            self.mode = RobotMode.ERROR
            self._finish_mission(False, self.last_result_code)
            return

        # ---- 성공 후 FSM 전이 ----
        if goal_name in ("GUIDE", "LUGGAGE_ASSIST"):
            # TODO(팀원): 도착 후 세부 동작(안내 멘트/추종 제어 등)을 여기서 호출
            self.get_logger().info(f"[TODO] {goal_name} 세부 동작 실행 - 기존 코드 연결 필요")
            self.mode = RobotMode.IDLE
            self._finish_mission(True, "IDLE")

        elif goal_name == "HOTPLACE_DISPATCH":
            self.get_logger().info(
                "HOTPLACE_DISPATCH 도착. 추가 핫플레이스 요청을 대기한다."
            )
            self.mode = RobotMode.IDLE
            self.schedule_patrol_resume_after_hotplace()
            self._finish_mission(True, "IDLE")

        elif goal_name == "EMERGENCY":
            self.start_clearing_people()

        elif goal_name == "FALLEN_PERSON":
            self.start_emergency_work()

        elif goal_name == "DOCK":
            self.mode = RobotMode.DOCKED
            self.latest_rgbd_point = None
            self.last_nav_status = "DOCKED"
            self._finish_mission(True, "DOCKED")

    def cancel_current_goal(self):
        if self.active_goal_handle is None:
            return
        self.last_nav_status = "CANCELING"
        self.active_goal_handle.cancel_goal_async().add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Cancel error: {e}")
            return
        self.last_nav_status = "CANCEL_ACCEPTED" if response.goals_canceling else "CANCEL_REJECTED"
        # 취소 완료 후, 대기 중인 pending goal(응급 등)이 있으면 이어서 실행
        if self.pending_goal_name is not None:
            self.send_pending_goal()

    @staticmethod
    def goal_status_to_text(status):
        return {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN", GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING", GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(status, str(status))

    # =====================================================
    # ESTOP / TIMER / STATUS
    # =====================================================
    def on_emergency_stop(self, request, response):
        self.get_logger().error(f"ESTOP: {request.reason}")
        self.mode = RobotMode.ESTOP
        self.cancel_all_timers()
        self.pending_goal_name = None
        self.pending_pose = None
        self.clear_pending_hotplace_start()
        if self._active_flow == "FOLLOW":
            self.request_follow_cancel()
        elif self.active_goal_handle is not None:
            self.cancel_current_goal()
        self.publish_zero_cmd()
        self._finish_mission(False, "ESTOP")
        response.success = True
        return response

    def _cancel_timer(self, attr_name):
        timer = getattr(self, attr_name)
        if timer is not None:
            timer.cancel()
            setattr(self, attr_name, None)

    def cancel_all_timers(self):
        for name in (
            "goal2_timer",
            "clearing_timer",
            "rgbd_wait_timer",
            "emergency_timer",
            "patrol_stop_ack_timer",
            "hotplace_resume_timer",
        ):
            self._cancel_timer(name)

    def publish_zero_cmd(self):
        self.cmd_vel_pub.publish(Twist())

    def estop_zero_timer(self):
        if self.mode == RobotMode.ESTOP:
            self.publish_zero_cmd()

    def publish_status(self):
        msg = RobotStatus()
        msg.robot_id = self.robot_id
        msg.mode = MODE_TO_ENUM.get(self.mode, RobotStatus.MODE_ERROR)
        msg.battery_pct = self.battery_pct
        msg.pose = PoseStamped()  # TODO(팀원): 실제 현재 pose(AMCL 등) 연결
        msg.pose.header.frame_id = self.frame_id
        msg.aed_loaded = self.aed_loaded
        msg.available = self.mode not in (RobotMode.ESTOP, RobotMode.ERROR)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotMissionFsm()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero_cmd()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()