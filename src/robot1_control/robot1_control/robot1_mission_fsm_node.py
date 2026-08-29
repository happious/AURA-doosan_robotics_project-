#!/usr/bin/env python3
"""
Robot1 mission FSM

지원 기능
---------
- 지정된 6개 좌표를 1→2→3→4→5→6 순서로 무한 순찰
- /rb1_standby=True 수신 시 즉시 정지
- GUIDE 목적지 안내 후 /rb1_service_end=True 수신 전까지 현장 대기
- /rb1_service_end=True 수신 시 일반 안내/대기 상태에서 순찰 복귀
- HOTPLACE_DISPATCH 목표 도착 후 3초 대기 뒤 순찰 복귀
- EMERGENCY_DISPATCH -> 현장 이동 -> 경고음 -> RGB-D 좌표 최종 접근
- /emergency_end=True 수신 시 응급 임무 종료 후 순찰 복귀
- EmergencyStop 서비스
- RobotStatus 발행
"""


import threading
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
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
    PATROLLING = "PATROLLING"
    GUIDE_RUNNING = "GUIDE_RUNNING"
    HOTPLACE_RUNNING = "HOTPLACE_RUNNING"

    EMERGENCY_DISPATCH = "EMERGENCY_DISPATCH"
    CLEARING_PEOPLE = "CLEARING_PEOPLE"
    RGBD_WAITING = "RGBD_WAITING"
    FINAL_APPROACH = "FINAL_APPROACH"
    EMERGENCY_WORKING = "EMERGENCY_WORKING"

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
    RobotMode.PATROLLING: RobotStatus.MODE_IDLE,
    RobotMode.GUIDE_RUNNING: RobotStatus.MODE_GUIDE,
    RobotMode.HOTPLACE_RUNNING: RobotStatus.MODE_HOTPLACE_DISPATCH,
    RobotMode.EMERGENCY_DISPATCH: RobotStatus.MODE_EMERGENCY_DISPATCH,
    RobotMode.CLEARING_PEOPLE: RobotStatus.MODE_EMERGENCY_DISPATCH,
    RobotMode.RGBD_WAITING: RobotStatus.MODE_EMERGENCY_DISPATCH,
    RobotMode.FINAL_APPROACH: RobotStatus.MODE_EMERGENCY_DISPATCH,
    RobotMode.EMERGENCY_WORKING: RobotStatus.MODE_EMERGENCY_DISPATCH,
    RobotMode.ESTOP: RobotStatus.MODE_ERROR,
    RobotMode.ERROR: RobotStatus.MODE_ERROR,
}

EMERGENCY_PRIORITY = 10


class RobotMissionFsm(Node):
    def __init__(self):
        super().__init__("robot_mission_fsm")
        self.cb_group = ReentrantCallbackGroup()

        # =========================
        # 파라미터
        # =========================
        self.declare_parameter("aed_loaded", False)
        self.declare_parameter("battery_pct", 100.0)
        self.aed_loaded = (
            self.get_parameter("aed_loaded").get_parameter_value().bool_value
        )
        self.battery_pct = (
            self.get_parameter("battery_pct").get_parameter_value().double_value
        )
        self.robot_id = self.get_namespace().strip("/") or "robot_unknown"

        self.frame_id = "map"
        self.clearing_wait_sec = 4.0
        self.rgbd_wait_timeout_sec = 10.0

        # 응급 지점 도착 후 경고음 재생 중 제자리 회전 속도(rad/s)
        # 양수는 반시계 방향, 음수는 시계 방향이다.
        self.emergency_rotate_speed = 0.35

        # =========================
        # 6개 지정 좌표 무한 순찰 설정
        # =========================
        # 사용자가 지정한 좌표를 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 1 순서로 반복한다.
        # 입력에 중복되어 있던 (-4.2049027420, 4.4259094788) 좌표는 한 번만 사용한다.
        self.patrol_enabled = True
        self.patrol_points = [
            (-2.565690142822833,  1.4118863657337248),  # 1
            (-3.150715584017791,  2.4975126588150810),  # 2
            (-2.354073191216989,  4.3149866230668900),  # 3
            (-4.204902742019720,  4.4259094788374720),  # 4
            (-3.8393029464814705, 2.8743053073133880),  # 5
            (-4.1767676154410625, 0.5401599172746740),  # 6
        ]

        self.patrol_index = 0
        self.patrol_retry_timer = None
        self.patrol_start_timer = None
        self.standby_active = False

        # =========================
        # FSM 상태 변수
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
        self.emergency_end_enabled = False

        # 응급 지점으로 이동 중 반복 경고음 타이머
        self.emergency_travel_beep_timer = None

        # 응급 지점 도착 후 경고음 재생 중 제자리 회전 타이머
        self.emergency_rotate_timer = None

        self.clearing_timer = None
        self.rgbd_wait_timer = None
        self.hotplace_wait_timer = None
        self.patrol_retry_timer = None
        self.patrol_start_timer = None

        # Action Server <-> 내부 FSM 콜백 브리지용 (threading.Event 사용 - asyncio 루프 불필요)
        self._action_goal_handle = None      # 현재 처리 중인 ExecuteMission goal_handle
        self._mission_done_event = threading.Event()
        self._mission_result = None          # (success: bool, final_state: str)

        # service_end / emergency_end 이후 순찰 복귀를 Action 종료 타이밍과 분리한다.
        self.resume_patrol_requested = False
        self.mission_end_reason = None

        # =========================
        # ROS Interface
        # =========================
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/robot1/navigate_to_pose",
            callback_group=self.cb_group,
        )

        self._mission_server = ActionServer(
            self, ExecuteMission, "execute_mission",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb_group)

        self.create_service(EmergencyStop, "emergency_stop", self.on_emergency_stop,
                             callback_group=self.cb_group)

        # 응급 중 최종 접근 좌표
        self.rgbd_point_sub = self.create_subscription(
            PointStamped, "/rgbd_fall_person_point", self.rgbd_point_callback, 10,
            callback_group=self.cb_group)

        self.standby_sub = self.create_subscription(
            Bool, "/rb1_standby", self.standby_callback, 10,
            callback_group=self.cb_group)
    
        self.emergency_end_sub = self.create_subscription(
            Bool, "/emergency_end", self.emergency_end_callback, 10,
            callback_group=self.cb_group)

        self.service_end_sub = self.create_subscription(
            Bool, "/rb1_service_end", self.service_end_callback, 10,
            callback_group=self.cb_group)

        self.status_pub = self.create_publisher(RobotStatus, "status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.audio_pub = self.create_publisher(AudioNoteVector, "cmd_audio", 10)

        self.create_timer(1.0, self.publish_status, callback_group=self.cb_group)
        self.create_timer(0.1, self.estop_zero_timer, callback_group=self.cb_group)

        # 노드 시작 직후 지정된 6개 좌표 순찰 시작
        self.patrol_start_timer = self.create_timer(
            2.0, self.start_patrol_once, callback_group=self.cb_group)

        self.get_logger().info(
            f"[robot_mission_fsm] {self.robot_id} 시작 (aed_loaded={self.aed_loaded})"
        )

    # =====================================================
    # Pose 생성
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

        if (self.mode != RobotMode.PATROLLING and
                self.active_goal_handle is not None and
                goal_request.priority <= self.current_priority):
            self.get_logger().info("[goal_cb] 우선순위 낮음, 거부")
            return GoalResponse.REJECT

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
        별도 스레드에서 계속 실행되어 _finish_mission()을 정상적으로 호출할 수 있다."""
        req = goal_handle.request
        self._action_goal_handle = goal_handle
        self.current_priority = req.priority
        self._mission_done_event = threading.Event()
        self._mission_result = None

        feedback = ExecuteMission.Feedback()
        feedback.current_mode = req.mode
        feedback.progress_pct = 0.0
        goal_handle.publish_feedback(feedback)

        if req.mode == "EMERGENCY_DISPATCH":
            self.preempt_to_emergency(req.dest_x, req.dest_y)
        elif req.mode == "GUIDE":
            self.preempt_patrol_to_mission(
                "GUIDE",
                self.make_pose_from_xy(req.dest_x, req.dest_y),
                RobotMode.GUIDE_RUNNING,
            )
        elif req.mode == "HOTPLACE_DISPATCH":
            self.preempt_patrol_to_mission(
                "HOTPLACE_DISPATCH",
                self.make_pose_from_xy(req.dest_x, req.dest_y),
                RobotMode.HOTPLACE_RUNNING,
            )
        else:
            self.get_logger().warn(f"알 수 없는 mode: {req.mode}")
            goal_handle.abort()
            return ExecuteMission.Result(success=False, final_state="ERROR")

        # 내부 FSM(콜백 체인)이 끝날 때까지 폴링 대기 (0.2초 간격, 취소 요청도 함께 체크)
        while not self._mission_done_event.wait(timeout=0.2):
            if goal_handle.is_cancel_requested:
                self.cancel_all_timers()
                if self.active_goal_handle is not None:
                    self.cancel_current_goal()
                # cancel_current_goal의 결과 콜백이 결국 _finish_mission을 호출할 때까지 계속 대기

        success, final_state = self._mission_result or (False, "ERROR")

        if goal_handle.is_cancel_requested and not success:
            goal_handle.canceled()
            self._action_goal_handle = None
            self.current_priority = 0
            return ExecuteMission.Result(success=False, final_state="PREEMPTED")

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self._action_goal_handle = None
        self.current_priority = 0

        # 종료 토픽이 먼저 들어온 경우 execute_cb가 완전히 끝난 뒤 순찰을 확실히 재개한다.
        if self.resume_patrol_requested:
            self.schedule_patrol_resume(0.1)

        return ExecuteMission.Result(success=success, final_state=final_state)

    def _finish_mission(self, success: bool, final_state: str):
        """내부 FSM이 최종 상태(성공/실패)에 도달했을 때 호출 -> execute_cb를 깨움."""
        self._mission_result = (success, final_state)
        if self._mission_done_event is not None:
            self._mission_done_event.set()

    def request_mission_end_and_patrol(self, reason: str):
        """현재 일반/응급 임무를 종료하고 Nav2 취소 완료 후 순찰로 복귀한다."""
        self.get_logger().info(f"{reason}: 현재 임무 종료 및 순찰 복귀 요청")

        self.resume_patrol_requested = True
        self.mission_end_reason = reason
        self.standby_active = False
        self.cancel_all_timers()
        self.pending_goal_name = None
        self.pending_pose = None
        self.latest_rgbd_point = None
        self.mode = RobotMode.IDLE
        self.publish_zero_cmd()

        # 중앙 ExecuteMission Action을 즉시 완료시켜 execute_cb의 대기를 해제한다.
        if self._action_goal_handle is not None:
            self._finish_mission(True, "IDLE")

        # 이동 중 종료 신호가 들어오면 Nav2 goal부터 취소한다.
        # 결과 콜백에서 resume_patrol_requested를 확인해 순찰을 시작한다.
        if self.active_goal_handle is not None:
            self.get_logger().warn(
                f"{reason}: 이동 중인 Nav2 goal [{self.active_goal_name}] 취소 후 순찰 복귀")
            self.cancel_current_goal()
        else:
            self.schedule_patrol_resume(0.1)

    # =====================================================
    # 구역 내부 무한 순찰
    # =====================================================
    def start_patrol_once(self):
        self._cancel_timer("patrol_start_timer")
        self.get_logger().info(
            f"지정 순찰점 {len(self.patrol_points)}개 로드 완료: 1→2→3→4→5→6 무한 반복")
        self.start_patrol(reset_path=True)

    def start_patrol(self, reset_path=False):
        if not self.patrol_enabled or self.standby_active:
            return

        if self.mode in EMERGENCY_SUBMODES or self.mode in (
                RobotMode.ESTOP,
                RobotMode.ERROR,
                RobotMode.GUIDE_RUNNING,
                RobotMode.HOTPLACE_RUNNING):
            return

        if self._action_goal_handle is not None:
            self.schedule_patrol_resume(0.5)
            return

        if self.active_goal_handle is not None:
            return

        if reset_path:
            self.patrol_index = 0

        if not self.patrol_points:
            self.get_logger().error("지정된 순찰점이 없습니다.")
            self.mode = RobotMode.ERROR
            return

        self.resume_patrol_requested = False
        self.mission_end_reason = None
        self.mode = RobotMode.PATROLLING
        x, y = self.patrol_points[self.patrol_index]

        # 각 지점에 도착했을 때 다음 순찰점을 바라보도록 yaw를 자동 계산한다.
        next_index = (self.patrol_index + 1) % len(self.patrol_points)
        next_x, next_y = self.patrol_points[next_index]
        yaw = math.atan2(next_y - y, next_x - x)

        self.send_nav_goal(
            f"PATROL_ZONE_{self.patrol_index + 1}",
            self.make_pose_from_xy(x, y, yaw),
        )

    def schedule_patrol_resume(self, delay_sec=0.5):
        if self.standby_active:
            return
        if not self.patrol_enabled or self.mode in (
                RobotMode.ESTOP, RobotMode.ERROR):
            return

        self._cancel_timer("patrol_retry_timer")
        self.patrol_retry_timer = self.create_timer(
            delay_sec,
            self.patrol_resume_timer_cb,
        )

    def patrol_resume_timer_cb(self):
        self._cancel_timer("patrol_retry_timer")

        if self.mode not in (
                RobotMode.IDLE,
                RobotMode.PATROLLING):
            return

        self.start_patrol(reset_path=False)

    def advance_patrol(self):
        self.patrol_index = (
            self.patrol_index + 1
        ) % len(self.patrol_points)

        self.mode = RobotMode.PATROLLING
        self.schedule_patrol_resume(0.2)


    def standby_callback(self, msg):
        """로봇을 즉시 정지시킨다.

        True:
          - 순찰 재시작 타이머 취소
          - pending 순찰/임무 goal 제거
          - 현재 Nav2 goal 취소
          - standby_active=True 동안 0.1초마다 cmd_vel=0 반복 발행

        False:
          - standby 플래그만 해제
          - 자동 순찰은 재개하지 않음
          - /service_end=True를 받아야 순찰 복귀
        """
        requested = bool(msg.data)

        if requested:
            # 같은 True가 반복되어도 현재 goal 취소를 다시 시도한다.
            self.standby_active = True

            self.get_logger().warn(
                f"/rb1_standby=True 수신: 즉시 정지 시작 "
                f"(mode={self.mode.value}, "
                f"active_goal={self.active_goal_name})"
            )

            # 순찰이 다시 시작될 가능성을 차단
            self._cancel_timer("patrol_retry_timer")
            self._cancel_timer("patrol_start_timer")

            # 취소 후 실행될 pending goal 제거
            self.pending_goal_name = None
            self.pending_pose = None

            # 상태를 대기로 전환
            self.mode = RobotMode.IDLE
            self.last_nav_status = "STANDBY"

            # 먼저 속도 0 발행
            self.publish_zero_cmd()

            # 현재 Nav2 goal이 이미 승인된 상태면 즉시 취소
            if self.active_goal_handle is not None:
                self.get_logger().warn(
                    f"현재 Nav2 goal [{self.active_goal_name}] 취소 요청")
                self.cancel_current_goal()
            else:
                self.get_logger().warn(
                    "현재 active goal handle 없음. "
                    "goal이 늦게 승인되면 승인 콜백에서 즉시 취소합니다.")
            return

        # False만으로는 순찰을 다시 시작하지 않는다.
        self.standby_active = False
        self.last_nav_status = "STANDBY_RELEASED"
        self.publish_zero_cmd()

        self.get_logger().info(
            "/rb1_standby=False 수신: standby 플래그만 해제. "
            "/rb1_service_end=True 수신 전까지 제자리 대기")

    def service_end_callback(self, msg):
        """GUIDE/standby/일반 서비스 종료 후 현재 이동을 정리하고 순찰로 복귀한다."""
        if not msg.data:
            return

        if self.mode in EMERGENCY_SUBMODES:
            self.get_logger().info(
                f"/rb1_service_end=True 수신, 현재 모드({self.mode.value})는 응급 대응 중이므로 무시. "
                "/emergency_end=True를 사용하세요.")
            return

        if self.mode == RobotMode.HOTPLACE_RUNNING:
            self.get_logger().info(
                "/rb1_service_end=True 수신: 핫플레이스 임무도 즉시 종료하고 순찰로 복귀")

        if self.mode in (RobotMode.ESTOP, RobotMode.ERROR):
            self.get_logger().warn(
                f"/rb1_service_end=True 수신, 현재 모드({self.mode.value})에서는 순찰 복귀 불가")
            return

        self.request_mission_end_and_patrol("/rb1_service_end=True")

    def preempt_patrol_to_mission(self, goal_name, pose, next_mode):
        self.cancel_all_timers()
        self.mode = next_mode
        self.pending_goal_name = goal_name
        self.pending_pose = pose

        if self.active_goal_handle is not None:
            self.get_logger().warn(
                f"{goal_name}: 순찰 goal 취소 후 임무로 전환")
            self.cancel_current_goal()
        else:
            self.send_pending_goal()

    # =====================================================
    # 응급 플로우
    # =====================================================
    def preempt_to_emergency(self, x: float, y: float):
        self.cancel_all_timers()
        self.emergency_end_enabled = False
        self.mode = RobotMode.EMERGENCY_DISPATCH

        # 응급 출동을 시작하는 즉시 경고음을 한 번 재생하고,
        # 목적지에 도착할 때까지 약 4초 간격으로 반복한다.
        self.start_emergency_travel_beep()

        emergency_pose = self.make_pose_from_xy(x, y)
        self.pending_goal_name = "EMERGENCY"
        self.pending_pose = emergency_pose

        if self.active_goal_handle is not None:
            self.get_logger().warn("Emergency dispatch: 기존 goal 선점")
            self.cancel_current_goal()
        else:
            self.send_pending_goal()

    def send_pending_goal(self):
        if self.pending_goal_name is None or self.active_goal_handle is not None:
            return
        name, pose = self.pending_goal_name, self.pending_pose
        self.pending_goal_name = None
        self.pending_pose = None
        self.send_nav_goal(name, pose)

    def start_emergency_travel_beep(self):
        """응급 지점으로 이동하는 동안 경고음을 반복 재생한다."""
        self._cancel_timer("emergency_travel_beep_timer")
        self.get_logger().warn(
            "Emergency dispatch started. 이동 중 경고음을 반복 재생합니다."
        )
        self.beep()
        self.emergency_travel_beep_timer = self.create_timer(
            4.0,
            self.emergency_travel_beep_callback,
            callback_group=self.cb_group,
        )

    def emergency_travel_beep_callback(self):
        """EMERGENCY 목적지 이동 상태일 때만 반복 경고음을 재생한다."""
        if self.mode != RobotMode.EMERGENCY_DISPATCH:
            self._cancel_timer("emergency_travel_beep_timer")
            return
        self.get_logger().info("응급 지점 이동 중 경고음 재생")
        self.beep()

    def start_clearing_people(self):
        # 응급 지점 도착 후 경고음과 제자리 회전을 동시에 시작한다.
        self._cancel_timer("emergency_travel_beep_timer")
        self._cancel_timer("emergency_rotate_timer")
        self.publish_zero_cmd()

        self.mode = RobotMode.CLEARING_PEOPLE
        self.get_logger().warn(
            "Emergency point arrived. 경고음과 제자리 회전을 시작합니다."
        )

        self.beep()

        self.emergency_rotate_timer = self.create_timer(
            0.1,
            self.emergency_rotate_callback,
            callback_group=self.cb_group,
        )

        self._cancel_timer("clearing_timer")
        self.clearing_timer = self.create_timer(
            self.clearing_wait_sec,
            self.clearing_done,
            callback_group=self.cb_group,
        )

    def emergency_rotate_callback(self):
        # CLEARING_PEOPLE 상태에서만 제자리 회전 명령을 발행한다.
        if self.mode != RobotMode.CLEARING_PEOPLE:
            self._cancel_timer("emergency_rotate_timer")
            self.publish_zero_cmd()
            return

        if self.standby_active:
            self._cancel_timer("emergency_rotate_timer")
            self.publish_zero_cmd()
            return

        cmd = Twist()
        cmd.angular.z = float(self.emergency_rotate_speed)
        self.cmd_vel_pub.publish(cmd)

    def clearing_done(self):
        # 경고음·회전 시간을 끝내고 RGB-D 최종 좌표 대기로 넘어간다.
        self._cancel_timer("clearing_timer")
        self._cancel_timer("emergency_rotate_timer")
        self.publish_zero_cmd()

        if self.mode != RobotMode.CLEARING_PEOPLE:
            return

        self.emergency_end_enabled = True
        self.get_logger().warn(
            "경고음 및 제자리 회전 완료. "
            "이제 /emergency_end=True를 보내면 즉시 순찰로 복귀합니다."
        )
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
        self.publish_zero_cmd()
        self.get_logger().warn(
            "Emergency work start. /emergency_end=True 수신 전까지 현재 위치에서 대기")

    def emergency_end_callback(self, msg):
        """응급 지점 도착 후 경고음이 끝난 시점부터 종료를 허용한다."""
        if not msg.data:
            return

        if self.mode not in EMERGENCY_SUBMODES:
            self.get_logger().info(
                f"/emergency_end=True 수신, 현재 모드({self.mode.value})는 응급 모드가 아니므로 무시")
            return

        if not self.emergency_end_enabled:
            self.get_logger().warn(
                "/emergency_end=True 수신했지만 아직 경고음이 끝나지 않아 무시합니다."
            )
            return

        self.emergency_end_enabled = False
        self.request_mission_end_and_patrol("/emergency_end=True")

    def beep(self):
        # 응급환자 위치 도착 시: 삐(880Hz) -> 뽀(440Hz) -> 삐 -> 뽀
        # 각 음은 0.3초 동안 재생한다.
        note = lambda freq: AudioNote(
            frequency=int(freq),
            max_runtime=Duration(sec=0, nanosec=300000000),
        )

        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            # 삐뽀삐뽀 1회
            note(880),  # 삐
            note(440),  # 뽀
            note(880),  # 삐
            note(440),  # 뽀

            # 삐뽀삐뽀 2회
            note(880),  # 삐
            note(440),  # 뽀
            note(880),  # 삐
            note(440),  # 뽀

            # 삐뽀삐뽀 3회
            note(880),  # 삐
            note(440),  # 뽀
            note(880),  # 삐
            note(440),  # 뽀
        ]
        self.audio_pub.publish(msg)

    # =====================================================
    # Nav2 Action
    # =====================================================
    def send_nav_goal(self, goal_name, pose):
        if self.mode == RobotMode.ESTOP:
            return

        # standby 상태에서는 새로운 순찰 goal 자체를 보내지 않는다.
        if self.resume_patrol_requested and not goal_name.startswith("PATROL_ZONE_"):
            self.get_logger().warn(
                f"[{goal_name}] 종료 신호 이후 늦게 승인됨. 즉시 취소합니다.")
            self.publish_zero_cmd()
            self.cancel_current_goal()
            return

        if self.standby_active and goal_name.startswith("PATROL_ZONE_"):
            self.get_logger().warn(
                f"[{goal_name}] 전송 차단: standby_active=True")
            self.publish_zero_cmd()
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                f"Nav2 Action Server 연결 대기 중: "
                f"{self.nav_client._action_name}"
            )
            self.schedule_nav_goal_retry(goal_name, pose)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.active_goal_name = goal_name
        self.last_nav_status = "SENDING"

        self.get_logger().info(
            f"Send goal [{goal_name}] x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}")

        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.nav_feedback_callback)
        future.add_done_callback(lambda f: self.nav_goal_response_callback(f, goal_name))

    def schedule_nav_goal_retry(self, goal_name, pose, delay_sec=1.0):
        """Nav2 서버가 아직 검색되지 않았으면 노드를 ERROR로 종료하지 않고 재시도한다."""

        # 같은 목표를 중복 전송하지 않도록 이전 재시도 타이머를 정리한다.
        retry_timer = getattr(self, "nav_server_retry_timer", None)
        if retry_timer is not None:
            retry_timer.cancel()
            self.nav_server_retry_timer = None

        self.pending_nav_retry_goal_name = goal_name
        self.pending_nav_retry_pose = pose

        self.nav_server_retry_timer = self.create_timer(
            delay_sec,
            self.nav_server_retry_callback,
            callback_group=self.cb_group,
        )

    def nav_server_retry_callback(self):
        """Nav2 Action Server를 발견할 때까지 1초 간격으로 목표 전송을 재시도한다."""

        retry_timer = getattr(self, "nav_server_retry_timer", None)
        if retry_timer is not None:
            retry_timer.cancel()
            self.nav_server_retry_timer = None

        goal_name = getattr(self, "pending_nav_retry_goal_name", None)
        pose = getattr(self, "pending_nav_retry_pose", None)

        if goal_name is None or pose is None:
            return

        # 대기 중 standby/ESTOP이 들어오면 목표를 보내지 않는다.
        if self.mode == RobotMode.ESTOP or self.standby_active:
            self.pending_nav_retry_goal_name = None
            self.pending_nav_retry_pose = None
            self.publish_zero_cmd()
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                "Nav2 Action Server가 아직 준비되지 않았습니다. "
                "1초 후 다시 확인합니다: /robot1/navigate_to_pose"
            )
            self.schedule_nav_goal_retry(goal_name, pose, 1.0)
            return

        self.get_logger().info(
            "Nav2 Action Server 연결 완료: /robot1/navigate_to_pose"
        )

        self.pending_nav_retry_goal_name = None
        self.pending_nav_retry_pose = None
        self.send_nav_goal(goal_name, pose)

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
        result_future.add_done_callback(
            lambda f: self.nav_result_callback(f, goal_name)
        )

        # /rb1_standby=True가 goal 전송 후, goal 승인 전에 들어온 경우.
        # 늦게 승인된 순찰 goal을 여기서 즉시 취소한다.
        if self.standby_active and goal_name.startswith("PATROL_ZONE_"):
            self.get_logger().warn(
                f"[{goal_name}] goal이 늦게 승인됨. "
                "standby 상태이므로 즉시 취소합니다.")
            self.publish_zero_cmd()
            self.cancel_current_goal()

    def nav_feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.last_nav_status = f"RUNNING {dist:.2f}m"
        if self._action_goal_handle is not None:
            fb = ExecuteMission.Feedback()
            fb.current_mode = self.mode.value
            fb.progress_pct = max(0.0, 100.0 - dist * 10.0)
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

        # service_end/emergency_end 때문에 취소된 goal은 실패로 처리하지 않는다.
        if self.resume_patrol_requested:
            self.get_logger().info(
                f"[{goal_name}] 정리 완료({self.last_result_code}). 순찰을 재개합니다.")
            self.mode = RobotMode.IDLE
            self.schedule_patrol_resume(0.1)
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Goal not succeeded: {goal_name}, {self.last_result_code}")

            # 순찰 중 취소는 외부 임무 선점일 수 있으므로 ERROR로 보내지 않는다.
            if goal_name.startswith("PATROL_ZONE_"):
                if self.standby_active:
                    self.get_logger().info(
                        "순찰 goal 취소 완료: /rb1_standby=True 상태로 현재 위치 대기")
                    self.mode = RobotMode.IDLE
                    self.publish_zero_cmd()
                    return

                if self.pending_goal_name is not None:
                    self.get_logger().warn(
                        f"순찰 중단 후 pending goal 전송: {self.pending_goal_name}")
                    self.send_pending_goal()
                else:
                    # 장애물 때문에 특정 순찰점에 못 가면 다음 점으로 넘어가 순찰을 지속한다.
                    self.get_logger().warn(
                        f"{goal_name} 접근 실패. 다음 지정 순찰점으로 건너뜁니다.")
                    self.advance_patrol()
                return

            if self.pending_goal_name is not None:
                self.get_logger().warn(f"Send pending goal after {self.last_result_code}")
                self.send_pending_goal()
                return

            if (self._action_goal_handle is not None and
                    self._action_goal_handle.is_cancel_requested):
                self.mode = RobotMode.IDLE
                self._finish_mission(False, "PREEMPTED")
                self.publish_zero_cmd()
                self.get_logger().info(
                    "임무 취소 완료. /rb1_service_end=True 수신 전까지 현재 위치에서 대기")
                return

            self.mode = RobotMode.ERROR
            self._finish_mission(False, self.last_result_code)
            return

        # ---- 성공 후 FSM 전이 ----
        if goal_name.startswith("PATROL_ZONE_"):
            self.get_logger().info(f"{goal_name} 도착. 다음 지정 순찰점으로 이동.")
            self.advance_patrol()

        elif goal_name == "GUIDE":
            self.mode = RobotMode.GUIDE_RUNNING
            self.publish_zero_cmd()
            self.get_logger().info(
                "GUIDE 목적지 도착. Action은 성공 처리하고 "
                "/rb1_service_end=True 수신 전까지 현재 위치에서 안내 대기합니다.")
            self._finish_mission(True, "GUIDE_RUNNING")

        elif goal_name == "HOTPLACE_DISPATCH":
            self.start_hotplace_wait()

        elif goal_name == "EMERGENCY":
            self.start_clearing_people()

        elif goal_name == "FALLEN_PERSON":
            self.start_emergency_work()



    def start_hotplace_wait(self):
        """핫플레이스 목표 도착 후 현재 위치에서 3초 대기한다."""
        self.mode = RobotMode.HOTPLACE_RUNNING
        self.publish_zero_cmd()
        self.get_logger().info(
            "HOTPLACE_DISPATCH 도착. 현재 위치에서 3초 대기합니다.")

        self._cancel_timer("hotplace_wait_timer")
        self.hotplace_wait_timer = self.create_timer(
            3.0,
            self.hotplace_wait_done,
            callback_group=self.cb_group,
        )

    def hotplace_wait_done(self):
        """핫플레이스 도착 후 3초 대기가 끝나면 미션을 완료하고 순찰로 복귀한다."""
        self._cancel_timer("hotplace_wait_timer")

        if self.mode != RobotMode.HOTPLACE_RUNNING:
            return

        self.get_logger().info(
            "HOTPLACE_DISPATCH 3초 대기 완료. 미션을 종료하고 순찰로 복귀합니다.")
        self.standby_active = False
        self.mode = RobotMode.IDLE
        self._finish_mission(True, "IDLE")
        self.schedule_patrol_resume(0.5)

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
        # 실제 goal result 콜백에서 active_goal_handle을 비운 뒤 pending goal을 전송한다.

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
        if self.active_goal_handle is not None:
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
            "emergency_travel_beep_timer",
            "emergency_rotate_timer",
            "clearing_timer",
            "rgbd_wait_timer",
            "hotplace_wait_timer",
            "patrol_retry_timer",
            "patrol_start_timer",
        ):
            self._cancel_timer(name)

    def publish_zero_cmd(self):
        self.cmd_vel_pub.publish(Twist())

    def estop_zero_timer(self):
        # ESTOP 또는 standby 상태에서는 10 Hz로 속도 0을 계속 발행한다.
        # Nav2 goal 취소가 처리되는 짧은 시간 동안에도 로봇이 계속 가지 않게 한다.
        if self.mode == RobotMode.ESTOP or self.standby_active:
            self.publish_zero_cmd()

    def publish_status(self):
        msg = RobotStatus()
        msg.robot_id = self.robot_id
        msg.mode = MODE_TO_ENUM.get(self.mode, RobotStatus.MODE_ERROR)
        msg.battery_pct = self.battery_pct
        msg.pose = PoseStamped()
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


