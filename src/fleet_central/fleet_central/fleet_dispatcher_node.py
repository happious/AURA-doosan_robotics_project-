#!/usr/bin/env python3
"""
fleet_dispatcher_node
======================
확정된 시나리오 반영:
- 로봇 이름: robot1, robot3 (동적으로 ROBOT_IDS 리스트로 관리, 하드코딩 지양)
- GUIDE / LUGGAGE_ASSIST 세부 로직은 로봇 내부에 이미 내장되어 있음.
  중앙 노드는 "이 미션을 수행하라"는 트리거(Action Goal)만 보낸다.
- 응급(EMERGENCY_DISPATCH): /fall_detection(Bool) + /fall_detection_point(Point)
  감지 시, RobotStatus.aed_loaded == True 인 로봇을 실시간으로 찾아서 그 로봇에게만 보낸다.
  (역할 고정 아님 - AED가 어느 로봇에 있든 동적으로 찾음)
- 강제 선점(기존 임무 취소 여부 판단)은 로봇 쪽 goal_cb/execute_cb 책임.
  Dispatcher는 "누구에게 보낼지"만 결정하고 그 이상 관여하지 않는다.
- RobotStatus를 통해 위치/배터리/AED 탑재 여부를 실시간으로 파악한다.

[신규] AURA 승객 모바일 UI 연동
--------------------------------
aura_mobile_ui_final.py(Flask)가 발행하는 아래 3개 topic을 구독해서
CLI 대신 실제 승객 UI 입력으로 GUIDE/LUGGAGE_ASSIST/복귀를 트리거한다.

  UI -> Dispatcher
  - /aura/robot_select    (std_msgs/String, JSON) : 승객이 선택한 AMR 정보 (참고/로그용)
  - /aura/service_request (std_msgs/String, JSON) : GUIDE 또는 LUGGAGE_ASSIST 요청
  - /aura/service_end     (std_msgs/String, JSON) : 서비스 종료 / 대기 위치 복귀 요청

  Dispatcher -> UI
  - /aura/arrival_status  (std_msgs/String, JSON) : 미션 종료(도착) 결과 통보

[설계 확정 사항] (사용자 확인 완료)
- AMR1 -> robot1, AMR3 -> robot3 고정 매핑. AMR2는 미지원(에러 로그만 남기고 무시).
- UI에서 승객이 이미 로봇을 선택했으므로, GUIDE/LUGGAGE_ASSIST 모두
  MultiAMRDispatcher의 거리/가용성 기반 재선정을 거치지 않고
  UI가 지정한 robot_id로 직행한다.
- goal_id(goal_1/goal_2/goal_3) -> (x, y) 좌표 매핑은 로봇별 GOAL_TABLE로 관리한다.
  현재 robot1의 goal_1, goal_2 좌표만 확정되어 있고 나머지는 TODO로 비워둠.
  (실제 좌표 확정되는 대로 GOAL_TABLE 채워 넣을 것)
"""

import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import PointStamped, PoseArray

from fleet_interfaces.msg import RobotStatus
from fleet_interfaces.action import ExecuteMission
from fleet_interfaces.srv import EmergencyStop


ROBOT_IDS = ['robot1', 'robot3']

# ===================== AURA UI 연동 상수 =====================
ROBOT_SELECT_TOPIC = '/aura/robot_select'
SERVICE_REQUEST_TOPIC = '/aura/service_request'
SERVICE_END_TOPIC = '/aura/service_end'
ARRIVAL_STATUS_TOPIC = '/aura/arrival_status'

# robot3 전용: GUIDE 요청을 받아도 바로 보내지 않고, 이 토픽으로 회전 완료 신호가
# 와야 (저장해둔) 좌표/모드를 그제서야 전송한다. robot1은 이 로직을 타지 않고 기존대로
# service_request를 받는 즉시 전송한다.
# 주의: robot3 네임스페이스가 붙은 '/robot3/turn_complete'가 아니라
# 네임스페이스 없는 전역 토픽 '/turn_complete'로 발행됨.
ROBOT3_TURN_COMPLETE_TOPIC = '/turn_complete'

# UI의 AMR1/AMR2/AMR3 선택을 실제 로봇 ID로 고정 매핑.
# AMR2는 아직 실제 로봇이 없으므로 매핑에서 제외(요청 시 에러 로그 후 무시).
AMR_TO_ROBOT = {
    'AMR1': 'robot1',
    'AMR3': 'robot3',
}
ROBOT_TO_AMR = {v: k for k, v in AMR_TO_ROBOT.items()}

# goal_id -> (x, y) 매핑. 로봇별로 관리한다.
# TODO: robot1 goal_3, robot3 전체 좌표는 아직 확정되지 않음. 확정되는 대로 채울 것.
GOAL_TABLE = {
    'robot1': {
        'goal1_1': (-2.92409, 2.94599),
        'goal1_2': (-3.97043, 3.462406),
        'goal_2': (-4.248990535736084, -1.706390142440796),
    },
    'robot3': {
        'goal1_1': (-2.92409, 2.94599),
        'goal1_2': (-3.97043, 3.462406),
        'goal_2': (-4.248990535736084, -1.706390142440796),
    },
}

GUIDE_PRIORITY = 5
LUGGAGE_PRIORITY = 5
HOTPLACE_PRIORITY = 3  # GUIDE(5)/EMERGENCY(10)보다 낮음 - 유휴 로봇만 파견하는 시나리오

# 혼잡구역(zone_a1 / zone_a3) 경계선. RViz "Publish Point"로 지도 위 대각선의
# 양 끝점을 찍어서 얻은 기준점이며, 실제 로봇 목적지가 아니라 판정 전용 기준점이다.
ZONE_LINE = {
    'point1': (-2.1238932609558105, 1.1382713317871094),   # a3 방향
    'point2': (-4.864102840423584, -2.2349956035614014),   # a1 방향
}


def get_zone_by_line(x: float, y: float) -> str:
    """경계선(ZONE_LINE) 기준 외적(cross product) 부호로 좌표가 어느 구역에 속하는지 판별한다."""
    x1, y1 = ZONE_LINE['point1']
    x2, y2 = ZONE_LINE['point2']
    cross = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
    return 'zone_a3' if cross > 0 else 'zone_a1'


# =========================================================
# AMR State Manager
# =========================================================
class AMRStateManager:
    """robot1/robot3의 최신 RobotStatus를 보관. 위치/배터리/AED 탑재 여부를 실시간 조회."""

    def __init__(self, logger):
        self.logger = logger
        self.robot_table: dict[str, RobotStatus] = {}

    def update(self, msg: RobotStatus):
        self.robot_table[msg.robot_id] = msg
        self.logger.debug(
            f'[AMRStateManager] {msg.robot_id} mode={msg.mode} '
            f'battery={msg.battery_pct:.1f} aed_loaded={msg.aed_loaded} '
            f'available={msg.available}'
        )

    def get(self, robot_id: str):
        return self.robot_table.get(robot_id)

    def find_robot_with_aed(self) -> str | None:
        """AED를 탑재하고 있는 로봇을 실시간으로 찾는다. 역할 고정이 아니라 상태 기반 탐색."""
        for robot_id, status in self.robot_table.items():
            if status.aed_loaded:
                return robot_id
        return None

    def available_for_guide(self) -> list[str]:
        """GUIDE 후보: available=True 이고 EMERGENCY 중이 아닌 로봇들."""
        result = []
        for robot_id, status in self.robot_table.items():
            if status.available and status.mode != RobotStatus.MODE_EMERGENCY_DISPATCH:
                result.append(robot_id)
        return result

    def has_valid_pose(self, robot_id: str) -> bool:
        """(0.0, 0.0) 기본값(아직 pose를 한 번도 못 받은 상태)을 걸러낸다.
        이 값을 그대로 접근지점 계산에 쓰면 원점 방향으로 위험하게 이동시킬 수 있다."""
        status = self.get(robot_id)
        if status is None:
            return False
        px = status.pose.pose.position.x
        py = status.pose.pose.position.y
        return not (px == 0.0 and py == 0.0)

    def available_for_hotplace(self) -> list[str]:
        """hot_place 파견 후보: MODE_IDLE인 로봇을 사용한다.
        robot1_fffinal.py 확인 결과 PATROLLING도 status.mode로는 MODE_IDLE로 보고되므로
        (MODE_TO_ENUM에서 PATROLLING -> MODE_IDLE 매핑), 이 조건 하나로 순찰 중인 로봇도 포함된다.
        RobotStatus.msg에 MODE_PATROL은 정의되어 있지 않다."""
        result = []
        for robot_id, status in self.robot_table.items():
            if status.mode == RobotStatus.MODE_IDLE and self.has_valid_pose(robot_id):
                result.append(robot_id)
        return result

    def distance_to(self, robot_id: str, x: float, y: float) -> float:
        status = self.get(robot_id)
        if status is None:
            return float('inf')
        px = status.pose.pose.position.x
        py = status.pose.pose.position.y
        return math.hypot(px - x, py - y)


# =========================================================
# Emergency Event Manager
# =========================================================
class EmergencyEventManager:
    """/fall_detection(_point)을 받아서 응급 이벤트를 생성.
    Point와 Bool은 서로 다른 토픽이라 도착 순서가 보장되지 않으므로,
    두 값을 각각 캐싱해두고 "둘 다 준비된 시점"에 트리거하는 방식으로 순서 무관하게 처리한다."""

    EMERGENCY_PRIORITY = 10

    def __init__(self, logger):
        self.logger = logger
        self._pending_point = None
        self._fail_detected = False
        self._dispatched = False  # 같은 응급 상황 중 중복 전송 방지

    def on_fall_detection_point(self, msg: PointStamped):
        self._pending_point = (msg.point.x, msg.point.y)

    def on_fall_detection_flag(self, detected: bool):
        self._fail_detected = detected
        if not detected:
            self._pending_point = None
            self._dispatched = False  # 상황 종료 -> 다음 응급 발생 시 다시 전송 가능

    def try_build_emergency_event(self, state_manager: AMRStateManager):
        """flag와 point가 어떤 순서로 도착했든, 둘 다 준비되면 이벤트를 생성한다.
        같은 응급 상황(연속 True) 동안은 한 번만 전송한다."""
        if not self._fail_detected or self._pending_point is None or self._dispatched:
            return None

        aed_robot = state_manager.find_robot_with_aed()
        if aed_robot is None:
            self.logger.error(
                '[EmergencyEventManager] AED 탑재 로봇을 찾을 수 없음! '
                '응급 대응 불가 - 관리자 확인 필요'
            )
            return None

        x, y = self._pending_point
        self.logger.warn(
            f'[EmergencyEventManager] 응급 이벤트: ({x:.2f}, {y:.2f}) -> '
            f'AED 탑재 로봇 "{aed_robot}"에게 전송'
        )
        self._dispatched = True
        return {
            'robot_id': aed_robot,
            'mode': 'EMERGENCY_DISPATCH',
            'dest_x': x,
            'dest_y': y,
            'priority': self.EMERGENCY_PRIORITY,
        }


# =========================================================
# Multi-AMR Dispatcher (GUIDE / LUGGAGE_ASSIST 트리거 판단)
# =========================================================
class MultiAMRDispatcher:
    """GUIDE/LUGGAGE_ASSIST는 "이 미션을 수행하라"는 트리거만 보낸다.
    세부 수행 로직은 로봇 내부에 이미 내장되어 있으므로 Dispatcher는 관여하지 않는다.

    [주의] 이 클래스는 CLI/자동 배정 시나리오를 위해 남겨둔 레거시 경로다.
    AURA UI 경로(on_aura_service_request)는 승객이 이미 로봇을 선택했으므로
    이 클래스의 거리/가용성 기반 재선정을 거치지 않고 바로 트리거를 만든다.
    """

    GUIDE_PRIORITY = GUIDE_PRIORITY
    LUGGAGE_PRIORITY = LUGGAGE_PRIORITY

    def __init__(self, state_manager: AMRStateManager, logger):
        self.state_manager = state_manager
        self.logger = logger

    def decide_for_guide(self, dest_x: float, dest_y: float):
        candidates = self.state_manager.available_for_guide()
        if not candidates:
            self.logger.warn('[MultiAMRDispatcher] GUIDE 가능한 로봇 없음 (대기열 정책 미확정)')
            return None
        target = min(candidates, key=lambda rid: self.state_manager.distance_to(rid, dest_x, dest_y))
        self.logger.info(f'[MultiAMRDispatcher] GUIDE -> {target} 트리거 전송 (candidates={candidates})')
        return {
            'robot_id': target,
            'mode': 'GUIDE',
            'dest_x': dest_x,
            'dest_y': dest_y,
            'priority': self.GUIDE_PRIORITY,
        }


# =========================================================
# Hot Place Dispatcher (혼잡구역 유휴/순찰 로봇 파견)
# =========================================================
class HotPlaceDispatcher:
    """/hot_place 좌표(zone_a1/zone_a3 판별 포함)를 받아 가장 가까운 로봇을 파견한다.

    - 목적지는 hot_place 원본 좌표가 아니라, 로봇 현재 위치 -> hot_place 방향선 상에서
      keepout 반경 + 여유거리만큼 못 미친 "접근 지점(approach point)"으로 계산해서 보낸다.
    - HOT_PLACE_KEEPOUT_RADIUS는 로봇팀이 실제 설정한 keepout 반경 값으로
      반드시 교체해야 한다 (현재 TODO placeholder).
    - /hot_place는 고빈도로 계속 들어올 수 있으므로, 같은 구역에 이미 로봇이 파견되어
      임무 수행 중이면 재파견하지 않고, 이미 다른 구역에 파견된 로봇도 후보에서 제외한다.
    - MODE_IDLE인 로봇을 후보로 사용한다 (available_for_hotplace 사용).
      robot1_fffinal.py 확인 결과 PATROLLING도 status.mode로는 MODE_IDLE로 보고되므로
      (MODE_TO_ENUM에서 PATROLLING -> MODE_IDLE 매핑), 순찰 중인 로봇도 이 조건으로 자연히 포함된다.
    - standby/resume 토픽 제어는 dispatcher가 하지 않는다. robot1_fffinal.py의
      preempt_patrol_to_mission()/hotplace_wait_done()이 순찰 goal 취소와 재개를
      FSM 내부에서 전부 자체 처리하기 때문이다 (dispatcher는 execute_mission goal만 보내면 됨).
    - mode 문자열은 'HOTPLACE_DISPATCH'를 그대로 사용한다 ('GUIDE'로 위장하지 않는다).
      robot1/robot3 FSM 양쪽 모두 HOTPLACE_DISPATCH 전용 모드를 지원해야 한다.
    """

    HOTPLACE_PRIORITY = HOTPLACE_PRIORITY

    # TODO(팀원 확인 필요): robot1/robot3 실제 hot_place keepout 반경(m)으로 교체.
    HOT_PLACE_KEEPOUT_RADIUS = 1.0
    HOT_PLACE_APPROACH_MARGIN = 0.3  # keepout 경계에서 추가로 띄우는 여유거리(m)

    def __init__(self, state_manager: AMRStateManager, logger):
        self.state_manager = state_manager
        self.logger = logger
        # zone_id -> robot_id : 현재 그 구역에 파견되어 임무 수행 중인 로봇
        self._busy_zones: dict[str, str] = {}

    def _busy_robot_ids(self) -> set[str]:
        return set(self._busy_zones.values())

    def _compute_approach_point(self, robot_x: float, robot_y: float, hot_x: float, hot_y: float):
        """로봇 현재위치 -> hot_place 방향선 상에서 keepout 바깥의 접근 지점 계산."""
        dx = hot_x - robot_x
        dy = hot_y - robot_y
        dist = math.hypot(dx, dy)
        stop_dist = self.HOT_PLACE_KEEPOUT_RADIUS + self.HOT_PLACE_APPROACH_MARGIN

        if dist <= stop_dist:
            # 이미 keepout 경계 안쪽(또는 매우 가까움) - 그 자리에서 정지 유지.
            return robot_x, robot_y

        ratio = (dist - stop_dist) / dist
        approach_x = robot_x + dx * ratio
        approach_y = robot_y + dy * ratio
        return approach_x, approach_y

    def decide_for_hotplace(self, zone_id: str, x: float, y: float):
        if zone_id in self._busy_zones:
            self.logger.debug(
                f'[HotPlaceDispatcher] {zone_id} 이미 {self._busy_zones[zone_id]} 파견 중 - 재파견 생략'
            )
            return None

        busy_robots = self._busy_robot_ids()
        candidates = [
            rid for rid in self.state_manager.available_for_hotplace()
            if rid not in busy_robots
        ]
        if not candidates:
            self.logger.warn(f'[HotPlaceDispatcher] {zone_id} 파견 가능한 유휴/순찰 로봇 없음')
            return None

        target = min(candidates, key=lambda rid: self.state_manager.distance_to(rid, x, y))
        target_status = self.state_manager.get(target)
        robot_x = target_status.pose.pose.position.x
        robot_y = target_status.pose.pose.position.y
        approach_x, approach_y = self._compute_approach_point(robot_x, robot_y, x, y)

        self._busy_zones[zone_id] = target

        self.logger.info(
            f'[HotPlaceDispatcher] {zone_id} hot_place ({x:.2f}, {y:.2f}) -> {target} 파견, '
            f'접근지점=({approach_x:.2f}, {approach_y:.2f}) '
            f'(keepout 반경 {self.HOT_PLACE_KEEPOUT_RADIUS}m 회피, candidates={candidates})'
        )
        return {
            'robot_id': target,
            'mode': 'HOTPLACE_DISPATCH',
            'dest_x': approach_x,
            'dest_y': approach_y,
            'priority': self.HOTPLACE_PRIORITY,
        }

    def on_mission_done(self, robot_id: str, zone_id: str):
        """해당 구역 파견 임무가 끝나면 재파견 가능 상태로 되돌린다.
        순찰 재개는 robot1_fffinal.py의 hotplace_wait_done()이 자체적으로 처리하므로
        여기서는 별도 토픽을 발행하지 않는다."""
        if self._busy_zones.get(zone_id) == robot_id:
            del self._busy_zones[zone_id]
            self.logger.info(f'[HotPlaceDispatcher] {zone_id} 파견 완료 -> 재파견 가능 상태로 전환')


# =========================================================
# Mission Manager
# =========================================================
class MissionManager:
    def __init__(self, logger):
        self.logger = logger

    def build_goal(self, decision: dict) -> ExecuteMission.Goal:
        goal = ExecuteMission.Goal()
        goal.mode = decision['mode']
        goal.dest_x = decision['dest_x']
        goal.dest_y = decision['dest_y']
        goal.priority = decision['priority']
        return goal

    def on_mission_result(self, robot_id: str, success: bool, final_state: str):
        self.logger.info(
            f'[MissionManager] {robot_id} 임무 종료: success={success}, final_state={final_state}'
        )


# =========================================================
# AURA UI Adapter
# =========================================================
class AuraUIAdapter:
    """AURA 모바일 UI(std_msgs/String, JSON payload)와 Dispatcher 내부 로직을 연결하는 어댑터.

    책임:
    1. AMR1/AMR2/AMR3 <-> robot1/robot3 ID 변환
    2. goal_id -> (x, y) 좌표 변환 (GUIDE)
    3. UI가 보낸 SERVICE_REQUEST JSON을 내부 decision dict(mission goal 입력)로 변환
       (승객이 이미 로봇을 선택했으므로 거리 기반 재선정 없이 그대로 사용)
    4. 미션 결과를 UI가 이해하는 arrival_status JSON으로 변환
    """

    def __init__(self, logger):
        self.logger = logger

    def amr_to_robot(self, amr_id: str) -> str | None:
        robot_id = AMR_TO_ROBOT.get(amr_id)
        if robot_id is None:
            self.logger.error(
                f'[AuraUIAdapter] 지원하지 않는 AMR id "{amr_id}" (매핑: {list(AMR_TO_ROBOT.keys())}) - 무시함'
            )
        return robot_id

    def robot_to_amr(self, robot_id: str) -> str:
        return ROBOT_TO_AMR.get(robot_id, robot_id)

    def resolve_goal_xy(self, robot_id: str, goal_id: str):
        table = GOAL_TABLE.get(robot_id, {})
        xy = table.get(goal_id)
        if xy is None:
            self.logger.error(
                f'[AuraUIAdapter] {robot_id}의 goal_id "{goal_id}" 좌표가 GOAL_TABLE에 없음 '
                '(TODO 좌표 채워넣기 필요) - 요청 무시'
            )
        return xy

    def build_decision_from_service_request(self, payload: dict):
        """UI의 SERVICE_REQUEST payload -> mission 트리거 decision dict.

        기대 payload 예시(GUIDE):
        {
          "event_type": "SERVICE_REQUEST",
          "robot_id": "AMR1",
          "service_type": "GUIDE",
          "mode": "NAVIGATION",
          "goal_id": "goal_1",
          "destination_label": "화장품 가게",
          ...
        }

        기대 payload 예시(LUGGAGE_ASSIST):
        {
          "robot_id": "AMR1",
          "service_type": "LUGGAGE_ASSIST",
          "mode": "FOLLOWING",
          "goal_id": null,
          ...
        }
        """
        amr_id = payload.get('robot_id')
        service_type = payload.get('service_type')

        robot_id = self.amr_to_robot(amr_id)
        if robot_id is None:
            return None

        if service_type == 'GUIDE':
            goal_id = payload.get('goal_id')
            if not goal_id:
                self.logger.error('[AuraUIAdapter] GUIDE 요청에 goal_id가 없음 - 무시')
                return None
            xy = self.resolve_goal_xy(robot_id, goal_id)
            if xy is None:
                return None
            x, y = xy
            self.logger.info(
                f'[AuraUIAdapter] GUIDE -> {robot_id} ({amr_id}) goal_id={goal_id} '
                f'dest=({x:.3f},{y:.3f})'
            )
            return {
                'robot_id': robot_id,
                'mode': 'GUIDE',
                'dest_x': x,
                'dest_y': y,
                'priority': GUIDE_PRIORITY,
            }

        if service_type == 'LUGGAGE_ASSIST':
            # LUGGAGE_ASSIST(동행)는 목적지 좌표 없이 FOLLOWING 트리거만 보낸다.
            # ExecuteMission.Goal이 dest_x/dest_y를 필수로 요구하므로 0.0으로 채우되,
            # 로봇 내부 FOLLOWING 로직은 이 좌표를 사용하지 않는다는 전제(기존 설계)를 따른다.
            self.logger.info(f'[AuraUIAdapter] LUGGAGE_ASSIST -> {robot_id} ({amr_id}) 트리거 전송')
            return {
                'robot_id': robot_id,
                'mode': 'LUGGAGE_ASSIST',
                'dest_x': 0.0,
                'dest_y': 0.0,
                'priority': LUGGAGE_PRIORITY,
            }

        self.logger.error(f'[AuraUIAdapter] 알 수 없는 service_type "{service_type}" - 무시')
        return None

    def build_arrival_payload(self, robot_id: str, success: bool, final_state: str,
                               service_type: str | None = None, goal_id: str | None = None,
                               destination_label: str | None = None) -> str:
        """미션 결과를 UI(/aura/arrival_status)가 기대하는 JSON 문자열로 변환한다.

        UI(_on_arrival_status)는 event_type 또는 arrival_status가
        ARRIVED/REACHED/SUCCEEDED/SUCCESS 계열일 때만 도착으로 인정한다.
        """
        payload = {
            'event_type': 'ARRIVED' if success else 'MISSION_FAILED',
            'arrival_status': 'REACHED' if success else 'FAILED',
            'robot_id': self.robot_to_amr(robot_id),
            'final_state': final_state,
        }
        if service_type:
            payload['service_type'] = service_type
        if goal_id:
            payload['goal_id'] = goal_id
        if destination_label:
            payload['destination_label'] = destination_label
        return json.dumps(payload, ensure_ascii=False)


# =========================================================
# Robot Command Publisher (Action/Service Client 실행부)
# =========================================================
class RobotCommandPublisher:
    def __init__(self, node: Node, mission_manager: MissionManager, cb_group, on_result=None):
        """
        on_result: 선택적 콜백. (robot_id, success, final_state, request_meta) 형태로 호출된다.
                   AURA UI로 도착 결과를 되돌려 보내는 데 사용한다.
        """
        self.node = node
        self.mission_manager = mission_manager
        self.logger = node.get_logger()
        self.on_result = on_result

        self.mission_clients = {
            rid: ActionClient(node, ExecuteMission, f'/{rid}/execute_mission', callback_group=cb_group)
            for rid in ROBOT_IDS
        }
        self.estop_clients = {
            rid: node.create_client(EmergencyStop, f'/{rid}/emergency_stop', callback_group=cb_group)
            for rid in ROBOT_IDS
        }

    def send_mission(self, robot_id: str, goal: ExecuteMission.Goal, request_meta: dict | None = None):
        """request_meta: service_type/goal_id/destination_label 등 UI 응답 시 되돌려줄 부가 정보."""
        client = self.mission_clients.get(robot_id)
        if client is None:
            self.logger.error(f'[RobotCommandPublisher] 알 수 없는 robot_id: {robot_id}')
            return
        if not client.wait_for_server(timeout_sec=1.0):
            self.logger.error(f'[RobotCommandPublisher] {robot_id} execute_mission 서버 응답 없음')
            return

        future = client.send_goal_async(goal, feedback_callback=self._make_feedback_cb(robot_id))
        future.add_done_callback(self._make_goal_response_cb(robot_id, request_meta))

    def _make_feedback_cb(self, robot_id):
        def cb(feedback_msg):
            fb = feedback_msg.feedback
            self.logger.info(f'[{robot_id} feedback] mode={fb.current_mode} progress={fb.progress_pct:.0f}%')
        return cb

    def _make_goal_response_cb(self, robot_id, request_meta):
        def cb(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.logger.warn(f'[RobotCommandPublisher] {robot_id} goal 거부됨 (재시도 정책 미확정)')
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._make_result_cb(robot_id, request_meta))
        return cb

    def _make_result_cb(self, robot_id, request_meta):
        def cb(future):
            result = future.result().result
            self.mission_manager.on_mission_result(robot_id, result.success, result.final_state)
            if self.on_result is not None:
                self.on_result(robot_id, result.success, result.final_state, request_meta or {})
        return cb

    def call_emergency_stop(self, robot_id: str, reason: str):
        client = self.estop_clients.get(robot_id)
        if client is None or not client.wait_for_service(timeout_sec=1.0):
            self.logger.error(f'[RobotCommandPublisher] {robot_id} emergency_stop 서비스 없음')
            return
        req = EmergencyStop.Request()
        req.reason = reason
        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self.logger.warn(f'[emergency_stop] {robot_id} 결과: {f.result().success}')
        )


# =========================================================
# ROS2 Topic Bridge + 메인 노드
# =========================================================
class FleetDispatcherNode(Node):
    def __init__(self):
        super().__init__('fleet_dispatcher_node')
        cb_group = ReentrantCallbackGroup()

        self.state_manager = AMRStateManager(self.get_logger())
        self.emergency_manager = EmergencyEventManager(self.get_logger())
        self.dispatcher = MultiAMRDispatcher(self.state_manager, self.get_logger())
        self.hotplace_dispatcher = HotPlaceDispatcher(self.state_manager, self.get_logger())
        self.mission_manager = MissionManager(self.get_logger())
        self.aura_adapter = AuraUIAdapter(self.get_logger())
        self.command_publisher = RobotCommandPublisher(
            self, self.mission_manager, cb_group, on_result=self._on_mission_result_for_aura)

        # robot3 GUIDE 전용 상태
        # - _last_robot3_guide: 가장 최근에 받은 GUIDE (decision, request_meta). 계속 유지/덮어쓰기.
        # - _robot3_turn_complete_latched: turn_complete=True가 "아직 소비되지 않고" 와있는 상태.
        #   실제 현장에서는 turn_complete가 GUIDE보다 먼저 오는 경우가 기본이라,
        #   GUIDE가 도착했을 때 이미 래치가 서 있으면 그 자리에서 즉시 전송해야 한다.
        #   (turn_complete 쪽에서도 last_guide가 있으면 그때그때 재전송 - 둘 다 지원)
        self._pending_lock = threading.Lock()
        self._last_robot3_guide = None
        self._robot3_turn_complete_latched = False

        # ---- 로봇 상태 구독 (robot1, robot3) ----
        for rid in ROBOT_IDS:
            self.create_subscription(
                RobotStatus, f'/{rid}/status', self.state_manager.update, 10,
                callback_group=cb_group)

        # ---- 응급 감지 구독 ----
        self.create_subscription(
            Bool, '/fall_detection', self.on_fall_detection, 10, callback_group=cb_group)
        self.create_subscription(
            PointStamped, '/fall_detection_point', self.on_fall_detection_point, 10,
            callback_group=cb_group)

        # ---- 혼잡구역(population) 중계 ----
        self.create_subscription(
            Int32, '/population', self.on_population_relay, 10, callback_group=cb_group)
        self.population_relay_pub = self.create_publisher(Int32, '/robot1/population', 10)

        # ---- 혼잡구역(hot_place) 수신 -> 유휴/순찰 로봇 파견 ----
        self.create_subscription(
            PoseArray, '/hot_place', self.on_hot_place_relay, 10, callback_group=cb_group)

        # ---- AURA 승객 모바일 UI 연동 ----
        self.create_subscription(
            String, ROBOT_SELECT_TOPIC, self.on_aura_robot_select, 10, callback_group=cb_group)
        self.create_subscription(
            String, SERVICE_REQUEST_TOPIC, self.on_aura_service_request, 10, callback_group=cb_group)
        self.create_subscription(
            String, SERVICE_END_TOPIC, self.on_aura_service_end, 10, callback_group=cb_group)
        self.arrival_status_pub = self.create_publisher(String, ARRIVAL_STATUS_TOPIC, 10)

        # ---- robot3 전용: 회전 완료 신호 (GUIDE 대기 중이던 좌표/모드를 이 시점에 전송) ----
        self.create_subscription(
            Bool, ROBOT3_TURN_COMPLETE_TOPIC, self.on_robot3_turn_complete, 10, callback_group=cb_group)

        # LUGGAGE_ASSIST 취소·복귀 요청을 로봇 저수준 제어 토픽으로 직접 발행.
        # execute_mission 액션(진행상태/완료 추적용)은 그대로 병행해서 보낸다 -
        # 이 토픽들은 실제 복귀 동작을 트리거하는 역할만 담당한다.
        # 2_back_move.py/4_front_move.py가 절대 경로("/...")로 구독하므로 동일하게 맞춘다.
        self.service_cancel_pub = self.create_publisher(Bool, '/service_cancel', 10)

        self.get_logger().info(
            f'[fleet_dispatcher_node] 초기화 완료. 관리 로봇: {ROBOT_IDS}'
        )

    # ---- 응급 콜백 ----
    def on_fall_detection(self, msg: Bool):
        self.emergency_manager.on_fall_detection_flag(msg.data)
        self._try_dispatch_emergency()

    def on_fall_detection_point(self, msg: PointStamped):
        self.emergency_manager.on_fall_detection_point(msg)
        self._try_dispatch_emergency()

    def _try_dispatch_emergency(self):
        event = self.emergency_manager.try_build_emergency_event(self.state_manager)
        if event is None:
            return
        goal = self.mission_manager.build_goal(event)
        self.command_publisher.send_mission(event['robot_id'], goal)

    # ---- 혼잡구역 중계 콜백 ----
    def on_population_relay(self, msg: Int32):
        self.population_relay_pub.publish(msg)
        self.get_logger().debug(f'[CrowdRelay] population 중계: {msg.data} -> /robot1/population')

    def on_hot_place_relay(self, msg: PoseArray):
        if not msg.poses:
            self.get_logger().debug('[CrowdRelay] hot_place PoseArray 비어있음 (좌표 없음)')
            return

        for i, pose in enumerate(msg.poses):
            x, y = pose.position.x, pose.position.y
            zone_id = get_zone_by_line(x, y)
            self.get_logger().info(
                f'[CrowdRelay] hot_place[{i}] 수신: ({x:.2f}, {y:.2f}) -> {zone_id} '
                f'(총 {len(msg.poses)}건)'
            )

            decision = self.hotplace_dispatcher.decide_for_hotplace(zone_id, x, y)
            if decision is None:
                continue
            goal = self.mission_manager.build_goal(decision)
            request_meta = {'hotplace_zone': zone_id}
            self.command_publisher.send_mission(
                decision['robot_id'], goal, request_meta=request_meta)

    # ---- AURA UI 콜백 ----
    def _parse_aura_json(self, msg: String, topic_name: str) -> dict | None:
        try:
            return json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'[AURA] {topic_name} JSON 파싱 실패: {e} raw="{msg.data}"')
            return None

    def on_aura_robot_select(self, msg: String):
        """참고/로그 용도. 실제 로봇 배정은 SERVICE_REQUEST에서 이루어진다."""
        payload = self._parse_aura_json(msg, ROBOT_SELECT_TOPIC)
        if payload is None:
            return
        amr_id = payload.get('robot_id')
        robot_id = self.aura_adapter.amr_to_robot(amr_id)
        self.get_logger().info(f'[AURA] robot_select: {amr_id} -> {robot_id}')

    def on_aura_service_request(self, msg: String):
        payload = self._parse_aura_json(msg, SERVICE_REQUEST_TOPIC)
        if payload is None:
            return

        decision = self.aura_adapter.build_decision_from_service_request(payload)
        if decision is None:
            return

        request_meta = {
            'service_type': payload.get('service_type'),
            'goal_id': payload.get('goal_id'),
            'destination_label': payload.get('destination_label'),
        }

        # robot3 GUIDE 전용 처리.
        # robot1(및 robot3의 다른 mode)은 기존 그대로 즉시 전송.
        if decision['robot_id'] == 'robot3' and decision['mode'] == 'GUIDE':
            with self._pending_lock:
                self._last_robot3_guide = {'decision': decision, 'request_meta': request_meta}
                # turn_complete가 GUIDE보다 먼저 와서 이미 래치돼 있는 경우 -> 즉시 소비해서 전송.
                send_now = self._robot3_turn_complete_latched
                if send_now:
                    self._robot3_turn_complete_latched = False

            if send_now:
                self.get_logger().info(
                    f'[AURA] robot3 GUIDE 요청 - 이미 도착해있던 turn_complete 래치 소비 -> 즉시 전송 '
                    f'goal_id={request_meta.get("goal_id")} dest=({decision["dest_x"]:.3f},{decision["dest_y"]:.3f})'
                )
                goal = self.mission_manager.build_goal(decision)
                self.command_publisher.send_mission(decision['robot_id'], goal, request_meta=request_meta)
                return

            self.get_logger().info(
                f'[AURA] robot3 GUIDE 좌표 갱신 - turn_complete 대기 중 '
                f'goal_id={request_meta.get("goal_id")} dest=({decision["dest_x"]:.3f},{decision["dest_y"]:.3f})'
            )
            return

        goal = self.mission_manager.build_goal(decision)
        self.command_publisher.send_mission(decision['robot_id'], goal, request_meta=request_meta)

    def on_robot3_turn_complete(self, msg: Bool):
        """robot3가 회전을 마쳤다는 신호.

        실제 현장 순서는 보통 turn_complete가 먼저 오고 GUIDE가 나중에 온다.
        - turn_complete=True가 오면: 래치를 세워둔다(다음 GUIDE 요청이 오면 즉시 전송되도록).
          이때 "최근 GUIDE"가 이미 있다면(재요청 없이 재전송하는 케이스) 그것도 바로 재전송한다.
        - GUIDE 요청 쪽(on_aura_service_request)에서도 래치가 서 있으면 그 자리에서 바로 소비한다.
        """
        with self._pending_lock:
            last = self._last_robot3_guide
            self._robot3_turn_complete_latched = bool(msg.data)

        self.get_logger().info(
            f'[AURA] /turn_complete 수신: data={msg.data}, '
            f'보유 중인 최근 GUIDE={"있음" if last is not None else "없음"}'
        )

        if not msg.data:
            return

        if last is None:
            self.get_logger().info(
                '[AURA] robot3 turn_complete=True - 아직 받은 GUIDE 요청이 없어 래치만 세움 '
                '(다음 GUIDE 요청 시 즉시 전송)'
            )
            return

        decision = last['decision']
        request_meta = last['request_meta']
        self.get_logger().info(
            f'[AURA] robot3 turn_complete=True -> 최근 GUIDE 재전송 '
            f'dest=({decision["dest_x"]:.3f},{decision["dest_y"]:.3f})'
        )
        goal = self.mission_manager.build_goal(decision)
        self.command_publisher.send_mission(decision['robot_id'], goal, request_meta=request_meta)

    def on_aura_service_end(self, msg: String):
        """서비스 종료 / 대기 위치 복귀 요청.

        /service_cancel=True를 직접 발행해 4_front_move.py가 즉시 복귀/도킹을
        시작하도록 한다. (execute_mission 액션 자체는 이 신호로 인해 로봇이
        복귀·도킹을 마치고 /robot3/mission_complete를 발행하면 자연스럽게 종료됨)
        """
        payload = self._parse_aura_json(msg, SERVICE_END_TOPIC)
        if payload is None:
            return
        amr_id = payload.get('robot_id')
        robot_id = self.aura_adapter.amr_to_robot(amr_id)
        if robot_id is None:
            return
        reason = payload.get('end_reason', 'UNKNOWN')

        cancel_msg = Bool()
        cancel_msg.data = True
        self.service_cancel_pub.publish(cancel_msg)
        self.get_logger().info(
            f'[AURA] service_end: {robot_id} ({amr_id}) reason={reason} '
            '-> /service_cancel=True 발행'
        )

    def _on_mission_result_for_aura(self, robot_id: str, success: bool, final_state: str, request_meta: dict):
        """RobotCommandPublisher 미션 결과 -> /aura/arrival_status로 UI에 통보."""
        zone_id = request_meta.get('hotplace_zone')
        if zone_id is not None:
            self.hotplace_dispatcher.on_mission_done(robot_id, zone_id)
            # hot_place 파견은 AURA UI의 service_type이 아니므로 arrival_status 통보는 생략한다.
            return

        data = self.aura_adapter.build_arrival_payload(
            robot_id=robot_id,
            success=success,
            final_state=final_state,
            service_type=request_meta.get('service_type'),
            goal_id=request_meta.get('goal_id'),
            destination_label=request_meta.get('destination_label'),
        )
        out_msg = String()
        out_msg.data = data
        self.arrival_status_pub.publish(out_msg)
        self.get_logger().info(f'[AURA] arrival_status 발행 -> {data}')


def main():
    rclpy.init()
    node = FleetDispatcherNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
