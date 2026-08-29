from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32, Int32


class PersonFollowCmdVelNode(Node):
    """
    TurtleBot4 전방 사람 추종 노드.

    동작 순서
    ---------
    1. 노드 시작 시 LOCKED 상태로 시작하고 /front_start=0을 주기적으로 발행한다.
    2. LOCKED 상태에서는 /robot3/cmd_vel을 발행하지 않아 제어권을 사용하지 않는다.
    3. /carrying=True를 받으면 FOLLOW 상태로 전환하고 /front_start=1을 주기적으로 발행한다.
    4. FOLLOW 상태에서 사람 감지 상태, 중앙점, 거리값이 모두 최신이고 유효할 때만 추종한다.
    5. /service_end=True를 받으면 정지 명령을 한 번 발행하고 LOCKED 상태로 돌아가 /front_start=0을 발행한다.
    6. 초기 상태로 돌아간 뒤 /robot3/mission_complete=True를 한 번 발행한다.
    7. 이후 /start_patrol=True를 한 번 발행한다.
    """

    # -------------------------------------------------------------------------
    # 토픽
    # -------------------------------------------------------------------------
    CMD_VEL_TOPIC = "/robot3/cmd_vel"

    TRACKING_TOPIC = "/tracking_rgbd"
    CENTER_TOPIC = "/tracking_rgbd_center_pixel"
    DEPTH_TOPIC = "/target_depth_rgbd"

    CARRYING_TOPIC = "/carrying"
    SERVICE_END_TOPIC = "/service_end"

    MISSION_COMPLETE_TOPIC = "/robot3/mission_complete"
    START_PATROL_TOPIC = "/start_patrol"
    FRONT_START_TOPIC = "/front_start"

    # -------------------------------------------------------------------------
    # 트래킹 상태
    # -------------------------------------------------------------------------
    TRACKING_NONE = 0
    TRACKING_VISIBLE = 1
    TRACKING_LOST = 2

    # -------------------------------------------------------------------------
    # 사람 추종 제어 파라미터
    # -------------------------------------------------------------------------
    IMAGE_WIDTH = 640.0
    CAMERA_CENTER_X = IMAGE_WIDTH * 0.5

    TARGET_STOP_DISTANCE_M = 0.7
    SLOW_START_DISTANCE_M = 1.0

    MAX_LINEAR_SPEED = 0.40
    MIN_LINEAR_SPEED = 0.12

    MAX_ANGULAR_SPEED = 0.55
    ANGULAR_KP = 0.0025

    CENTER_DEADBAND_PX = 35.0
    ALIGN_ONLY_ERROR_PX = 140.0

    COMMAND_RATE_HZ = 20.0
    FRONT_START_RATE_HZ = 10.0
    INPUT_TIMEOUT_SEC = 0.7

    # -------------------------------------------------------------------------
    # 동작 모드
    # -------------------------------------------------------------------------
    MODE_LOCKED = "LOCKED"
    MODE_FOLLOW = "FOLLOW"
    MODE_STOPPED = "STOPPED"

    def __init__(self) -> None:
        super().__init__("front_move")

        self.state_lock = threading.Lock()

        self.latest_tracking_state = self.TRACKING_NONE
        self.latest_center_pixel = -1.0
        self.latest_target_depth = -1.0

        self.tracking_time = 0.0
        self.center_time = 0.0
        self.depth_time = 0.0

        self.mode = self.MODE_LOCKED

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            Int32,
            self.TRACKING_TOPIC,
            self.tracking_callback,
            qos,
        )
        self.create_subscription(
            Float32,
            self.CENTER_TOPIC,
            self.center_callback,
            qos,
        )
        self.create_subscription(
            Float32,
            self.DEPTH_TOPIC,
            self.depth_callback,
            qos,
        )
        self.create_subscription(
            Bool,
            self.CARRYING_TOPIC,
            self.carrying_callback,
            qos,
        )
        self.create_subscription(
            Bool,
            self.SERVICE_END_TOPIC,
            self.service_end_callback,
            qos,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.CMD_VEL_TOPIC,
            qos,
        )
        self.mission_complete_pub = self.create_publisher(
            Bool,
            self.MISSION_COMPLETE_TOPIC,
            qos,
        )
        self.start_patrol_pub = self.create_publisher(
            Bool,
            self.START_PATROL_TOPIC,
            qos,
        )
        self.front_start_pub = self.create_publisher(
            Int32,
            self.FRONT_START_TOPIC,
            qos,
        )

        timer_period = 1.0 / max(self.COMMAND_RATE_HZ, 1.0)
        self.control_timer = self.create_timer(
            timer_period,
            self.control_loop,
        )

        front_start_period = 1.0 / max(self.FRONT_START_RATE_HZ, 1.0)
        self.front_start_timer = self.create_timer(
            front_start_period,
            self.publish_front_start_state,
        )

        # 초기 LOCKED 상태를 즉시 알린다. 이후에도 주기적으로 0을 발행한다.
        self.publish_front_start(0)

        self.get_logger().info("Front person-follow node started")
        self.get_logger().info(f"initial mode: {self.MODE_LOCKED}")
        self.get_logger().info(
            f"sub: {self.TRACKING_TOPIC}, "
            f"{self.CENTER_TOPIC}, "
            f"{self.DEPTH_TOPIC}"
        )
        self.get_logger().info(
            f"sub: {self.CARRYING_TOPIC}, "
            f"{self.SERVICE_END_TOPIC}"
        )
        self.get_logger().info(f"pub: {self.CMD_VEL_TOPIC}")
        self.get_logger().info(f"pub: {self.MISSION_COMPLETE_TOPIC}")
        self.get_logger().info(f"pub: {self.START_PATROL_TOPIC}")
        self.get_logger().info(
            f"pub: {self.FRONT_START_TOPIC} "
            f"({self.FRONT_START_RATE_HZ:.1f} Hz, 0=locked, 1=follow)"
        )

    # -------------------------------------------------------------------------
    # 입력 콜백
    # -------------------------------------------------------------------------
    def tracking_callback(self, msg: Int32) -> None:
        now = time.monotonic()

        with self.state_lock:
            self.latest_tracking_state = int(msg.data)
            self.tracking_time = now

    def center_callback(self, msg: Float32) -> None:
        now = time.monotonic()

        with self.state_lock:
            self.latest_center_pixel = float(msg.data)
            self.center_time = now

    def depth_callback(self, msg: Float32) -> None:
        now = time.monotonic()

        with self.state_lock:
            self.latest_target_depth = float(msg.data)
            self.depth_time = now

    def carrying_callback(self, msg: Bool) -> None:
        if not bool(msg.data):
            return

        with self.state_lock:
            current_mode = self.mode

            if current_mode == self.MODE_LOCKED:
                self.mode = self.MODE_FOLLOW
                changed = True
            else:
                changed = False

        if changed:
            # FOLLOW 전환을 즉시 알리고, 이후 타이머가 1을 계속 발행한다.
            self.publish_front_start(1)

            self.get_logger().info(
                "carrying=True received: "
                f"{self.MODE_LOCKED} -> {self.MODE_FOLLOW}"
            )
        elif current_mode == self.MODE_FOLLOW:
            self.get_logger().info(
                "carrying=True ignored: already following"
            )
        elif current_mode == self.MODE_STOPPED:
            self.get_logger().info(
                "carrying=True ignored: node is stopped"
            )

    def service_end_callback(self, msg: Bool) -> None:
        """
        /service_end=True가 들어오면 현재 추종을 종료하고 초기 LOCKED 상태로 돌아간다.

        front_move가 마지막으로 발행했던 속도 명령을 제거하기 위해 정지 명령을 한 번만
        발행한다. 이후 LOCKED 상태에서는 cmd_vel을 발행하지 않으므로 순찰 노드가
        제어권을 사용할 수 있다.
        """
        if not bool(msg.data):
            return

        with self.state_lock:
            previous_mode = self.mode
            self.mode = self.MODE_LOCKED

            # 다음 FOLLOW 진입 시 이전 미션의 오래된 트래킹 값을 사용하지 않도록 초기화한다.
            self.latest_tracking_state = self.TRACKING_NONE
            self.latest_center_pixel = -1.0
            self.latest_target_depth = -1.0

            self.tracking_time = 0.0
            self.center_time = 0.0
            self.depth_time = 0.0

        # front_move가 마지막으로 발행했던 이동 명령을 정지시킨다.
        self.publish_stop()

        # LOCKED 전환을 즉시 알리고, 이후 타이머가 0을 계속 발행한다.
        self.publish_front_start(0)

        # 중앙 FSM에 전방 운반 미션 완료를 알린다.
        self.publish_mission_complete(True)

        # 순찰 노드에 제어 시작을 요청한다.
        self.publish_start_patrol(True)

        self.get_logger().info(
            "service_end=True received: "
            f"{previous_mode} -> {self.MODE_LOCKED}, "
            f"{self.MISSION_COMPLETE_TOPIC}=True, "
            f"{self.START_PATROL_TOPIC}=True"
        )

    # -------------------------------------------------------------------------
    # 사람 추종 제어
    # -------------------------------------------------------------------------
    def control_loop(self) -> None:
        state = self.read_latest_state()

        if state["mode"] == self.MODE_LOCKED:
            # LOCKED 상태에서는 다른 제어 노드의 cmd_vel을 덮어쓰지 않는다.
            return

        if state["mode"] != self.MODE_FOLLOW:
            self.publish_stop()
            return

        if not state["tracking_fresh"]:
            self.publish_stop()
            return

        if state["tracking_state"] != self.TRACKING_VISIBLE:
            self.publish_stop()
            return

        if not state["center_fresh"]:
            self.publish_stop()
            return

        if not state["depth_fresh"]:
            self.publish_stop()
            return

        center_pixel = state["center_pixel"]
        target_depth = state["target_depth"]

        if not self.is_valid_center(center_pixel):
            self.publish_stop()
            return

        if not self.is_valid_depth(target_depth):
            self.publish_stop()
            return

        if target_depth <= self.TARGET_STOP_DISTANCE_M:
            self.publish_stop()
            return

        cmd = self.make_follow_cmd(
            center_pixel,
            target_depth,
        )
        self.cmd_vel_pub.publish(cmd)

    def read_latest_state(self) -> dict:
        now = time.monotonic()

        with self.state_lock:
            tracking_state = self.latest_tracking_state
            center_pixel = self.latest_center_pixel
            target_depth = self.latest_target_depth

            tracking_time = self.tracking_time
            center_time = self.center_time
            depth_time = self.depth_time

            mode = self.mode

        tracking_age = (
            math.inf
            if tracking_time <= 0.0
            else now - tracking_time
        )
        center_age = (
            math.inf
            if center_time <= 0.0
            else now - center_time
        )
        depth_age = (
            math.inf
            if depth_time <= 0.0
            else now - depth_time
        )

        return {
            "tracking_state": tracking_state,
            "center_pixel": center_pixel,
            "target_depth": target_depth,
            "tracking_fresh": tracking_age <= self.INPUT_TIMEOUT_SEC,
            "center_fresh": center_age <= self.INPUT_TIMEOUT_SEC,
            "depth_fresh": depth_age <= self.INPUT_TIMEOUT_SEC,
            "mode": mode,
        }

    def make_follow_cmd(
        self,
        center_pixel: float,
        target_depth: float,
    ) -> Twist:
        cmd = Twist()

        center_error = center_pixel - self.CAMERA_CENTER_X

        if abs(center_error) > self.CENTER_DEADBAND_PX:
            cmd.angular.z = self.clamp(
                -self.ANGULAR_KP * center_error,
                -self.MAX_ANGULAR_SPEED,
                self.MAX_ANGULAR_SPEED,
            )
        else:
            cmd.angular.z = 0.0

        if abs(center_error) >= self.ALIGN_ONLY_ERROR_PX:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.compute_linear_speed(target_depth)

        return cmd

    def compute_linear_speed(self, target_depth: float) -> float:
        if target_depth <= self.TARGET_STOP_DISTANCE_M:
            return 0.0

        if target_depth >= self.SLOW_START_DISTANCE_M:
            return self.MAX_LINEAR_SPEED

        ratio = (
            target_depth - self.TARGET_STOP_DISTANCE_M
        ) / max(
            self.SLOW_START_DISTANCE_M
            - self.TARGET_STOP_DISTANCE_M,
            1e-6,
        )

        speed = (
            self.MIN_LINEAR_SPEED
            + ratio
            * (self.MAX_LINEAR_SPEED - self.MIN_LINEAR_SPEED)
        )

        return self.clamp(
            speed,
            self.MIN_LINEAR_SPEED,
            self.MAX_LINEAR_SPEED,
        )

    # -------------------------------------------------------------------------
    # 유틸리티
    # -------------------------------------------------------------------------
    @staticmethod
    def is_valid_center(center_pixel: float) -> bool:
        return (
            math.isfinite(center_pixel)
            and center_pixel >= 0.0
        )

    @staticmethod
    def is_valid_depth(depth: float) -> bool:
        return (
            math.isfinite(depth)
            and depth > 0.0
        )

    @staticmethod
    def clamp(
        value: float,
        low: float,
        high: float,
    ) -> float:
        return max(
            low,
            min(high, value),
        )

    def publish_stop(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def publish_mission_complete(self, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        self.mission_complete_pub.publish(msg)

    def publish_start_patrol(self, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        self.start_patrol_pub.publish(msg)

    def publish_front_start(self, value: int) -> None:
        msg = Int32()
        msg.data = 1 if int(value) == 1 else 0
        self.front_start_pub.publish(msg)

    def publish_front_start_state(self) -> None:
        """현재 모드에 따라 /front_start 상태를 주기적으로 발행한다."""
        with self.state_lock:
            mode = self.mode

        self.publish_front_start(
            1 if mode == self.MODE_FOLLOW else 0
        )

    def shutdown_node(self) -> None:
        with self.state_lock:
            self.mode = self.MODE_STOPPED

        self.publish_stop()
        self.publish_front_start(0)


def main() -> None:
    rclpy.init()

    node = PersonFollowCmdVelNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested by Ctrl+C")

    finally:
        node.shutdown_node()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()