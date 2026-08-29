from __future__ import annotations

import json
import threading
import time
from datetime import datetime
from typing import Any, Callable

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Int32, String

    ROS2_AVAILABLE = True
    ROS2_IMPORT_ERROR = None
except Exception as exc:  # ROS2가 없는 PC에서도 화면 단독 실행 허용
    rclpy = None
    Duration = None
    DurabilityPolicy = HistoryPolicy = QoSProfile = ReliabilityPolicy = None
    Bool = Int32 = String = None
    ROS2_AVAILABLE = False
    ROS2_IMPORT_ERROR = exc


class AuraRosBridge:
    """Flask UI와 ROS2 Topic graph 사이의 통신 어댑터.

    AMR1(CENTRAL_JSON)은 미션/상태 통신은 std_msgs/msg/String(JSON)으로 중앙을 거치지만,
    승객 터치/종료/응급 종료처럼 즉시 제어가 필요한 AMR1 명령은 Bool(True) 직접 명령 + DDS ACK로 보낸다.

    AMR3(DIRECT_TOPIC)는 통신 목적을 분리한다.
    - 로봇3 직접 명령: /rb3_standby, /service_end, /carrying,
      /turn_around에 std_msgs/msg/Bool(data=True)를 1회 발행
    - 중앙 통신: /aura/service_request, /aura/service_end 등의
      std_msgs/msg/String(JSON) 구조 유지
    - 회전 완료 수신: /turn_complete Bool
    - 비전 상태 수신: /tracking_web, /tracking_rgbd Int32
    """

    def __init__(self, *, config, state_store):
        self.config = config
        self.state_store = state_store
        self.enabled = False
        self.node = None
        self.spin_thread = None
        self.stop_event = threading.Event()
        self._publishers: dict[str, Any] = {}

        # Flask는 threaded=True로 여러 요청을 동시에 처리할 수 있다.
        # AMR3 직접 명령은 구독자 확인 → 1회 발행 → DDS ACK 확인 과정을
        # 한 번에 하나씩 수행해서 버튼 중복 요청이 겹치지 않게 한다.
        self._direct_publish_lock = threading.Lock()

        # AMR3/기존 방식 전용 publisher
        self.retarget_web_pub = None
        self.turn_around_pub = None
        self.carrying_pub = None
        self.direct_service_end_pub = None
        self.rb1_standby_pub = None
        self.rb1_service_end_pub = None
        self.emergency_end_pub = None
        self.rb3_standby_pub = None

        self._on_emergency_detected_handler: Callable[[dict[str, Any]], None] | None = None
        self._on_emergency_cleared_handler: Callable[[dict[str, Any]], None] | None = None

        if not ROS2_AVAILABLE:
            print(
                f"[AURA UI] ROS2 import 실패, print-only mode: {ROS2_IMPORT_ERROR}",
                flush=True,
            )
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self.node = rclpy.create_node(config.ROS_NODE_NAME)

            # AMR1·AMR3가 중앙으로 보내는 이벤트는 String(JSON) publisher로 등록한다.
            # AMR3 직접 Bool 명령과 중앙용 String(JSON)은 서로 다른 Topic이다.
            string_publish_topics = {
                config.ROBOT_SELECT_TOPIC,
                config.SERVICE_REQUEST_TOPIC,
                config.SERVICE_END_TOPIC,
                config.LUGGAGE_LOAD_CONFIRM_TOPIC,
            }
            if config.ROBOT_ID != "AMR1":
                string_publish_topics.add(config.EMERGENCY_END_TOPIC)

            if config.ALIGNMENT_INPUT_MODE == "legacy_bool":
                string_publish_topics.update(
                    {config.ALIGN_REQUEST_TOPIC, config.ALIGN_CONFIRM_TOPIC}
                )

            for topic in string_publish_topics:
                self._publishers[topic] = self.node.create_publisher(String, topic, 10)

            # 직접 제어 명령은 과거 명령을 재전달하지 않도록
            # RELIABLE + VOLATILE로 고정한다. 늦게 연결된 구독자가
            # 과거 True를 받아 오동작하면 안 된다.
            direct_command_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )

            # AMR1 직접 제어 명령: 로봇1과 Bool(True) 직접 통신 + DDS ACK.
            if config.ROBOT_ID == "AMR1":
                self.rb1_standby_pub = self.node.create_publisher(
                    Bool, config.RB1_STANDBY_TOPIC, direct_command_qos
                )
                self.rb1_service_end_pub = self.node.create_publisher(
                    Bool, config.RB1_SERVICE_END_TOPIC, direct_command_qos
                )
                self.emergency_end_pub = self.node.create_publisher(
                    Bool, config.EMERGENCY_END_TOPIC, direct_command_qos
                )

            # 정렬 방식별 ROS2 연결
            if config.ALIGNMENT_INPUT_MODE == "legacy_bool":
                self.retarget_web_pub = self.node.create_publisher(
                    Bool, config.RETARGET_WEB_TOPIC, 10
                )
                self.node.create_subscription(
                    String, config.ALIGN_STATUS_TOPIC, self._on_alignment_status, 10
                )
                self.node.create_subscription(
                    Bool,
                    config.REAR_PERSON_DETECTED_TOPIC,
                    self._on_rear_person_detected,
                    10,
                )
                self.node.create_subscription(
                    Bool,
                    config.FRONT_LEG_DETECTED_TOPIC,
                    self._on_front_leg_detected,
                    10,
                )
            elif config.ALIGNMENT_INPUT_MODE == "tracking_int32":
                self.turn_around_pub = self.node.create_publisher(
                    Bool, config.TURN_AROUND_TOPIC, direct_command_qos
                )
                self.carrying_pub = self.node.create_publisher(
                    Bool, config.CARRYING_TOPIC, direct_command_qos
                )
                self.direct_service_end_pub = self.node.create_publisher(
                    Bool, config.DIRECT_SERVICE_END_TOPIC, direct_command_qos
                )
                self.rb3_standby_pub = self.node.create_publisher(
                    Bool, config.RB3_STANDBY_TOPIC, direct_command_qos
                )
                self.node.create_subscription(
                    Bool, config.TURN_COMPLETE_TOPIC, self._on_turn_complete, 10
                )
                self.node.create_subscription(
                    Int32, config.TRACKING_WEB_TOPIC, self._on_tracking_web, 10
                )
                self.node.create_subscription(
                    Int32, config.TRACKING_RGBD_TOPIC, self._on_tracking_rgbd, 10
                )
            # manual은 카메라/검출 Topic이 필요 없다.

            # 도착 정보는 모든 로봇이 중앙에서 String(JSON)으로 받는다.
            self.node.create_subscription(
                String, config.ARRIVAL_STATUS_TOPIC, self._on_arrival_status, 10
            )

            if config.CONTROL_MODE == "CENTRAL_JSON":
                # AMR1: 중앙이 raw 센서/로봇 상태를 JSON으로 정규화해서 UI에 전달한다.
                self.node.create_subscription(
                    String,
                    config.EMERGENCY_EVENT_TOPIC,
                    self._on_emergency_event,
                    10,
                )
                self.node.create_subscription(
                    String, config.ROBOT_STATUS_TOPIC, self._on_robot_status, 10
                )
                self.node.create_subscription(
                    String, config.MISSION_STATUS_TOPIC, self._on_mission_status, 10
                )

            # CCTV/웹캠 원시 응급 감지 입력은 AMR1 UI에서만 직접 처리한다.
            # AMR3는 응급상황 로직을 태우지 않도록 /fall_detection을 구독하지 않는다.
            if config.FALL_DETECTION_EMERGENCY_ENABLED:
                self.node.create_subscription(
                    Bool, config.FALL_DETECTION_TOPIC, self._on_fall_detection_bool, 10
                )
                self.node.create_subscription(
                    Bool, config.EMERGENCY_CLEAR_TOPIC, self._on_emergency_clear_bool, 10
                )

            self.enabled = True
            self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self.spin_thread.start()

            print(
                "[AURA UI] ROS2 bridge ready: "
                f"node={config.ROS_NODE_NAME}, robot={config.ROBOT_ID}, "
                f"control_mode={config.CONTROL_MODE}, "
                f"alignment_mode={config.ALIGNMENT_INPUT_MODE}",
                flush=True,
            )
        except Exception as exc:
            print(
                f"[AURA UI] ROS2 bridge 초기화 실패, print-only mode: {exc}",
                flush=True,
            )
            self.enabled = False

    def set_emergency_handlers(
        self,
        *,
        on_detected: Callable[[dict[str, Any]], None],
        on_cleared: Callable[[dict[str, Any]], None],
    ) -> None:
        self._on_emergency_detected_handler = on_detected
        self._on_emergency_cleared_handler = on_cleared

    def _spin_loop(self) -> None:
        while self.enabled and not self.stop_event.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as exc:
                print(f"[AURA UI] spin warning: {exc}", flush=True)
                time.sleep(0.1)

    @staticmethod
    def _parse_json_object(raw: str, label: str) -> dict[str, Any] | None:
        try:
            payload = json.loads(raw)
            if not isinstance(payload, dict):
                raise ValueError("JSON object가 아닙니다.")
            return payload
        except Exception as exc:
            print(f"[AURA UI] 잘못된 {label} JSON 무시: {exc}", flush=True)
            return None

    def _payload_targets_this_robot(self, payload: dict[str, Any]) -> bool:
        target = payload.get("target_robot_id") or payload.get("robot_id")
        return not target or str(target).upper() == self.config.ROBOT_ID

    def _on_arrival_status(self, msg) -> None:
        print(
            f"[AURA UI SUBSCRIBE] {self.config.ARRIVAL_STATUS_TOPIC} -> {msg.data}",
            flush=True,
        )
        payload = self._parse_json_object(msg.data, "arrival")
        if payload is None or not self._payload_targets_this_robot(payload):
            return

        event_type = str(payload.get("event_type", "")).upper()
        status = str(payload.get("arrival_status", payload.get("status", ""))).upper()
        if event_type not in {"ARRIVED", "ARRIVAL_STATUS", "MISSION_ARRIVED"} and status not in {
            "ARRIVED", "REACHED", "SUCCEEDED", "SUCCESS"
        }:
            return

        if self.state_store.accept_arrival(payload):
            print("[AURA UI] 현재 미션의 도착 메시지 반영", flush=True)
        else:
            print("[AURA UI] 현재 미션과 맞지 않는 도착 메시지 무시", flush=True)

    # -------------------- AMR1 중앙 JSON 수신 --------------------
    def _on_emergency_event(self, msg) -> None:
        print(
            f"[AURA UI SUBSCRIBE] {self.config.EMERGENCY_EVENT_TOPIC} -> {msg.data}",
            flush=True,
        )
        payload = self._parse_json_object(msg.data, "emergency_event")
        if payload is None or not self._payload_targets_this_robot(payload):
            return

        event_type = str(payload.get("event_type", "")).upper()
        status = str(payload.get("status", "")).upper()
        active = payload.get("active")

        cleared = (
            event_type in {"EMERGENCY_END", "EMERGENCY_CLEARED", "EMERGENCY_RESOLVED"}
            or status in {"CLEARED", "RESOLVED", "ENDED"}
            or active is False
        )
        if cleared:
            if self._on_emergency_cleared_handler is not None:
                self._on_emergency_cleared_handler(payload)
            else:
                self.state_store.clear_emergency(self.config.ROBOT_ID)
            return

        detected = (
            event_type in {"FALL_DETECTED", "EMERGENCY_DETECTED", "EMERGENCY_START"}
            or status in {"ACTIVE", "DETECTED", "DISPATCHED"}
            or active is True
        )
        if not detected:
            return

        if self._on_emergency_detected_handler is not None:
            self._on_emergency_detected_handler(payload)
        else:
            self.state_store.activate_emergency(payload)

    def _on_robot_status(self, msg) -> None:
        payload = self._parse_json_object(msg.data, "robot_status")
        if payload is None or not self._payload_targets_this_robot(payload):
            return
        self.state_store.record_robot_status(payload)

    def _on_mission_status(self, msg) -> None:
        payload = self._parse_json_object(msg.data, "mission_status")
        if payload is None or not self._payload_targets_this_robot(payload):
            return
        self.state_store.record_mission_status(payload)

    # -------------------- 기존 Bool/Int32 정렬 방식 --------------------
    def _on_alignment_status(self, msg) -> None:
        payload = self._parse_json_object(msg.data, "alignment")
        if payload is None or not self._payload_targets_this_robot(payload):
            return
        detected = bool(payload.get("detected"))
        status = str(payload.get("status", "")).upper()
        if status in {"DETECTED", "CENTERED", "SUCCESS", "SUCCEEDED"}:
            detected = True
        service_type = str(payload.get("service_type") or "").upper() or None
        self.state_store.record_alignment_signal(
            detected=detected,
            source=self.config.ALIGN_STATUS_TOPIC,
            payload=payload,
            expected_service_type=service_type,
        )

    def _on_rear_person_detected(self, msg) -> None:
        detected = bool(msg.data)
        self.state_store.record_alignment_signal(
            detected=detected,
            source=self.config.REAR_PERSON_DETECTED_TOPIC,
            payload={"detected": detected, "camera": "REAR", "detector": "PERSON_YOLO"},
            expected_service_type="GUIDE",
        )

    def _on_front_leg_detected(self, msg) -> None:
        detected = bool(msg.data)
        self.state_store.record_alignment_signal(
            detected=detected,
            source=self.config.FRONT_LEG_DETECTED_TOPIC,
            payload={"detected": detected, "camera": "FRONT", "detector": "LEG_YOLO"},
            expected_service_type="LUGGAGE_ASSIST",
        )

    def _on_turn_complete(self, msg) -> None:
        if not bool(msg.data):
            return
        changed = self.state_store.mark_guide_turn_complete()
        if changed:
            print(
                f"[AURA UI TURN] {self.config.TURN_COMPLETE_TOPIC}=True "
                "-> tracking_web 판정 시작",
                flush=True,
            )

    def _on_tracking_web(self, msg) -> None:
        value = int(msg.data)
        accepted = self.state_store.record_tracking_state(
            value=value,
            source=self.config.TRACKING_WEB_TOPIC,
            expected_service_type="GUIDE",
        )

        # IDLE/PREALIGN/이미 DETECTED 상태에서 반복 수신되는 값은 조용히 무시한다.
        # 실제 상태 전환이 일어난 경우에만 로그를 남겨 터미널 로그 폭주를 막는다.
        if accepted:
            state = self.state_store.get()
            print(
                f"[AURA UI TRACKING] {self.config.TRACKING_WEB_TOPIC}={value} "
                f"accepted=True, status={state.get('alignment_status')}",
                flush=True,
            )

    def _on_tracking_rgbd(self, msg) -> None:
        value = int(msg.data)
        accepted = self.state_store.record_tracking_state(
            value=value,
            source=self.config.TRACKING_RGBD_TOPIC,
            expected_service_type="LUGGAGE_ASSIST",
        )
        if accepted:
            state = self.state_store.get()
            print(
                f"[AURA UI TRACKING] {self.config.TRACKING_RGBD_TOPIC}={value} "
                f"accepted=True, status={state.get('alignment_status')}",
                flush=True,
            )

    # -------------------- AMR1 Bool 응급 입력 --------------------
    def _on_fall_detection_bool(self, msg) -> None:
        if not bool(msg.data):
            return
        payload = {
            "event_type": "FALL_DETECTED",
            "source": "fall_detection",
            "detected": True,
            "robot_id": self.config.ROBOT_ID,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
        }
        if self._on_emergency_detected_handler is not None:
            self._on_emergency_detected_handler(payload)
        else:
            self.state_store.activate_emergency(payload)

    def _on_emergency_clear_bool(self, msg) -> None:
        if not bool(msg.data):
            return
        payload = {
            "event_type": "EMERGENCY_CLEARED",
            "source": "central_system",
            "robot_id": self.config.ROBOT_ID,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
        }
        if self._on_emergency_cleared_handler is not None:
            self._on_emergency_cleared_handler(payload)
        else:
            self.state_store.clear_emergency(self.config.ROBOT_ID)


    # -------------------- 로봇 직접 Bool(True) 명령 --------------------
    def _publish_direct_true(
        self,
        *,
        topic: str,
        publisher,
        before_publish: Callable[[], None] | None = None,
        require_subscriber: bool = True,
    ) -> bool:
        """로봇 직접 Bool(True) 명령을 DDS ACK 방식으로 정확히 1회 전송한다.

        기본 순서:
        1. DDS graph에서 subscriber가 나타날 때까지 제한 시간 동안 기다린다.
        2. Bool(data=True)를 정확히 한 번 publish한다.
        3. RELIABLE QoS의 ``wait_for_all_acked``로 DDS ACK를 기다린다.

        ``require_subscriber=False``인 /turn_around는 discovery 지연 때문에
        명령 자체가 막히지 않도록 subscriber 사전 대기 없이 바로 발행한다.

        DDS ACK는 구독 측 DDS 미들웨어가 메시지를 수신했다는 의미이며,
        로봇 동작 완료 자체를 의미하지는 않는다.
        """
        if require_subscriber:
            print(f"[AURA UI WAIT] {topic} subscriber 확인 중...", flush=True)

        # ROS2가 없는 PC에서는 화면 단독 확인을 위해 기존 print-only 동작을 유지한다.
        if not self.enabled or publisher is None:
            print(
                f"[AURA UI PRINT-ONLY] {topic} -> True "
                "(ROS2 publisher 비활성화)",
                flush=True,
            )
            return False

        subscriber_timeout_sec = float(
            self.config.DIRECT_COMMAND_WAIT_TIMEOUT_SECONDS
        )
        poll_interval = float(self.config.DIRECT_COMMAND_WAIT_POLL_SECONDS)
        ack_timeout_sec = float(self.config.DIRECT_COMMAND_ACK_TIMEOUT_SECONDS)

        with self._direct_publish_lock:
            if require_subscriber:
                deadline = time.monotonic() + subscriber_timeout_sec
                while publisher.get_subscription_count() <= 0:
                    if time.monotonic() >= deadline:
                        print(
                            f"[AURA UI ERROR] {topic}: "
                            f"{subscriber_timeout_sec:.1f}초 동안 subscriber를 "
                            "찾지 못해 발행하지 않았습니다.",
                            flush=True,
                        )
                        return False
                    time.sleep(poll_interval)

            subscriber_count = publisher.get_subscription_count()

            # GUIDE 회전처럼 수신 상태를 먼저 WAITING으로 무장해야 하는 경우,
            # 실제 publish 직전에 콜백을 실행한다. 이렇게 해야 /tracking_web=1이
            # 매우 빠르게 들어와도 PREALIGN 상태에서 버려지지 않는다.
            if before_publish is not None:
                try:
                    before_publish()
                except Exception as exc:
                    print(
                        f"[AURA UI ERROR] {topic}: publish 준비 콜백 실패 ({exc})",
                        flush=True,
                    )
                    return False

            msg = Bool()
            msg.data = True

            # 반복 전송하지 않고 정확히 한 번만 publish한다.
            try:
                publisher.publish(msg)
            except Exception as exc:
                print(
                    f"[AURA UI ERROR] {topic}: publish 실패 ({exc})",
                    flush=True,
                )
                return False
            print(
                f"[AURA UI PUBLISH] {topic} -> True "
                f"(1회 발행, subscriptions={subscriber_count})",
                flush=True,
            )

            acked = None
            try:
                acked = publisher.wait_for_all_acked(
                    timeout=Duration(seconds=ack_timeout_sec)
                )
            except Exception as exc:
                # 일부 RMW/rclpy 조합에서 ACK API 결과를 제공하지 못하더라도
                # 이미 1회 publish한 사실은 유지한다.
                print(
                    f"[AURA UI ACK WARNING] {topic}: "
                    f"DDS ACK 확인 불가 ({exc})",
                    flush=True,
                )

            if acked is True:
                print(
                    f"[AURA UI ACK] {topic} -> True "
                    f"(1회 발행, subscriptions={subscriber_count}, "
                    "dds_ack=True)",
                    flush=True,
                )
            elif acked is False:
                print(
                    f"[AURA UI ACK TIMEOUT] {topic} -> True "
                    f"(1회 발행, subscriptions={subscriber_count}, "
                    f"ack_timeout={ack_timeout_sec:.2f}s)",
                    flush=True,
                )
            else:
                print(
                    f"[AURA UI ACK UNAVAILABLE] {topic} "
                    f"(subscriptions={subscriber_count})",
                    flush=True,
                )

            # 반환값은 '메시지를 1회 publish했는지'를 나타낸다.
            # ACK timeout은 중복 재발행 사유로 사용하지 않는다.
            return True

    def publish_turn_around(
        self,
        before_publish: Callable[[], None] | None = None,
    ) -> bool:
        return self._publish_direct_true(
            topic=self.config.TURN_AROUND_TOPIC,
            publisher=self.turn_around_pub,
            before_publish=before_publish,
            require_subscriber=False,
        )

    # V4 코드의 오타 메서드명 호환. 새 코드에서는 publish_turn_around() 사용.
    def publish_turn_arround(self) -> bool:
        return self.publish_turn_around()

    def publish_carrying(self) -> bool:
        return self._publish_direct_true(
            topic=self.config.CARRYING_TOPIC,
            publisher=self.carrying_pub,
        )

    def publish_direct_service_end(self) -> bool:
        return self._publish_direct_true(
            topic=self.config.DIRECT_SERVICE_END_TOPIC,
            publisher=self.direct_service_end_pub,
        )

    def publish_rb1_standby(self) -> bool:
        return self._publish_direct_true(
            topic=self.config.RB1_STANDBY_TOPIC,
            publisher=self.rb1_standby_pub,
        )

    def publish_rb1_service_end(self) -> bool:
        return self._publish_direct_true(
            topic=self.config.RB1_SERVICE_END_TOPIC,
            publisher=self.rb1_service_end_pub,
        )

    def publish_emergency_end(self) -> bool:
        return self._publish_direct_true(
            topic=self.config.EMERGENCY_END_TOPIC,
            publisher=self.emergency_end_pub,
        )

    def publish_rb3_standby(self) -> bool:
        return self._publish_direct_true(
            topic=self.config.RB3_STANDBY_TOPIC,
            publisher=self.rb3_standby_pub,
        )

    def publish_retarget_web(self, value: bool = True) -> bool:
        if self.enabled and self.retarget_web_pub is not None:
            msg = Bool()
            msg.data = bool(value)
            self.retarget_web_pub.publish(msg)
        return bool(value)

    # -------------------- 공통 String(JSON) 발행 --------------------
    def publish_json(self, topic_name: str, payload: dict[str, Any]) -> str:
        payload = dict(payload)
        payload.setdefault("sent_at", datetime.now().isoformat(timespec="seconds"))
        data = json.dumps(payload, ensure_ascii=False)
        print(f"[AURA UI PUBLISH] {topic_name} -> {data}", flush=True)

        if not self.enabled:
            return data
        publisher = self._publishers.get(topic_name)
        if publisher is None:
            raise ValueError(f"등록되지 않은 String(JSON) publish topic: {topic_name}")
        msg = String()
        msg.data = data
        publisher.publish(msg)
        return data

    def shutdown(self) -> None:
        if not self.enabled:
            return
        try:
            self.stop_event.set()
            if self.spin_thread and self.spin_thread.is_alive():
                self.spin_thread.join(timeout=0.5)
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            print(f"[AURA UI] shutdown warning: {exc}", flush=True)
