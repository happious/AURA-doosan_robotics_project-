"""Robot3 목적지 mission_cmd 발행 테스트용 UI 실행.

기존 run_amr3.py는 변경하지 않는다.
이 파일로 실행한 경우에만 tracking_web 인식 성공 후
/robot3/mission_cmd (std_msgs/msg/String)를 1회 발행한다.
"""
import os

os.environ["AURA_ROBOT3_MISSION_CMD_TEST_ENABLED"] = "1"
os.environ.setdefault("AURA_ROBOT3_MISSION_CMD_TOPIC", "/robot3/mission_cmd")

from launcher import launch_ui


if __name__ == "__main__":
    launch_ui(
        robot_id="AMR3",
        port=5003,
        alignment_input_mode="tracking_int32",
        tracking_web_topic="/tracking_web",
        tracking_rgbd_topic="/tracking_rgbd",
        turn_around_topic="/turn_around",
        turn_complete_topic="/turn_complete",
        carrying_topic="/carrying",
        rb3_standby_topic="/rb3_standby",
        direct_service_end_topic="/service_end",
    )
