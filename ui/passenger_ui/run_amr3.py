"""AMR3 승객 UI 실행: python3 run_amr3.py

로봇3는 Action Server 없이 비전 노드의 Int32 추적 상태를 직접 사용한다.
"""
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
