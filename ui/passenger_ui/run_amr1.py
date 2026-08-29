"""AMR1 승객 UI 실행: python3 run_amr1.py

AMR1은 카메라 정렬 없이 중앙 fleet_dispatcher_node.py와
std_msgs/msg/String(JSON) Topic으로만 통신한다.
"""
from launcher import launch_ui


if __name__ == "__main__":
    launch_ui(
        robot_id="AMR1",
        port=5001,
        alignment_input_mode="manual",
    )
