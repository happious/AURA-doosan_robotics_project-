"""AURA UI 접속 주소 표시용 네트워크 정보."""

from __future__ import annotations

import re
import subprocess


def get_lan_ip() -> str | None:
    """현재 PC가 기본 네트워크 통신에 사용하는 IPv4 주소를 반환한다.

    1. `ip route get`의 src 주소를 우선 사용한다.
    2. 찾지 못하면 `hostname -I`에서 loopback이 아닌 IPv4를 사용한다.
    3. 확인할 수 없으면 None을 반환한다.
    """
    try:
        result = subprocess.run(
            ["ip", "route", "get", "1.1.1.1"],
            check=True,
            capture_output=True,
            text=True,
        )
        match = re.search(r"\bsrc\s+(\d+\.\d+\.\d+\.\d+)", result.stdout)
        if match and not match.group(1).startswith("127."):
            return match.group(1)
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass

    try:
        result = subprocess.run(
            ["hostname", "-I"],
            check=True,
            capture_output=True,
            text=True,
        )
        for value in result.stdout.split():
            if re.fullmatch(r"\d+\.\d+\.\d+\.\d+", value) and not value.startswith("127."):
                return value
    except (FileNotFoundError, subprocess.CalledProcessError):
        pass

    return None


def print_access_urls(robot_ports: dict[str, int]) -> None:
    """같은 PC용 주소와 다른 기기용 주소를 함께 출력한다."""
    lan_ip = get_lan_ip()

    print("[AURA UI] 같은 PC에서 접속:", flush=True)
    for robot_id, port in robot_ports.items():
        print(f"  {robot_id}: http://127.0.0.1:{port}", flush=True)

    if lan_ip:
        print("[AURA UI] 같은 네트워크의 휴대폰/다른 PC에서 접속:", flush=True)
        for robot_id, port in robot_ports.items():
            print(f"  {robot_id}: http://{lan_ip}:{port}", flush=True)
    else:
        print(
            "[AURA UI] LAN IP를 자동 확인하지 못했습니다. "
            "터미널에서 `hostname -I`로 확인하세요.",
            flush=True,
        )
