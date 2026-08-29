"""현재 프로젝트에서 사용하는 전체 UI(AMR1 + AMR3)를 동시에 실행한다."""
from __future__ import annotations
import subprocess
import sys
import time
from pathlib import Path
from network_info import print_access_urls

PROJECT_ROOT = Path(__file__).resolve().parent


def main() -> None:
    ui_files = ["run_amr1.py", "run_amr3.py"]
    processes: list[subprocess.Popen] = []
    try:
        for ui_file in ui_files:
            processes.append(subprocess.Popen([sys.executable, str(PROJECT_ROOT / ui_file)], cwd=PROJECT_ROOT))
        print_access_urls({"AMR1": 5001, "AMR3": 5003})
        print("[AURA UI] AMR1/AMR3 UI를 종료하려면 Ctrl+C", flush=True)
        while True:
            for ui_file, process in zip(ui_files, processes):
                if process.poll() is not None:
                    raise RuntimeError(f"{ui_file}가 종료되었습니다. returncode={process.returncode}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[AURA UI] AMR1/AMR3 UI 종료 중...", flush=True)
    except RuntimeError as exc:
        print(f"[AURA UI] 오류: {exc}", flush=True)
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()

if __name__ == "__main__":
    main()
