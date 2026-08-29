# third_party

외부(서드파티) 라이브러리를 **git submodule**로 참조합니다. 저장소에는
소스를 직접 커밋하지 않습니다.

## UniDepth (UniDepthV2)

`robot3_control/guide_rear_tracker_node.py`(후방 카메라 가이드 추적)가
단안(monocular) 깊이 추정을 위해 사용합니다.

```bash
# 최초 클론 후 1회
git submodule update --init --recursive

# editable 설치 (가상환경 권장)
pip install -e third_party/UniDepth
```

- 업스트림: https://github.com/lpiccinelli-eth/UniDepth
- 라이선스는 업스트림 저장소를 따릅니다.
