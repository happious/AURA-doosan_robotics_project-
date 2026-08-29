(() => {
  const body = document.body;
  const stateUrl = body.dataset.alignmentStateUrl;
  const retryUrl = body.dataset.retryUrl;
  const confirmUrl = body.dataset.confirmUrl;
  const isRetryPage = body.dataset.isRetry === 'true';
  const autoConfirm = body.dataset.autoConfirm === 'true';

  const overlay = document.querySelector('.overlay');
  const statusPill = document.getElementById('statusPill');
  const statusText = document.getElementById('statusText');
  const detectionLabel = document.getElementById('detectionLabel');
  const captionTitle = document.getElementById('captionTitle');
  const captionText = document.getElementById('captionText');
  const confirmButton = document.getElementById('alignDoneBtn');
  const retryForm = document.getElementById('retryForm');

  let redirecting = false;

  function sourceName(state) {
    if (state.alignment_input_mode === 'tracking_int32') {
      return state.tracking_topic || '/tracking_web';
    }
    return state.detector || 'YOLO';
  }

  function setButtonDisabled(disabled) {
    if (confirmButton && confirmButton.tagName === 'BUTTON') {
      confirmButton.disabled = disabled;
    }
  }

  function showPrealign() {
    overlay?.classList.remove('detected');
    statusPill?.classList.remove('warning');
    if (statusText) statusText.textContent = isRetryPage ? '다시 정렬해주세요' : '정렬 완료 버튼을 눌러주세요';
    if (detectionLabel) detectionLabel.textContent = isRetryPage ? '전방 재정렬 준비' : '전방 정렬 준비';
    if (captionTitle) captionTitle.textContent = isRetryPage
      ? '다시 한번 로봇을 정면으로 봐주세요'
      : '사람이 로봇을 정면으로 봐주세요';
    if (captionText) captionText.textContent = '로봇 전방 중앙에 선 뒤 정렬 완료 버튼을 눌러주세요';
    setButtonDisabled(false);
    retryForm?.classList.add('hidden');
  }

  function showTurningToRear(state) {
    overlay?.classList.remove('detected');
    statusPill?.classList.remove('warning');
    if (statusText) statusText.textContent = '로봇 회전 중';
    if (detectionLabel) detectionLabel.textContent = '회전 완료 신호 대기 중';
    if (captionTitle) captionTitle.textContent = '로봇이 후방 카메라 방향으로 회전하고 있습니다';
    if (captionText) captionText.textContent = '회전이 완료되면 후방 카메라가 승객을 확인합니다';
    setButtonDisabled(true);
    retryForm?.classList.add('hidden');
  }

  function showWaiting(state) {
    overlay?.classList.remove('detected');
    statusPill?.classList.remove('warning');
    if (statusText) statusText.textContent = '승객 확인 중';
    if (detectionLabel) {
      detectionLabel.textContent = '후방 카메라 인식 대기 중';
    }
    if (captionTitle) captionTitle.textContent = '후방 승객을 확인하고 있습니다';
    if (captionText) captionText.textContent = '연속 기준 프레임을 만족하면 자동으로 운행을 시작합니다';
    setButtonDisabled(true);
    retryForm?.classList.add('hidden');
  }

  function showDetected(state) {
    overlay?.classList.add('detected');
    statusPill?.classList.remove('warning');
    if (statusText) statusText.textContent = '승객 확인 완료';
    if (detectionLabel) detectionLabel.textContent = '운행 준비 완료';
    if (captionTitle) captionTitle.textContent = '승객 인식이 완료되었습니다';
    if (captionText) captionText.textContent = '운행 상태로 전환합니다';
    setButtonDisabled(false);
    retryForm?.classList.add('hidden');
  }

  function goTo(url) {
    if (!url || redirecting) return;
    redirecting = true;
    window.location.replace(url);
  }

  async function pollAlignment() {
    if (!stateUrl || redirecting) return;

    try {
      const response = await fetch(stateUrl, {
        cache: 'no-store',
        credentials: 'same-origin',
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);

      const state = await response.json();

      if (state.status === 'PREALIGN') {
        // 최초 화면이 아닌 곳에서 재정렬 PREALIGN으로 바뀌면 반드시
        // retry 템플릿으로 다시 렌더링해 정렬 완료 버튼 form을 복구한다.
        if (state.retry && !isRetryPage && retryUrl) {
          goTo(retryUrl);
          return;
        }
        showPrealign();
        return;
      }

      if (state.status === 'TURNING_TO_REAR' || state.turning_to_rear) {
        showTurningToRear(state);
        return;
      }


      if (state.detected || state.status === 'DETECTED') {
        showDetected(state);

        // tracking_web=1이면 추가 버튼 입력 없이 서버의 서비스 시작 route로 이동한다.
        if (autoConfirm && confirmUrl) {
          const separator = confirmUrl.includes('?') ? '&' : '?';
          goTo(`${confirmUrl}${separator}auto=1`);
        }
        return;
      }


      showWaiting(state);
    } catch (error) {
      console.error('[AURA] alignment polling failed:', error);
    }
  }

  window.setInterval(pollAlignment, 250);
  pollAlignment();
})();
