(() => {
  const body = document.body;
  if (body.dataset.luggageTracking !== 'true') return;

  const stateUrl = body.dataset.alignmentStateUrl;
  const confirmUrl = body.dataset.confirmUrl;
  const successDelayMs = Number(body.dataset.successDelayMs || 2000);

  const title = document.getElementById('luggageTitle');
  const subtitle = document.getElementById('luggageSubtitle');
  const caption = document.getElementById('luggageCaption');
  const note = document.getElementById('trackingNote');
  const button = document.getElementById('luggageWaitBtn');

  let redirecting = false;

  function showRetry(value) {
    if (title) title.textContent = '다시 인식해 주세요';
    if (subtitle) subtitle.textContent = '로봇 앞에 다시 서주세요';
    if (caption) caption.textContent = '다시 로봇 앞에 서주세요';

    if (note) {
      note.classList.remove('success');
      note.textContent = value === 2
        ? '승객을 다시 찾고 있습니다'
        : '승객 인식 대기 중';
    }

    if (button) {
      button.classList.remove('success');
      button.textContent = '승객 재인식 중';
    }
  }

  function showSuccess() {
    if (title) title.textContent = '인식이 완료되었습니다';
    if (subtitle) subtitle.textContent = '로봇이 출발할 준비를 마쳤습니다';
    if (caption) caption.textContent = '이제 출발하시면 됩니다';

    if (note) {
      note.classList.add('success');
      note.textContent = '출발 준비가 완료되었습니다';
    }

    if (button) {
      button.classList.add('success');
      button.textContent = '출발 준비 완료';
    }
  }

  async function pollTracking() {
    if (!stateUrl || !confirmUrl || redirecting) return;

    try {
      const response = await fetch(stateUrl, {
        cache: 'no-store',
        credentials: 'same-origin',
      });
      const state = await response.json();

      if (state.service_type !== 'LUGGAGE_ASSIST') return;

      const trackingValue = Number(state.tracking_value || 0);

      if (state.detected || state.status === 'DETECTED' || trackingValue === 1) {
        redirecting = true;
        showSuccess();

        // 성공 화면을 충분히 보여준 뒤 기존 출발 처리 route로 이동한다.
        window.setTimeout(() => {
          window.location.replace(confirmUrl);
        }, successDelayMs);
        return;
      }

      // 사용자 요구: 0과 2 모두 같은 재정렬 안내 화면을 표시한다.
      if (trackingValue === 0 || trackingValue === 2) {
        showRetry(trackingValue);
      }
    } catch (error) {
      console.error('[AURA] luggage tracking polling failed:', error);
      if (title) title.textContent = '인식 상태 확인 중';
      if (subtitle) subtitle.textContent = '로봇 앞에서 잠시 기다려주세요';
      if (caption) caption.textContent = '다시 로봇 앞에 서주세요';
      if (note) {
        note.classList.remove('success');
        note.textContent = '인식 상태를 확인하지 못했습니다. 다시 시도합니다';
      }
    }
  }

  window.setInterval(pollTracking, 400);
  pollTracking();
})();
