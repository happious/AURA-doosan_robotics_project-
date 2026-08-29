(() => {
  const body = document.body;
  const timeoutUrl = body.dataset.timeoutUrl;
  const seconds = Number(body.dataset.timeoutSeconds || 0);
  if (!timeoutUrl || !seconds) return;

  let timerId = null;
  let sent = false;

  async function sendTimeout() {
    if (sent) return;
    sent = true;
    try {
      const response = await fetch(timeoutUrl, {
        method: 'POST',
        credentials: 'same-origin',
        cache: 'no-store'
      });
      const result = await response.json();
      if (response.ok && result.redirect_url) {
        window.location.replace(result.redirect_url);
        return;
      }
      sent = false;
      resetTimer();
    } catch (error) {
      console.error('[AURA] inactivity timeout failed:', error);
      sent = false;
      resetTimer();
    }
  }

  function resetTimer() {
    if (sent) return;
    window.clearTimeout(timerId);
    timerId = window.setTimeout(sendTimeout, seconds * 1000);
  }

  ['click', 'touchstart', 'keydown', 'input', 'change'].forEach((eventName) => {
    document.addEventListener(eventName, resetTimer, { passive: true });
  });

  resetTimer();
})();
