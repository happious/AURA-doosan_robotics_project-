(() => {
  const body = document.body;
  const stateUrl = body.dataset.systemStateUrl;
  if (!stateUrl) return;

  let redirecting = false;

  function pathOf(url) {
    return new URL(url, window.location.origin).pathname;
  }

  async function pollEmergencyState() {
    if (redirecting) return;

    try {
      const response = await fetch(stateUrl, {
        cache: 'no-store',
        credentials: 'same-origin',
      });
      const state = await response.json();
      const emergencyPath = state.emergency_url ? pathOf(state.emergency_url) : '/emergency';
      const currentPath = window.location.pathname;

      if (state.emergency_active && currentPath !== emergencyPath) {
        redirecting = true;
        window.location.replace(state.emergency_url);
        return;
      }

      if (!state.emergency_active && currentPath === emergencyPath && state.emergency_resolved_url) {
        redirecting = true;
        window.location.replace(state.emergency_resolved_url);
      }
    } catch (error) {
      console.error('[AURA] emergency state polling failed:', error);
    }
  }

  window.setInterval(pollEmergencyState, 500);
  pollEmergencyState();
})();
