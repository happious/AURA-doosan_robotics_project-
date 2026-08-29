(() => {
  const body = document.body;
  const stateUrl = body.dataset.missionStateUrl;
  if (!stateUrl) return;

  async function pollArrival() {
    try {
      const response = await fetch(stateUrl, { cache: 'no-store' });
      const state = await response.json();
      if (state.arrived && state.arrival_url) {
        window.location.replace(state.arrival_url);
      }
    } catch (error) {
      console.error('[AURA] mission state polling failed:', error);
    }
  }

  window.setInterval(pollArrival, 1000);
  pollArrival();
})();
