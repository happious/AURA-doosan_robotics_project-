(() => {
  function fitKiosk() {
    const frame = document.querySelector('.frame');
    if (!frame) return;
    const width = Number(frame.dataset.designWidth || 1080);
    const height = Number(frame.dataset.designHeight || 3240);
    const scale = Math.min(window.innerWidth / width, window.innerHeight / height) * 0.94;
    frame.style.transform = `scale(${scale})`;
  }

  window.addEventListener('resize', fitKiosk);
  window.addEventListener('DOMContentLoaded', fitKiosk);
  fitKiosk();
})();
