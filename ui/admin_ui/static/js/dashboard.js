const $ = (id) => document.getElementById(id);
let lastError = "";

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;").replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;").replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function formatDate(value) {
  if (!value) return "-";
  const normalized = String(value).includes("T") ? value : String(value).replace(" ", "T") + "Z";
  const date = new Date(normalized);
  if (Number.isNaN(date.getTime())) return value;
  return new Intl.DateTimeFormat("ko-KR", {
    month: "2-digit", day: "2-digit", hour: "2-digit", minute: "2-digit", second: "2-digit"
  }).format(date);
}

function formatTime(value) {
  if (!value) return "--:--:--";
  const normalized = String(value).includes("T") ? value : String(value).replace(" ", "T") + "Z";
  const date = new Date(normalized);
  if (Number.isNaN(date.getTime())) return value;
  return date.toLocaleTimeString("ko-KR", { hour12: false });
}

function modeLabel(mode) {
  if (mode === null || mode === undefined) return "상태 미수신";
  return `RobotStatus mode ${mode}`;
}

function serviceLabel(type) {
  const labels = { GUIDE: "승객 안내", LUGGAGE_ASSIST: "짐 보조·동행", ACCOMPANY: "동행", EMERGENCY_DISPATCH: "응급 출동", HOTPLACE_DISPATCH: "혼잡구역 대응" };
  return labels[type] || type || "-";
}

function setPill(element, kind, text) {
  element.className = `status-pill ${kind || ""}`;
  element.querySelector("span").textContent = text;
}

function showToast(message) {
  if (!message || message === lastError) return;
  lastError = message;
  const toast = $("toast");
  toast.textContent = message;
  toast.classList.add("show");
  setTimeout(() => toast.classList.remove("show"), 5000);
}

function renderRobots(robots) {
  const grid = $("robotGrid");
  if (!robots.length) {
    grid.innerHTML = '<div class="empty-row">로봇 상태 토픽을 기다리고 있습니다.</div>';
    return;
  }
  grid.innerHTML = robots.map(robot => {
    const online = robot.last_seen_age_sec !== null && Number(robot.last_seen_age_sec) <= 5;
    const battery = Math.max(0, Math.min(100, Number(robot.battery_pct || 0)));
    return `
      <article class="robot-card ${online ? "" : "offline"}">
        <div class="robot-title">
          <strong>${escapeHtml(robot.amr_id || robot.robot_id)}</strong>
          <span class="badge ${online ? "" : "offline"}">${online ? "ONLINE" : "OFFLINE"}</span>
        </div>
        <div class="robot-mode">${escapeHtml(modeLabel(robot.mode))}</div>
        <div class="metric-row">
          <div class="metric"><span>배터리</span><b>${battery.toFixed(0)}%</b></div>
          <div class="metric"><span>AED 탑재</span><b>${robot.aed_loaded ? "탑재" : "미탑재"}</b></div>
          <div class="metric"><span>사용 가능</span><b>${robot.available ? "가능" : "불가"}</b></div>
          <div class="metric"><span>현재 위치</span><b>${Number(robot.pose_x || 0).toFixed(2)}, ${Number(robot.pose_y || 0).toFixed(2)}</b></div>
        </div>
        <div class="battery"><i style="width:${battery}%"></i></div>
      </article>`;
  }).join("");
  $("robotUpdated").textContent = `최근 갱신 ${new Date().toLocaleTimeString("ko-KR", {hour12:false})}`;
}

function renderEmergencies(items) {
  const list = $("emergencyList");
  if (!items.length) {
    list.innerHTML = '<div class="empty-row">등록된 응급 이벤트가 없습니다.</div>';
    return;
  }
  list.innerHTML = items.slice(0, 8).map(item => {
    const active = item.status !== "CLEARED";
    const location = item.point_x === null ? "위치 대기 중" : `(${Number(item.point_x).toFixed(2)}, ${Number(item.point_y).toFixed(2)})`;
    return `
      <div class="emergency-item ${active ? "active" : ""}">
        <div class="emergency-icon">!</div>
        <div class="emergency-copy">
          <strong>응급 이벤트 #${item.id} · ${escapeHtml(item.status)}</strong>
          <span>${escapeHtml(location)} · 출동 ${escapeHtml(item.dispatched_robot_id || "미배정")}</span>
        </div>
        <span class="subtle">${escapeHtml(formatTime(item.detected_at))}</span>
      </div>`;
  }).join("");
}

function renderSnapshots(images) {
  const grid = $("snapshotGrid");
  if (!images.length) {
    grid.innerHTML = '<div class="empty-row">저장된 낙상 장면이 없습니다.</div>';
    return;
  }
  grid.innerHTML = images.slice(0, 8).map(image => `
    <figure class="snapshot">
      <a href="/event-image/${image.id}" target="_blank" rel="noopener">
        <img src="/event-image/${image.id}" alt="낙상 감지 장면 ${image.id}" loading="lazy">
      </a>
      <figcaption>이벤트 #${escapeHtml(image.emergency_id || "-")}<br>${escapeHtml(formatDate(image.captured_at))}</figcaption>
    </figure>`).join("");
}

function renderLogs(logs) {
  const list = $("logList");
  if (!logs.length) {
    list.innerHTML = '<div class="empty-row">시스템 로그가 없습니다.</div>';
    return;
  }
  list.innerHTML = logs.slice(0, 40).map(log => `
    <div class="log-item">
      <span class="log-time">${escapeHtml(formatTime(log.created_at))}</span>
      <span class="log-level ${escapeHtml(log.level)}">${escapeHtml(log.level)}</span>
      <span class="log-message" title="${escapeHtml(log.message)}">${escapeHtml(log.message)}</span>
    </div>`).join("");
}

function renderCamera(camera) {
  if (camera.receiving) {
    setPill($("cameraStatus"), "ok", "CCTV 수신 중");
  } else if (camera.enabled) {
    setPill($("cameraStatus"), "warn", "CCTV 대기 중");
  } else {
    setPill($("cameraStatus"), "danger", "CCTV 비활성");
  }
  const resolution = camera.width && camera.height ? `${camera.width} × ${camera.height} · ${camera.encoding || ""}` : "영상 수신 대기";
  $("cameraResolution").textContent = resolution;
  $("cameraLastSeen").textContent = camera.last_received_at ? `최근 ${formatTime(camera.last_received_at)}` : (camera.error || "프레임 없음");
}


function renderPopulation(population) {
  const value = population && population.value !== null && population.value !== undefined
    ? Number(population.value)
    : null;

  $("summaryPopulation").textContent = value === null
    ? "-"
    : value.toLocaleString("ko-KR");

  if (population?.receiving) {
    setPill($("populationStatus"), "ok", "인원수 수신 중");
    $("summaryPopulationStatus").textContent = population.last_received_at
      ? `최근 ${formatTime(population.last_received_at)} · /population`
      : "/population 수신 중";
  } else if (population?.enabled) {
    setPill($("populationStatus"), "warn", "인원수 대기 중");
    $("summaryPopulationStatus").textContent = value === null
      ? "/population 수신 대기"
      : `마지막 수신 ${formatTime(population.last_received_at)}`;
  } else {
    setPill($("populationStatus"), "danger", "인원수 비활성");
    $("summaryPopulationStatus").textContent = population?.error || "ROS2 연결 필요";
  }
}

function renderDatabaseUnavailable(message) {
  $("summaryRobots").textContent = "-";
  $("summaryEmergencies").textContent = "-";
  $("summaryCompleted").textContent = "-";
  $("robotGrid").innerHTML = '<div class="empty-row">DB 연결 후 로봇 기록이 표시됩니다.</div>';
  $("emergencyList").innerHTML = '<div class="empty-row">DB 연결 후 응급 기록이 표시됩니다.</div>';
  $("snapshotGrid").innerHTML = '<div class="empty-row">DB 연결 후 저장 장면이 표시됩니다.</div>';
  $("logList").innerHTML = '<div class="empty-row">DB 연결 후 시스템 로그가 표시됩니다.</div>';
  $("robotUpdated").textContent = "DB 연결 대기 중";
  if (message) showToast(message);
}

async function updateDashboard() {
  try {
    const response = await fetch("/api/dashboard", { cache: "no-store" });
    const data = await response.json();

    // ROS2 실시간 항목은 DB 성공 여부와 무관하게 항상 먼저 갱신한다.
    renderCamera(data.camera || {});
    renderPopulation(data.population || {});

    if (!data.ok) {
      throw new Error(data.error || "관리자 서버 데이터 조회 실패");
    }

    if (!data.db_ok) {
      setPill($("dbStatus"), "danger", "DB 연결 실패");
      renderDatabaseUnavailable(data.db_error || "DB 연결 실패");
      return;
    }

    setPill($("dbStatus"), "ok", "DB 연결됨");
    $("summaryRobots").textContent = data.summary.robot_count;
    $("summaryEmergencies").textContent = data.summary.emergency_count;
    $("summaryCompleted").textContent = data.summary.completed_service_count;

    renderRobots(data.robots || []);
    renderEmergencies(data.emergencies || []);
    renderSnapshots(data.images || []);
    renderLogs(data.logs || []);
    lastError = "";
  } catch (error) {
    setPill($("dbStatus"), "danger", "서버 연결 실패");
    showToast(error.message || String(error));
  }
}

function updateClock() {
  const now = new Date();
  $("clockTime").textContent = now.toLocaleTimeString("ko-KR", { hour12: false });
  $("clockDate").textContent = now.toLocaleDateString("ko-KR", { year: "numeric", month: "long", day: "numeric", weekday: "short" });
}



let populationEventSource = null;

function startPopulationStream() {
  if (!("EventSource" in window)) {
    console.warn("EventSource 미지원: 1초 폴링으로 인원수를 갱신합니다.");
    return;
  }

  if (populationEventSource) {
    populationEventSource.close();
  }

  populationEventSource = new EventSource("/api/population/stream");

  populationEventSource.addEventListener("population", (event) => {
    try {
      const population = JSON.parse(event.data);
      renderPopulation(population);
    } catch (error) {
      console.error("population SSE 파싱 실패", error);
    }
  });

  populationEventSource.onopen = () => {
    console.info("/population 실시간 스트림 연결됨");
  };

  populationEventSource.onerror = () => {
    // EventSource가 자동 재연결하므로 화면 전체 오류로 처리하지 않는다.
    console.warn("/population 실시간 스트림 재연결 중");
  };
}

window.addEventListener("beforeunload", () => {
  if (populationEventSource) populationEventSource.close();
});


updateClock();
setInterval(updateClock, 1000);
updateDashboard();
startPopulationStream();
// DB/로봇/로그는 1초 폴링 유지. /population은 SSE로 토픽 수신 즉시 갱신한다.
setInterval(updateDashboard, 1000);
