// 핑키 상세: the panel that opens when a Pinky card is tapped. Its live state and every sensor reading, the route
// and safety monitor, its marker, its waypoints, and the operator actions that used to live on the fleet dashboard:
// 정지, 주행준비 (with the map to load), 위치 재설정, Nav2 복구, 웨이포인트 기록 and 삭제.
//
// Loaded after pinky.js (pinkyScreen, pinkySheet, pinkyById, pinkyName, sendPinkyCommand, stopPinky, setHtmlIfChanged,
// batteryStatHtml, readinessChipHtml, markerChipHtml, lastStopHtml, commandOutcome, pinkyRouteLabels,
// pinkyActionLabels) and pinky-map.js (pinkyMapState).
//
// The panel is built once per opened Pinky and then updated in place on every read, so a typed waypoint name or the
// chosen map is never lost to a poll.

const pinkySheetTitle = document.querySelector('#pinkySheetTitle');
const pinkySheetBody = document.querySelector('#pinkySheetBody');
const pinkyDetail = { pinkyId: null };

// The maps the fleet dashboard can bring up; map-proto.yaml carries the keepout mask.
const pinkyMaps = [
  ['map-proto.yaml', 'map-proto (금지 구역)'], ['my_map.yaml', 'my_map'], ['pinklab.yaml', 'pinklab'],
];

function routeProgressText(route) {
  if (!route?.status) return route?.current_label || '경로 없음';
  const parts = [pinkyRouteLabels[route.status] || route.status];
  if (route.current_label) parts.push(`웨이포인트 ${route.current_label}`);
  if (route.total) parts.push(`${route.completed ?? 0}/${route.total}`);
  return parts.join(', ');
}

function detailSkeleton(pinkyId) {
  const id = escapeHtml(pinkyId);
  return `
    <section class="sheet-status">
      <div class="sheet-status-main">
        <p class="sheet-identity" data-field="identity"></p>
        <p class="sheet-phase" data-field="phase"></p>
        <div class="pinky-chips" data-field="chips"></div>
      </div>
      <div data-field="battery"></div>
    </section>
    <button type="button" class="pinky-stop sheet-stop" data-pinky-stop="${id}" data-field="stop">정지</button>
    <div data-field="last-stop"></div>

    <section class="sheet-group" aria-label="주행 준비">
      <h3>주행 준비</h3>
      <div class="sheet-action">
        <div><strong>주행준비</strong><small>고른 지도로 Nav2를 띄우도록 핑키 관제에 요청해요.</small></div>
        <div class="sheet-action-controls">
          <label class="visually-hidden" for="pinkyMapChoice">불러올 지도</label>
          <select id="pinkyMapChoice" data-field="map">${pinkyMaps.map(([value, label]) => `<option value="${value}">${label}</option>`).join('')}</select>
          <button type="button" data-pinky-action="ready">주행준비</button>
        </div>
      </div>
      <div class="sheet-action">
        <div><strong>위치 재설정</strong><small>핑키가 자기 주차장에 서 있을 때만 쓰세요. 주차장 위치로 지도 위 위치를 다시 맞춰요.</small></div>
        <button type="button" class="secondary" data-pinky-action="initial-pose">위치 재설정</button>
      </div>
      <div class="sheet-action">
        <div><strong>Nav2 복구</strong><small>Nav2를 다시 띄우도록 요청해요.</small></div>
        <button type="button" class="secondary" data-pinky-action="nav-restart">Nav2 복구</button>
      </div>
    </section>

    <section class="sheet-group" aria-label="웨이포인트">
      <h3>웨이포인트</h3>
      <form class="sheet-waypoint-form" data-waypoint-form novalidate>
        <label class="visually-hidden" for="pinkyWaypointLabel">기록할 지점 이름</label>
        <input id="pinkyWaypointLabel" name="label" list="waypointLabelOptions" maxlength="48" placeholder="지점 이름 (예: 5)" autocomplete="off">
        <button type="submit" data-pinky-action="waypoints">지금 위치 기록</button>
      </form>
      <p class="sheet-hint" data-field="waypoint-hint"></p>
      <ul class="sheet-waypoints" data-field="waypoints"></ul>
    </section>

    <section class="sheet-group" aria-label="센서">
      <h3>센서</h3>
      <ul class="sensor-list" data-field="sensors"></ul>
    </section>

    <section class="sheet-group" aria-label="경로와 안전">
      <h3>경로와 안전</h3>
      <dl class="pinky-facts" data-field="facts"></dl>
    </section>

    <section class="sheet-group" aria-label="이 핑키에 보낸 명령">
      <h3>이 핑키에 보낸 명령</h3>
      <ol class="sheet-log" data-field="log"></ol>
    </section>`;
}

function detailField(name) {
  return pinkySheetBody.querySelector(`[data-field="${name}"]`);
}

function sensorRowsHtml(pinky) {
  const details = pinky.sensor_details || [];
  if (!details.length) {
    return `<li class="muted">${pinky.simulated && devMode ? '모의 핑키는 센서 값이 없어요' : '센서 값을 받지 못했어요'}</li>`;
  }
  return details.map((sensor) => {
    const age = sensor.age_s == null ? '안 들어옴' : `${sensor.age_s < 1 ? '방금' : `${sensor.age_s.toFixed(1)}초 전`}`;
    return `<li class="${sensor.fresh ? '' : 'muted'}">
      <span class="sensor-label">${escapeHtml(sensor.label)}</span>
      <span class="sensor-text">${escapeHtml(sensor.text || '값 없음')}</span>
      <span class="sensor-age">${sensor.fresh ? '' : escapeHtml(age)}</span></li>`;
  }).join('');
}

function detailFactsHtml(pinky) {
  const marker = pinky.marker;
  const markerText = !marker ? '마커 정보 없음'
    : marker.registered ? `마커 ${marker.marker_id}, 정차 위치 ${marker.spot_id} 등록됨`
      : `마커 ${marker.marker_id}: ${marker.problem || '정차 위치가 등록되지 않았습니다'}`;
  const rows = [
    ['경로', routeProgressText(pinky.route), ''],
    ['안전 감시', pinky.safety_label || '정보 없음', pinky.safety_label ? '' : 'muted'],
    ['메시지', pinky.message || '없음', pinky.message ? '' : 'muted'],
    ['빠진 센서', pinky.missing.length ? pinky.missing.join(', ') : '없음', pinky.missing.length ? 'bad' : 'good'],
    ['마커', markerText, marker && !marker.registered ? 'bad' : ''],
  ];
  return rows.map(([label, value, tone]) => `<div><dt>${label}</dt><dd${tone ? ` class="${tone}"` : ''}>${escapeHtml(value)}</dd></div>`).join('');
}

// Waypoints come from the map read. Deleting needs the waypoint's id, which only an answer that carries it offers.
function waypointRowsHtml(pinkyId) {
  const data = pinkyMapState.data;
  if (!data) return '<li class="muted">지도를 불러오는 중…</li>';
  const points = data.waypoints?.[pinkyId] || [];
  if (!points.length) return '<li class="muted">기록된 웨이포인트가 없어요</li>';
  return points.map((point) => `
    <li><strong>${escapeHtml(point.label)}</strong><span>x ${point.x_m.toFixed(2)} m, y ${point.y_m.toFixed(2)} m</span>
      ${point.id ? `<button type="button" class="secondary small" data-waypoint-delete="${escapeHtml(point.id)}" data-waypoint-label="${escapeHtml(point.label)}">삭제</button>` : ''}</li>`).join('');
}

function waypointHint() {
  const zones = (pinkyMapState.data?.zones || []).filter((zone) => zone.waypoint);
  const zoneText = zones.map((zone) => `${zone.label}는 경로 마지막 웨이포인트 ${zone.waypoint}의 위치를 써요.`).join(' ');
  return `핑키가 지금 서 있는 곳을 이 이름으로 기록해요. 경로는 1–9 번호를 씁니다. ${zoneText}`;
}

function detailLogHtml(pinkyId) {
  const entries = (pinkyScreen.view?.log || []).filter((entry) => entry.pinky_id === pinkyId).slice(0, 6);
  if (!entries.length) return '<li class="muted">보낸 명령이 없어요</li>';
  return entries.map((entry) => {
    const outcome = commandOutcome(entry);
    return `<li><time>${escapeHtml(formatClock(entry.at))}</time>
      <span>${escapeHtml(pinkyActionLabels[entry.action] || entry.action)}</span>
      <span class="result-chip ${outcome.kind}">${outcome.word}</span>
      <small>${escapeHtml(commandMessage(entry))}</small></li>`;
  }).join('');
}

function renderPinkyDetail() {
  if (pinkySheet.hidden || !pinkyDetail.pinkyId) return;
  const pinky = pinkyById(pinkyDetail.pinkyId);
  if (!pinky) return;
  const id = pinky.pinky_id;
  pinkySheetTitle.innerHTML = `${pinkyBadge(id, { offline: !pinky.connected })} ${escapeHtml(pinky.robot_name || '핑키')}${pinky.simulated ? ' <span class="sim-tag">모의</span>' : ''}`;
  pinkySheet.classList.toggle('offline', !pinky.connected);
  detailField('identity').textContent = pinky.ip || (pinky.simulated && devMode ? '모의 세계' : 'IP 모름');
  detailField('phase').textContent = pinky.connected ? (pinky.phase_label || pinky.phase || '단계 모름') : '단계 알 수 없음';
  setHtmlIfChanged(detailField('chips'), `${readinessChipHtml(pinky)}${markerChipHtml(pinky)}${pinky.stop_pending ? '<span class="ready-chip stopping">정지 적용 중</span>' : ''}`);
  setHtmlIfChanged(detailField('battery'), batteryStatHtml(pinky));
  const stopping = pinkyScreen.sending.has(`${id}:stop`);
  detailField('stop').textContent = stopping ? '정지 요청 보내는 중…' : `${pinky.name} 정지`;
  setHtmlIfChanged(detailField('last-stop'), lastStopHtml(id));
  pinkySheetBody.querySelectorAll('[data-pinky-action]').forEach((button) => {
    button.disabled = pinkyScreen.sending.has(`${id}:${button.dataset.pinkyAction}`);
  });
  setHtmlIfChanged(detailField('sensors'), sensorRowsHtml(pinky));
  setHtmlIfChanged(detailField('facts'), detailFactsHtml(pinky));
  setHtmlIfChanged(detailField('waypoints'), waypointRowsHtml(id));
  detailField('waypoint-hint').textContent = waypointHint();
  setHtmlIfChanged(detailField('log'), detailLogHtml(id));
}

function openPinkyDetail(pinkyId, opener) {
  if (pinkyDetail.pinkyId !== pinkyId || !pinkySheetBody.childElementCount) {
    pinkyDetail.pinkyId = pinkyId;
    pinkySheetBody.innerHTML = detailSkeleton(pinkyId);
  }
  openOverlay(pinkySheet, opener);
  renderPinkyDetail();
  refreshPinky({ force: true });
  refreshPinkyMap({ force: true });
}

// ---- actions ----

const pinkyDetailActions = {
  ready: { path: 'ready', label: '주행준비', body: () => ({ map: detailField('map').value }) },
  'initial-pose': {
    path: 'initial-pose', label: '위치 재설정', body: () => ({}),
    confirm: (name) => `${name} 핑키가 자기 주차장에 서 있나요?\n위치 재설정은 주차장 위치를 지금 위치로 맞춥니다.`,
  },
  'nav-restart': { path: 'nav-restart', label: 'Nav2 복구', body: () => ({}) },
};

pinkySheetBody.addEventListener('click', (event) => {
  const pinkyId = pinkyDetail.pinkyId;
  if (!pinkyId) return;
  const name = pinkyName(pinkyId);
  if (event.target.closest('[data-pinky-stop]')) {
    stopPinky(pinkyId);
    return;
  }
  const remove = event.target.closest('[data-waypoint-delete]');
  if (remove) {
    if (!window.confirm(`${name} 웨이포인트 ${remove.dataset.waypointLabel} 기록을 지울까요?`)) return;
    sendPinkyCommand(pinkyId, 'waypoint_delete', {
      path: `waypoints/${encodeURIComponent(remove.dataset.waypointDelete)}/delete`, body: {}, label: '웨이포인트 삭제',
    });
    return;
  }
  const button = event.target.closest('[data-pinky-action]');
  const action = button && pinkyDetailActions[button.dataset.pinkyAction];
  if (!action) return;
  if (action.confirm && !window.confirm(action.confirm(name))) return;
  sendPinkyCommand(pinkyId, button.dataset.pinkyAction, { path: action.path, body: action.body(), label: action.label });
});

pinkySheetBody.addEventListener('submit', async (event) => {
  event.preventDefault();
  const form = event.target.closest('[data-waypoint-form]');
  const pinkyId = pinkyDetail.pinkyId;
  if (!form || !pinkyId) return;
  const label = form.elements.label.value.trim();
  if (!label) {
    showToast('지점 이름을 넣어 주세요', '예: 1–9 번호나 수령 구역 이름', 'warning');
    form.elements.label.focus();
    return;
  }
  const entry = await sendPinkyCommand(pinkyId, 'waypoints', { path: 'waypoints', body: { label }, label: `웨이포인트 ${label} 기록` });
  if (entry?.ok) form.elements.label.value = '';
});

document.addEventListener('pinkyview', renderPinkyDetail);
document.addEventListener('pinkymap', renderPinkyDetail);
