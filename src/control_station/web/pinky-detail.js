// 핑키 상세: the panel that opens when a Pinky card is tapped. Its live state and every sensor reading, the route
// and safety monitor, its marker, its waypoints, and the operator actions that used to live on the fleet dashboard:
// 정지, 라이다 위치 찾기 (with the candidate places to pick from when the cell's symmetry leaves two), 주차장 좌표로
// 맞추기, 주행준비 (with the map to load), Nav2 복구, 웨이포인트 기록 and 삭제.
//
// Loaded after pinky.js (pinkyScreen, pinkySheet, pinkyById, pinkyName, sendPinkyCommand, stopPinky, setHtmlIfChanged,
// batteryStatHtml, readinessChipHtml, markerChipHtml, lastStopHtml, commandOutcome, pinkyRouteLabels,
// pinkyActionLabels) and pinky-map.js (pinkyMapState).
//
// The panel is built once per opened Pinky and then updated in place on every read, so a typed waypoint name or the
// chosen map is never lost to a poll.

const pinkySheetTitle = document.querySelector('#pinkySheetTitle');
const pinkySheetBody = document.querySelector('#pinkySheetBody');
const pinkyDetail = { pinkyId: null, preview: null, previewing: false };  // preview: GET locate answer for this Pinky

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

    <section class="sheet-group" aria-label="지도 위치">
      <h3>지도 위치</h3>
      <p class="sheet-hint" data-field="loc-summary"></p>
      <div class="sheet-action">
        <div><strong>라이다로 위치 찾기</strong><small>라이다로 지도에서 자기 자리를 찾아 넣어요. 비밀번호는 필요 없고 Nav2만 떠 있으면 돼요. 경기장이 대칭이라 헷갈리면 후보를 보여 드려요.</small></div>
        <div class="sheet-action-controls">
          <button type="button" class="secondary" data-locate-preview>후보 보기</button>
          <button type="button" data-pinky-action="localize">위치 찾기</button>
        </div>
      </div>
      <ul class="sheet-candidates" data-field="candidates"></ul>
      <div class="sheet-action" data-dev-only>
        <div><strong>주차장 좌표로 맞추기</strong><small>라이다로 못 찾을 때 예비로 써요. 핑키가 자기 주차 칸에 서 있다고 보고 그 좌표를 넣어요.</small></div>
        <button type="button" class="secondary" data-pinky-action="initial-pose">주차장 좌표로</button>
      </div>
    </section>

    <section class="sheet-group" aria-label="주행 준비">
      <h3>주행 준비</h3>
      <div class="sheet-action">
        <div><strong>주행준비</strong><small>고른 지도로 Nav2를 띄우고, 끝나면 라이다로 자기 위치를 찾아요. 못 찾으면 위 후보에서 골라 주세요.</small></div>
        <div class="sheet-action-controls">
          <label class="visually-hidden" for="pinkyMapChoice">불러올 지도</label>
          <select id="pinkyMapChoice" data-field="map" data-dev-only>${pinkyMaps.map(([value, label]) => `<option value="${value}">${label}</option>`).join('')}</select>
          <button type="button" data-pinky-action="ready">주행준비</button>
        </div>
      </div>
      <div class="sheet-action" data-dev-only>
        <div><strong>Nav2 복구</strong><small>Nav2를 다시 띄우도록 요청해요.</small></div>
        <button type="button" class="secondary" data-pinky-action="nav-restart">Nav2 복구</button>
      </div>
    </section>

    <section class="sheet-group" aria-label="웨이포인트" data-dev-only>
      <h3>웨이포인트</h3>
      <form class="sheet-waypoint-form" data-waypoint-form novalidate>
        <label class="visually-hidden" for="pinkyWaypointLabel">기록할 지점 이름</label>
        <input id="pinkyWaypointLabel" name="label" list="waypointLabelOptions" maxlength="48" placeholder="지점 이름 (예: 5)" autocomplete="off">
        <button type="submit" data-pinky-action="waypoints">지금 위치 기록</button>
      </form>
      <p class="sheet-hint" data-field="waypoint-hint"></p>
      <ul class="sheet-waypoints" data-field="waypoints"></ul>
    </section>

    <section class="sheet-group" aria-label="센서" data-dev-only>
      <h3>센서</h3>
      <ul class="sensor-list" data-field="sensors"></ul>
    </section>

    <section class="sheet-group" aria-label="경로와 안전">
      <h3>경로와 안전</h3>
      <dl class="pinky-facts" data-field="facts"></dl>
    </section>

    <section class="sheet-group" aria-label="이 핑키에 보낸 명령" data-dev-only>
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

// ---- LiDAR localization: the current map pose, the last result and the places to pick from ----

const FACING = ['→ 오른쪽', '↗', '↑ 위쪽', '↖', '← 왼쪽', '↙', '↓ 아래쪽', '↘'];

function facingText(yaw) {
  const step = Math.round(((yaw % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI) / (Math.PI / 4)) % 8;
  return `${FACING[step]}을 봄`;
}

// The nearest named place on the map (parking slots, station 5, junction 7) to say where a pose is.
function placeText(x, y) {
  const data = pinkyMapState.data || {};
  const named = [
    ...(data.parking || []).map((spot) => ({ label: `${(spot.label || '').replace(' 주차장', '')} 칸`, x: spot.x_m, y: spot.y_m })),
    ...(data.landmarks || []).map((mark) => ({ label: mark.label, x: mark.x_m, y: mark.y_m })),
  ].filter((spot) => spot.x != null && spot.y != null);
  if (!named.length) return `x ${x.toFixed(2)}, y ${y.toFixed(2)} m`;
  const near = named.map((spot) => ({ ...spot, d: Math.hypot(spot.x - x, spot.y - y) })).sort((a, b) => a.d - b.d)[0];
  return `${near.label} ${near.d < 0.15 ? '자리' : `에서 ${Math.round(near.d * 100)} cm`} (x ${x.toFixed(2)}, y ${y.toFixed(2)})`;
}

function locSummaryText(pinky) {
  const pose = pinky.pose || {};
  const now = pose.x_m == null ? '지금 지도 위치가 없어요.' : `지금 위치: ${placeText(pose.x_m, pose.y_m)}, ${facingText(pose.yaw_rad)}.`;
  const loc = pinky.localization;
  if (!loc) return now;
  const when = loc.at ? `${formatClock(new Date(loc.at * 1000).toISOString())} ` : '';
  const result = loc.applied ? '라이다로 위치를 넣었어요' : loc.ok ? '라이다가 위치를 찾았지만 아직 넣지 않았어요' : loc.reason;
  return `${now} ${when}라이다: ${result}`;
}

function candidatesFor(pinky) {
  if (pinkyDetail.preview?.pinkyId === pinky.pinky_id) return pinkyDetail.preview.candidates;
  const loc = pinky.localization;
  return loc && !loc.applied && !loc.ok ? loc.candidates || [] : [];
}

function candidateRowsHtml(pinky) {
  if (pinkyDetail.previewing) return '<li class="muted">라이다 후보를 계산하는 중…</li>';
  if (pinkyDetail.preview?.pinkyId === pinky.pinky_id && pinkyDetail.preview.error) {
    return `<li class="muted">${escapeHtml(pinkyDetail.preview.error)}</li>`;
  }
  const candidates = candidatesFor(pinky);
  if (!candidates.length) return '';
  return candidates.map((c, i) => `
    <li><strong>후보 ${i + 1}</strong><span>${escapeHtml(placeText(c.x_m, c.y_m))}, ${escapeHtml(facingText(c.yaw_rad))} · 일치도 ${Number(c.score).toFixed(2)}</span>
      <button type="button" class="small" data-pick-candidate="${i}">이 자리</button></li>`).join('');
}

async function previewCandidates(pinkyId) {
  pinkyDetail.previewing = true;
  renderPinkyDetail();
  let preview;
  try {
    const response = await fetch(`/api/pinky/${encodeURIComponent(pinkyId)}/locate`, { cache: 'no-store' });
    const result = await response.json();
    preview = response.ok ? { pinkyId, candidates: result.match.candidates || [] }
      : { pinkyId, candidates: [], error: apiErrorMessage(result, '라이다 후보를 받지 못했어요') };
  } catch {
    preview = { pinkyId, candidates: [], error: '관제 서버의 답을 받지 못했어요' };
  }
  pinkyDetail.preview = preview;
  pinkyDetail.previewing = false;
  renderPinkyDetail();
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
  detailField('loc-summary').textContent = locSummaryText(pinky);
  setHtmlIfChanged(detailField('candidates'), candidateRowsHtml(pinky));
  setHtmlIfChanged(detailField('sensors'), sensorRowsHtml(pinky));
  setHtmlIfChanged(detailField('facts'), detailFactsHtml(pinky));
  setHtmlIfChanged(detailField('waypoints'), waypointRowsHtml(id));
  detailField('waypoint-hint').textContent = waypointHint();
  setHtmlIfChanged(detailField('log'), detailLogHtml(id));
}

function openPinkyDetail(pinkyId, opener) {
  if (pinkyDetail.pinkyId !== pinkyId || !pinkySheetBody.childElementCount) {
    pinkyDetail.pinkyId = pinkyId;
    pinkyDetail.preview = null;
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
  localize: { path: 'localize', label: '라이다 위치 찾기', body: () => ({}) },
  'initial-pose': {
    path: 'initial-pose', label: '주차장 좌표로 맞추기', body: () => ({}),
    confirm: (name) => `${name} 핑키가 자기 주차장에 서 있나요?\n주차장 좌표를 지금 위치로 넣습니다.`,
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
  if (event.target.closest('[data-locate-preview]')) {
    previewCandidates(pinkyId);
    return;
  }
  const pick = event.target.closest('[data-pick-candidate]');
  if (pick) {
    const candidate = candidatesFor(pinkyById(pinkyId))[Number(pick.dataset.pickCandidate)];
    if (!candidate) return;
    const where = `${placeText(candidate.x_m, candidate.y_m)}, ${facingText(candidate.yaw_rad)}`;
    if (!window.confirm(`${name} 핑키가 지금 여기에 있나요?\n${where}\n이 자리를 AMCL 시작 위치로 넣고, 그다음부터는 라이다로 따라가요.`)) return;
    pinkyDetail.preview = null;
    sendPinkyCommand(pinkyId, 'localize', {
      path: 'localize', label: '고른 자리로 위치 넣기',
      body: { pose: { x_m: candidate.x_m, y_m: candidate.y_m, yaw_rad: candidate.yaw_rad } },
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
