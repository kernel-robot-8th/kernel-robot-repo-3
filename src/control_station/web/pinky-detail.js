// 핑키 상세: the panel that opens when a Pinky card is tapped. Its live state and every sensor reading, the route
// and safety monitor, its marker, its waypoints, and the operator actions that used to live on the fleet dashboard:
// 정지, 위치 다시 잡기 (the operator puts it on the enlarged map when the automatic place is wrong), 라이다 위치 찾기
// (with the candidate places to pick from when the cell's symmetry leaves two), 주차장 좌표로
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
        <div><strong>위치 다시 잡기</strong><small>지도에 보이는 자리가 실제와 다르면 눌러요. 큰 지도에서 핑키가 있는 곳을 누른 채, 바라보는 방향으로 끌었다가 놓으면 돼요. 라이다는 그 근처(30 cm·30°)에서만 다듬어요.</small></div>
        <button type="button" data-set-pose>↗ 위치 다시 잡기</button>
      </div>
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

    <section class="sheet-group" aria-label="시운전">
      <h3>시운전 (주문 없이)</h3>
      <div class="sheet-action">
        <div><strong>5번 적재 → 배달 → 주차장</strong><small>지금 자리에서 출발해 5번에서 멈춰 적재를 기다리고, 고른 웨이포인트에서 수령을 기다린 뒤 자기 주차 칸으로 돌아와요. 길은 Nav2가 짜고, 교차로·5번·주차장 출입은 한 대씩 들어가요.</small></div>
        <div class="sheet-action-controls">
          <label class="visually-hidden" for="pinkyDropoff">배달 목적지</label>
          <select id="pinkyDropoff" data-field="dropoff"></select>
          <label class="visually-hidden" for="pinkyVia">경유 웨이포인트</label>
          <input id="pinkyVia" data-field="via" placeholder="경유 (예: 9,7)" autocomplete="off" size="9">
          <button type="button" data-pinky-action="test-mission">시험 임무 시작</button>
        </div>
      </div>
      <p class="sheet-hint" data-field="mission-status"></p>
      <div class="sheet-signal" data-field="mission-signal"></div>
      <div class="sheet-action">
        <div><strong>정지 뒤 정리</strong><small>정지하면 받던 경로를 비우고 명령을 기다려요. 1분 동안 명령이 없으면 자기 주차 칸으로 돌아가요(정지를 한 번 더 누르거나, 그사이 옮기거나 위치를 다시 잡으면 안 가요). 초기 세팅은 정지·경로 지우기 뒤 주행준비를 처음부터 다시 해요.</small></div>
        <div class="sheet-action-controls">
          <button type="button" class="secondary" data-pinky-action="return-home">주차장 복귀</button>
          <button type="button" class="secondary" data-pinky-action="route-clear">기존 경로 날리기</button>
          <button type="button" class="secondary" data-pinky-action="reset">초기 세팅</button>
        </div>
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
  if (!loc) return pose.x_m == null ? `${now} ↗ 위치 다시 잡기로 지도에서 직접 넣을 수 있어요.` : now;
  const when = loc.at ? `${formatClock(new Date(loc.at * 1000).toISOString())} ` : '';
  if (loc.method === 'operator') return `${now} ${when}${loc.reason || '직접 지정한 위치예요'}`;
  const result = loc.applied ? '라이다로 위치를 잡았어요. 지도에서 맞는지 보고, 틀리면 ↗ 위치 다시 잡기를 눌러 주세요.'
    : loc.ok ? '라이다가 위치를 찾았지만 아직 넣지 않았어요' : `${loc.reason} ↗ 위치 다시 잡기로 직접 넣어 주세요.`;
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

// ---- 시운전: a test mission without an order, and the operator's signals along it ----

const MISSION_SIGNALS = {
  docking_manual: ['dock_complete', '도킹완료', '5번에 섰어요. 제자리에 붙었으면 눌러 주세요.'],
  docked_manual: ['received', '적재 완료 · 출발', '물건을 실었으면 눌러 주세요. 20 cm 뒤로 빠진 뒤 배달 목적지로 가요.'],
  course_complete: ['mission_complete', '수령 완료 · 복귀', '배달 목적지에 섰어요. 물건을 받았으면 눌러 주세요. 자기 주차 칸으로 돌아가요.'],
};

// Every recorded waypoint of the map (a receiving zone recorded by any Pinky counts), except 5, the loading spot.
function dropoffLabels() {
  const all = Object.values(pinkyMapState.data?.waypoints || {}).flat().map((point) => point.label);
  return [...new Set(all)].filter((label) => label !== '5');
}

// "9,7" → ['9', '7']: waypoints to pass on the way out (and back in reverse), e.g. the passage past the middle wall.
function viaLabels() {
  return detailField('via').value.split(/[,\s→]+/).map((label) => label.trim()).filter(Boolean);
}

function renderDropoffs() {
  const select = detailField('dropoff');
  // 원점: load at 5, back out, and drive back to where it started (no hand-over stop).
  const labels = ['주차장', '원점', ...dropoffLabels()];
  const names = { 주차장: '주차장 (적재 후 바로 복귀)', 원점: '원점 (적재 후 후진·복귀)' };
  const html = labels.map((label) => `<option value="${escapeHtml(label)}">${escapeHtml(names[label] || label)}</option>`).join('');
  if (pinkyScreen.rendered.get(select) === html) return;
  const previous = select.value;
  select.innerHTML = html;
  pinkyScreen.rendered.set(select, html);
  select.value = labels.includes(previous) ? previous : (labels.find((label) => label.includes('수령')) || labels[0] || '');
}

function missionStatusText(pinky) {
  if (!pinky.mission) return '진행 중인 시험 임무가 없어요. 배달 목적지는 수령존에서 [지금 위치 기록]으로 찍은 웨이포인트를 고르면 돼요.';
  const route = pinky.route?.status ? (pinkyRouteLabels[pinky.route.status] || pinky.route.status) : '';
  const left = pinky.mission.auto_return_at ? Math.max(0, Math.round(pinky.mission.auto_return_at - Date.now() / 1000)) : null;
  const wait = left == null ? '' : ` · ${left}초 뒤 주차장으로 자동 복귀 (취소: 정지 한 번 더 또는 기존 경로 날리기)`;
  return `${pinky.mission.message || '시험 임무'} · ${route}${pinky.phase_label ? ` · 지금: ${pinky.phase_label}` : ''}${wait}`;
}

function missionSignalHtml(pinky) {
  const signal = MISSION_SIGNALS[pinky.phase];
  if (!signal) return '';
  const [action, label, hint] = signal;
  return `<div class="sheet-action"><div><strong>${escapeHtml(label)}</strong><small>${escapeHtml(hint)}</small></div>
    <button type="button" data-pinky-signal="${action}">${escapeHtml(label)}</button></div>`;
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
  renderDropoffs();
  detailField('mission-status').textContent = missionStatusText(pinky);
  setHtmlIfChanged(detailField('mission-signal'), missionSignalHtml(pinky));
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
  'test-mission': {
    path: 'test-mission', label: '시험 임무 시작',
    body: () => ({ dropoff: detailField('dropoff').value, via: viaLabels() }),
    confirm: (name) => `${name} 핑키를 출발시킬까요?\n${viaLabels().length ? `${viaLabels().join(' → ')} 경유 → ` : ''}5번 적재 → ${detailField('dropoff').value === '원점' ? '후진 → 출발한 자리로 복귀' : detailField('dropoff').value === '주차장' ? '후진 → 주차장 복귀' : `${detailField('dropoff').value} 배달 → 주차장 복귀`}\n로봇 옆에 지켜보는 사람이 있어야 해요.`,
  },
  'initial-pose': {
    path: 'initial-pose', label: '주차장 좌표로 맞추기', body: () => ({}),
    confirm: (name) => `${name} 핑키가 자기 주차장에 서 있나요?\n주차장 좌표를 지금 위치로 넣습니다.`,
  },
  'nav-restart': { path: 'nav-restart', label: 'Nav2 복구', body: () => ({}) },
  'return-home': {
    path: 'return-home', label: '주차장 복귀', body: () => ({ via: viaLabels().slice().reverse() }),
    confirm: (name) => `${name} 핑키를 지금 자리에서 자기 주차 칸으로 보낼까요?${viaLabels().length ? `\n경유: ${viaLabels().slice().reverse().join(' → ')}` : ''}`,
  },
  'route-clear': {
    path: 'route-clear', label: '기존 경로 날리기', body: () => ({}),
    confirm: (name) => `${name} 핑키의 경로·주행선·자동 복귀를 지울까요?\n주행 중이면 먼저 세워요.`,
  },
  reset: {
    path: 'reset', label: '초기 세팅', body: () => ({ map: detailField('map')?.value || 'map-proto.yaml' }),
    confirm: (name) => `${name} 핑키를 초기 세팅할까요?\n정지 잠금 → 경로 지우기 → 주행준비를 처음부터 다시 해요(1~2분). 끝나면 라이다로 위치를 찾아요.`,
  },
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
  const signal = event.target.closest('[data-pinky-signal]');
  if (signal) {
    const action = signal.dataset.pinkySignal;
    const label = Object.values(MISSION_SIGNALS).find(([a]) => a === action)?.[1] || action;
    sendPinkyCommand(pinkyId, action, { path: 'signal', body: { action }, label });
    return;
  }
  const setPose = event.target.closest('[data-set-pose]');
  if (setPose) {
    startMapPick(pinkyId, setPose);
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
