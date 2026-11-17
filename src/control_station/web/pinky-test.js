// 전체 주행 테스트 (핑키 화면): every Pinky through the whole course without an order.
//   자기 주차 칸 → (9번 통로, 한 대씩) → 대기 자리 4 → 적재 자리 5 (정면 진입) → [도킹완료] → [적재 완료]
//   → 20 cm 후진 → 수령 구역 → [수령 완료] → 자기 주차 칸
// [전체 출발] starts the Pinkys in FLEET_TEST_ORDER; the fleet dashboard's traffic zones make them wait in turn (the
// passage one at a time, one at the queue spot, station 5 in the order they asked). Each row shows where its Pinky
// is and the one button its phase needs. The chip shows the ceiling camera's judgment of the loading spot.
//
// Loaded after pinky-detail.js (sendPinkyCommand, pinkyBadge, pinkyScreen, setHtmlIfChanged, dropoffLabels).

const FLEET_TEST_ORDER = ['12', '10', '11'];  // 12 carries the registered ArUco marker (21), so it loads first
const FLEET_TEST_SPOT = 'pinky_dock_1';       // the ceiling camera's registered loading spot
const FLEET_TEST_SIGNALS = {
  docking_manual: [['dock_complete', '도킹완료'], ['redock', '다시 맞추기', 'secondary']],
  docked_manual: [['received', '적재 완료 · 출발']],
  course_complete: [['mission_complete', '수령 완료 · 주차장으로']],
};
const FLEET_TEST_ACTIVE = new Set(['starting', 'running', 'cancel_requested']);
const PARKING_WORDS = { IN_POSITION: '도착 (정상 위치)', OFF_POSITION: '어긋남', NOT_IN_VIEW: '핑키 안 보임', VIEW_UNVERIFIED: '카메라 시야 확인 안 됨', MOVING: '움직이는 중' };

const fleetTestRows = document.querySelector('#fleetTestRows');
const fleetTestCamera = document.querySelector('#fleetTestCamera');
const fleetTestAuto = document.querySelector('#fleetTestAuto');
// okStreak: camera polls in a row that said IN_POSITION; told / sent: per Pinky, already announced or signalled for
// this stop at the loading spot (cleared when it leaves the docking_manual phase).
const fleetTest = { zone: {}, busy: false, camera: null, okStreak: 0, told: {}, sent: {} };

// The ceiling camera's verdict for a Pinky standing at the loading spot: [tone, text], or null when it is not there.
function fleetTestVerdict(pinky) {
  if (pinky.phase !== 'docking_manual' || !FLEET_TEST_ACTIVE.has(pinky.route?.status)) return null;
  const live = fleetTest.camera;
  const offset = live?.offset;
  const numbers = offset?.distance_mm != null
    ? ` · ${(offset.distance_mm / 10).toFixed(1)} cm / ${Math.abs(offset.dyaw_deg ?? 0).toFixed(1)}°` : '';
  if (live?.state === 'IN_POSITION') return ['good', `✓ 도착 위치 OK${numbers} · 도킹완료를 눌러도 돼요`];
  if (live?.state === 'OFF_POSITION') return ['attention', `위치 어긋남${numbers} · 다시 맞추기를 눌러 주세요`];
  if (live?.state === 'NOT_IN_VIEW') return ['', '카메라에 마커가 안 보여요 · 눈으로 확인하고 도킹완료'];
  return ['', `카메라 판정: ${PARKING_WORDS[live?.state] || live?.state || '확인 중'}`];
}

function fleetTestZones() {
  return dropoffLabels().filter((label) => label.includes('수령'));
}

function fleetTestPinkies() {
  const all = pinkyScreen.view?.pinkies || [];
  return [...all].sort((a, b) => FLEET_TEST_ORDER.indexOf(a.pinky_id) - FLEET_TEST_ORDER.indexOf(b.pinky_id));
}

function fleetTestZoneFor(pinkyId, index, zones) {
  if (!zones.includes(fleetTest.zone[pinkyId])) fleetTest.zone[pinkyId] = zones[index % Math.max(1, zones.length)];
  return fleetTest.zone[pinkyId];
}

// Battery as the fleet dashboard reports it: [text, tone]. Under 20 % the Pinky should be charged before it drives
// (weak motors and an unstable Pi, 2026-11-17: P0 at 14 % lost its base and kept failing to bring Nav2 up).
function fleetTestBattery(pinky) {
  const percent = pinky.battery_percent;
  if (!pinky.connected || percent == null) return ['배터리 모름', ''];
  const volts = pinky.battery_volts == null ? '' : ` · ${Number(pinky.battery_volts).toFixed(2)} V`;
  return [`🔋 ${Math.round(percent)}%${volts}`, percent < 20 ? 'danger' : percent < 30 ? 'attention' : 'good'];
}

function fleetTestRowHtml(pinky, index, zones) {
  const id = escapeHtml(pinky.pinky_id);
  const active = FLEET_TEST_ACTIVE.has(pinky.route?.status);
  const zone = fleetTestZoneFor(pinky.pinky_id, index, zones);
  const where = !pinky.connected ? ((pinky.holding || []).length ? `연결 끊김 · ${pinky.holding.join(', ')}을(를) 잡은 채 꺼졌어요` : '연결 끊김')
    : pinky.waiting ? pinky.waiting.text
    : active ? (pinky.mission?.message || pinky.phase_label || '주행 중') : (pinky.readiness_label || pinky.phase_label || '');
  const signals = active ? (FLEET_TEST_SIGNALS[pinky.phase] || []) : [];
  const buttons = signals.map(([action, label, tone]) =>
    `<button type="button" class="${tone || ''}" data-fleet-signal="${action}" data-pinky="${id}">${label}</button>`).join('')
    || (!active && pinky.connected && pinky.ready
      ? `<button type="button" class="secondary" data-fleet-start="${id}">출발</button>` : '')
    // A Pinky that went off while holding a zone (the passage, the loading spot): the others wait for it for ever.
    || (!pinky.connected && (pinky.holding || []).length
      ? `<button type="button" data-fleet-release="${id}">치웠어요 · 자리 비우기</button>` : '');
  const select = `<select data-fleet-zone="${id}" aria-label="${escapeHtml(pinky.name)} 수령 구역" ${active ? 'disabled' : ''}>${
    zones.map((label) => `<option value="${escapeHtml(label)}" ${label === zone ? 'selected' : ''}>${escapeHtml(label)}</option>`).join('')
    || '<option value="">수령 구역 없음</option>'}</select>`;
  return `<li class="${signals.length ? 'needs-you' : ''} ${fleetTestVerdict(pinky)?.[0] === 'good' ? 'arrived' : ''} ${pinky.waiting ? 'waiting' : ''}">
    <span class="fleet-test-order">${index + 1}</span>${pinkyBadge(pinky.pinky_id, { offline: !pinky.connected })}
    <span class="fleet-test-where"><strong>${pinky.waiting ? '⏳ ' : ''}${escapeHtml(pinky.phase_label && active ? pinky.phase_label : (active ? '주행 중' : '대기'))}</strong>
      <small>${escapeHtml(where)}</small></span>
    <span class="chip fleet-test-battery ${fleetTestBattery(pinky)[1]}">${escapeHtml(fleetTestBattery(pinky)[0])}</span>
    ${select}<span class="fleet-test-buttons">${buttons}</span>
    ${fleetTestVerdict(pinky) ? `<span class="fleet-test-verdict chip ${fleetTestVerdict(pinky)[0]}">${escapeHtml(fleetTestVerdict(pinky)[1])}</span>` : ''}</li>`;
}

function renderFleetTest() {
  if (activeView !== 'pinky' || !fleetTestRows) return;
  const zones = fleetTestZones();
  const pinkies = fleetTestPinkies();
  setHtmlIfChanged(fleetTestRows, pinkies.map((pinky, index) => fleetTestRowHtml(pinky, index, zones)).join('')
    || '<li class="muted">핑키 상태를 불러오는 중…</li>');
  const startable = pinkies.some((pinky) => pinky.connected && pinky.ready && !FLEET_TEST_ACTIVE.has(pinky.route?.status));
  document.querySelector('#fleetTestStart').disabled = fleetTest.busy || !startable || !zones.length;
  document.querySelector('#fleetTestHome').disabled = fleetTest.busy || !startable;
  const live = fleetTest.camera;
  const offset = live?.offset;
  fleetTestCamera.textContent = !live ? '적재 자리 카메라: 확인 중'
    : `적재 자리 카메라: ${PARKING_WORDS[live.state] || live.state}${offset?.distance_mm != null
      ? ` · ${(offset.distance_mm / 10).toFixed(1)} cm / ${Math.abs(offset.dyaw_deg ?? 0).toFixed(1)}°` : ''}`;
  fleetTestCamera.className = `chip ${live?.state === 'IN_POSITION' ? 'good' : live?.state === 'OFF_POSITION' ? 'attention' : ''}`;
}

function fleetTestStartOne(pinky) {
  return sendPinkyCommand(pinky.pinky_id, 'test-mission', {
    path: 'test-mission', label: '전체 주행 테스트 출발', body: { dropoff: fleetTest.zone[pinky.pinky_id], via: [] },
  });
}

async function fleetTestEach(run) {
  fleetTest.busy = true;
  renderFleetTest();
  try {
    for (const pinky of fleetTestPinkies()) {
      if (!pinky.connected || !pinky.ready || FLEET_TEST_ACTIVE.has(pinky.route?.status)) continue;
      await run(pinky);
      await new Promise((resolve) => window.setTimeout(resolve, 1500));  // in order, so the first asks for the zones first
    }
  } finally {
    fleetTest.busy = false;
    renderFleetTest();
  }
}

document.querySelector('#fleetTestStart').addEventListener('click', () => {
  const names = fleetTestPinkies().filter((pinky) => pinky.connected && pinky.ready && !FLEET_TEST_ACTIVE.has(pinky.route?.status))
    .map((pinky) => `${pinky.name} → ${fleetTest.zone[pinky.pinky_id]}`);
  if (!window.confirm(`핑키를 차례로 출발시킬까요?\n${names.join('\n')}\n적재 자리 → 수령 구역 → 자기 주차 칸. 로봇 옆에 지켜보는 사람이 있어야 해요.`)) return;
  fleetTestEach(fleetTestStartOne);
});
document.querySelector('#fleetTestHome').addEventListener('click', () => {
  if (!window.confirm('서 있는 핑키를 모두 자기 주차 칸으로 보낼까요? 적재 자리에 있는 핑키는 먼저 20 cm 후진해요.')) return;
  fleetTestEach((pinky) => sendPinkyCommand(pinky.pinky_id, 'return_home', { path: 'return-home', label: '주차장 복귀', body: { via: [] } }));
});
document.querySelector('#fleetTestBattery').addEventListener('click', async () => {
  await refreshPinky({ force: true });  // read the Pinkys again now, then say it in one line
  const pinkies = fleetTestPinkies();
  const low = pinkies.filter((pinky) => pinky.connected && pinky.battery_percent != null && pinky.battery_percent < 20);
  showToast('핑키 배터리', pinkies.map((pinky) => `${pinky.name} ${fleetTestBattery(pinky)[0].replace('🔋 ', '')}`).join('  ·  ')
    + (low.length ? `  — ${low.map((pinky) => pinky.name).join(', ')} 충전이 필요해요` : ''), low.length ? 'error' : 'info', 10000);
  renderFleetTest();
});
fleetTestRows.addEventListener('change', (event) => {
  const select = event.target.closest('[data-fleet-zone]');
  if (select) fleetTest.zone[select.dataset.fleetZone] = select.value;
});
fleetTestRows.addEventListener('click', (event) => {
  const signal = event.target.closest('[data-fleet-signal]');
  if (signal) {
    const action = signal.dataset.fleetSignal;
    sendPinkyCommand(signal.dataset.pinky, action, { path: 'signal', body: { action }, label: signal.textContent });
    return;
  }
  const release = event.target.closest('[data-fleet-release]');
  if (release) {
    const id = release.dataset.fleetRelease;
    if (window.confirm(`${pinkyName(id)} 핑키를 그 자리에서 치우셨나요?\n잡고 있던 자리를 비워 다른 핑키가 지나가게 해요.`)) {
      sendPinkyCommand(id, 'clear_route', { path: 'route-clear', body: {}, label: '자리 비우기' });
    }
    return;
  }
  const start = event.target.closest('[data-fleet-start]');
  const pinky = start && pinkyById(start.dataset.fleetStart);
  if (pinky && window.confirm(`${pinky.name} 핑키를 출발시킬까요?\n적재 자리 → ${fleetTest.zone[pinky.pinky_id]} → 자기 주차 칸`)) fleetTestStartOne(pinky);
});

async function pollFleetTestCamera() {
  if (activeView !== 'pinky' || document.hidden) return;
  try {
    const response = await fetch(`/api/parking-spots/${FLEET_TEST_SPOT}/live`, { cache: 'no-store', signal: AbortSignal.timeout(4000) });
    const result = await response.json();
    fleetTest.camera = response.ok ? result.live : { state: result.error_code || '판정 노드 없음' };
  } catch {
    fleetTest.camera = { state: '관제 응답 없음' };
  }
  fleetTest.okStreak = fleetTest.camera?.state === 'IN_POSITION' ? fleetTest.okStreak + 1 : 0;
  pinkyMapState.dockState = fleetTest.camera?.state || null;  // the map colours the loading spot with it
  for (const pinky of pinkyScreen.view?.pinkies || []) {
    const id = pinky.pinky_id;
    if (!fleetTestVerdict(pinky)) {
      delete fleetTest.told[id];
      delete fleetTest.sent[id];
      continue;
    }
    if (fleetTest.okStreak < 2) continue;  // two polls in a row, so one lucky frame is not an arrival
    if (!fleetTest.told[id]) {
      fleetTest.told[id] = true;
      showToast(`${pinky.name} 적재 자리 도착 OK`, fleetTestAuto.checked ? '도킹완료를 자동으로 보내요' : '도킹완료를 눌러 주세요', 'info', 8000);
    }
    if (fleetTestAuto.checked && !fleetTest.sent[id]) {
      fleetTest.sent[id] = true;
      sendPinkyCommand(id, 'dock_complete', { path: 'signal', body: { action: 'dock_complete' }, label: '도킹완료 (카메라 OK, 자동)' });
    }
  }
  renderFleetTest();
}

document.addEventListener('pinkyview', renderFleetTest);
document.addEventListener('pinkymap', renderFleetTest);
document.addEventListener('viewchange', () => { renderFleetTest(); pollFleetTestCamera(); });
window.setInterval(pollFleetTestCamera, 1500);
