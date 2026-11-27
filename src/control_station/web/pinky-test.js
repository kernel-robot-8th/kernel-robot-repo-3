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
  docked_manual: [['received', '적재 완료 · 출발'], ['redock', '다시 들어오기', 'secondary']],
  course_complete: [['mission_complete', '수령 완료 · 주차장으로']],
};
const FLEET_TEST_ACTIVE = new Set(['starting', 'running', 'cancel_requested']);
const PARKING_WORDS = { IN_POSITION: '도착 (정상 위치)', OFF_POSITION: '어긋남', NOT_IN_VIEW: '핑키 안 보임', VIEW_UNVERIFIED: '카메라 시야 확인 안 됨', MOVING: '움직이는 중' };

const fleetTestRows = document.querySelector('#fleetTestRows');
const fleetTestCamera = document.querySelector('#fleetTestCamera');
const fleetTestAuto = document.querySelector('#fleetTestAuto');
const fleetTestRepeat = document.querySelector('#fleetTestRepeat');
// dock: per Pinky, the server's last answer for its stop at the loading spot (POST /api/pinky/{id}/dock-handshake:
// the camera's judgment at that Pinky's own registration, what to do about it, what was sent, the stop's log);
// told: the log lines already shown as a toast. Both are cleared when the Pinky leaves the loading spot.
// picked: which Pinkys the two buttons on top act on (all by default); one Pinky alone can be run this way.
const fleetTest = { zone: {}, busy: false, camera: null, dock: {}, told: {}, picked: {} };
const fleetTestPicked = (id) => fleetTest.picked[id] !== false;
const FLEET_TEST_DOCK_PHASES = new Set(['docking_manual', 'redocking']);
const FLEET_TEST_DOCK_TONE = { COMPLETE: 'good', OPERATOR: 'attention', REDOCK: 'attention' };

// What the camera <-> Pinky exchange says for a Pinky at the loading spot: [tone, text], or null when it is not there.
// The deciding is the server's (control_station/dock_handshake.py), the same as an order's: seen in place twice ->
// 도킹완료; steadily off -> 다시 맞추기 with the camera's offset, then wait until it is back; no registration, a blind
// camera or three redocks used -> a person looks. With 자동 off the server only judges and the buttons send.
function fleetTestVerdict(pinky) {
  // Docked: the station's loading chain (arm -> YOLO tray count -> received), when 자동 처리 is on (load_relay.py).
  const loading = pinkyScreen.view?.loading?.[pinky.pinky_id];
  if (pinky.phase === 'docked_manual' && loading?.active) {
    return [loading.tone || '', `적재 · ${loading.message || ''}`];
  }
  if (!FLEET_TEST_DOCK_PHASES.has(pinky.phase) || !FLEET_TEST_ACTIVE.has(pinky.route?.status)) return null;
  const dock = fleetTest.dock[pinky.pinky_id];
  if (!dock) return ['', '카메라 판정을 확인하는 중'];
  if (dock.error) return ['', dock.error];
  if (!dock.active) return ['', dock.message || '카메라 판정을 확인하는 중'];
  const decision = dock.decision || {};
  const offset = decision.offset;
  const numbers = offset?.distance_mm != null
    ? ` · ${(offset.distance_mm / 10).toFixed(1)} cm / ${Math.abs(offset.dyaw_deg ?? 0).toFixed(1)}°` : '';
  const settled = decision.reason === 'SETTLING' || decision.reason === 'COMPLETE_SENT';
  const hint = dock.auto || dock.done ? ''
    : decision.action === 'COMPLETE' ? ' · 도킹완료를 눌러 주세요'
    : decision.action === 'REDOCK' ? ' · 다시 맞추기를 눌러 주세요' : '';
  const last = (dock.log || []).filter((entry) => entry.event === 'REDOCK_RESULT').at(-1);
  return [settled ? 'good' : FLEET_TEST_DOCK_TONE[decision.action] || '',
    `${decision.action === 'COMPLETE' || settled ? '✓ ' : ''}${decision.message || ''}${numbers}${hint}${last ? ` · ${last.message}` : ''}${dock.repeat ? ` · 반복 테스트 성공 ${dock.repeat.successes}/${dock.repeat.target} (들어온 횟수 ${dock.repeat.trials})` : ''}`];
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
      ? `<button type="button" data-fleet-start="${id}">이 핑키만 출발</button><button type="button" class="secondary" data-fleet-home="${id}">주차장으로</button>` : '')
    // A Pinky that went off while holding a zone (the passage, the loading spot): the others wait for it for ever.
    || (!pinky.connected && (pinky.holding || []).length
      ? `<button type="button" data-fleet-release="${id}">치웠어요 · 자리 비우기</button>` : '');
  const select = `<select data-fleet-zone="${id}" aria-label="${escapeHtml(pinky.name)} 수령 구역" ${active ? 'disabled' : ''}>${
    zones.map((label) => `<option value="${escapeHtml(label)}" ${label === zone ? 'selected' : ''}>${escapeHtml(label)}</option>`).join('')
    || '<option value="">수령 구역 없음</option>'}</select>`;
  return `<li class="${signals.length ? 'needs-you' : ''} ${fleetTestVerdict(pinky)?.[0] === 'good' ? 'arrived' : ''} ${pinky.waiting ? 'waiting' : ''}">
    <input type="checkbox" class="fleet-test-pick" data-fleet-pick="${id}" aria-label="${escapeHtml(pinky.name)} 포함" title="위의 출발·복귀 버튼에 이 핑키를 포함" ${fleetTestPicked(pinky.pinky_id) ? 'checked' : ''}>
    <span class="fleet-test-order">${index + 1}</span>${pinkyBadge(pinky.pinky_id, { offline: !pinky.connected })}
    <span class="fleet-test-where"><strong>${pinky.waiting ? '⏳ ' : ''}${escapeHtml(pinky.phase_label && active ? pinky.phase_label : (active ? '주행 중' : '대기'))}</strong>
      <small>${escapeHtml(where)}</small></span>
    <span class="chip fleet-test-battery ${fleetTestBattery(pinky)[1]}">${escapeHtml(fleetTestBattery(pinky)[0])}</span>
    ${select}<span class="fleet-test-buttons">${buttons}</span>
    ${fleetTestVerdict(pinky) ? `<span class="fleet-test-verdict chip ${fleetTestVerdict(pinky)[0]}">${escapeHtml(fleetTestVerdict(pinky)[1])}</span>` : ''}</li>`;
}

// The box is the control station's own switch (dock_auto in GET /api/pinky): the station steps the camera <-> Pinky
// exchange itself, so it goes on with this screen closed. Ticking it here changes the station's setting.
let fleetTestAutoBusy = false;
fleetTestAuto.addEventListener('change', async () => {
  const auto = fleetTestAuto.checked;
  fleetTestAutoBusy = true;
  try {
    const response = await fetch('/api/pinky/dock-auto', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ auto }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, '자동 처리 설정을 바꾸지 못했습니다'));
    showToast(auto ? '적재 자리 도착 자동 처리를 켰어요' : '적재 자리 도착 자동 처리를 껐어요',
      auto ? '카메라가 맞다고 보면 도킹완료, 어긋나면 다시 맞추기를 알아서 보내요' : '카메라는 판정만 하고, 도킹완료·다시 맞추기는 직접 눌러요');
  } catch (error) {
    fleetTestAuto.checked = !auto;
    showToast('자동 처리 설정을 바꾸지 못했어요', error.message, 'error');
  } finally {
    fleetTestAutoBusy = false;
  }
});

function renderFleetTest() {
  if (!fleetTestAutoBusy && typeof pinkyScreen.view?.dock_auto === 'boolean') fleetTestAuto.checked = pinkyScreen.view.dock_auto;
  if (activeView !== 'pinky' || !fleetTestRows) return;
  const zones = fleetTestZones();
  const pinkies = fleetTestPinkies();
  setHtmlIfChanged(fleetTestRows, pinkies.map((pinky, index) => fleetTestRowHtml(pinky, index, zones)).join('')
    || '<li class="muted">핑키 상태를 불러오는 중…</li>');
  const ready = pinkies.filter((pinky) => fleetTestPicked(pinky.pinky_id) && pinky.connected && pinky.ready && !FLEET_TEST_ACTIVE.has(pinky.route?.status));
  const startable = ready.length > 0;
  const everyone = pinkies.every((pinky) => fleetTestPicked(pinky.pinky_id));
  const who = everyone ? '전체' : ready.length === 1 ? `${ready[0].name}만` : `고른 ${ready.length}대`;
  document.querySelector('#fleetTestStart').textContent = `${who} 출발`;
  document.querySelector('#fleetTestHome').textContent = `${who} 주차장 복귀`;
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
    path: 'test-mission', label: '핑키 상태 확인 출발', body: { dropoff: fleetTest.zone[pinky.pinky_id], via: [] },
  });
}

async function fleetTestEach(run) {
  fleetTest.busy = true;
  renderFleetTest();
  try {
    for (const pinky of fleetTestPinkies()) {
      if (!fleetTestPicked(pinky.pinky_id) || !pinky.connected || !pinky.ready || FLEET_TEST_ACTIVE.has(pinky.route?.status)) continue;
      await run(pinky);
      await new Promise((resolve) => window.setTimeout(resolve, 1500));  // in order, so the first asks for the zones first
    }
  } finally {
    fleetTest.busy = false;
    renderFleetTest();
  }
}

document.querySelector('#fleetTestStart').addEventListener('click', () => {
  const names = fleetTestPinkies().filter((pinky) => fleetTestPicked(pinky.pinky_id) && pinky.connected && pinky.ready && !FLEET_TEST_ACTIVE.has(pinky.route?.status))
    .map((pinky) => `${pinky.name} → ${fleetTest.zone[pinky.pinky_id]}`);
  if (!window.confirm(`핑키를 차례로 출발시킬까요?\n${names.join('\n')}\n적재 자리 → 수령 구역 → 자기 주차 칸. 로봇 옆에 지켜보는 사람이 있어야 해요.`)) return;
  fleetTestEach(fleetTestStartOne);
});
document.querySelector('#fleetTestHome').addEventListener('click', () => {
  const names = fleetTestPinkies().filter((pinky) => fleetTestPicked(pinky.pinky_id) && pinky.connected && pinky.ready && !FLEET_TEST_ACTIVE.has(pinky.route?.status)).map((pinky) => pinky.name);
  if (!window.confirm(`${names.join(', ')} 핑키를 자기 주차 칸으로 보낼까요? 적재 자리에 있는 핑키는 먼저 20 cm 후진해요.`)) return;
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
  const pick = event.target.closest('[data-fleet-pick]');
  if (pick) {
    fleetTest.picked[pick.dataset.fleetPick] = pick.checked;
    renderFleetTest();
  }
});
fleetTestRows.addEventListener('click', (event) => {
  const signal = event.target.closest('[data-fleet-signal]');
  if (signal) {
    const action = signal.dataset.fleetSignal;
    if (action === 'redock') {
      fleetTestRedock(signal.dataset.pinky, signal.textContent);
      return;
    }
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
  const home = event.target.closest('[data-fleet-home]');
  const homing = home && pinkyById(home.dataset.fleetHome);
  if (homing) {
    if (window.confirm(`${homing.name} 핑키만 자기 주차 칸으로 보낼까요? 적재 자리에 있으면 먼저 20 cm 후진해요.`)) {
      sendPinkyCommand(homing.pinky_id, 'return_home', { path: 'return-home', label: '주차장 복귀', body: { via: [] } });
    }
    return;
  }
  const start = event.target.closest('[data-fleet-start]');
  const pinky = start && pinkyById(start.dataset.fleetStart);
  if (pinky && window.confirm(`${pinky.name} 핑키만 출발시킬까요? 다른 핑키는 그대로 있어요.\n적재 자리 → ${fleetTest.zone[pinky.pinky_id]} → 자기 주차 칸`)) fleetTestStartOne(pinky);
});

// 다시 맞추기: a fresh camera judgment (the check over new frames, not the live preview) goes along, so the Pinky
// comes in corrected by what the camera measured. The judgment is this Pinky's own: each carries its own marker with
// its own registration of the loading spot (config/pinky-markers.json), and another Pinky's offset would send it to
// the wrong place. Without a registration or a usable judgment it comes in again at the same spot, and the label says so.
async function fleetTestRedock(id, label) {
  const marker = pinkyById(id)?.marker;
  let guide = null;
  let why = '카메라 등록 없음';
  if (marker?.registered && marker.spot_id) {
    why = '카메라가 못 봄';
    try {
      const response = await fetch(`/api/parking-spots/${encodeURIComponent(marker.spot_id)}/check`, { cache: 'no-store', signal: AbortSignal.timeout(8000) });
      const seen = response.ok ? (await response.json()).observation?.offset : null;
      if (seen && [seen.forward_mm, seen.left_mm, seen.dyaw_deg].every(Number.isFinite)) {
        guide = { forward_mm: seen.forward_mm, left_mm: seen.left_mm, dyaw_deg: seen.dyaw_deg };
      }
    } catch { /* no judgment: plain redock */ }
  }
  sendPinkyCommand(id, 'redock', {
    path: 'signal', body: guide ? { action: 'redock', offset: guide } : { action: 'redock' },
    label: guide ? `${label} (카메라 보정)` : `${label} (보정 없이 · ${why})`,
  });
}

async function pollFleetTestCamera() {
  if (activeView !== 'pinky' || document.hidden || fleetTest.polling) return;
  fleetTest.polling = true;  // a judgment takes several frames: the next tick waits for this one
  try {
    await pollFleetTestDock();
  } finally {
    fleetTest.polling = false;
  }
}

async function pollFleetTestDock() {
  try {
    const response = await fetch(`/api/parking-spots/${FLEET_TEST_SPOT}/live`, { cache: 'no-store', signal: AbortSignal.timeout(4000) });
    const result = await response.json();
    fleetTest.camera = response.ok ? result.live : { state: result.error_code || '판정 노드 없음' };
  } catch {
    fleetTest.camera = { state: '관제 응답 없음' };
  }
  pinkyMapState.dockState = fleetTest.camera?.state || null;  // the map colours the loading spot with it
  for (const pinky of pinkyScreen.view?.pinkies || []) {
    const id = pinky.pinky_id;
    if (!FLEET_TEST_DOCK_PHASES.has(pinky.phase) || !FLEET_TEST_ACTIVE.has(pinky.route?.status)) {
      delete fleetTest.dock[id];
      delete fleetTest.told[id];
      continue;
    }
    try {
      const response = await fetch(`/api/pinky/${encodeURIComponent(id)}/dock-handshake`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ auto: fleetTestAuto.checked, repeat: Math.max(0, Math.min(20, parseInt(fleetTestRepeat?.value, 10) || 0)) }), signal: AbortSignal.timeout(10000),
      });
      const result = await response.json();
      fleetTest.dock[id] = response.ok ? result : { error: apiErrorMessage(result, '카메라 판정을 받지 못했어요') };
    } catch {
      fleetTest.dock[id] = { error: '관제 서버의 답을 받지 못했어요' };
    }
    // Each new line of the stop's log once, as a toast: what was sent, how a redock turned out, when a person is needed.
    const log = fleetTest.dock[id].log || [];
    const shown = fleetTest.told[id] || 0;
    log.filter((entry) => entry.at > shown).forEach((entry) => showToast(`${pinky.name} · 적재 자리`, entry.message, 'info', 8000));
    if (log.length) fleetTest.told[id] = log.at(-1).at;
  }
  renderFleetTest();
}

document.addEventListener('pinkyview', renderFleetTest);
document.addEventListener('pinkymap', renderFleetTest);
document.addEventListener('viewchange', () => { renderFleetTest(); pollFleetTestCamera(); });
window.setInterval(pollFleetTestCamera, 1500);
