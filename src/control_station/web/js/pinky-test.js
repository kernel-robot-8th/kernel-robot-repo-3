// 전체 주행 테스트(핑키 화면): 주문 없이 모든 핑키가 코스 전체를 돈다.
//   자기 주차 칸 → (9번 통로, 한 대씩) → 대기 자리 4 → 적재 자리 5 (정면 진입) → [도킹완료] → [적재 완료]
//   → 20 cm 후진 → 수령 구역 → [수령 완료] → 자기 주차 칸
// [전체 출발]은 FLEET_TEST_ORDER 순서로 핑키를 출발시키고, 핑키 관제의 교통 구역이 차례로 기다리게 한다(통로는
// 한 대씩, 대기 자리에 한 대, 5번은 요청한 순서대로). 줄마다 그 핑키가 어디 있는지와 지금 단계에 필요한 버튼
// 하나가 나온다. 칩은 천장 카메라가 적재 자리를 어떻게 판정하는지 보여 준다.
//
// pinky-detail.js(dropoffLabels) 뒤에 로드된다. sendPinkyCommand·pinkyScreen·setHtmlIfChanged는 pinky.js,
// pinkyBadge는 app.js에 있다.

const FLEET_TEST_ORDER = ['12', '10', '11'];  // 12번이 등록된 ArUco 마커(21)를 달고 있어 먼저 싣는다
const FLEET_TEST_SPOT = 'pinky_dock_1';       // 천장 카메라에 등록된 적재 자리
// 핑키 단계 → [신호, 버튼 이름, 버튼 모양]. 'redock'은 카메라 보정값을 붙여 보낸다(fleetTestRedock).
const FLEET_TEST_SIGNALS = {
  docking_manual: [['dock_complete', '도킹완료'], ['redock', '다시 맞추기', 'secondary']],
  docked_manual: [['received', '적재 완료 · 출발'], ['redock', '다시 들어오기', 'secondary']],
  course_complete: [['mission_complete', '수령 완료 · 주차장으로']],
};
// 경로가 진행 중인 상태. PARKING_WORDS는 적재 자리 카메라 실시간 상태의 화면 말이다.
const FLEET_TEST_ACTIVE = new Set(['starting', 'running', 'cancel_requested']);
const PARKING_WORDS = { IN_POSITION: '도착 (정상 위치)', OFF_POSITION: '어긋남', NOT_IN_VIEW: '핑키 안 보임', VIEW_UNVERIFIED: '카메라 시야 확인 안 됨', MOVING: '움직이는 중' };

const fleetTestRows = document.querySelector('#fleetTestRows');
const fleetTestCamera = document.querySelector('#fleetTestCamera');
const fleetTestAuto = document.querySelector('#fleetTestAuto');
const fleetTestRepeat = document.querySelector('#fleetTestRepeat');
// dock: 핑키마다 적재 자리 정차에 대한 서버의 마지막 답(POST /api/pinky/{id}/dock-handshake: 그 핑키 자신의
// 등록 위치에서의 카메라 판정, 할 일, 보낸 것, 그 정차의 기록). told: 이미 알림으로 보여 준 기록 줄.
// 둘 다 핑키가 적재 자리를 떠나면 지운다.
// picked: 위의 두 버튼이 움직일 핑키(기본은 모두). 이렇게 핑키 한 대만 돌릴 수도 있다.
const fleetTest = { zone: {}, busy: false, camera: null, dock: {}, told: {}, picked: {} };
const fleetTestPicked = (id) => fleetTest.picked[id] !== false;
// 적재 자리에서 카메라 판정을 주고받는 단계와, 판정 동작별 색.
const FLEET_TEST_DOCK_PHASES = new Set(['docking_manual', 'redocking']);
const FLEET_TEST_DOCK_TONE = { COMPLETE: 'good', OPERATOR: 'attention', REDOCK: 'attention' };

// 적재 자리에 선 핑키에 대해 카메라 <-> 핑키 주고받기가 말하는 것: [색, 글]. 거기 없으면 null.
// 결정은 서버가 한다(control_station/pinky/docking.py, 주문과 같다): 제자리로 두 번 보이면 -> 도킹완료,
// 계속 어긋나면 -> 카메라 오프셋과 함께 다시 맞추기, 그 뒤 돌아올 때까지 기다림. 등록이 없거나 카메라가 못 보거나
// 다시 맞추기를 세 번 다 쓰면 -> 사람이 본다. 자동이 꺼져 있으면 서버는 판정만 하고 버튼이 보낸다.
function fleetTestVerdict(pinky) {
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

// 고를 수 있는 수령 구역: 이름에 '수령'이 든 웨이포인트.
function fleetTestZones() {
  return dropoffLabels().filter((label) => label.includes('수령'));
}

// FLEET_TEST_ORDER 순서로 정렬한 핑키 목록.
function fleetTestPinkies() {
  const all = pinkyScreen.view?.pinkies || [];
  return [...all].sort((a, b) => FLEET_TEST_ORDER.indexOf(a.pinky_id) - FLEET_TEST_ORDER.indexOf(b.pinky_id));
}

// 핑키마다 수령 구역을 하나 정해 둔다. 아직 없거나 사라진 구역이면 순서대로 돌려 나눈다.
function fleetTestZoneFor(pinkyId, index, zones) {
  if (!zones.includes(fleetTest.zone[pinkyId])) fleetTest.zone[pinkyId] = zones[index % Math.max(1, zones.length)];
  return fleetTest.zone[pinkyId];
}

// 핑키 관제가 알려 준 배터리: [글, 색]. 20 % 미만이면 주행 전에 충전해야 한다
// (모터가 약해지고 Pi가 불안정해진다. 2026-11-17: 14 %의 P0가 베이스를 잃고 Nav2를 계속 띄우지 못했다).
function fleetTestBattery(pinky) {
  const percent = pinky.battery_percent;
  if (!pinky.connected || percent == null) return ['배터리 모름', ''];
  const volts = pinky.battery_volts == null ? '' : ` · ${Number(pinky.battery_volts).toFixed(2)} V`;
  return [`🔋 ${Math.round(percent)}%${volts}`, percent < 20 ? 'danger' : percent < 30 ? 'attention' : 'good'];
}

// 핑키 한 줄: 포함 체크, 순서, 지금 하는 일, 배터리, 수령 구역(주행 중에는 못 바꿈), 눌러야 할 버튼, 카메라 판정.
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
    // 구역(통로, 적재 자리)을 잡은 채 꺼진 핑키: 다른 핑키들이 그 핑키를 끝없이 기다린다.
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

// 이 상자는 관제 자체의 스위치다(GET /api/pinky의 dock_auto): 관제가 카메라 <-> 핑키 주고받기를 스스로 진행하므로
// 이 화면을 닫아도 계속된다. 여기서 체크하면 관제의 설정이 바뀐다(POST /api/pinky/dock-auto).
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

// 전체 주행 테스트 카드를 다시 그린다(핑키 뷰일 때만). 위 버튼은 고른 핑키 중 연결되고 준비됐고 주행 중이 아닌
// 핑키가 있어야 켜지고, 출발은 수령 구역이 하나라도 있어야 한다.
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

// 핑키 하나를 시험 임무로 출발시킨다(경유 없이, 고른 수령 구역으로).
function fleetTestStartOne(pinky) {
  return sendPinkyCommand(pinky.pinky_id, 'test-mission', {
    path: 'test-mission', label: '전체 주행 테스트 출발', body: { dropoff: fleetTest.zone[pinky.pinky_id], via: [] },
  });
}

// 고른 핑키 중 출발할 수 있는 핑키마다 차례로 run을 부른다. 그동안 위 버튼을 막는다.
async function fleetTestEach(run) {
  fleetTest.busy = true;
  renderFleetTest();
  try {
    for (const pinky of fleetTestPinkies()) {
      if (!fleetTestPicked(pinky.pinky_id) || !pinky.connected || !pinky.ready || FLEET_TEST_ACTIVE.has(pinky.route?.status)) continue;
      await run(pinky);
      await new Promise((resolve) => window.setTimeout(resolve, 1500));  // 차례를 지켜 앞 핑키가 구역을 먼저 요청하게 한다
    }
  } finally {
    fleetTest.busy = false;
    renderFleetTest();
  }
}

// 위 버튼들: 고른 핑키 출발, 주차장 복귀, 배터리 확인.
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
  await refreshPinky({ force: true });  // 지금 다시 읽은 뒤 한 줄로 알린다
  const pinkies = fleetTestPinkies();
  const low = pinkies.filter((pinky) => pinky.connected && pinky.battery_percent != null && pinky.battery_percent < 20);
  showToast('핑키 배터리', pinkies.map((pinky) => `${pinky.name} ${fleetTestBattery(pinky)[0].replace('🔋 ', '')}`).join('  ·  ')
    + (low.length ? `  — ${low.map((pinky) => pinky.name).join(', ')} 충전이 필요해요` : ''), low.length ? 'error' : 'info', 10000);
  renderFleetTest();
});
// 줄 안의 수령 구역 선택과 포함 체크.
fleetTestRows.addEventListener('change', (event) => {
  const select = event.target.closest('[data-fleet-zone]');
  if (select) fleetTest.zone[select.dataset.fleetZone] = select.value;
  const pick = event.target.closest('[data-fleet-pick]');
  if (pick) {
    fleetTest.picked[pick.dataset.fleetPick] = pick.checked;
    renderFleetTest();
  }
});
// 줄 안의 버튼: 신호(다시 맞추기는 카메라 보정과 함께), 자리 비우기, 이 핑키만 복귀·출발.
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

// 다시 맞추기: 새 카메라 판정(실시간 미리보기가 아니라 새 프레임으로 한 확정 판정)을 함께 보내, 핑키가 카메라가
// 잰 만큼 고쳐서 들어오게 한다. 판정은 그 핑키 자신의 것이다: 핑키마다 자기 마커와 자기 적재 자리 등록이 있고
// (config/pinky-markers.json), 다른 핑키의 오프셋은 엉뚱한 곳으로 보낸다. 등록이나 쓸 만한 판정이 없으면 같은
// 자리로 다시 들어오고, 버튼 이름표에 그렇게 적는다.
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
    } catch { /* 판정 없음: 보정 없이 다시 맞추기 */ }
  }
  sendPinkyCommand(id, 'redock', {
    path: 'signal', body: guide ? { action: 'redock', offset: guide } : { action: 'redock' },
    label: guide ? `${label} (카메라 보정)` : `${label} (보정 없이 · ${why})`,
  });
}

// 1.5초마다(핑키 뷰이고 탭이 보일 때만): 적재 자리 카메라와, 적재 자리에 선 핑키별 판정을 읽는다.
async function pollFleetTestCamera() {
  if (activeView !== 'pinky' || document.hidden || fleetTest.polling) return;
  fleetTest.polling = true;  // 판정은 여러 프레임이 걸린다. 다음 주기는 이번 것을 기다린다
  try {
    await pollFleetTestDock();
  } finally {
    fleetTest.polling = false;
  }
}

// GET /api/parking-spots/{FLEET_TEST_SPOT}/live로 적재 자리 상태를 읽고, 적재 자리 단계인 핑키마다
// POST /api/pinky/{id}/dock-handshake를 보낸다. auto면 서버가 도킹완료·다시 맞추기를 알아서 보내고, repeat은
// 반복 진입 테스트에서 성공해야 할 횟수(0~20)다. 적재 자리를 떠난 핑키의 기록은 지운다.
async function pollFleetTestDock() {
  try {
    const response = await fetch(`/api/parking-spots/${FLEET_TEST_SPOT}/live`, { cache: 'no-store', signal: AbortSignal.timeout(4000) });
    const result = await response.json();
    fleetTest.camera = response.ok ? result.live : { state: result.error_code || '판정 노드 없음' };
  } catch {
    fleetTest.camera = { state: '관제 응답 없음' };
  }
  pinkyMapState.dockState = fleetTest.camera?.state || null;  // 지도가 이 값으로 적재 자리를 칠한다
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
    // 그 정차 기록의 새 줄마다 한 번씩 알림으로: 보낸 것, 다시 맞추기 결과, 사람이 필요할 때.
    const log = fleetTest.dock[id].log || [];
    const shown = fleetTest.told[id] || 0;
    log.filter((entry) => entry.at > shown).forEach((entry) => showToast(`${pinky.name} · 적재 자리`, entry.message, 'info', 8000));
    if (log.length) fleetTest.told[id] = log.at(-1).at;
  }
  renderFleetTest();
}

// 핑키·지도 갱신과 뷰 전환 때 다시 그리고, 카메라 판정은 1.5초마다 읽는다.
document.addEventListener('pinkyview', renderFleetTest);
document.addEventListener('pinkymap', renderFleetTest);
document.addEventListener('viewchange', () => { renderFleetTest(); pollFleetTestCamera(); });
window.setInterval(pollFleetTestCamera, 1500);
