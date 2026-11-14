// 종합: the first screen of the control station. One look says what needs a person now, how the orders, Pinkys,
// arms, stock and devices are, and where the Pinkys are on the map. Every tile and row leads to the screen that
// acts on it; this screen itself sends no command.
//
// Loaded after app.js (flowSummary, intake, showView, setStat-style helpers, pinkyBadge, orderSummary, objectLabel,
// formatClock, elapsed), pinky.js (pinkyScreen, setHtmlIfChanged, pinkyActionLabels, commandPhrase), pinky-detail.js
// (openPinkyDetail) and pinky-map.js (pinkyMapState, drawPinkyMap, openMapViewer). /api/flow and the intake are
// already read by app.js every 1–2 s, /api/pinky and the map by their scripts while this view is open; this file
// reads the rest (health, cameras, stock, order history) itself, and only while it is on screen.

const DASH_POLL_MS = 3000;
const DASH_WORKER_FRESH_MS = 45000;  // an arm worker long-polls at most every 20 s; past this it is not there
const DASH_EVENTS_SHOWN = 6;

const dashView = document.querySelector('#view-dashboard');
const dashNow = document.querySelector('#dashNow');
const dashNowTitle = document.querySelector('#dashNowTitle');
const dashNowDetail = document.querySelector('#dashNowDetail');
const dashTiles = document.querySelector('#dashTiles');
const dashMapSvg = document.querySelector('#dashMap');
const dashMapEmpty = document.querySelector('#dashMapEmpty');
const dashMapOpen = document.querySelector('#dashMapOpen');
const dashMapNote = document.querySelector('#dashMapNote');
const dashProgress = document.querySelector('#dashProgress');
const dashProgressCount = document.querySelector('#dashProgressCount');
const dashDevices = document.querySelector('#dashDevices');
const dashTodo = document.querySelector('#dashTodo');
const dashTodoCount = document.querySelector('#dashTodoCount');
const dashEvents = document.querySelector('#dashEvents');
const dashStock = document.querySelector('#dashStock');
const dashStockNote = document.querySelector('#dashStockNote');

const dash = {
  health: null,
  cameras: null,
  inventory: null,
  tasks: null,
  loadError: null,
  polling: false,
};

const dashArmNames = { 'arm-01': '왼쪽 팔', 'arm-02': '오른쪽 팔' };
const dashCameraRoles = {
  HEAD: '천장 캠', CEILING: '천장 캠', JOINT: '조인트 캠', GRIPPER: '조인트 캠', WRIST: '조인트 캠',
  OVERHEAD: '탑 캠', TOP: '탑 캠', FRONT: '정면 캠', SHELF: '선반 캠',
};
const dashRunning = ['VALIDATING', 'RUNNING', 'CANCELING'];
const dashStopped = ['FAILED', 'NEEDS_INTERVENTION'];

function dashOnScreen() {
  return !document.hidden && activeView === 'dashboard';
}

// ---- reading ----

async function dashGet(path) {
  const response = await fetch(path, { cache: 'no-store', signal: AbortSignal.timeout(8000) });
  const result = await response.json();
  if (!response.ok) throw new Error(apiErrorMessage(result, `${path} 응답 없음`));
  return result;
}

async function refreshDashboard({ force = false } = {}) {
  if (!force && (!dashOnScreen() || dash.polling)) return;
  dash.polling = true;
  const [health, cameras, inventory, tasks] = await Promise.allSettled([
    dashGet('/api/health'), dashGet('/api/cameras'), dashGet('/api/inventory'), dashGet('/api/tasks'),
  ]);
  dash.polling = false;
  // A failed read keeps the last answer; the banner says the server is not answering only when health fails.
  if (health.status === 'fulfilled') dash.health = health.value;
  if (cameras.status === 'fulfilled') dash.cameras = cameras.value.cameras || [];
  if (inventory.status === 'fulfilled') dash.inventory = inventory.value;
  if (tasks.status === 'fulfilled') dash.tasks = tasks.value.tasks || [];
  dash.loadError = health.status === 'rejected' ? '관제 서버가 응답하지 않아요' : null;
  renderDashboard();
}

// ---- what the screen is made of ----

function dashIsToday(iso) {
  if (!iso) return false;
  return new Date(iso).toDateString() === new Date().toDateString();
}

function dashTodaysOrders() {
  return (dash.tasks || []).filter((task) => Array.isArray(task.order_items) && dashIsToday(task.created_at));
}

function dashFlowView() {
  return flowSummary.flow;  // app.js reads /api/flow every second
}

function dashWaitingOrders() {
  return intake.orders || [];  // app.js reads the intake every two seconds
}

function dashPickupWaiting(flow) {
  return (flow?.orders || []).filter((order) => order.active_step === 'CONFIRM_PICKUP');
}

function dashArmState(arm) {
  if (arm.state === 'busy') return { tone: 'active', text: arm.doing?.text || '작업 중', key: 'busy' };
  const lastPoll = arm.worker?.last_poll_at ? Date.parse(arm.worker.last_poll_at) : NaN;
  if (Number.isFinite(lastPoll) && Date.now() - lastPoll < DASH_WORKER_FRESH_MS) return { tone: 'ok', text: '준비', key: 'ready' };
  return { tone: 'off', text: '연결 전', key: 'off' };
}

function dashCameraName(camera) {
  if (camera.label) return camera.label;
  const role = dashCameraRoles[String(camera.role || camera.position || '').toUpperCase()] || camera.id;
  const arm = camera.arm_label || dashArmNames[camera.arm_id];
  return arm && !role.startsWith('천장') && !role.startsWith('선반') ? `${arm} ${role}` : role;
}

function dashCameraState(camera) {
  if (camera.stream_state === 'STREAMING') {
    return camera.frame_freshness === 'STALE' ? { tone: 'warn', text: '영상 늦음' } : { tone: 'ok', text: '영상 들어옴' };
  }
  if (camera.configured === false) return { tone: 'off', text: '연결 전' };
  if (camera.connection_state === 'CONNECTED') return { tone: 'warn', text: '영상 기다리는 중' };
  if (camera.connection_state === 'DISCONNECTED') return { tone: 'bad', text: '연결 끊김' };
  return { tone: 'off', text: '연결 전' };  // never opened: no device path set, or this mode does not open it
}

function dashPinkyRowState(pinky) {
  if (!pinky.connected) return { tone: 'off', text: '연결 끊김' };
  if (pinky.stop_pending) return { tone: 'bad', text: '정지 적용 중' };
  const phase = pinky.phase_label || pinky.phase || '단계 모름';
  if (!pinky.ready) return { tone: 'warn', text: `${phase} · ${pinky.simulated && !devMode ? '주행 불가' : pinky.readiness_label || '주행 불가'}` };
  return { tone: 'ok', text: phase };
}

// ---- the sentence on top ----

function dashNowLine(flow) {
  if (dash.loadError && !dash.health) return { tone: 'bad', title: dash.loadError, detail: '다시 연결하는 중이에요' };
  const errors = (flow?.alerts || []).filter((alert) => alert.level === 'error');
  const warnings = (flow?.alerts || []).filter((alert) => alert.level !== 'error' && alert.code !== 'PICKUP_WAITING');
  const pickup = dashPickupWaiting(flow);
  const waiting = dashWaitingOrders();
  const running = (flow?.orders || []).filter((order) => dashRunning.includes(order.status));
  const parts = [];
  if (waiting.length) parts.push(`새 주문 ${waiting.length}건 시작 대기`);
  if (pickup.length) parts.push(`수령 확인 대기 ${pickup.length}건`);
  if (warnings.length) parts.push(`확인할 경보 ${warnings.length}건`);
  if (errors.length) {
    return { tone: 'bad', title: `멈춘 주문이 ${errors.length}건 있어요`, detail: [errors[0].title, ...parts].filter(Boolean).join(' · ') };
  }
  if (waiting.length) {
    return { tone: 'warn', title: `새 주문 ${waiting.length}건이 시작을 기다려요`, detail: parts.slice(1).join(' · ') || '주문 진행에서 시작할 수 있어요' };
  }
  if (pickup.length || warnings.length) return { tone: 'warn', title: parts[0], detail: parts.slice(1).join(' · ') };
  if (running.length) {
    const first = running[0];
    return { tone: 'active', title: `주문 ${running.length}건을 처리하고 있어요`, detail: `${first.pinky_name ? `핑키 ${first.pinky_name} · ` : ''}${first.current || ''}` };
  }
  return { tone: 'ok', title: '모두 정상이에요', detail: '처리할 일이 없어요' };
}

function dashRenderNow(flow) {
  const line = dashNowLine(flow);
  dashNow.className = `dash-now ${line.tone}`;
  dashNowTitle.textContent = line.title;
  dashNowDetail.textContent = line.detail || '';
}

// ---- big numbers ----

function dashTileHtml({ view, label, value, total = null, tone = '', sub = '' }) {
  const shown = value == null ? '–' : escapeHtml(value);
  return `<button type="button" class="dash-tile ${tone}" data-dash-goto="${view}" aria-label="${escapeHtml(`${label} ${value ?? '모름'}${total == null ? '' : `/${total}`}, ${sub}`)}">
    <span class="dash-tile-label">${escapeHtml(label)}</span>
    <strong class="dash-tile-value">${shown}${total == null ? '' : `<small>/${escapeHtml(total)}</small>`}</strong>
    <span class="dash-tile-sub">${sub}</span>
  </button>`;
}

function dashRenderTiles(flow) {
  const today = dash.tasks ? dashTodaysOrders() : null;
  const waiting = dashWaitingOrders().length;
  const running = today ? today.filter((task) => dashRunning.includes(task.status)).length : null;
  const done = today ? today.filter((task) => task.status === 'SUCCEEDED').length : null;
  const pinkies = pinkyScreen.view?.pinkies || [];
  const connected = pinkies.filter((pinky) => pinky.connected).length;
  const ready = pinkies.filter((pinky) => pinky.connected && pinky.ready).length;
  const arms = (flow?.arms || []).map(dashArmState);
  const armReady = arms.filter((state) => state.key !== 'off').length;
  const armBusy = arms.filter((state) => state.key === 'busy').length;
  const products = dash.inventory?.products || [];
  const soldOut = products.filter((product) => product.status === 'sold_out').length;
  const unknown = products.filter((product) => product.status === 'unknown').length;
  const alerts = (flow?.alerts || []).filter((alert) => alert.code !== 'PICKUP_WAITING');  // waiting is not trouble
  const pickup = dashPickupWaiting(flow).length;
  const orderSub = today
    ? `대기 <b>${waiting}</b> · 진행 <b>${running}</b> · 완료 <b>${done}</b>`
    : '불러오는 중';
  setHtmlIfChanged(dashTiles, [
    dashTileHtml({ view: 'flow', label: '오늘 주문', value: today ? today.length : null, tone: waiting ? 'attention' : running ? 'active' : '', sub: orderSub }),
    dashTileHtml({
      view: 'pinky', label: '운행 가능 핑키', value: pinkyScreen.view ? ready : null, total: pinkies.length || 3,
      tone: pinkyScreen.view && !ready ? 'attention' : '', sub: pinkyScreen.view ? `연결 <b>${connected}</b>대` : '불러오는 중',
    }),
    dashTileHtml({
      view: 'manual', label: '준비된 팔', value: flow ? armReady : null, total: arms.length || 2,
      tone: armBusy ? 'active' : flow && !armReady ? 'zero' : '', sub: flow ? (armBusy ? `작업 중 <b>${armBusy}</b>대` : armReady ? '대기 중' : '팔 연결 전') : '불러오는 중',
    }),
    dashTileHtml({
      view: 'inventory', label: '품절', value: dash.inventory ? soldOut : null,
      tone: soldOut ? 'danger' : dash.inventory ? 'zero' : '', sub: unknown ? `재고 모름 <b>${unknown}</b>개` : dash.inventory ? `물건 ${products.length}개 중` : '불러오는 중',
    }),
    dashTileHtml({
      view: 'flow', label: '경보', value: flow ? alerts.length : null,
      tone: alerts.some((alert) => alert.level === 'error') ? 'danger' : alerts.length ? 'attention' : flow ? 'zero' : '',
      sub: pickup ? `수령 대기 <b>${pickup}</b>건` : flow ? '멈춘 주문 없음' : '불러오는 중',
    }),
  ].join(''));
}

// ---- the order on its way ----

// The delivery half (after the arms) is followed by the order's active step: 배달, 수령, then the Pinky going home.
const dashDeliverySteps = [['NOTIFY_PINKY_RECEIVED', 'WAIT_AT_RECEIVING_ZONE'], ['CONFIRM_PICKUP'], ['WAIT_PINKY_PARKED', 'FINALIZE_TASK']];

function dashFoldStages(stages) {
  if (!stages.length) return 'todo';
  if (stages.some((stage) => stage.state === 'failed')) return 'failed';
  if (stages.some((stage) => stage.state === 'active')) return 'active';
  if (stages.every((stage) => ['done', 'skipped'].includes(stage.state))) return 'done';
  return stages.some((stage) => stage.state === 'done') ? 'active' : 'todo';
}

// The flow view's stages folded into the seven a person follows: 접수 → 준비 → 핑키 도착 → 적재 → 배달 → 수령 → 완료.
function dashOrderStages(order) {
  const byKey = new Map((order.stages || []).map((stage) => [stage.key, stage]));
  const pick = (keys) => keys.map((key) => byKey.get(key)).filter(Boolean);
  const finished = order.status === 'SUCCEEDED';
  const stopped = dashStopped.includes(order.status);
  const delivery = dashDeliverySteps.findIndex((steps) => steps.includes(order.active_step));
  const front = [
    ['접수', pick(['ORDER'])], ['준비', pick(['STOCK', 'PLAN'])], ['핑키 도착', pick(['PINKY'])],
    [order.total ? `적재 ${order.loaded ?? 0}/${order.total}` : '적재', pick(['LOAD', 'TOTAL'])],
  ].map(([label, stages]) => ({ label, state: finished || delivery >= 0 ? 'done' : dashFoldStages(stages) }));
  const back = ['배달', '수령', '완료'].map((label, index) => {
    let state = 'todo';
    if (finished) state = 'done';
    else if (delivery >= 0) state = index < delivery ? 'done' : index === delivery ? (stopped ? 'failed' : 'active') : 'todo';
    else if (index === 0) state = dashFoldStages(pick(['SIGNAL']));  // the completion signal starts the delivery
    return { label: index === 2 && state === 'active' ? '복귀 중' : label, state };
  });
  return [...front, ...back];
}

function dashProgressCardHtml(order) {
  const stopped = dashStopped.includes(order.status);
  return `<article class="dash-order${stopped ? ' stopped' : ''}">
    <header>
      ${order.pinky_id ? pinkyBadge(order.pinky_id) : ''}
      <strong>${escapeHtml(orderSummary(order.order_items))}</strong>
      <span class="dash-order-meta"><span>${escapeHtml(order.id)}</span>${order.started_at ? `<time>${elapsed(order.started_at, order.finished_at)}</time>` : ''}</span>
    </header>
    <ol class="dash-stages">${dashOrderStages(order).map((stage) => `
      <li class="${stage.state}"><span class="dash-stage-dot" aria-hidden="true"></span><span>${escapeHtml(stage.label)}</span></li>`).join('')}</ol>
    <p class="dash-order-now${stopped ? ' stopped' : ''}"><b>지금</b> ${escapeHtml(order.current || (stopped ? '멈춤' : ''))}</p>
    <div class="dash-order-plan">${planChips(order)}</div>
  </article>`;
}

function dashRenderProgress(flow) {
  const orders = flow?.orders || [];
  const active = orders.filter((order) => dashRunning.includes(order.status) || dashStopped.includes(order.status));
  dashProgressCount.textContent = flow ? (active.length ? `${active.length}건` : '없음') : '';
  if (!flow) {
    setHtmlIfChanged(dashProgress, '<p class="dash-empty">주문을 불러오는 중…</p>');
    return;
  }
  if (!active.length) {
    const last = orders.find((order) => order.status === 'SUCCEEDED');
    setHtmlIfChanged(dashProgress, `<div class="dash-idle">
        <p class="dash-empty">진행 중인 주문이 없어요</p>
        ${last ? `<p class="dash-last">마지막 완료 · ${escapeHtml(orderSummary(last.order_items))}${last.finished_at ? ` · ${escapeHtml(formatClock(last.finished_at))}` : ''}</p>` : ''}
        <a class="dash-link" href="/order" target="_blank" rel="noopener">고객 주문 앱에서 주문 넣기</a>
      </div>`);
    return;
  }
  // Stopped orders first: a person has to look at them.
  const sorted = [...active].sort((a, b) => Number(dashStopped.includes(b.status)) - Number(dashStopped.includes(a.status)));
  setHtmlIfChanged(dashProgress, sorted.slice(0, 2).map(dashProgressCardHtml).join('')
    + (sorted.length > 2 ? `<button type="button" class="dash-more" data-dash-goto="flow">${sorted.length - 2}건 더 보기</button>` : ''));
}

// ---- devices ----

function dashDeviceRow({ name, state, detail = '', attrs = '' }) {
  return `<li class="dash-device ${state.tone}"${attrs}>
    <span class="dash-dot" aria-hidden="true"></span>
    <span class="dash-device-name">${name}</span>
    <span class="dash-device-state">${escapeHtml(state.text)}${detail ? `<small>${escapeHtml(detail)}</small>` : ''}</span>
  </li>`;
}

function dashRenderDevices(flow) {
  const groups = [];
  const pinkies = pinkyScreen.view?.pinkies;
  groups.push(['핑키', pinkies
    ? pinkies.map((pinky) => {
      const percent = pinky.battery_percent == null ? '' : `${Math.round(pinky.battery_percent)}%`;
      return dashDeviceRow({
        name: `${pinkyBadge(pinky.pinky_id, { offline: !pinky.connected })}`,
        state: dashPinkyRowState(pinky), detail: pinky.connected ? percent : '',
        attrs: ` data-dash-pinky="${escapeHtml(pinky.pinky_id)}" role="button" tabindex="0" aria-label="핑키 ${escapeHtml(pinky.name)} 자세히 보기"`,
      });
    }).join('')
    : dashDeviceRow({ name: '핑키', state: { tone: 'off', text: pinkyScreen.loadError || '불러오는 중' } })]);
  groups.push(['팔', (flow?.arms || []).map((arm) => dashDeviceRow({
    name: escapeHtml(arm.label || dashArmNames[arm.arm_id] || arm.arm_id), state: dashArmState(arm),
    detail: arm.shelf_id ? (arm.shelf_id === 'shelf_a' ? '선반 A' : arm.shelf_id === 'shelf_b' ? '선반 B' : arm.shelf_id) : '',
  })).join('') || dashDeviceRow({ name: '팔', state: { tone: 'off', text: flow ? '설정 없음' : '불러오는 중' } })]);
  groups.push(['카메라', dash.cameras
    ? (dash.cameras.length
      ? dash.cameras.map((camera) => dashDeviceRow({ name: escapeHtml(dashCameraName(camera)), state: dashCameraState(camera) })).join('')
      : dashDeviceRow({ name: '카메라', state: { tone: 'off', text: '설정된 카메라 없음' } }))
    : dashDeviceRow({ name: '카메라', state: { tone: 'off', text: '불러오는 중' } })]);
  const parking = dash.health?.parking;
  const inventory = dash.inventory;
  const stockLive = inventory && !inventory.simulated && inventory.state === 'live';
  groups.push(['판정', [
    dashDeviceRow({
      name: '핑키 도착 판정',
      state: parking?.camera_id ? { tone: 'ok', text: parking.live_tracking ? '실시간 추적 중' : '확정 판정만' } : { tone: 'off', text: '연결 전' },
    }),
    dashDeviceRow({
      name: '재고 인식',
      state: !inventory ? { tone: 'off', text: '불러오는 중' }
        : stockLive ? { tone: 'ok', text: '카메라로 세는 중' }
          : inventory.simulated ? { tone: 'off', text: '연결 전' } : { tone: 'warn', text: inventory.reason || '확인 필요' },
    }),
  ].join('')]);
  setHtmlIfChanged(dashDevices, groups.map(([title, rows]) => `
    <div class="dash-device-group"><h4>${escapeHtml(title)}</h4><ul>${rows}</ul></div>`).join(''));
}

// ---- things a person should do ----

function dashRenderTodo(flow) {
  const items = [];
  const waiting = dashWaitingOrders();
  waiting.slice(0, 3).forEach((order) => items.push({
    tone: 'warn', view: 'flow', title: `새 주문 ${order.id}`, text: `${orderSummary(order.order_items)} · 시작 또는 거절`,
  }));
  if (waiting.length > 3) items.push({ tone: 'warn', view: 'flow', title: `새 주문 ${waiting.length - 3}건 더`, text: '주문 진행에서 확인' });
  (flow?.alerts || []).forEach((alert) => items.push({
    tone: alert.level === 'error' ? 'bad' : 'warn', view: 'flow',
    title: alert.title || '확인 필요', text: [alert.pinky_name ? `핑키 ${alert.pinky_name}` : '', alert.message].filter(Boolean).join(' · '),
  }));
  (dash.inventory?.products || []).filter((product) => product.status === 'unknown').forEach((product) => items.push({
    tone: 'warn', view: 'inventory', title: `${product.label} 재고 모름`, text: '주문을 닫아 두었어요 · 직접 수량을 넣을 수 있어요',
  }));
  (pinkyScreen.view?.pinkies || []).filter((pinky) => pinky.connected && !pinky.ready && !pinky.simulated).forEach((pinky) => items.push({
    tone: 'warn', view: 'pinky', title: `핑키 ${pinky.name} ${pinky.readiness_label || '주행 불가'}`, text: '핑키 화면에서 주행준비',
  }));
  dashTodoCount.textContent = items.length ? `${items.length}건` : '';
  setHtmlIfChanged(dashTodo, items.length
    ? `<ul>${items.slice(0, 6).map((item) => `
        <li><button type="button" class="dash-todo ${item.tone}" data-dash-goto="${item.view}">
          <span class="dash-dot" aria-hidden="true"></span>
          <span><strong>${escapeHtml(item.title)}</strong><small>${escapeHtml(item.text)}</small></span>
          <span class="dash-chevron" aria-hidden="true">›</span>
        </button></li>`).join('')}</ul>`
    : '<p class="dash-empty good">처리할 일이 없어요</p>');
}

// ---- recent events: orders, stock changes, Pinky commands, newest first ----

function dashOrderEvents() {
  const words = { SUCCEEDED: ['완료', 'ok'], FAILED: ['멈춤', 'bad'], CANCELED: ['취소', 'off'], NEEDS_INTERVENTION: ['확인 필요', 'bad'] };
  const events = [];
  dashTodaysOrders().forEach((task) => {
    const summary = orderSummary(task.order_items);
    events.push({ at: task.created_at, tone: 'warn', word: '접수', text: `${task.id} · ${summary}` });
    if (task.started_at) events.push({ at: task.started_at, tone: 'active', word: '시작', text: `${task.id}${task.pinky_id ? ` · 핑키 ${pinkyNames[task.pinky_id] || task.pinky_id}` : ''}` });
    if (task.finished_at && words[task.status]) {
      const [word, tone] = words[task.status];
      events.push({ at: task.finished_at, tone, word, text: `${task.id} · ${summary}` });
    }
  });
  return events;
}

function dashStockEvents() {
  const words = { RESTOCKED: ['채움', 'ok'], REMOVED_OUTSIDE: ['줄어듦', 'warn'], OVERRIDE: ['재고 수정', 'off'] };
  return (dash.inventory?.events || []).map((event) => {
    const [word, tone] = words[event.kind] || [event.kind, 'off'];
    const counts = event.from != null && event.to != null ? ` ${event.from}→${event.to}` : '';
    return { at: event.at, tone, word, text: `${objectLabel(event.object_class)}${counts}` };
  });
}

function dashPinkyEvents() {
  return (pinkyScreen.view?.log || []).map((entry) => ({
    at: entry.at, tone: entry.ok ? 'active' : 'bad', word: '핑키',
    text: `${pinkyNames[entry.pinky_id] || entry.pinky_id || '전체'} ${commandPhrase(entry, pinkyActionLabels[entry.action] || entry.action)}`,
  }));
}

function dashRenderEvents() {
  const events = [...dashOrderEvents(), ...dashStockEvents(), ...dashPinkyEvents()]
    .filter((event) => event.at)
    .sort((a, b) => Date.parse(b.at) - Date.parse(a.at))
    .slice(0, DASH_EVENTS_SHOWN);
  setHtmlIfChanged(dashEvents, events.length
    ? `<ol>${events.map((event) => `
        <li class="${event.tone}"><time datetime="${escapeHtml(event.at)}">${escapeHtml(formatClock(event.at))}</time>
          <span class="dash-event-word">${escapeHtml(event.word)}</span><span class="dash-event-text">${escapeHtml(event.text)}</span></li>`).join('')}</ol>`
    : '<p class="dash-empty">오늘 있었던 일이 아직 없어요</p>');
}

// ---- stock band ----

function dashRenderStock() {
  const inventory = dash.inventory;
  if (!inventory) {
    setHtmlIfChanged(dashStock, '<p class="dash-empty">재고를 불러오는 중…</p>');
    return;
  }
  const words = { sold_out: '품절', unknown: '모름', closed: '판매 중지' };
  setHtmlIfChanged(dashStock, (inventory.products || []).map((product) => {
    const count = product.status === 'available' || product.status === 'sold_out' ? product.available : null;
    return `<button type="button" class="dash-stock-item ${escapeHtml(product.status)}" data-dash-goto="inventory">
      <span class="dash-stock-name">${escapeHtml(product.label)}</span>
      <strong>${count == null ? escapeHtml(words[product.status] || '–') : `${escapeHtml(count)}<small>개</small>`}</strong>
      ${product.reserved ? `<span class="dash-stock-sub">주문이 ${escapeHtml(product.reserved)}개 잡아 둠</span>` : product.status !== 'available' && count != null ? `<span class="dash-stock-sub">${escapeHtml(words[product.status] || '')}</span>` : ''}
    </button>`;
  }).join('') || '<p class="dash-empty">파는 물건이 없어요</p>');
  dashStockNote.textContent = inventory.simulated ? '재고 카메라 연결 전' : `갱신 ${formatClock(inventory.checked_at)}`;
}

// ---- map ----

function renderDashMap() {
  if (activeView !== 'dashboard') return;
  drawPinkyMap(dashMapSvg, dashMapEmpty);
  const note = mapNote();
  dashMapNote.hidden = !note;
  dashMapNote.textContent = note;
}

// ---- all of it ----

function renderDashboard() {
  if (activeView !== 'dashboard') return;
  const flow = dashFlowView();
  dashRenderNow(flow);
  dashRenderTiles(flow);
  dashRenderProgress(flow);
  dashRenderDevices(flow);
  dashRenderTodo(flow);
  dashRenderEvents();
  dashRenderStock();
}

dashView.addEventListener('click', (event) => {
  const goto = event.target.closest('[data-dash-goto]');
  if (goto) {
    showView(goto.dataset.dashGoto);
    window.scrollTo(0, 0);
    return;
  }
  const pinky = event.target.closest('[data-dash-pinky]');
  if (pinky) openPinkyDetail(pinky.dataset.dashPinky, pinky);
});
dashView.addEventListener('keydown', (event) => {
  const pinky = event.target.closest('[data-dash-pinky]');
  if (pinky && (event.key === 'Enter' || event.key === ' ')) {
    event.preventDefault();
    openPinkyDetail(pinky.dataset.dashPinky, pinky);
  }
});
dashMapOpen.addEventListener('click', () => openMapViewer(dashMapOpen));

document.addEventListener('pinkymap', renderDashMap);
document.addEventListener('pinkyview', renderDashboard);
document.addEventListener('viewchange', (event) => {
  if (event.detail.view !== 'dashboard') return;
  renderDashboard();
  renderDashMap();
  refreshDashboard({ force: true });
});
document.addEventListener('visibilitychange', () => refreshDashboard());

// Flow and intake are refreshed by app.js; redraw from them every second without reading anything more.
window.setInterval(() => { if (dashOnScreen()) renderDashboard(); }, 1000);
window.setInterval(refreshDashboard, DASH_POLL_MS);
if (activeView === 'dashboard') {
  renderDashboard();
  refreshDashboard({ force: true });
}
