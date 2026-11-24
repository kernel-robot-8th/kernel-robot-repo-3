// 핑키 화면: 우리 서버를 거쳐 보는 핑키 세 대(작업자는 이제 핑키 관제(fleet dashboard) 화면을 직접 열지 않는다).
// 먼저 차분한 요약: 핑키 관제 연결 상태, 핑키마다 정지 버튼이 있는 작은 카드, 작은 지도, 경로 노선도.
// 카드를 누르면 센서와 작업자 동작이 있는 상세(pinky-detail.js)가 열린다.
//
// app.js(escapeHtml, apiErrorMessage, formatClock, showToast, setStat, pinkyBadge, pinkyNames, activeView,
// openOverlay, closeOverlay), route-strip.js, pinky-map.js 뒤에 로드된다. 머리글의 [전체 정지]도 이 파일이 맡는다.
//
// 모든 명령은 {command: {ok, http_status, simulated, message}}로 답한다. "접수"는 핑키 관제가 요청을 받았다는
// 뜻일 뿐이고, 핑키가 해냈는지는 나중에 상태에 나타난다. 401은 핑키 관제에 SSH 비밀번호가 필요하다는 뜻이다:
// 비밀번호 창이 열리고 작업자가 명령을 다시 보낼 수 있다. 무엇도 저절로 다시 보내지 않으며, 비밀번호는
// 입력칸에서 서버로만 가고 그 밖의 어디에도 남지 않는다.

// 읽기 주기 1.5초(핑키·종합 뷰이거나 상세가 열려 있을 때만). 명령 결과 알림은 12초 동안 보인다.
const PINKY_POLL_MS = 1500;
const PINKY_READ_TIMEOUT_MS = 10000;  // 서버는 핑키 관제에 3초 제한으로 묻는다. 멈춘 읽기 하나가 폴링을 막으면 안 된다
const PINKY_COMMAND_TOAST_MS = 12000;

const pinkyCards = document.querySelector('#pinkyCards');
const pinkyLink = document.querySelector('#pinkyLink');
const pinkyLinkTitle = document.querySelector('#pinkyLinkTitle');
const pinkyLinkDetail = document.querySelector('#pinkyLinkDetail');
const pinkyLog = document.querySelector('#pinkyLog');
const pinkyLogCount = document.querySelector('#pinkyLogCount');
const pinkyStopAll = document.querySelector('#pinkyStopAll');
const stopAllLabel = document.querySelector('#stopAllLabel');
const pinkyPrepareAll = document.querySelector('#pinkyPrepareAll');
const pinkyPrepareHint = document.querySelector('#pinkyPrepareHint');
const pinkyPasswordOpen = document.querySelector('#pinkyPasswordOpen');
const pinkyStatConnected = document.querySelector('#pinkyStatConnected');
const pinkyStatReady = document.querySelector('#pinkyStatReady');
const pinkyStatStopping = document.querySelector('#pinkyStatStopping');
const pinkyMapSmall = document.querySelector('#pinkyMapSmall');
const pinkyMapSmallEmpty = document.querySelector('#pinkyMapSmallEmpty');
const pinkyMapTag = document.querySelector('#pinkyMapTag');
const pinkyMapOff = document.querySelector('#pinkyMapOff');
const pinkyMapNote = document.querySelector('#pinkyMapNote');
const pinkySheet = document.querySelector('#pinkySheet');
const passwordPrompt = document.querySelector('#passwordPrompt');
const passwordPromptTitle = document.querySelector('#passwordPromptTitle');
const passwordPromptReason = document.querySelector('#passwordPromptReason');
const pinkyPasswordForm = document.querySelector('#pinkyPasswordForm');
const pinkyPassword = document.querySelector('#pinkyPassword');
const pinkyPasswordMessage = document.querySelector('#pinkyPasswordMessage');
const passwordRetry = document.querySelector('#passwordRetry');

const pinkyScreen = {
  view: null,              // 마지막 GET /api/pinky 응답
  loadError: null,         // 마지막 GET이 실패한 이유. 그때 화면은 마지막으로 받은 응답을 보여 준다
  requestSeq: 0,           // 가장 최근 GET의 응답만 그린다
  polling: false,          // 읽는 중. 타이머는 요청을 쌓지 않고 건너뛴다
  sending: new Set(),      // 요청이 오가는 중인 `${pinky id}:${action}`
  stopAllSending: 0,      // 오가는 전체 정지 요청 수(연달아 누를 수 있다)
  prepareAllSending: false,  // '주행준비하고 위치 잡기' 요청 중
  retry: null,             // 비밀번호가 받아들여진 뒤 다시 보내기로 내미는 { label, run }
  rendered: new WeakMap(), // 요소 -> 보여 주는 HTML, 바뀌지 않은 폴링은 버튼을 건드리지 않는다
};

// 핑키 관제의 경로 상태(route.status) 이름.
const pinkyRouteLabels = {
  starting: '경로 시작 중', running: '주행 중', complete: '경로 끝', failed: '경로 중단',
  cancel_requested: '경로 취소 요청 중', cancelled: '경로 취소됨',
};
// 서버 기록은 동작마다 이름(action)을 붙인다. 화면은 이 말로 보여 준다.
const pinkyActionLabels = {
  stop: '정지', password: '비밀번호', prepare: '주행준비', prepare_all: '주행준비하고 위치 잡기',
  initial_pose: '주차장 좌표로 맞추기', localize: '라이다 위치 찾기', set_pose: '위치 다시 잡기', clear_route: '기존 경로 날리기', return_home: '주차장 복귀', reset: '초기 세팅', nav_restart: 'Nav2 복구',
  test_mission: '시험 임무 시작', dock_complete: '도킹완료', received: '적재 완료', mission_complete: '수령 완료',
  waypoint: '웨이포인트 기록', waypoint_delete: '웨이포인트 삭제',
};

// HTML이 바뀔 때만 innerHTML을 쓴다. 다른 핑키 스크립트와 dashboard.js도 쓴다.
function setHtmlIfChanged(element, html) {
  if (pinkyScreen.rendered.get(element) === html) return;
  element.innerHTML = html;
  pinkyScreen.rendered.set(element, html);
}

function pinkyById(pinkyId) {
  return pinkyScreen.view?.pinkies.find((item) => item.pinky_id === pinkyId);
}

function pinkyName(pinkyId) {
  return pinkyById(pinkyId)?.name || pinkyNames[pinkyId] || pinkyId;
}

// ---- 핑키 관제의 답을 화면의 말로. 접수는 완료가 아니다: 여기의 어떤 말도 핑키가 해냈다고 하지 않는다. ----

// 명령 기록 한 건을 { kind, word }로: ok(접수), mock(보내지 않음), unknown(응답 없음), refused(거절, 401은 비밀번호 필요).
function commandOutcome(entry) {
  if (entry.ok) return { kind: 'ok', word: '접수' };
  if (entry.simulated) return { kind: 'mock', word: devMode ? '모의' : '보내지 않음' };
  if (entry.http_status == null) return { kind: 'unknown', word: '응답 없음' };
  if (entry.http_status === 401) return { kind: 'refused', word: '비밀번호 필요' };
  return { kind: 'refused', word: '거절' };
}

// 알림 제목과 기록에 쓸 한 구절.
function commandPhrase(entry, label) {
  if (entry.ok) return `${label} 요청 접수`;
  if (entry.simulated) return `${label} 보내지 않음${devWord(' (모의)')}`;
  if (entry.http_status == null) return `${label} 결과 모름, 핑키 관제 응답 없음`;
  if (entry.http_status === 401) return `${label} 거절, SSH 비밀번호 필요`;
  return `${label} 거절`;
}

// 서버의 말을 그대로 쓰되, 보내지 않은 명령(모의 모드)은 모드 이름 없이 그렇다고만 말한다.
function commandMessage(entry) {
  return entry.simulated && !devMode ? '핑키 관제에 연결되기 전이라 보내지 않았어요' : entry.message || '';
}

const pinkyToastKinds = { ok: 'info', mock: 'warning', unknown: 'error', refused: 'error' };

// 그 핑키에 보낸 그 동작의 가장 최근 기록(log는 최근 것이 앞).
function lastCommandOf(pinkyId, action) {
  return pinkyScreen.view?.log.find((entry) => entry.action === action && entry.pinky_id === pinkyId);
}

// ---- 요약 ----

// 핑키 관제 연결 상태 한 줄: 읽기 실패(마지막 응답 시각과 함께), 모의 세계, 연결됨, 응답 없음.
function renderPinkyLink() {
  const view = pinkyScreen.view;
  let state;
  let title;
  let detail;
  if (pinkyScreen.loadError) {
    state = 'error';
    title = '핑키 상태를 읽지 못했어요';
    detail = `${pinkyScreen.loadError}${view ? `. 아래는 ${formatClock(view.checked_at)}에 받은 상태예요` : ''}`;
  } else if (!view) {
    return;
  } else {
    const checked = `${formatClock(view.checked_at)}에 확인했어요`;
    if (view.source === 'mock_world') {
      state = 'mock';
      title = devMode ? '모의 핑키를 보여 주고 있어요' : '핑키 관제에 연결하기 전이에요';
      detail = devMode
        ? `핑키 관제 주소가 없어 모의 세계의 핑키를 보여 주며, 명령은 보내지 않습니다. ${checked}`
        : `핑키 관제 주소가 설정되지 않아 명령은 보내지 않아요. ${checked}`;
    } else if (view.reachable) {
      state = 'ok';
      title = '핑키 관제와 연결돼 있어요';
      detail = checked;
    } else {
      state = 'error';
      title = '핑키 관제가 응답하지 않아요';
      detail = `${view.error || '응답이 없습니다'}. ${checked}`;
    }
  }
  pinkyLink.className = `pinky-link ${state}`;
  pinkyLinkTitle.innerHTML = `${escapeHtml(title)}${view?.source === 'mock_world' ? ' <span class="sim-tag">모의</span>' : ''}`;
  pinkyLinkDetail.textContent = detail;
  renderPinkyStats();
  renderPinkyPrepare();
}

// 핑키 관제는 세 핑키에 쓰는 SSH 비밀번호 하나를 메모리에만 두고, 갖고 있는지만 알려 준다(password_set).
const PREPARE_HINT = '준비 안 된 핑키만 한 대씩 차례로 준비하고, 끝나면 라이다로 각자 지도 위치를 찾아요. 못 찾은 핑키는 상세에서 후보를 골라 주세요.';

// '주행준비하고 위치 잡기' 버튼: 핑키 관제와 실제로 연결돼 있고 진행 중인 작업(busy)이 없을 때만 누를 수 있다.
// busy는 핑키 관제가 지금 작업 중인 대상이다('all'이면 차례로 준비 중, 아니면 그 핑키 id).
function renderPinkyPrepare() {
  const view = pinkyScreen.view;
  const live = Boolean(view && view.source !== 'mock_world' && view.reachable && !pinkyScreen.loadError);
  const busy = live ? view.busy : null;
  pinkyPasswordOpen.textContent = view?.password_set ? 'SSH 비밀번호 넣음 · 다시 보내기' : 'SSH 비밀번호 넣기';
  pinkyPrepareAll.disabled = !live || Boolean(busy) || pinkyScreen.prepareAllSending;
  pinkyPrepareAll.textContent = pinkyScreen.prepareAllSending ? '보내는 중…'
    : busy === 'all' ? '차례로 준비하는 중…'
      : busy ? `${pinkyName(busy)} 작업이 끝나면 누를 수 있어요` : '주행준비하고 위치 잡기';
  pinkyPrepareHint.textContent = live && view.password_set === false
    ? 'SSH 비밀번호가 아직 없어요. 누르면 먼저 물어보고, 세 대 모두 같은 비밀번호로 접속해요.'
    : PREPARE_HINT;
}

// 요약 숫자: 연결된 핑키, 주행 가능, 정지 적용 중.
function renderPinkyStats() {
  const pinkies = pinkyScreen.view?.pinkies;
  if (!pinkies) return;
  const connected = pinkies.filter((pinky) => pinky.connected).length;
  const ready = pinkies.filter((pinky) => pinky.connected && pinky.ready).length;
  const stopping = pinkies.filter((pinky) => pinky.stop_pending).length;
  setStat(pinkyStatConnected, connected, { total: pinkies.length, tone: connected < pinkies.length ? 'attention' : '' });
  setStat(pinkyStatReady, ready, { tone: ready ? '' : 'zero' });
  setStat(pinkyStatStopping, stopping, { tone: stopping ? 'danger' : 'zero' });
}

// ---- 간단한 카드 ----

// 배터리 숫자(20% 미만은 빨강)와 전압.
function batteryStatHtml(pinky) {
  const percent = pinky.battery_percent == null ? null : Math.round(pinky.battery_percent);
  const volts = pinky.battery_volts == null ? '' : ` ${Number(pinky.battery_volts).toFixed(2)} V`;
  const label = percent == null && !volts ? '배터리 정보 없음' : `배터리${volts}`;
  const tone = percent != null && percent < 20 ? ' danger' : '';
  return `<dl class="pinky-battery"><div class="stat${tone}"><dt>${escapeHtml(label)}</dt>
    <dd>${percent == null ? '–' : `${percent}<small>%</small>`}</dd></div></dl>`;
}

function readinessChipHtml(pinky) {
  if (!pinky.connected) return '<span class="ready-chip offline">연결 끊김</span>';
  const fallback = pinky.ready ? '주행 가능' : '주행 불가';
  const label = pinky.simulated && !devMode ? fallback : pinky.readiness_label || fallback;  // 모의는 "모의 핑키"라고 말한다
  return `<span class="ready-chip ${pinky.ready ? 'ready' : 'not-ready'}">${escapeHtml(label)}</span>`;
}

// 핑키가 단 ArUco 마커와, 도착 판정용 정차 위치가 등록됐는지.
function markerChipHtml(pinky) {
  const marker = pinky.marker;
  if (!marker) return '';
  return marker.registered
    ? `<span class="chip good">마커 ${escapeHtml(marker.marker_id)}</span>`
    : `<span class="chip attention" title="${escapeHtml(marker.problem || '')}">마커 ${escapeHtml(marker.marker_id)} 미등록</span>`;
}

// 이 핑키에 보낸 마지막 정지 요청과 결과 한 줄.
function lastStopHtml(pinkyId) {
  const lastStop = lastCommandOf(pinkyId, 'stop');
  if (!lastStop) return '';
  return `<p class="pinky-last-stop ${commandOutcome(lastStop).kind}" title="${escapeHtml(commandMessage(lastStop))}">
    <time datetime="${escapeHtml(lastStop.at)}">${escapeHtml(formatClock(lastStop.at))}</time> ${escapeHtml(commandPhrase(lastStop, '정지'))}</p>`;
}

// 핑키 카드: 단계, 칩, 배터리, [정지]·[자세히 보기]. 카드의 다른 곳을 눌러도 상세가 열린다.
function pinkyCardHtml(pinky) {
  const id = escapeHtml(pinky.pinky_id);
  const name = escapeHtml(pinky.name);
  const readiness = !pinky.connected ? 'offline' : pinky.ready ? 'ready' : 'not-ready';
  const sending = pinkyScreen.sending.has(`${pinky.pinky_id}:stop`);
  return `
    <article class="pinky-card ${readiness}" data-pinky="${id}" aria-label="핑키 ${name}">
      <header class="pinky-card-head">
        <h3>${pinkyBadge(pinky.pinky_id, { offline: !pinky.connected })}<span>${escapeHtml(pinky.robot_name || '핑키')}</span></h3>
        <div class="pinky-card-tags">${pinky.simulated ? '<span class="sim-tag">모의</span>' : ''}${readinessChipHtml(pinky)}</div>
      </header>
      <div class="pinky-body">
        <div class="pinky-phase">
          <strong>${escapeHtml(pinky.connected ? (pinky.phase_label || pinky.phase || '단계 모름') : '단계 알 수 없음')}</strong>
          <span class="pinky-chips">${markerChipHtml(pinky)}${pinky.stop_pending ? '<span class="ready-chip stopping">정지 적용 중</span>' : ''}</span>
        </div>
        ${batteryStatHtml(pinky)}
      </div>
      <div class="pinky-card-actions">
        <button type="button" class="pinky-stop" data-pinky-stop="${id}" aria-label="핑키 ${name} 정지">${sending ? '보내는 중…' : '정지'}</button>
        <button type="button" class="secondary" data-pinky-open="${id}">자세히 보기</button>
      </div>
      ${lastStopHtml(pinky.pinky_id)}
    </article>`;
}

function renderPinkyCards() {
  const view = pinkyScreen.view;
  if (!view) return;
  setHtmlIfChanged(pinkyCards, view.pinkies.length
    ? view.pinkies.map(pinkyCardHtml).join('')
    : '<p class="pinky-empty">핑키 관제가 응답하지 않아 핑키 상태를 모릅니다. 위 연결 상태를 확인하세요.</p>');
}

// ---- 작은 지도 ----

function renderSmallMap() {
  drawPinkyMap(pinkyMapSmall, pinkyMapSmallEmpty);
  const data = pinkyMapState.data;
  pinkyMapTag.innerHTML = data?.simulated ? '<span class="sim-tag">모의</span>' : '';
  pinkyMapOff.innerHTML = mapZoneNotesHtml();
  const note = mapNote();
  pinkyMapNote.hidden = !note;
  pinkyMapNote.textContent = note;
}

// ---- 명령 기록 ----

// 명령 기록 한 줄. 메시지가 'HTTP nnn' 그대로가 아닐 때만 상태 코드를 따로 붙인다.
function pinkyLogRowHtml(entry) {
  const outcome = commandOutcome(entry);
  const status = entry.http_status && entry.message !== `HTTP ${entry.http_status}` ? entry.http_status : null;
  const label = pinkyActionLabels[entry.action] || entry.action;
  const what = entry.pinky_id ? `${pinkyName(entry.pinky_id)} ${label}` : label;
  return `
    <li>
      <time datetime="${escapeHtml(entry.at)}">${escapeHtml(formatClock(entry.at))}</time>
      <span class="pinky-log-what">${escapeHtml(what)}</span>
      <span class="result-chip ${outcome.kind}">${outcome.word}</span>
      <span class="pinky-log-message">${escapeHtml(commandMessage(entry))}${status ? ` <small>HTTP ${escapeHtml(status)}</small>` : ''}</span>
    </li>`;
}

function renderPinkyLog() {
  const log = pinkyScreen.view?.log;
  if (!log) return;
  pinkyLogCount.textContent = `${log.length}건`;
  setHtmlIfChanged(pinkyLog, log.length
    ? log.map(pinkyLogRowHtml).join('')
    : '<li class="pinky-log-empty">보낸 명령이 없습니다.</li>');
}

// 핑키 화면 전체를 그리고 'pinkyview' 이벤트를 보내 상세·전체 주행 테스트·종합 화면이 따라 그리게 한다.
function renderPinkyScreen() {
  renderPinkyLink();
  if (pinkyScreen.view) renderRouteStrip(pinkyScreen.view);
  renderPinkyCards();
  renderPinkyLog();
  document.dispatchEvent(new CustomEvent('pinkyview'));
}

// ---- 읽기 ----

// 탭이 보이고, 핑키·종합 뷰이거나 상세 패널이 열려 있을 때만 읽는다.
function pinkyOnScreen() {
  return !document.hidden && (['pinky', 'dashboard'].includes(activeView) || !pinkySheet.hidden);
}

// GET /api/pinky(1.5초마다, 명령 직후에는 force로 바로). 응답이 늦으면 10초에 끊는다.
async function refreshPinky({ force = false } = {}) {
  if (!force && (!pinkyOnScreen() || pinkyScreen.polling)) return;
  const seq = ++pinkyScreen.requestSeq;
  pinkyScreen.polling = true;
  let view = null;
  let loadError = null;
  try {
    const response = await fetch('/api/pinky', { cache: 'no-store', signal: AbortSignal.timeout(PINKY_READ_TIMEOUT_MS) });
    const result = await response.json();
    if (response.ok) view = result;
    else loadError = apiErrorMessage(result, '핑키 상태를 불러오지 못했습니다');
  } catch {
    loadError = '핑키 상태를 가져오지 못했습니다';
  }
  if (seq !== pinkyScreen.requestSeq) return;  // 더 새 읽기를 보냈다. 그 응답을 쓰고 polling 표시도 그쪽이 끝낸다
  pinkyScreen.polling = false;
  if (view) pinkyScreen.view = view;
  pinkyScreen.loadError = loadError;
  renderPinkyScreen();
}

// ---- 명령 ----

// POST하고 상태 코드와 JSON 본문을 함께 돌려준다. 본문이 없으면 JSON 머리글도 붙이지 않는다.
async function postPinky(url, body) {
  const options = { method: 'POST' };
  if (body) Object.assign(options, { headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const response = await fetch(url, options);
  return { status: response.status, result: await response.json() };
}

// 핑키 한 대에 명령 하나를 보내고(POST /api/pinky/{id}/{path}) 돌아온 결과를 알린다. 기록 항목을 돌려준다(없으면 null).
// 401이면 비밀번호 창을 열고, 같은 명령을 다시 보낼 수 있게 retry를 건넨다(저절로 다시 보내지는 않는다).
async function sendPinkyCommand(pinkyId, action, { path, body = {}, label }) {
  const key = `${pinkyId}:${action}`;
  const name = pinkyName(pinkyId);
  const retry = () => sendPinkyCommand(pinkyId, action, { path, body, label });
  pinkyScreen.sending.add(key);
  renderPinkyCards();
  document.dispatchEvent(new CustomEvent('pinkyview'));
  try {
    const { result } = await postPinky(`/api/pinky/${encodeURIComponent(pinkyId)}/${path}`, body);
    const entry = result.command;
    if (!entry) {
      showToast(`${name} ${label} 거절`, apiErrorMessage(result, '관제 서버가 요청을 받지 않았습니다'), 'error', PINKY_COMMAND_TOAST_MS);
      return null;
    }
    const phrase = commandPhrase(entry, label);
    const message = commandMessage(entry);
    showToast(`${name} ${phrase}`, message && message !== phrase ? message : '',
      pinkyToastKinds[commandOutcome(entry).kind], PINKY_COMMAND_TOAST_MS);
    if (entry.http_status === 401) openPasswordPrompt({ reason: `${name} ${label} 요청이 거절됐어요.`, retryLabel: `${name} ${label}`, retry });
    return entry;
  } catch {
    // 응답이 없다고 거절된 것은 아니다: 요청이 들어갔을 수 있으니 다시 보내지 말고 확인해 달라고 한다.
    showToast(`${name} ${label} 결과 모름`, '관제 서버의 답을 받지 못했습니다. 핑키를 현장에서 확인하세요.', 'error', PINKY_COMMAND_TOAST_MS);
    return null;
  } finally {
    pinkyScreen.sending.delete(key);
    refreshPinky({ force: true });
    refreshPinkyMap({ force: true });
  }
}

// 카드·상세의 [정지]: POST /api/pinky/{id}/stop. 확인 창 없이 바로 보낸다.
function stopPinky(pinkyId) {
  return sendPinkyCommand(pinkyId, 'stop', { path: 'stop', body: null, label: '정지' });
}

// 전체 정지 결과 알림의 제목과 종류.
function stopAllSummary(commands, failed) {
  const accepted = commands.filter((entry) => entry.ok).length;
  if (failed) return { title: '전체 정지, 일부 요청 실패', kind: 'error' };
  if (commands.length && accepted === commands.length) return { title: '전체 정지 요청 접수', kind: 'info' };
  if (commands.length && commands.every((entry) => entry.simulated)) {
    return { title: `핑키 정지 요청은 보내지 않음${devWord(' (모의)')}`, kind: 'warning' };
  }
  return { title: `전체 정지, 핑키 ${commands.length}대 중 ${accepted}대 접수`, kind: 'error' };
}

// 핑키별 정지 결과 한 줄.
function stopAllLine(entry) {
  const phrase = commandPhrase(entry, '정지');
  const showMessage = !entry.ok && !entry.simulated && entry.http_status !== 401 && entry.message;
  return `${pinkyName(entry.pinky_id)} ${phrase}${showMessage ? `: ${commandMessage(entry)}` : ''}`;
}

// /api/stop-all 응답 errors[].part의 이름.
const STOP_PART_NAMES = { arm_sequence: '팔 연속 동작', arm_commands: '팔 명령', policy_run: '정책 실행', pinkies: '핑키' };

// 전체 정지 응답(POST /api/stop-all)이 팔에 대해 말하는 것. 취소는 요청이다: 명령은 팔 워커가 스스로 끝내므로
// 문구는 "요청"이라고만 하고 "멈춤"이라고 하지 않는다.
function armStopLines(result) {
  const lines = [];
  const commands = result.arm_commands || [];
  if (commands.length) lines.push(`팔 명령 ${commands.length}개 취소 요청`);
  if (result.arm_sequence) lines.push('팔 연속 동작 취소 요청');
  if (result.policy_run) lines.push('정책 실행 중단 신호 보냄');
  if (!lines.length) lines.push('팔: 진행 중인 동작 없음');
  for (const error of result.errors || []) lines.push(`${STOP_PART_NAMES[error.part] || error.part} 요청 실패: ${error.message}`);
  return lines;
}

// 큰 빨간 버튼: 팔과 모든 핑키를 요청 하나로. /api/stop-all이 없던 예전 서버(404)에도 핑키 정지는 보내므로,
// 버튼이 예전보다 덜 하는 일은 없다.
async function requestStopAll() {
  const all = await postPinky('/api/stop-all');
  if (all.status !== 404) return { result: all.result, commands: all.result.pinkies || [], withArms: true };
  const pinkyOnly = await postPinky('/api/pinky/stop-all');
  return { result: pinkyOnly.result, commands: pinkyOnly.result.commands || [], withArms: false };
}

// 머리글의 [전체 정지]. 연달아 눌러도 모두 보내고, 마지막 요청이 끝날 때 버튼 글자를 되돌린다.
async function stopEverything() {
  pinkyScreen.stopAllSending += 1;
  pinkyStopAll.setAttribute('aria-busy', 'true');
  stopAllLabel.textContent = '정지 보내는 중…';
  try {
    const { result, commands, withArms } = await requestStopAll();
    const failed = withArms && (result.errors || []).length > 0;
    const { title, kind } = stopAllSummary(commands, failed);
    const mockOnly = commands.length && commands.every((entry) => entry.simulated);
    const pinkyLines = mockOnly ? [commandMessage(commands[0])] : commands.map(stopAllLine);
    const armLines = withArms ? armStopLines(result) : ['팔에는 보내지 못했어요 (이 관제 서버는 핑키 정지만 받아요)'];
    const detail = [...pinkyLines, ...armLines].filter(Boolean).join('\n');
    showToast(title, detail || apiErrorMessage(result, '관제 서버가 요청을 받지 않았습니다'), withArms ? kind : 'error', PINKY_COMMAND_TOAST_MS);
    if (commands.some((entry) => entry.http_status === 401)) {
      openPasswordPrompt({ reason: '정지 요청 일부가 거절됐어요.', retryLabel: '전체 정지', retry: stopEverything });
    }
  } catch {
    showToast('전체 정지 요청 결과 모름', '관제 서버의 답을 받지 못했습니다. 핑키와 팔을 현장에서 확인하세요.', 'error', PINKY_COMMAND_TOAST_MS);
  } finally {
    pinkyScreen.stopAllSending -= 1;
    if (!pinkyScreen.stopAllSending) {
      pinkyStopAll.removeAttribute('aria-busy');
      stopAllLabel.textContent = '전체 정지';
    }
    refreshPinky({ force: true });
  }
}

// 핑키 관제가 아직 준비하지 않은 핑키마다 차례로 주행준비를 한다. 각각 끝에 AMCL에 자기 주차 자세를 넣는다.
// 접수는 핑키 관제가 시작했다는 뜻이고, 핑키별 진행은 각 카드에 나타난다.
async function prepareAllPinkies({ afterPassword = false } = {}) {
  const label = '주행준비하고 위치 잡기';
  const retry = () => prepareAllPinkies({ afterPassword: true });
  // 비밀번호를 막 넣은 직후에는 마지막 상태가 아직 "없음"일 수 있다. 정말 없으면 핑키 관제가 401로 답한다.
  if (!afterPassword && pinkyScreen.view?.password_set === false) {
    openPasswordPrompt({ reason: '주행준비는 핑키에 SSH로 접속해서 해요.', retryLabel: label, retry });
    return;
  }
  pinkyScreen.prepareAllSending = true;
  renderPinkyPrepare();
  try {
    const { result } = await postPinky('/api/pinky/prepare-all', { map: 'map-proto.yaml' });
    const entry = result.command;
    if (!entry) {
      showToast(`${label} 거절`, apiErrorMessage(result, '관제 서버가 요청을 받지 않았습니다'), 'error', PINKY_COMMAND_TOAST_MS);
      return;
    }
    showToast(commandPhrase(entry, label), commandMessage(entry), pinkyToastKinds[commandOutcome(entry).kind], PINKY_COMMAND_TOAST_MS);
    if (entry.http_status === 401) openPasswordPrompt({ reason: `${label} 요청이 거절됐어요.`, retryLabel: label, retry });
  } catch {
    showToast(`${label} 결과 모름`, '관제 서버의 답을 받지 못했습니다. 핑키 카드에서 준비 상태를 확인하세요.', 'error', PINKY_COMMAND_TOAST_MS);
  } finally {
    pinkyScreen.prepareAllSending = false;
    refreshPinky({ force: true });
  }
}

// ---- SSH 비밀번호: 핑키 관제가 필요로 할 때만 묻는다 ----

function setPasswordMessage(text, tone = '') {
  pinkyPasswordMessage.textContent = text;
  pinkyPasswordMessage.className = `pinky-password-message${tone ? ` ${tone}` : ''}`;
}

// 비밀번호 창을 연다. retry가 있으면 '필요해요' 제목으로 열고, 보낸 뒤 거절됐던 명령의 다시 보내기 버튼을 내민다.
function openPasswordPrompt({ reason = '', retryLabel = '', retry = null } = {}) {
  pinkyScreen.retry = retry ? { label: retryLabel, run: retry } : null;
  passwordPromptTitle.textContent = retry ? 'SSH 비밀번호가 필요해요' : 'SSH 비밀번호 보내기';
  passwordPromptReason.textContent = `${reason ? `${reason} ` : ''}핑키 관제가 핑키에 SSH로 접속할 때 씁니다. 이 관제는 저장하지 않고 그대로 넘깁니다.`;
  passwordRetry.hidden = true;
  setPasswordMessage('');
  openOverlay(passwordPrompt);
}

// POST /api/pinky/credentials. 이 관제는 저장하지 않고 핑키 관제로 그대로 넘긴다. 200이면 입력칸을 바로 비운다.
async function sendPinkyPassword(event) {
  event.preventDefault();
  if (!pinkyPassword.value) {
    setPasswordMessage('비밀번호를 입력하세요.', 'error');
    return;
  }
  const button = pinkyPasswordForm.querySelector('button[type="submit"]');
  button.disabled = true;
  setPasswordMessage('핑키 관제에 보내는 중…');
  try {
    const { status, result } = await postPinky('/api/pinky/credentials', { password: pinkyPassword.value });
    const entry = result.command;
    if (status === 200) {
      pinkyPassword.value = '';
      setPasswordMessage('핑키 관제에 전달했어요 (관제는 저장하지 않아요)', 'ok');
      // 거절됐던 명령은 다시 보내기로 내밀 뿐, 저절로 다시 보내지 않는다.
      if (pinkyScreen.retry) {
        passwordRetry.textContent = `${pinkyScreen.retry.label} 다시 보내기`;
        passwordRetry.hidden = false;
        passwordRetry.focus();
      }
    } else if (entry?.simulated) {
      setPasswordMessage(commandMessage(entry), 'warning');
    } else {
      setPasswordMessage(`전달하지 못했어요: ${entry?.message || apiErrorMessage(result, '관제 서버가 요청을 받지 않았습니다')}`, 'error');
    }
  } catch {
    setPasswordMessage('관제 서버의 답을 받지 못해 전달됐는지 몰라요. 다시 보내도 됩니다.', 'error');
  } finally {
    button.disabled = false;
    refreshPinky({ force: true });
  }
}

passwordRetry.addEventListener('click', () => {
  const retry = pinkyScreen.retry;
  pinkyScreen.retry = null;
  closeOverlay(passwordPrompt);
  retry?.run();
});
// 입력한 비밀번호는 창이 닫히면 남지 않는다.
passwordPrompt.addEventListener('overlayclose', () => { pinkyPassword.value = ''; pinkyScreen.retry = null; });

// ---- 연결 ----

pinkyStopAll.addEventListener('click', stopEverything);
pinkyPrepareAll.addEventListener('click', () => prepareAllPinkies());
pinkyCards.addEventListener('click', (event) => {
  const stop = event.target.closest('[data-pinky-stop]');
  if (stop) {
    stopPinky(stop.dataset.pinkyStop);
    return;
  }
  const card = event.target.closest('[data-pinky]');
  if (card) openPinkyDetail(card.dataset.pinky, event.target.closest('button') || card.querySelector('[data-pinky-open]'));
});
document.querySelector('#pinkyMapOpen').addEventListener('click', (event) => openMapViewer(event.currentTarget));
pinkyPasswordOpen.addEventListener('click', () => openPasswordPrompt());
pinkyPasswordForm.addEventListener('submit', sendPinkyPassword);
// 지도를 읽을 때마다 작은 지도를 다시 그리고, 뷰 전환·탭 복귀 때는 바로 읽고, 1.5초마다 읽는다(화면에 없으면 건너뜀).
document.addEventListener('pinkymap', renderSmallMap);
document.addEventListener('viewchange', () => refreshPinky());
document.addEventListener('visibilitychange', () => refreshPinky());
window.setInterval(refreshPinky, PINKY_POLL_MS);
refreshPinky();
