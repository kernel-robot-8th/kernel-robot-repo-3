// 핑키 화면: the three Pinkys as the fleet dashboard reports them, a stop per Pinky, a stop for all of them in the
// header of every view, and the SSH password the dashboard needs before it can reach a Pinky.
//
// Loaded after app.js and uses its helpers: escapeHtml, apiErrorMessage, formatClock, showToast, pinkyNames.
//
// "Accepted" (202) only means the fleet dashboard took the stop request; whether the Pinky stopped shows up later in
// its card (safety monitor, stop pending). Stops go out at once without a confirm dialog, and nothing here resends a
// command by itself. The password is read from the field when it is sent and goes nowhere else.

const PINKY_POLL_MS = 1500;
const PINKY_READ_TIMEOUT_MS = 10000;  // the server asks the dashboard with a 3 s timeout; a hung read must not stop polling
const PINKY_STOP_TOAST_MS = 12000;

const pinkyView = document.querySelector('#view-pinky');
const pinkyCards = document.querySelector('#pinkyCards');
const pinkyCount = document.querySelector('#pinkyCount');
const pinkyLink = document.querySelector('#pinkyLink');
const pinkyLinkTitle = document.querySelector('#pinkyLinkTitle');
const pinkyLinkDetail = document.querySelector('#pinkyLinkDetail');
const pinkyLog = document.querySelector('#pinkyLog');
const pinkyPasswordForm = document.querySelector('#pinkyPasswordForm');
const pinkyPassword = document.querySelector('#pinkyPassword');
const pinkyPasswordMessage = document.querySelector('#pinkyPasswordMessage');
const pinkyStopAll = document.querySelector('#pinkyStopAll');

const pinkyScreen = {
  view: null,              // last GET /api/pinky answer
  loadError: null,         // why the last GET got no answer; the cards then show the last answer that came
  requestSeq: 0,           // only the newest GET answer is drawn
  polling: false,          // a poll is in flight; the timer skips instead of piling up requests
  sending: new Set(),      // Pinky ids with a stop request in flight
  stopAllSending: 0,
  rendered: new WeakMap(), // element -> the HTML it shows, so an unchanged poll leaves the buttons alone
};

const pinkyRouteLabels = {
  starting: '경로 시작 중', running: '주행 중', complete: '경로 끝', failed: '경로 중단',
  cancel_requested: '경로 취소 요청 중', cancelled: '경로 취소됨',
};
const pinkyActionLabels = { stop: '정지', password: '비밀번호' };

function setHtmlIfChanged(element, html) {
  if (pinkyScreen.rendered.get(element) === html) return;
  element.innerHTML = html;
  pinkyScreen.rendered.set(element, html);
}

function pinkyName(pinkyId) {
  const pinky = pinkyScreen.view?.pinkies.find((item) => item.pinky_id === pinkyId);
  return pinky?.name || pinkyNames[pinkyId] || pinkyId;
}

// A dashboard answer in the screen's words. Accepted is not done: the words never say the Pinky stopped.
function commandOutcome(entry) {
  if (entry.ok) return { kind: 'ok', word: '접수' };
  if (entry.simulated) return { kind: 'mock', word: '모의' };
  if (entry.http_status == null) return { kind: 'unknown', word: '응답 없음' };
  return { kind: 'refused', word: '거절' };
}

function stopPhrase(entry) {
  if (entry.ok) return '정지 요청 접수';
  if (entry.simulated) return '정지 요청 보내지 않음 (모의)';
  if (entry.http_status == null) return '정지 요청 결과 모름 · 핑키 관제 응답 없음';
  if (entry.http_status === 401) return '정지 요청 거절 · SSH 비밀번호 필요';
  return '정지 요청 거절';
}

const pinkyToastKinds = { ok: 'info', mock: 'warning', unknown: 'error', refused: 'error' };

// ---- 표시 ----

function batteryText(pinky) {
  const parts = [];
  if (pinky.battery_percent != null) parts.push(`${Math.round(pinky.battery_percent)}%`);
  if (pinky.battery_volts != null) parts.push(`${Number(pinky.battery_volts).toFixed(2)} V`);
  return parts.join(' · ');
}

function routeText(route) {
  if (!route?.status) return route?.current_label || '경로 없음';
  const parts = [pinkyRouteLabels[route.status] || route.status];
  if (route.current_label) parts.push(`웨이포인트 ${route.current_label}`);
  if (route.total) parts.push(`${route.completed ?? 0}/${route.total}`);
  return parts.join(' · ');
}

function factHtml(label, value, tone = '') {
  return `<div><dt>${label}</dt><dd${tone ? ` class="${tone}"` : ''}>${escapeHtml(value)}</dd></div>`;
}

function lastStopOf(pinkyId) {
  return pinkyScreen.view?.log.find((entry) => entry.action === 'stop' && entry.pinky_id === pinkyId);
}

function pinkyCardHtml(pinky) {
  const id = escapeHtml(pinky.pinky_id);
  const name = escapeHtml(pinky.name);
  const readiness = !pinky.connected ? 'offline' : pinky.ready ? 'ready' : 'not-ready';
  const sensorTotal = Object.keys(pinky.sensors || {}).length;
  const battery = batteryText(pinky);
  const message = pinky.message && pinky.message !== pinky.phase_label ? pinky.message : '';
  const lastStop = lastStopOf(pinky.pinky_id);
  const lastStopOutcome = lastStop && commandOutcome(lastStop);
  const sending = pinkyScreen.sending.has(pinky.pinky_id);
  return `
    <article class="pinky-card ${readiness}" data-pinky="${id}" aria-label="핑키 ${name}">
      <header class="pinky-card-head">
        <div>
          <h3><span class="pinky-badge pinky-${id}">${name}</span>${escapeHtml(pinky.robot_name || '핑키')}</h3>
          <small>${escapeHtml(pinky.ip || (pinky.simulated ? '모의 세계' : 'IP 모름'))}</small>
        </div>
        <div class="pinky-card-tags">
          ${pinky.simulated ? '<span class="sim-tag">모의</span>' : ''}
          <span class="ready-chip ${readiness}">${escapeHtml(pinky.readiness_label || (pinky.ready ? '주행 가능' : '주행 불가'))}</span>
        </div>
      </header>
      <div class="pinky-phase">
        <strong>${escapeHtml(pinky.connected ? (pinky.phase_label || pinky.phase || '단계 모름') : '단계 알 수 없음')}</strong>
        ${pinky.stop_pending ? '<span class="ready-chip stopping">정지 적용 중</span>' : ''}
      </div>
      <dl class="pinky-facts">
        ${factHtml('배터리', battery || '배터리 정보 없음', battery ? '' : 'muted')}
        ${pinky.missing.length
          ? factHtml('센서', `${pinky.missing.length}/${sensorTotal} 안 들어옴 · ${pinky.missing.join(', ')}`, 'bad')
          : factHtml('센서', '센서 모두 수신', 'good')}
        ${factHtml('경로', routeText(pinky.route))}
        ${factHtml('안전 감시', pinky.safety_label || '정보 없음', pinky.safety_label ? '' : 'muted')}
      </dl>
      ${message ? `<p class="pinky-message">${escapeHtml(message)}</p>` : ''}
      <div class="pinky-card-actions">
        <button type="button" class="pinky-stop" data-pinky-stop="${id}" aria-label="핑키 ${name} 정지">${sending ? '보내는 중…' : '정지'}</button>
        ${lastStop ? `<span class="pinky-last-stop ${lastStopOutcome.kind}" title="${escapeHtml(lastStop.message || '')}">
          <time datetime="${escapeHtml(lastStop.at)}">${escapeHtml(formatClock(lastStop.at))}</time> ${escapeHtml(stopPhrase(lastStop))}</span>` : ''}
      </div>
    </article>`;
}

function renderPinkyLink() {
  const view = pinkyScreen.view;
  let state;
  let title;
  let detail;
  if (pinkyScreen.loadError) {
    state = 'error';
    title = '핑키 상태를 읽지 못함';
    detail = `${pinkyScreen.loadError}${view ? ` · 아래는 ${formatClock(view.checked_at)}에 받은 상태` : ''}`;
  } else if (!view) {
    return;
  } else {
    const connected = view.pinkies.filter((pinky) => pinky.connected).length;
    const ready = view.pinkies.filter((pinky) => pinky.connected && pinky.ready).length;
    const summary = `연결 ${connected}/${view.pinkies.length}대 · 주행 가능 ${ready}대 · 확인 ${formatClock(view.checked_at)}`;
    if (view.source === 'mock_world') {
      state = 'mock';
      title = '모의 핑키';
      detail = `핑키 관제 주소가 없어 모의 세계의 핑키를 보여 주며, 명령은 보내지 않습니다 · ${summary}`;
    } else if (view.reachable) {
      state = 'ok';
      title = '핑키 관제 연결됨';
      detail = summary;
    } else {
      state = 'error';
      title = '핑키 관제 응답 없음';
      detail = `${view.error || '응답이 없습니다'} · 확인 ${formatClock(view.checked_at)}`;
    }
  }
  pinkyLink.className = `pinky-link ${state}`;
  pinkyLinkTitle.innerHTML = `${escapeHtml(title)}${view?.source === 'mock_world' ? ' <span class="sim-tag">모의</span>' : ''}`;
  pinkyLinkDetail.textContent = detail;
}

function renderPinkyCards() {
  const view = pinkyScreen.view;
  if (!view) return;
  pinkyCount.textContent = view.pinkies.length ? `${view.pinkies.length}대` : '';
  setHtmlIfChanged(pinkyCards, view.pinkies.length
    ? view.pinkies.map(pinkyCardHtml).join('')
    : '<p class="pinky-empty">핑키 관제가 응답하지 않아 핑키 상태를 모릅니다. 위 연결 상태를 확인하세요.</p>');
}

function pinkyLogRowHtml(entry) {
  const outcome = commandOutcome(entry);
  const status = entry.http_status && entry.message !== `HTTP ${entry.http_status}` ? entry.http_status : null;
  const what = entry.action === 'stop'
    ? `${pinkyName(entry.pinky_id)} ${pinkyActionLabels.stop}`
    : pinkyActionLabels[entry.action] || entry.action;
  return `
    <li>
      <time datetime="${escapeHtml(entry.at)}">${escapeHtml(formatClock(entry.at))}</time>
      <span class="pinky-log-what">${escapeHtml(what)}</span>
      <span class="result-chip ${outcome.kind}">${outcome.word}</span>
      <span class="pinky-log-message">${escapeHtml(entry.message || '')}${status ? ` <small>HTTP ${escapeHtml(status)}</small>` : ''}</span>
    </li>`;
}

function renderPinkyLog() {
  const log = pinkyScreen.view?.log;
  if (!log) return;
  setHtmlIfChanged(pinkyLog, log.length
    ? log.map(pinkyLogRowHtml).join('')
    : '<li class="pinky-log-empty">보낸 명령이 없습니다.</li>');
}

function renderPinkyScreen() {
  renderPinkyLink();
  renderPinkyCards();
  renderPinkyLog();
}

// ---- 읽기 ----

// The timer polls only while the view is on screen; commands ask for a fresh look right after their answer.
async function refreshPinky({ force = false } = {}) {
  if (!force && (document.hidden || pinkyView.hidden || pinkyScreen.polling)) return;
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
  if (seq !== pinkyScreen.requestSeq) return;  // a newer read was sent; its answer wins and ends the polling flag
  pinkyScreen.polling = false;
  if (view) pinkyScreen.view = view;
  pinkyScreen.loadError = loadError;
  renderPinkyScreen();
}

// ---- 명령 ----

async function postPinky(url, body) {
  const options = { method: 'POST' };
  if (body) Object.assign(options, { headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const response = await fetch(url, options);
  return { status: response.status, result: await response.json() };
}

async function stopPinky(pinkyId) {
  const name = pinkyName(pinkyId);
  pinkyScreen.sending.add(pinkyId);
  renderPinkyCards();
  try {
    const { result } = await postPinky(`/api/pinky/${encodeURIComponent(pinkyId)}/stop`);
    const entry = result.command;
    if (entry) {
      const phrase = stopPhrase(entry);
      showToast(`${name} ${phrase}`, entry.message && entry.message !== phrase ? entry.message : '',
        pinkyToastKinds[commandOutcome(entry).kind], PINKY_STOP_TOAST_MS);
    } else {
      showToast(`${name} 정지 요청 거절`, apiErrorMessage(result, '관제 서버가 요청을 받지 않았습니다'), 'error', PINKY_STOP_TOAST_MS);
    }
  } catch {
    // No answer is not a refusal: the request may have gone through, so ask for a look instead of resending.
    showToast(`${name} 정지 요청 결과 모름`, '관제 서버의 답을 받지 못했습니다. 핑키를 현장에서 확인하세요.', 'error', PINKY_STOP_TOAST_MS);
  } finally {
    pinkyScreen.sending.delete(pinkyId);
    renderPinkyCards();
    refreshPinky({ force: true });
  }
}

function stopAllSummary(commands) {
  const accepted = commands.filter((entry) => entry.ok).length;
  if (commands.length && accepted === commands.length) return { title: '핑키 전체 정지 요청 접수', kind: 'info' };
  if (commands.length && commands.every((entry) => entry.simulated)) {
    return { title: '핑키 전체 정지 요청 보내지 않음 (모의)', kind: 'warning' };
  }
  return { title: `핑키 전체 정지 · ${commands.length}대 중 ${accepted}대 접수`, kind: 'error' };
}

function stopAllLine(entry) {
  const phrase = stopPhrase(entry);
  const showMessage = !entry.ok && !entry.simulated && entry.http_status !== 401 && entry.message;
  return `${pinkyName(entry.pinky_id)} ${phrase}${showMessage ? ` · ${entry.message}` : ''}`;
}

async function stopAllPinkies() {
  pinkyScreen.stopAllSending += 1;
  pinkyStopAll.setAttribute('aria-busy', 'true');
  pinkyStopAll.textContent = '정지 요청 보내는 중…';
  try {
    const { result } = await postPinky('/api/pinky/stop-all');
    const commands = result.commands || [];
    const { title, kind } = stopAllSummary(commands);
    const mockOnly = commands.length && commands.every((entry) => entry.simulated);
    const detail = mockOnly ? commands[0].message : commands.map(stopAllLine).join('\n');
    showToast(title, detail || apiErrorMessage(result, '관제 서버가 요청을 받지 않았습니다'), kind, PINKY_STOP_TOAST_MS);
  } catch {
    showToast('핑키 전체 정지 요청 결과 모름', '관제 서버의 답을 받지 못했습니다. 핑키를 현장에서 확인하세요.', 'error', PINKY_STOP_TOAST_MS);
  } finally {
    pinkyScreen.stopAllSending -= 1;
    if (!pinkyScreen.stopAllSending) {
      pinkyStopAll.removeAttribute('aria-busy');
      pinkyStopAll.textContent = '핑키 전체 정지';
    }
    refreshPinky({ force: true });
  }
}

function setPasswordMessage(text, tone = '') {
  pinkyPasswordMessage.textContent = text;
  pinkyPasswordMessage.className = `pinky-password-message${tone ? ` ${tone}` : ''}`;
}

async function sendPinkyPassword(event) {
  event.preventDefault();
  if (!pinkyPassword.value) return;
  const button = pinkyPasswordForm.querySelector('button[type="submit"]');
  button.disabled = true;
  setPasswordMessage('핑키 관제에 보내는 중…');
  try {
    const { status, result } = await postPinky('/api/pinky/credentials', { password: pinkyPassword.value });
    const entry = result.command;
    if (status === 200) {
      pinkyPassword.value = '';
      setPasswordMessage('핑키 관제에 전달했습니다 (관제는 저장하지 않음)', 'ok');
    } else if (entry?.simulated) {
      setPasswordMessage(entry.message, 'warning');
    } else {
      setPasswordMessage(`전달하지 못했습니다 · ${entry?.message || apiErrorMessage(result, '관제 서버가 요청을 받지 않았습니다')}`, 'error');
    }
  } catch {
    setPasswordMessage('관제 서버의 답을 받지 못해 전달됐는지 모릅니다. 다시 보내도 됩니다.', 'error');
  } finally {
    button.disabled = false;
    refreshPinky({ force: true });
  }
}

pinkyStopAll.addEventListener('click', stopAllPinkies);
pinkyCards.addEventListener('click', (event) => {
  const button = event.target.closest('[data-pinky-stop]');
  if (button) stopPinky(button.dataset.pinkyStop);
});
pinkyPasswordForm.addEventListener('submit', sendPinkyPassword);
// app.js shows the view on the same click first, so this look already counts as "on screen".
document.querySelector('.nav-item[data-view="pinky"]').addEventListener('click', () => refreshPinky());
document.addEventListener('visibilitychange', () => refreshPinky());
window.setInterval(refreshPinky, PINKY_POLL_MS);
