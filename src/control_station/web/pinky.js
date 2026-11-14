// 핑키 화면: the three Pinkys through our server (the fleet dashboard itself is no longer opened by operators).
// A calm summary first: the link to the fleet dashboard, a compact card per Pinky with a stop, the small map and the
// route strip. Tapping a card opens its detail (pinky-detail.js) with the sensors and the operator actions.
//
// Loaded after app.js (escapeHtml, apiErrorMessage, formatClock, showToast, setStat, pinkyBadge, pinkyNames,
// activeView, openOverlay, closeOverlay), route-strip.js and pinky-map.js.
//
// Every command answers {command: {ok, http_status, simulated, message}}. "Accepted" only means the fleet dashboard
// took the request; whether the Pinky did it shows up later in its state. 401 means the dashboard needs the SSH
// password: the prompt opens and the operator may send the command again. Nothing is ever resent by itself, and
// the password goes from the field to the server and nowhere else.

const PINKY_POLL_MS = 1500;
const PINKY_READ_TIMEOUT_MS = 10000;  // the server asks the dashboard with a 3 s timeout; a hung read must not stop polling
const PINKY_COMMAND_TOAST_MS = 12000;

const pinkyCards = document.querySelector('#pinkyCards');
const pinkyLink = document.querySelector('#pinkyLink');
const pinkyLinkTitle = document.querySelector('#pinkyLinkTitle');
const pinkyLinkDetail = document.querySelector('#pinkyLinkDetail');
const pinkyLog = document.querySelector('#pinkyLog');
const pinkyLogCount = document.querySelector('#pinkyLogCount');
const pinkyStopAll = document.querySelector('#pinkyStopAll');
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
  view: null,              // last GET /api/pinky answer
  loadError: null,         // why the last GET got no answer; the screen then shows the last answer that came
  requestSeq: 0,           // only the newest GET answer is drawn
  polling: false,          // a read is in flight; the timer skips instead of piling up requests
  sending: new Set(),      // `${pinky id}:${action}` with a request in flight
  stopAllSending: 0,
  retry: null,             // { label, run } offered after the password is accepted
  rendered: new WeakMap(), // element -> the HTML it shows, so an unchanged poll leaves the buttons alone
};

const pinkyRouteLabels = {
  starting: '경로 시작 중', running: '주행 중', complete: '경로 끝', failed: '경로 중단',
  cancel_requested: '경로 취소 요청 중', cancelled: '경로 취소됨',
};
// The server's log names each action; the screen uses these words for them.
const pinkyActionLabels = {
  stop: '정지', password: '비밀번호', prepare: '주행준비', initial_pose: '위치 재설정', nav_restart: 'Nav2 복구',
  waypoint: '웨이포인트 기록', waypoint_delete: '웨이포인트 삭제',
};

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

// ---- the dashboard's answer in the screen's words. Accepted is not done: nothing here says the Pinky did it. ----

function commandOutcome(entry) {
  if (entry.ok) return { kind: 'ok', word: '접수' };
  if (entry.simulated) return { kind: 'mock', word: '모의' };
  if (entry.http_status == null) return { kind: 'unknown', word: '응답 없음' };
  if (entry.http_status === 401) return { kind: 'refused', word: '비밀번호 필요' };
  return { kind: 'refused', word: '거절' };
}

function commandPhrase(entry, label) {
  if (entry.ok) return `${label} 요청 접수`;
  if (entry.simulated) return `${label} 보내지 않음 (모의)`;
  if (entry.http_status == null) return `${label} 결과 모름, 핑키 관제 응답 없음`;
  if (entry.http_status === 401) return `${label} 거절, SSH 비밀번호 필요`;
  return `${label} 거절`;
}

const pinkyToastKinds = { ok: 'info', mock: 'warning', unknown: 'error', refused: 'error' };

function lastCommandOf(pinkyId, action) {
  return pinkyScreen.view?.log.find((entry) => entry.action === action && entry.pinky_id === pinkyId);
}

// ---- summary ----

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
      title = '모의 핑키를 보여 주고 있어요';
      detail = `핑키 관제 주소가 없어 모의 세계의 핑키를 보여 주며, 명령은 보내지 않습니다. ${checked}`;
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
}

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

// ---- compact cards ----

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
  return `<span class="ready-chip ${pinky.ready ? 'ready' : 'not-ready'}">${escapeHtml(pinky.readiness_label || (pinky.ready ? '주행 가능' : '주행 불가'))}</span>`;
}

// The ArUco marker the Pinky carries, and whether its stopping spot is registered for the arrival check.
function markerChipHtml(pinky) {
  const marker = pinky.marker;
  if (!marker) return '';
  return marker.registered
    ? `<span class="chip good">마커 ${escapeHtml(marker.marker_id)}</span>`
    : `<span class="chip attention" title="${escapeHtml(marker.problem || '')}">마커 ${escapeHtml(marker.marker_id)} 미등록</span>`;
}

function lastStopHtml(pinkyId) {
  const lastStop = lastCommandOf(pinkyId, 'stop');
  if (!lastStop) return '';
  return `<p class="pinky-last-stop ${commandOutcome(lastStop).kind}" title="${escapeHtml(lastStop.message || '')}">
    <time datetime="${escapeHtml(lastStop.at)}">${escapeHtml(formatClock(lastStop.at))}</time> ${escapeHtml(commandPhrase(lastStop, '정지'))}</p>`;
}

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

// ---- small map ----

function renderSmallMap() {
  drawPinkyMap(pinkyMapSmall, pinkyMapSmallEmpty);
  const data = pinkyMapState.data;
  pinkyMapTag.innerHTML = data?.simulated ? '<span class="sim-tag">모의</span>' : '';
  pinkyMapOff.innerHTML = mapZoneNotesHtml();
  pinkyMapNote.hidden = !data?.note;
  pinkyMapNote.textContent = data?.note || '';
}

// ---- command log ----

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
      <span class="pinky-log-message">${escapeHtml(entry.message || '')}${status ? ` <small>HTTP ${escapeHtml(status)}</small>` : ''}</span>
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

function renderPinkyScreen() {
  renderPinkyLink();
  if (pinkyScreen.view) renderRouteStrip(pinkyScreen.view);
  renderPinkyCards();
  renderPinkyLog();
  document.dispatchEvent(new CustomEvent('pinkyview'));
}

// ---- reading ----

function pinkyOnScreen() {
  return !document.hidden && (['pinky', 'dashboard'].includes(activeView) || !pinkySheet.hidden);
}

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
  if (seq !== pinkyScreen.requestSeq) return;  // a newer read was sent; its answer wins and ends the polling flag
  pinkyScreen.polling = false;
  if (view) pinkyScreen.view = view;
  pinkyScreen.loadError = loadError;
  renderPinkyScreen();
}

// ---- commands ----

async function postPinky(url, body) {
  const options = { method: 'POST' };
  if (body) Object.assign(options, { headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  const response = await fetch(url, options);
  return { status: response.status, result: await response.json() };
}

// Sends one command for one Pinky and says what came back. Returns the entry (or null without one).
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
    showToast(`${name} ${phrase}`, entry.message && entry.message !== phrase ? entry.message : '',
      pinkyToastKinds[commandOutcome(entry).kind], PINKY_COMMAND_TOAST_MS);
    if (entry.http_status === 401) openPasswordPrompt({ reason: `${name} ${label} 요청이 거절됐어요.`, retryLabel: `${name} ${label}`, retry });
    return entry;
  } catch {
    // No answer is not a refusal: the request may have gone through, so ask for a look instead of resending.
    showToast(`${name} ${label} 결과 모름`, '관제 서버의 답을 받지 못했습니다. 핑키를 현장에서 확인하세요.', 'error', PINKY_COMMAND_TOAST_MS);
    return null;
  } finally {
    pinkyScreen.sending.delete(key);
    refreshPinky({ force: true });
    refreshPinkyMap({ force: true });
  }
}

function stopPinky(pinkyId) {
  return sendPinkyCommand(pinkyId, 'stop', { path: 'stop', body: null, label: '정지' });
}

function stopAllSummary(commands) {
  const accepted = commands.filter((entry) => entry.ok).length;
  if (commands.length && accepted === commands.length) return { title: '핑키 전체 정지 요청 접수', kind: 'info' };
  if (commands.length && commands.every((entry) => entry.simulated)) {
    return { title: '핑키 전체 정지 요청 보내지 않음 (모의)', kind: 'warning' };
  }
  return { title: `핑키 전체 정지, ${commands.length}대 중 ${accepted}대 접수`, kind: 'error' };
}

function stopAllLine(entry) {
  const phrase = commandPhrase(entry, '정지');
  const showMessage = !entry.ok && !entry.simulated && entry.http_status !== 401 && entry.message;
  return `${pinkyName(entry.pinky_id)} ${phrase}${showMessage ? `: ${entry.message}` : ''}`;
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
    showToast(title, detail || apiErrorMessage(result, '관제 서버가 요청을 받지 않았습니다'), kind, PINKY_COMMAND_TOAST_MS);
    if (commands.some((entry) => entry.http_status === 401)) {
      openPasswordPrompt({ reason: '정지 요청 일부가 거절됐어요.', retryLabel: '핑키 전체 정지', retry: stopAllPinkies });
    }
  } catch {
    showToast('핑키 전체 정지 요청 결과 모름', '관제 서버의 답을 받지 못했습니다. 핑키를 현장에서 확인하세요.', 'error', PINKY_COMMAND_TOAST_MS);
  } finally {
    pinkyScreen.stopAllSending -= 1;
    if (!pinkyScreen.stopAllSending) {
      pinkyStopAll.removeAttribute('aria-busy');
      pinkyStopAll.textContent = '핑키 전체 정지';
    }
    refreshPinky({ force: true });
  }
}

// ---- SSH password: asked for only when the fleet dashboard needs it ----

function setPasswordMessage(text, tone = '') {
  pinkyPasswordMessage.textContent = text;
  pinkyPasswordMessage.className = `pinky-password-message${tone ? ` ${tone}` : ''}`;
}

function openPasswordPrompt({ reason = '', retryLabel = '', retry = null } = {}) {
  pinkyScreen.retry = retry ? { label: retryLabel, run: retry } : null;
  passwordPromptTitle.textContent = retry ? 'SSH 비밀번호가 필요해요' : 'SSH 비밀번호 보내기';
  passwordPromptReason.textContent = `${reason ? `${reason} ` : ''}핑키 관제가 핑키에 SSH로 접속할 때 씁니다. 이 관제는 저장하지 않고 그대로 넘깁니다.`;
  passwordRetry.hidden = true;
  setPasswordMessage('');
  openOverlay(passwordPrompt);
}

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
      // The refused command is offered again, never sent again by itself.
      if (pinkyScreen.retry) {
        passwordRetry.textContent = `${pinkyScreen.retry.label} 다시 보내기`;
        passwordRetry.hidden = false;
        passwordRetry.focus();
      }
    } else if (entry?.simulated) {
      setPasswordMessage(entry.message, 'warning');
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
// The typed password never outlives the prompt.
passwordPrompt.addEventListener('overlayclose', () => { pinkyPassword.value = ''; pinkyScreen.retry = null; });

// ---- wiring ----

pinkyStopAll.addEventListener('click', stopAllPinkies);
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
document.querySelector('#pinkyPasswordOpen').addEventListener('click', () => openPasswordPrompt());
pinkyPasswordForm.addEventListener('submit', sendPinkyPassword);
document.addEventListener('pinkymap', renderSmallMap);
document.addEventListener('viewchange', () => refreshPinky());
document.addEventListener('visibilitychange', () => refreshPinky());
window.setInterval(refreshPinky, PINKY_POLL_MS);
refreshPinky();
