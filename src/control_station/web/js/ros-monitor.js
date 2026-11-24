// ROS 모니터 (개발자 화면): what the Pinky fleet dashboard receives over ROS for one Pinky, like a terminal.
// The table shows every topic it reads (value, how long ago, fresh or not) and refreshes every second. The terminal
// below prints one line, with the time, whenever something changes: the preparation stage and its message or error,
// the route and mission phase, the line controller and final safety gate states, the lamp, a topic going stale or
// coming back, the LiDAR localization result. [Nav2 로그] prints the error lines of the Pinky's Nav2 log.
//
// Loaded after app.js (escapeHtml, apiErrorMessage, activeView, pinkyNames). Nothing here sends a command.

const rosPinkySelect = document.querySelector('#rosPinky');
const rosTopics = document.querySelector('#rosTopics');
const rosTerminal = document.querySelector('#rosTerminal');
const rosFilter = document.querySelector('#rosFilter');
const rosPause = document.querySelector('#rosPause');
const rosStatus = document.querySelector('#rosStatus');

const ROS_POLL_MS = 1000;
const ROS_MAX_LINES = 600;
const ROS_TOPIC_NAMES = {
  scan: '/scan', ir: '/ir_sensor/range', ultrasonic: '/us_sensor/range', batt_state: '/batt_state', imu: '/imu_raw',
  battery_percent: '/battery/percent', battery_voltage: '/battery/voltage', odom: '/odom', joints: '/joint_states',
  map: '/map', amcl_pose: '/amcl_pose', tf_pose: '/tf (map→base)', line_control: '/line/status',
  safety_status: '/safety/status', indicator: '/fleet/indicator_status', display: '/fleet/display_status',
};

const ros = { pinkyId: '10', last: null, paused: false, lines: [], polling: false, seq: 0 };

function rosClock() {
  return new Date().toLocaleTimeString('ko-KR', { hour12: false });
}

function rosPrint(text, tone = '') {
  for (const line of String(text).split('\n')) ros.lines.push({ at: rosClock(), text: line, tone });
  if (ros.lines.length > ROS_MAX_LINES) ros.lines.splice(0, ros.lines.length - ROS_MAX_LINES);
  renderRosTerminal();
}

function renderRosTerminal() {
  const filter = rosFilter.value.trim().toLowerCase();
  const lines = filter ? ros.lines.filter((line) => line.text.toLowerCase().includes(filter)) : ros.lines;
  rosTerminal.innerHTML = lines.map((line) =>
    `<div class="ros-line ${line.tone}"><time>${line.at}</time>${escapeHtml(line.text)}</div>`).join('');
  if (!ros.paused) rosTerminal.scrollTop = rosTerminal.scrollHeight;
}

// The things worth a terminal line, each as text, so a change is a different string.
function rosWatched(view) {
  const t = view.topics || {};
  const state = (key) => (t[key]?.value || {}).state;
  const watched = {
    'SSH': view.ssh_reachable ? '연결됨' : '끊김',
    '준비 단계': view.stage,
    '주행 판정': view.operational?.label,
    '메시지': view.message,
    '오류': view.error ? String(view.error).split('\n').filter(Boolean).slice(-1)[0]?.slice(0, 200) : null,
    '경로': view.route ? [view.route.status, view.route.phase, view.route.current_label].filter(Boolean).join(' · ') : null,
    '경로 메시지': view.route?.message,
    '주행 제어 /line/status': state('line_control'),
    '안전 감시 /safety/status': state('safety_status'),
    '램프': t.indicator?.value ? `${t.indicator.value.color} ${t.indicator.value.mode} (${(t.indicator.value.devices || []).join(',') || '장치 없음'})` : null,
    '화면': t.display?.value ? (t.display.value.problem || (t.display.value.shown || []).join(' ')) : null,
    '라이다 위치': view.localization ? `${view.localization.applied ? '적용' : view.localization.ok ? '찾음' : '못 정함'} · ${view.localization.reason || ''}` : null,
  };
  for (const [key, topic] of Object.entries(t)) watched[`수신 ${ROS_TOPIC_NAMES[key] || key}`] = topic.fresh ? '받는 중' : '끊김';
  return watched;
}

function rosToneFor(key, value) {
  if (key === '오류' || /끊김|fault|stop|오류|실패/.test(String(value))) return 'bad';
  if (/prepared|받는 중|가능|적용|연결됨/.test(String(value))) return 'good';
  return '';
}

function rosTopicRows(view) {
  return Object.entries(view.topics || {}).map(([key, topic]) => {
    const age = topic.age_s == null ? '안 들어옴' : `${Number(topic.age_s).toFixed(1)}초 전`;
    const json = topic.value == null ? '' : JSON.stringify(topic.value);
    return `<tr class="${topic.fresh ? '' : 'stale'}"><td>${escapeHtml(key)}</td><td><code>${escapeHtml(ROS_TOPIC_NAMES[key] || key)}</code></td>
      <td>${topic.fresh ? '● ' : '○ '}${escapeHtml(age)}</td><td class="ros-value" title="${escapeHtml(json)}">${escapeHtml(json.slice(0, 220))}</td></tr>`;
  }).join('');
}

async function pollRos() {
  if (activeView !== 'ros' || document.hidden || ros.polling) return;
  ros.polling = true;
  const seq = ++ros.seq;
  const pinkyId = ros.pinkyId;
  try {
    const response = await fetch(`/api/pinky/${encodeURIComponent(pinkyId)}/ros`, { cache: 'no-store', signal: AbortSignal.timeout(8000) });
    const view = await response.json();
    if (seq !== ros.seq || pinkyId !== ros.pinkyId) return;
    if (!view.ok) {
      rosStatus.textContent = view.error || '받지 못했어요';
      if (ros.last !== 'error') rosPrint(`! ${view.error || '핑키 관제에서 받지 못했어요'}`, 'bad');
      ros.last = 'error';
      return;
    }
    rosStatus.textContent = `${view.name} · ${view.robot_name || ''} · 도메인 ${view.domain ?? '?'} · ${rosClock()}`;
    rosTopics.innerHTML = rosTopicRows(view);
    const now = rosWatched(view);
    const before = ros.last && ros.last !== 'error' ? ros.last : null;
    if (!before) rosPrint(`── ${view.name} (${view.robot_name || pinkyId}, 도메인 ${view.domain}) 지켜보기 시작 ──`, 'log');
    for (const [key, value] of Object.entries(now)) {
      if (value == null || value === '') continue;
      if (!before) rosPrint(`${key}: ${value}`, rosToneFor(key, value));
      else if (before[key] !== value) rosPrint(`${key}: ${before[key] ?? '없음'} → ${value}`, rosToneFor(key, value));
    }
    ros.last = now;
  } catch {
    if (seq === ros.seq) rosStatus.textContent = '관제 서버 응답 없음';
  } finally {
    if (seq === ros.seq) ros.polling = false;
  }
}

async function printNavLog() {
  const pinkyId = ros.pinkyId;
  rosPrint(`── ${pinkyNames[pinkyId] || pinkyId} Nav2 로그 읽는 중 (SSH, 몇 초 걸려요) ──`, 'log');
  try {
    const response = await fetch(`/api/pinky/${encodeURIComponent(pinkyId)}/nav-log`, { cache: 'no-store' });
    const result = await response.json();
    if (!response.ok) {
      rosPrint(`! ${apiErrorMessage(result, 'Nav2 로그를 받지 못했어요')}`, 'bad');
      return;
    }
    const lines = String(result.output || '').split('\n').filter((line) => line.trim()).slice(-200);
    rosPrint(lines.join('\n') || '(빈 로그)', result.ok ? 'log' : 'bad');
  } catch {
    rosPrint('! 관제 서버의 답을 받지 못했어요', 'bad');
  }
}

function startRosPinky(pinkyId) {
  ros.pinkyId = pinkyId;
  ros.last = null;
  ros.polling = false;
  ros.seq += 1;
  rosTopics.innerHTML = '';
  pollRos();
}

rosPinkySelect.innerHTML = ['10', '11', '12'].map((id) => `<option value="${id}">${escapeHtml(pinkyNames[id] || id)} (${id})</option>`).join('');
rosPinkySelect.addEventListener('change', () => startRosPinky(rosPinkySelect.value));
rosFilter.addEventListener('input', renderRosTerminal);
rosPause.addEventListener('click', () => {
  ros.paused = !ros.paused;
  rosPause.textContent = ros.paused ? '이어서 보기' : '멈춤';
  if (!ros.paused) renderRosTerminal();
});
document.querySelector('#rosClear').addEventListener('click', () => { ros.lines = []; renderRosTerminal(); });
document.querySelector('#rosNavLog').addEventListener('click', printNavLog);
document.addEventListener('viewchange', () => pollRos());
window.setInterval(pollRos, ROS_POLL_MS);
