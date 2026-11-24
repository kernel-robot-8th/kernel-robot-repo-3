// 관제 화면(index.html)의 기본 스크립트다. 가장 먼저 로드되며, 다른 스크립트가 함께 쓰는 전역 도우미와 상태를 둔다.
//
// 로드 순서: index.html의 <script defer>가 적힌 순서대로 실행된다.
//   app.js → intake-plan.js → route-strip.js → pinky-map.js → pinky.js → pinky-detail.js → pinky-test.js
//   → inventory.js → dashboard.js → ros-monitor.js
// 모듈이 아닌 일반 스크립트라서 최상위 const/let/function은 모두 같은 전역 범위를 공유한다. 뒤에 로드되는 스크립트가
// 여기의 escapeHtml, apiErrorMessage, formatClock, setStat, showToast, pinkyBadge, pinkyNames, objectLabel,
// orderSummary, planChips, elapsed, activeView, devMode, devWord, openOverlay/closeOverlay, flowSummary, intake를
// 그대로 쓴다. 반대로 여기서도 뒤 스크립트가 만든 window.IntakePlan을 쓰는데, 호출 시점에는 이미 로드돼 있다.
// 뷰가 바뀌면 document에 `viewchange` 이벤트를 보내고, 다른 스크립트는 메뉴 대신 이 이벤트를 듣는다.
//
// 이 파일이 맡는 화면과 주기 폴링(맨 아래 setInterval):
//   - 서버 상태·장비·카메라(GET /api/health, /api/robots, /api/cameras, /api/arm/status): 3초
//   - 주문 진행(GET /api/flow): 1초, 주문 접수(GET /api/orders): 2초, 배정 미리보기(GET /api/orders/{id}/assignment): 3초
//   - 작업 이력(GET /api/tasks, /api/tasks/{id}): 1초
//   - 팔 명령(GET /api/arm/commands)·연속 실행(GET /api/arm/sequences): 1초
//   - 핑키 도착 실시간 판정(GET /api/parking-spots/{spot}/live): 0.5초, 장비·카메라 뷰가 보일 때만
// 핑키·지도·재고·종합·ROS 모니터 화면은 각자의 스크립트가 따로 읽는다.
//
// 목차 (구역 머리 주석 `// ==== n.`으로 찾는다)
//   1. 전역 DOM 참조와 상태
//   2. 운영/개발자 화면과 뷰 전환
//   3. 화면 위 패널(오버레이)
//   4. 공통 도우미
//   5. 장비·카메라: 요약, 로봇 표, 카메라 카드, 선반 칸 오버레이
//   6. 수동 조작: 직접 실행(텔레오퍼레이션·정책 1회 실행)
//   7. 수동 조작: 팔 선택, 명령 목록, 확인 바, 정책으로 집기
//   8. 수동 조작(개발용): 정책 연속 실행
//   9. 주문 표시 공통(물체 이름·핑키 배지)과 토스트 알림
//  10. 수동 조작: 칸으로 이동(개발용)과 원상복귀
//  11. 작업 이력용 상태·단계·이벤트 이름표
//  12. 장비·카메라: 핑키 도착 확인(주차 판정) 패널
//  13. 작업 이력 화면
//  14. 서버 상태 주기 갱신(refresh)
//  15. 주문 진행 화면: 경보, 로봇 상태, 주문 목록, 모의 세계
//  16. 주문 접수와 자동 출발
//  17. 시작과 폴링 주기

// ==== 1. 전역 DOM 참조와 상태 ====
const serverStatus = document.querySelector('#serverStatus');
const robotRows = document.querySelector('#robotRows');
const cameraGrid = document.querySelector('#cameraGrid');
const updatedAt = document.querySelector('#updatedAt');
const armState = document.querySelector('#armState');
const armDetail = document.querySelector('#armDetail');
const armStart = document.querySelector('#armStart');
const armStop = document.querySelector('#armStop');
const policyRunState = document.querySelector('#policyRunState');
const policyRunDetail = document.querySelector('#policyRunDetail');
const policyRunStart = document.querySelector('#policyRunStart');
const policyRunStop = document.querySelector('#policyRunStop');
const policyProfile = document.querySelector('#policyProfile');
const detectorClassOptions = document.querySelector('#detectorClassOptions');
const taskForm = document.querySelector('#taskForm');
const taskList = document.querySelector('#taskList');
const taskDetail = document.querySelector('#taskDetail');
const taskCount = document.querySelector('#taskCount');
const taskMessage = document.querySelector('#taskMessage');
const viewTitle = document.querySelector('#viewTitle');
const parkingSpotInput = document.querySelector('#parkingSpot');
const parkingCheckButton = document.querySelector('#parkingCheck');
const parkingSteps = document.querySelector('#parkingSteps');
const parkingConfirm = document.querySelector('#parkingConfirm');
const parkingLiveMode = document.querySelector('#parkingLiveMode');
const parkingMap = document.querySelector('#parkingMap');
const parkingStatus = document.querySelector('#parkingStatus');
const parkingDetails = document.querySelector('#parkingDetails');
const parkingUpdated = document.querySelector('#parkingUpdated');
const parkingBanner = document.querySelector('#parkingBanner');
const parkingSummary = document.querySelector('#parkingSummary');
const parkingForward = document.querySelector('#parkingForward');
const parkingLeft = document.querySelector('#parkingLeft');
const parkingYaw = document.querySelector('#parkingYaw');
const parkingGauges = document.querySelector('#parkingGauges');
const parkingAdvice = document.querySelector('#parkingAdvice');
const parkingStream = document.querySelector('#parkingStream');
const parkingStreamEmpty = document.querySelector('#parkingStreamEmpty');
const parkingOverlay = document.querySelector('#parkingOverlay');
const parkingPinky = document.querySelector('#parkingPinky');
const navItems = [...document.querySelectorAll('.nav-item[data-view]')];
const views = [...document.querySelectorAll('.view[id^="view-"]')];
// selectedTaskId: 작업 이력에서 상세를 펼친 작업(주문 포함) id다.
// parkingCameraId: GET /api/health의 parking.camera_id(주차 판정 카메라)다.
// controlMode: GET /api/health의 mode('mock' | 'ros2')다. 작업 시작·취소 버튼은 mock 모드에서만 나온다.
let selectedTaskId = null;
let parkingCameraId = null;
let controlMode = 'unknown';

// 뷰 id → 머리글 제목. 여기 없는 이름을 요청하면 showView가 종합으로 돌린다.
const viewNames = {
  dashboard: '종합', flow: '주문 진행', pinky: '핑키', inventory: '재고', overview: '장비·카메라', manual: '수동 조작', tasks: '작업 이력',
  ros: 'ROS 모니터', api: 'API',
};
const controlModeLabels = {
  mock: '모의',
  ros2: 'ROS 2 상태 관측',
};

// 지금 보이는 뷰. 다른 스크립트(pinky.js, dashboard.js 등)가 폴링할지 정할 때 읽는다.
let activeView = 'dashboard';

// ==== 2. 운영/개발자 화면과 뷰 전환 ====
// 모의 흔적(모의 세계 패널, "모의" 태그, 모드 이름)은 ?dev=1일 때만 보인다. 모든 스크립트가 이 함수 하나에 물으므로
// 규칙이 한곳에만 있다.
function isDevView() {
  return new URLSearchParams(window.location.search).get('dev') === '1';
}

// 페이지를 열 때 한 번만 정한다. body.dev 클래스가 있어야 CSS가 [data-dev-only]·.dev-only 요소를 보여 준다.
const devMode = isDevView();
document.body.classList.toggle('dev', devMode);

// 운영 화면과 개발자 화면을 오가는 링크는 지금 보고 있는 뷰(#해시)를 주소에 그대로 유지한다.
const screenModeSwitch = document.querySelector('#screenModeSwitch');
function updateScreenModeSwitch() {
  const params = new URLSearchParams(window.location.search);
  if (devMode) params.delete('dev'); else params.set('dev', '1');
  const query = params.toString();
  screenModeSwitch.href = `${window.location.pathname}${query ? `?${query}` : ''}${window.location.hash}`;
  screenModeSwitch.textContent = devMode ? '운영 화면으로' : '개발자 화면';
}
updateScreenModeSwitch();
window.addEventListener('hashchange', updateScreenModeSwitch);
document.addEventListener('viewchange', updateScreenModeSwitch);

// 개발자 화면이면 모의 표시 단어를 그대로, 아니면 빈 문자열을 돌려준다.
function devWord(word) {
  return devMode ? word : '';
}

// 뷰는 주소(#pinky, #inventory…)에 담기므로 새로고침하거나 링크를 공유해도 같은 뷰가 열린다.
// 다른 스크립트는 메뉴를 직접 듣지 않고 `viewchange` 이벤트로 뷰 전환을 따라간다.
// 운영 화면(기본)은 장비·수동 조작·API 뷰를 빼고 보여 주며, ?dev=1이면 모두 보인다.
// (지금 코드에서 개발자 전용 뷰는 아래 DEV_ONLY_VIEWS의 manual, ros, api이고, 장비·카메라 뷰는 운영 화면에도 나온다.)
const DEV_ONLY_VIEWS = new Set(['manual', 'ros', 'api']);

// 요청한 뷰를 보여 준다. 없는 뷰이거나 운영 화면에서 개발자 전용 뷰를 요청하면 종합으로 돌린다.
// 열린 오버레이를 닫고 메뉴 표시·제목·주소 해시를 맞춘 뒤 `viewchange` 이벤트를 보낸다.
function showView(requested) {
  const selected = viewNames[requested] && (devMode || !DEV_ONLY_VIEWS.has(requested)) ? requested : 'dashboard';
  activeView = selected;
  document.body.dataset.view = selected;
  closeAllOverlays();
  views.forEach((view) => { view.hidden = view.id !== `view-${selected}`; });
  navItems.forEach((item) => {
    const active = item.dataset.view === selected;
    item.classList.toggle('active', active);
    if (active) item.setAttribute('aria-current', 'page');
    else item.removeAttribute('aria-current');
    if (active) keepInNavView(item);
  });
  viewTitle.textContent = viewNames[selected];
  // 방문 기록이 쌓이지 않도록 pushState가 아니라 replaceState로 해시를 바꾼다.
  if (window.location.hash !== `#${selected}`) window.history.replaceState(null, '', `#${selected}`);
  document.dispatchEvent(new CustomEvent('viewchange', { detail: { view: selected } }));
}

// 휴대폰 폭에서는 메뉴가 옆으로 스크롤되는 한 줄이 된다. 페이지는 움직이지 않고 현재 항목이 보이게 메뉴만 스크롤한다.
function keepInNavView(item) {
  const nav = item.parentElement;
  if (nav.scrollWidth <= nav.clientWidth) return;
  nav.scrollLeft = item.offsetLeft - (nav.clientWidth - item.offsetWidth) / 2;
}

navItems.forEach((item) => item.addEventListener('click', () => showView(item.dataset.view)));

// ==== 3. 화면 위 패널(오버레이) ====
// ---- 페이지 위에 뜨는 패널(핑키 상세, 크게 본 지도, 비밀번호 창) ----
// 고정된 머리글 아래에서 열리므로 패널이 열려 있어도 전체 정지가 보이고 누를 수 있다. 어두운 바깥을 누르거나
// Esc를 누르면 맨 위 패널이 닫히고, 포커스는 그 패널을 연 요소로 돌아간다.
const overlayScrim = document.querySelector('#overlayScrim');
const pageHeader = document.querySelector('.header');
const overlayStack = [];  // [{ panel, opener }], 마지막 것이 맨 위

// 머리글 아래쪽 위치를 CSS 변수 --overlay-top으로 넘겨, 패널과 어두운 배경이 그 아래에서 시작하게 한다.
function placeOverlays() {
  const top = Math.max(0, Math.round(pageHeader.getBoundingClientRect().bottom));
  document.documentElement.style.setProperty('--overlay-top', `${top}px`);
}

// 패널을 스택 맨 위에 연다. 이미 열려 있으면 무시하고, z-index를 쌓인 순서대로 올린 뒤 [data-overlay-focus]에 포커스를 준다.
function openOverlay(panel, opener = document.activeElement) {
  if (overlayStack.some((entry) => entry.panel === panel)) return;
  placeOverlays();
  overlayStack.push({ panel, opener });
  panel.style.zIndex = String(30 + overlayStack.length);
  panel.hidden = false;
  overlayScrim.hidden = false;
  (panel.querySelector('[data-overlay-focus]') || panel).focus();
}

// 패널을 스택에서 빼고 닫는다. 'overlayclose' 이벤트를 보내 각 스크립트가 정리하게 한다(비밀번호 지우기, 위치 지정 끝내기).
function closeOverlay(panel) {
  const index = overlayStack.findIndex((entry) => entry.panel === panel);
  if (index < 0) return;
  const [{ opener }] = overlayStack.splice(index, 1);
  panel.hidden = true;
  overlayScrim.hidden = !overlayStack.length;
  panel.dispatchEvent(new Event('overlayclose'));
  if (opener?.isConnected) opener.focus();
}

function closeAllOverlays() {
  [...overlayStack].reverse().forEach(({ panel }) => closeOverlay(panel));
}

overlayScrim.addEventListener('click', () => { if (overlayStack.length) closeOverlay(overlayStack.at(-1).panel); });
document.addEventListener('keydown', (event) => {
  if (event.key === 'Escape' && overlayStack.length) closeOverlay(overlayStack.at(-1).panel);
});
document.addEventListener('click', (event) => {
  const close = event.target.closest('[data-overlay-close]');
  if (close) closeOverlay(close.closest('.overlay-panel'));
});
// 스크롤·창 크기가 바뀌면 머리글 위치가 달라질 수 있으니 열린 패널의 위치를 다시 잡는다.
window.addEventListener('scroll', () => { if (overlayStack.length) placeOverlays(); }, { passive: true });
window.addEventListener('resize', () => { if (overlayStack.length) placeOverlays(); });
// 주소의 해시가 바뀌면(뒤로 가기, 링크) 그 뷰로 바꾼다.
window.addEventListener('hashchange', () => showView(window.location.hash.slice(1)));

// ==== 4. 공통 도우미 ====
// innerHTML에 넣을 값을 HTML 이스케이프한다. 서버나 사용자에게서 온 문자열은 모두 이것을 거친다.
function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (character) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
  })[character]);
}

// API 오류 응답({error_code, message} 또는 {error})을 화면용 한 줄로 만든다. 코드가 있으면 'CODE · 메시지'다.
function apiErrorMessage(result, fallback) {
  const message = result.message || result.error || fallback;
  return result.error_code ? `${result.error_code} · ${message}` : message;
}

// 시:분:초 24시간 표기(ko-KR).
const clockFormat = new Intl.DateTimeFormat('ko-KR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });

// 요약 카드의 큰 숫자 하나. tone: attention(사람이 처리해야 함) | danger | active | zero.
function setStat(stat, value, { total = null, tone = '' } = {}) {
  const shown = value == null ? '–' : escapeHtml(value);
  stat.querySelector('dd').innerHTML = total == null ? shown : `${shown}<small>/${escapeHtml(total)}</small>`;
  stat.className = `stat${tone ? ` ${tone}` : ''}`;
}

// ==== 5. 장비·카메라: 요약, 로봇 표, 카메라 카드, 선반 칸 오버레이 ====
const overviewSummaryTitle = document.querySelector('#overviewSummaryTitle');
const overviewStatRobots = document.querySelector('#overviewStatRobots');
const overviewStatCameras = document.querySelector('#overviewStatCameras');
// 카메라 카드로 보여 줄 카메라 id와 순서. 여기에 없는 카메라는 요약 숫자에서도 빠진다.
const CAMERA_PREVIEW_IDS = [
  'head_camera', 'front_camera',
  'left_ceiling_camera', 'left_joint_camera',
  'right_ceiling_camera', 'right_joint_camera',
];

// 장비·카메라 뷰 맨 위 요약 카드. health가 없으면(서버 연결 실패) 숫자를 비운다.
function renderOverviewSummary(health, robots, cameras) {
  if (!health) {
    overviewSummaryTitle.textContent = '관제 서버에 연결하지 못했어요';
    setStat(overviewStatRobots, null);
    setStat(overviewStatCameras, null);
    return;
  }
  overviewSummaryTitle.textContent = health.mode !== 'mock' || devMode
    ? `API 정상, ${controlModeLabels[health.mode] || health.mode} 모드로 실행 중이에요`
    : '관제 서버가 정상이에요';
  const reachable = robots.filter((robot) => robot.state === 'available').length;
  const previewCameras = cameras.filter((camera) => CAMERA_PREVIEW_IDS.includes(camera.id));
  const streaming = previewCameras.filter((camera) => camera.stream_state === 'STREAMING').length;
  setStat(overviewStatRobots, reachable, { total: robots.length, tone: reachable < robots.length ? 'attention' : '' });
  setStat(overviewStatCameras, streaming, { total: previewCameras.length, tone: streaming < previewCameras.length ? 'attention' : '' });
}

// 로봇 인터페이스 표(GET /api/robots). state(available/waiting/stale/invalid)를 색 클래스로 바꾼다.
function renderRobots(robots) {
  if (!robots.length) {
    robotRows.innerHTML = '<tr><td colspan="3" class="loading">등록된 로봇이 없습니다.</td></tr>';
    return;
  }
  robotRows.innerHTML = robots.map((robot) => {
    const stateClass = robot.state === 'available' ? 'available' : robot.state === 'waiting' ? 'waiting' : robot.state === 'stale' ? 'stale' : '';
    const communicationClass = robot.state === 'invalid' ? 'invalid' : stateClass;
    return `<tr><td>${escapeHtml(robot.name)}</td><td><span class="state ${communicationClass}">${escapeHtml(robot.communication)}</span></td><td>${escapeHtml(robot.detail)}</td></tr>`;
  }).join('');
}

// 탑재 목록: 적재 카메라가 가장 최근 프레임에서 핑키 쟁반 위에 보는 것(GET /api/cameras의 pinky_load).
// pinkyArrivalText는 그중 핑키 도착 여부(마커 기준)를 한 줄 글로 만든다. 표시 전용이다.
function pinkyArrivalText(load) {
  const pinky = load.pinky;
  if (load.state !== 'READY' || !pinky) return load.message || load.state;
  if (pinky.state === 'ARRIVED') {
    const seen = pinky.marker_visible ? `${pinky.offset_px}px` : `마커 가려짐 · 마지막 확인 ${clockFormat.format(new Date(pinky.last_seen_at))}`;
    return `${pinky.name} 적재 자리 도착 ✓ (마커 ${pinky.marker_id}, ${seen})`;
  }
  if (pinky.state === 'NOT_AT_SPOT') return `${pinky.name} 도착 전 (마커 ${pinky.marker_id}, 자리에서 ${pinky.offset_px}px)`;
  if (pinky.state === 'NOT_SEEN' && Object.keys(load.counts || {}).length) return '핑키 마커가 물건에 가려져 있습니다 (쟁반 위 물건은 아래에 표시)';
  return pinky.message || pinky.state;
}

// 카메라의 영역별 집계(pinky_load·head_shelf·head_middle)를 물체 칩으로 만든다. seen_now가 false인 물체는
// 이번 프레임에는 안 보였지만 최근에 본 것이라 노란 칩(.recent)으로 표시한다.
function headCountBlocks(view) {
  if (view.state === 'NO_TRAY_REGION') return '검출 영역을 설정해 주세요';
  if (view.state !== 'READY') return escapeHtml(view.message || view.state);
  if (view.pinky?.state === 'NOT_AT_SPOT') return '핑키 도착 대기';
  const entries = Object.entries(view.counts || {});
  if (!entries.length) return '비어 있음';
  const items = new Map((view.items || []).map((item) => [item.class_name, item]));
  return entries.map(([name, count]) => {
    const item = items.get(name);
    const recent = item?.seen_now === false;
    return `<span class="load-chip${recent ? ' recent' : ''}">${escapeHtml(objectLabel(name))} ${count}개</span>`;
  }).join('');
}

// 정면 카메라(FRONT)의 YOLO 검출을 종류별 개수 칩으로 만든다. 신뢰도 0.5 미만은 세지 않는다.
function cameraDetectionBlocks(camera) {
  if (camera.detector_state === 'loading') return '모델 불러오는 중';
  if (camera.detector_state === 'error') return '모델 오류';
  if (camera.detector_state !== 'ready') return '모델 미설정';
  if (camera.stream_state !== 'STREAMING' || camera.frame_freshness !== 'FRESH') return '새 프레임 대기 중';
  const counts = new Map();
  for (const item of camera.detections || []) {
    if (item.confidence < 0.5) continue;
    counts.set(item.class_name, (counts.get(item.class_name) || 0) + 1);
  }
  if (!counts.size) return '검출된 물체 없음';
  return [...counts].map(([name, count]) =>
    `<span class="load-chip">${escapeHtml(objectLabel(name))} ${count}개</span>`
  ).join('');
}

// 카메라 카드 격자. 카드 구성(어떤 카메라가 있는지)이 바뀔 때만 HTML을 새로 만들고, 그 밖에는 data-field 칸의
// 글자만 바꾼다. 그래야 3초마다 갱신해도 <img> 스트림이 끊기지 않는다.
function renderCameras(cameras, pinkyLoad = null, headShelf = null, headMiddle = null) {
  const camerasById = new Map(cameras.map((camera) => [camera.id, camera]));
  const visibleCameras = CAMERA_PREVIEW_IDS.map((id) => camerasById.get(id)).filter(Boolean);
  const names = {
    LEFT_CEILING: '왼쪽 천장 카메라', LEFT_JOINT: '왼쪽 관절 카메라',
    RIGHT_CEILING: '오른쪽 천장 카메라', RIGHT_JOINT: '오른쪽 관절 카메라',
    HEAD: '천장 카메라', JOINT: '조인트 카메라', FRONT: '정면 카메라', SHELF: '선반 카메라 (정면)',
  };
  // 서버는 카메라마다 하는 일로 이름을 붙여 보낸다("왼쪽 팔 탑 캠 (ACT 정책)"). 위 표는 예전 서버용이다.
  const cameraName = (camera) => camera.role || names[camera.position] || camera.id;
  const connectionLabel = {
    CONNECTED: '연결됨', DISCONNECTED: '연결 끊김', UNKNOWN: '확인 필요',
  };
  const streamLabel = {
    STREAMING: '수신 중', NO_FRAMES: '프레임 미수신', STOPPED: '중지',
    ERROR: '오류', UNKNOWN: '확인 필요',
  };
  const freshnessLabel = { FRESH: '최신', STALE: '지연', UNKNOWN: '확인 필요' };
  const cameraIds = visibleCameras.map((camera) => camera.id).join(',');
  // 준비된 검출기의 클래스 이름을 모아(대소문자 무시 중복 제거) 수동 조작 '정면 캠에서 찾을 물건' 입력의 자동완성으로 쓴다.
  const availableClasses = [];
  const availableClassKeys = new Set();
  for (const camera of cameras.filter((item) => item.detector_state === 'ready')) {
    for (const className of camera.detector_classes || []) {
      const key = String(className).toLocaleLowerCase('en-US');
      if (availableClassKeys.has(key)) continue;
      availableClassKeys.add(key);
      availableClasses.push(className);
    }
  }
  detectorClassOptions.replaceChildren(...availableClasses.map((className) => {
    const option = document.createElement('option');
    option.value = className;
    option.label = objectLabel(className);
    return option;
  }));
  // 카메라 목록이 바뀐 경우에만 카드를 다시 만든다(dataset.cameraIds로 비교).
  if (cameraGrid.dataset.cameraIds !== cameraIds) {
    cameraGrid.dataset.cameraIds = cameraIds;
    cameraGrid.innerHTML = visibleCameras.map((camera) => `
      <article class="camera-card" data-camera-id="${escapeHtml(camera.id)}">
        <header class="camera-head"><h3>${escapeHtml(cameraName(camera))}</h3><span class="camera-position">${escapeHtml(camera.position)}</span></header>
        <div class="camera-preview" data-field="preview"><span>영상 스트림 미연결</span></div>
        <div class="head-counts" data-field="head-counts" hidden>
          <section data-field="head-shelf-group" hidden>
            <h4>왼쪽 선반</h4><div class="head-count-items" data-field="head-shelf"></div>
          </section>
          <section data-field="head-middle-group" hidden>
            <h4>가운데 작업대</h4><div class="head-count-items" data-field="head-middle"></div>
          </section>
          <section data-field="pinky-load-group" hidden>
            <h4>오른쪽 핑키 탑재</h4><div class="head-count-items" data-field="pinky-load"></div>
          </section>
        </div>
        <section class="camera-detection-panel" data-field="camera-detection-panel" hidden>
          <h4>정면 카메라 검출</h4>
          <div class="head-count-items" data-field="camera-detection-blocks"></div>
        </section>
        <div class="camera-details">
          <div class="camera-detail"><span>연결</span><strong data-field="connection"></strong></div>
          <div class="camera-detail"><span>스트림</span><strong data-field="stream"></strong></div>
          <div class="camera-detail"><span>프레임 상태</span><strong data-field="freshness"></strong></div>
          <div class="camera-detail"><span>마지막 프레임</span><strong data-field="last-frame"></strong></div>
          <div class="camera-detail"><span>프레임 ID / 해상도</span><strong data-field="frame-info"></strong></div>
          <div class="camera-detail"><span>프레임률</span><strong data-field="frame-rate"></strong></div>
          <div class="camera-detail" data-field="detections-row" hidden><span>YOLO 검출</span><strong data-field="detections"></strong></div>
          <div class="camera-detail" data-field="pinky-arrival-row" hidden><span>핑키 도착 · 표시 전용</span><strong data-field="pinky-arrival"></strong></div>
          <div class="camera-detail" data-field="slot-zones-row" hidden><span>선반 칸 · 표시 전용</span><strong data-field="slot-zones"></strong></div>
          <div class="camera-detail" data-field="detector-provenance-row" hidden><span>YOLO 설정 / 모델</span><strong data-field="detector-provenance"></strong></div>
        </div>
        <p class="note" data-field="error"></p>
      </article>`).join('');
  }
  visibleCameras.forEach((camera) => {
    const card = cameraGrid.querySelector(`[data-camera-id="${camera.id}"]`);
    if (!card) return;
    card.querySelector('[data-field="connection"]').textContent = connectionLabel[camera.connection_state] || camera.connection_state;
    card.querySelector('[data-field="stream"]').textContent = streamLabel[camera.stream_state] || camera.stream_state;
    card.querySelector('[data-field="freshness"]').textContent = freshnessLabel[camera.frame_freshness] || camera.frame_freshness;
    card.querySelector('[data-field="last-frame"]').textContent = camera.last_frame_at || '없음';
    const resolution = camera.frame_width && camera.frame_height
      ? `${camera.frame_width}×${camera.frame_height}`
      : '—';
    card.querySelector('[data-field="frame-info"]').textContent = `${camera.frame_sequence || 0} / ${resolution}`;
    card.querySelector('[data-field="frame-rate"]').textContent = camera.frame_rate == null ? '—' : `${camera.frame_rate} fps`;
    card.querySelector('[data-field="error"]').textContent = camera.error_message || '';
    const detectorLabel = { loading: '모델 불러오는 중', error: '모델 오류' };
    const detectionPanel = card.querySelector('[data-field="camera-detection-panel"]');
    detectionPanel.hidden = camera.position !== 'FRONT';
    if (!detectionPanel.hidden) {
      detectionPanel.querySelector('[data-field="camera-detection-blocks"]').innerHTML = cameraDetectionBlocks(camera);
    }
    // YOLO 검출 줄은 검출기가 있고, 천장(HEAD) 카메라가 아니고, 위의 정면 카메라 검출 칸이 없을 때만 보인다.
    card.querySelector('[data-field="detections-row"]').hidden = !camera.detector_state || camera.position === 'HEAD' || !detectionPanel.hidden;
    card.querySelector('[data-field="detections-row"] span').textContent = camera.detector_model_type === 'YOLO_WORLD_OPEN_VOCAB'
      ? 'YOLO-World 후보 · 관측 전용'
      : 'YOLO 검출 · 관측 전용';
    card.querySelector('[data-field="detections"]').textContent = camera.detector_state === 'ready'
      ? (camera.detections.map((item) => `${objectLabel(item.class_name)} ${Math.round(item.confidence * 100)}% @ (${item.center_px.join(', ')})`).join(', ') || '없음')
      : (detectorLabel[camera.detector_state] || '');
    // 영역별 집계는 그 집계를 만든 카메라(camera_id)의 카드에만 붙는다.
    const loadGroup = card.querySelector('[data-field="pinky-load-group"]');
    const shelfGroup = card.querySelector('[data-field="head-shelf-group"]');
    const middleGroup = card.querySelector('[data-field="head-middle-group"]');
    middleGroup.hidden = !headMiddle || headMiddle.camera_id !== camera.id;
    if (!middleGroup.hidden) {
      middleGroup.querySelector('[data-field="head-middle"]').innerHTML = headCountBlocks(headMiddle);
    }
    loadGroup.hidden = !pinkyLoad || pinkyLoad.camera_id !== camera.id;
    shelfGroup.hidden = !headShelf || headShelf.camera_id !== camera.id;
    card.querySelector('[data-field="head-counts"]').hidden = loadGroup.hidden && shelfGroup.hidden && middleGroup.hidden;
    const arrivalRow = card.querySelector('[data-field="pinky-arrival-row"]');
    arrivalRow.hidden = loadGroup.hidden;
    if (!loadGroup.hidden) {
      arrivalRow.querySelector('[data-field="pinky-arrival"]').textContent = pinkyArrivalText(pinkyLoad);
      arrivalRow.dataset.state = pinkyLoad.pinky?.state || pinkyLoad.state;
      loadGroup.querySelector('[data-field="pinky-load"]').innerHTML = headCountBlocks(pinkyLoad);
    }
    if (!shelfGroup.hidden) {
      shelfGroup.querySelector('[data-field="head-shelf"]').innerHTML = headCountBlocks(headShelf);
    }
    // 검출기 출처(모델 종류·임계값·가중치 해시 등)를 한 줄로 보여 주고, 전체 해시와 프롬프트는 title(마우스 올림)에 둔다.
    const hasDetectorProvenance = Boolean(camera.detector_confidence_threshold != null
      || camera.detector_weights_sha256 || camera.detector_ultralytics_version
      || camera.detector_model_type);
    const provenanceRow = card.querySelector('[data-field="detector-provenance-row"]');
    provenanceRow.hidden = !hasDetectorProvenance;
    if (hasDetectorProvenance) {
      const weightsHash = camera.detector_weights_sha256 || 'hash 없음';
      const promptCount = camera.detector_prompts?.length || 0;
      const promptSummary = camera.detector_prompts?.slice(0, 4).join(', ') || '';
      const provenance = [
        camera.detector_model_type === 'YOLO_WORLD_OPEN_VOCAB' ? 'YOLO-World' : 'YOLO',
        promptCount ? `${promptSummary}${promptCount > 4 ? ` 외 ${promptCount - 4}개` : ''}` : '',
        `conf ${camera.detector_confidence_threshold ?? '—'}`,
        `imgsz ${camera.detector_inference_image_size ?? '—'}`,
        `device ${camera.detector_device || '자동'}`,
        `ultralytics ${camera.detector_ultralytics_version || '—'}`,
        `sha256 ${weightsHash.slice(0, 12)}`,
      ].join(' · ');
      const provenanceValue = card.querySelector('[data-field="detector-provenance"]');
      provenanceValue.textContent = provenance;
      provenanceValue.title = [
        camera.detector_weights_sha256 || '',
        ...(camera.detector_prompts || []),
      ].filter(Boolean).join('\n');
    }
    // 스트림 <img>는 한 번만 만든다. 다시 만들면 MJPEG 연결이 끊겼다 다시 붙는다. 오류가 나면 안내 글로 바꾸고,
    // 다음 갱신 때 <img>가 없으므로 다시 만든다.
    const preview = card.querySelector('[data-field="preview"]');
    if (camera.stream_url && !preview.querySelector('img')) {
      const image = document.createElement('img');
      image.src = camera.stream_url;
      image.alt = `${cameraName(camera)} 영상`;
      image.addEventListener('error', () => {
        if (!image.isConnected) return;
        preview.replaceChildren(document.createTextNode('영상 스트림 연결 대기 중'));
      }, { once: true });
      preview.replaceChildren(image);
    } else if (!camera.stream_url && !preview.querySelector('img')) {
      preview.replaceChildren(document.createTextNode(camera.error_message || '영상 스트림 미연결'));
    }
    renderSlotZones(card, preview, camera);
  });
}

// 선반 칸 보정 상태(slot_zones.status)와 검출의 칸 배정 상태(slot_assignment.status) 표시 이름.
const slotZoneStatusLabels = {
  READY: '보정 적용', INVALID: '보정 파일 오류', SLOT_MAP_MISMATCH: '칸 정의 버전 다름',
  SLOT_MAP_UNAVAILABLE: '칸 정의 없음', IMAGE_MISMATCH: '카메라 설정 다름',
};
const slotAssignmentLabels = { OUTSIDE: '칸 밖', AMBIGUOUS: '칸 경계·애매', NO_BBOX: 'bbox 없음' };
const svgNamespace = 'http://www.w3.org/2000/svg';

function renderSlotZones(card, preview, camera) {
  // 표시 전용: 스트림 위에 칸 영역 윤곽을 그리고, 검출마다 바닥 가운데 점이 들어간 칸을 보여 준다.
  const zones = camera.slot_zones;
  const row = card.querySelector('[data-field="slot-zones-row"]');
  row.hidden = !zones;
  let overlay = preview.querySelector('svg.slot-overlay');
  if (!zones || zones.status !== 'READY' || !preview.querySelector('img')) {
    overlay?.remove();
    if (zones) {
      card.querySelector('[data-field="slot-zones"]').textContent = `${slotZoneStatusLabels[zones.status] || zones.status}${zones.detail ? ` · ${zones.detail}` : ''}`;
    }
    return;
  }
  if (!overlay) {
    overlay = document.createElementNS(svgNamespace, 'svg');
    overlay.classList.add('slot-overlay');
    overlay.setAttribute('preserveAspectRatio', 'xMidYMid meet');
    overlay.setAttribute('aria-hidden', 'true');
    preview.append(overlay);
  }
  overlay.setAttribute('viewBox', `0 0 ${camera.frame_width} ${camera.frame_height}`);
  const occupied = new Set();
  const detections = camera.detections || [];
  detections.forEach((item) => { if (item.slot_assignment?.status === 'IN_SLOT') occupied.add(item.slot_assignment.slot_id); });
  const shapes = zones.slots.map((slot) => {
    // 채운 도형은 칸을 정하는 기준(anchor) 영역이고, 점선 윤곽은 칸 전체 공간이다.
    const volume = document.createElementNS(svgNamespace, 'polygon');
    volume.setAttribute('points', (slot.volume_polygon_px || []).map(([x, y]) => `${x},${y}`).join(' '));
    volume.setAttribute('class', 'slot-volume');
    const polygon = document.createElementNS(svgNamespace, 'polygon');
    polygon.setAttribute('points', slot.polygon_px.map(([x, y]) => `${x},${y}`).join(' '));
    polygon.setAttribute('class', occupied.has(slot.slot_id) ? 'slot-zone occupied' : 'slot-zone');
    const label = document.createElementNS(svgNamespace, 'text');
    const xs = slot.polygon_px.map(([x]) => x);
    const ys = slot.polygon_px.map(([, y]) => y);
    label.setAttribute('x', String((Math.min(...xs) + Math.max(...xs)) / 2));
    label.setAttribute('y', String(Math.min(...ys) - 6));
    label.setAttribute('class', 'slot-label');
    label.textContent = slot.label.replace(' 선반 ', ' ');
    return [volume, polygon, label];
  }).flat();
  // 검출마다 칸 판정에 쓴 기준점. 칸 안이면 초록, 애매하면 빨강(.unsure)이다.
  const anchors = detections.filter((item) => item.slot_assignment?.anchor_px).map((item) => {
    const dot = document.createElementNS(svgNamespace, 'circle');
    dot.setAttribute('cx', String(item.slot_assignment.anchor_px[0]));
    dot.setAttribute('cy', String(item.slot_assignment.anchor_px[1]));
    dot.setAttribute('r', '4');
    dot.setAttribute('class', item.slot_assignment.status === 'IN_SLOT' ? 'slot-anchor' : 'slot-anchor unsure');
    return dot;
  });
  overlay.replaceChildren(...shapes, ...anchors);
  const summary = detections.map((item) => {
    const assignment = item.slot_assignment;
    const where = assignment?.status === 'IN_SLOT' ? assignment.label : (slotAssignmentLabels[assignment?.status] || '—');
    return `${item.class_name} → ${where}`;
  });
  card.querySelector('[data-field="slot-zones"]').textContent = `보정 적용 · 재투영 ${zones.reprojection_rms_px} px${summary.length ? ` · ${summary.join(', ')}` : ' · 검출 없음'}`;
}

// ==== 6. 수동 조작: 직접 실행(텔레오퍼레이션·정책 1회 실행) ====
// 텔레오퍼레이션 프로세스 상태(GET /api/arm/status의 arm). 기능이 꺼져 있거나 이미 돌고 있으면 시작 버튼을 막고,
// 프로세스가 있을 때만 종료 요청을 누를 수 있다.
function renderArm(arm) {
  const labels = {
    DISABLED: '비활성화', STOPPED: '중지', STARTING: '시작 중',
    RUNNING: '프로세스 실행 중', STOPPING: '종료 중', ERROR: '오류',
  };
  armState.textContent = labels[arm.state] || arm.state;
  armDetail.textContent = arm.detail;
  armStart.disabled = !arm.enabled || arm.process_alive;
  armStop.disabled = !arm.process_alive;
  showDirectBlock(manualTeleop, arm.enabled || arm.process_alive);
}

// 텔레오퍼레이션과 정책 1회 실행은 이 PC에서 LeRobot을 직접 띄운다. 서버를 그 기능과 함께 켰을 때만 켜지며
// (run-control-station.sh --arm, CONTROL_STATION_POLICY_ENABLED=1), 꺼진 블록은 숨긴다.
const manualDirect = document.querySelector('#manualDirect');
const manualTeleop = document.querySelector('#manualTeleop');
const manualPolicyRun = document.querySelector('#manualPolicyRun');

// 기능이 켜졌거나 개발자 화면이면 블록을 보인다. 두 블록이 모두 숨으면 '직접 실행' 묶음 전체를 숨긴다.
function showDirectBlock(block, enabled) {
  block.hidden = !(enabled || devMode);
  manualDirect.hidden = manualTeleop.hidden && manualPolicyRun.hidden;
}

// 정책 1회 실행 상태(GET /api/arm/status의 policy_run). 정책 목록이 바뀔 때만 <select>를 다시 채운다(고른 값 유지).
// 정책이 2개 이상이고 실행 중이 아닐 때만 고를 수 있고, 총 액션 상한(max_action_steps)이 없으면 시작할 수 없다.
function renderPolicyRun(policyRun) {
  const labels = {
    DISABLED: '비활성화', STOPPED: '중지', STARTING: '시작 중',
    RUNNING: '정책 실행 중', STOPPING: '중단 중', LIMIT_REACHED: '액션 상한 도달',
    INTERRUPTED: '중단 · 결과 미확정', ERROR: '오류',
  };
  const profiles = policyRun.policy_profiles || [];
  const expectedOptions = profiles.map((profile) => profile.id).join('|');
  const currentOptions = [...policyProfile.options].map((option) => option.value).join('|');
  if (expectedOptions !== currentOptions) {
    policyProfile.replaceChildren(...profiles.map((profile) => {
      const option = document.createElement('option');
      option.value = profile.id;
      option.textContent = profile.name;
      return option;
    }));
    if (profiles.some((profile) => profile.id === policyRun.current_profile_id)) {
      policyProfile.value = policyRun.current_profile_id;
    }
  }
  policyProfile.disabled = !policyRun.enabled || profiles.length < 2 || policyRun.process_alive;
  policyRunState.textContent = labels[policyRun.state] || policyRun.state;
  policyRunDetail.textContent = policyRun.max_action_steps
    ? `${policyRun.detail} · 총 액션 상한 ${policyRun.max_action_steps}`
    : `${policyRun.detail} · CONTROL_STATION_POLICY_MAX_ACTION_STEPS 설정 필요`;
  policyRunStart.disabled = !policyRun.enabled || !policyRun.max_action_steps || policyRun.process_alive;
  policyRunStop.disabled = !policyRun.process_alive;
  showDirectBlock(manualPolicyRun, policyRun.enabled || policyRun.process_alive);
}

// ==== 7. 수동 조작: 팔 선택, 명령 목록, 확인 바, 정책으로 집기 ====
// 팔 명령은 관제 서버의 대기열에 들어가고, 그 팔의 워커(scripts/omx_lerobot_worker.py)가 long-poll로 가져가
// 수락해야 실제로 움직인다. 이 화면은 대기열에 넣고 상태를 보여 줄 뿐이다.
const armCommandForm = document.querySelector('#armCommandForm');
const armCommandMessage = document.querySelector('#armCommandMessage');
const armCommandRows = document.querySelector('#armCommandRows');
const armWorkerState = document.querySelector('#armWorkerState');
const armPicker = document.querySelector('#armPicker');
const manualNow = document.querySelector('#manualNow');
const manualNowArm = document.querySelector('#manualNowArm');
const armCommandCount = document.querySelector('#armCommandCount');
// 팔 명령 상태 이름(명령 기록 표용).
const armCommandStateLabels = {
  QUEUED: '대기 (워커 미수신)', DISPATCHED: '워커 수신 · 수락 대기', ACCEPTED: '수락',
  RUNNING: '실행 중', SUCCEEDED: '성공', FAILED: '실패', REJECTED: '워커 거부',
  CANCELED: '취소됨', EXPIRED: '만료', UNKNOWN: '결과 불명 · 현장 확인 필요',
};
// 최신 명령의 큰 상태 칩에 쓸 짧은 말과 색.
const armCommandStateShort = {
  QUEUED: ['워커 기다리는 중', 'attention'], DISPATCHED: ['수락 기다리는 중', 'attention'], ACCEPTED: ['수락됨', 'active'],
  RUNNING: ['움직이는 중', 'active'], SUCCEEDED: ['끝남', 'good'], FAILED: ['실패', 'danger'], REJECTED: ['워커 거부', 'danger'],
  CANCELED: ['취소됨', ''], EXPIRED: ['만료', ''], UNKNOWN: ['결과 불명', 'attention'],
};
const WORKER_LIVE_S = 30;  // 이 시간(초) 안에 폴링한 워커는 붙어 있다고 본다

const manual = {
  armId: 'arm-01',   // 버튼이 명령을 보낼 팔
  workers: [],       // GET /api/arm/commands에서 받음
  commands: [],
  policies: [],      // 학습된 정책 목록, 정책마다 녹화한 팔이 있다(arm_id null = 어느 팔이나)
  busy: false,       // 집기나 원상복귀 요청을 보내는 중
};

// ISO 시각을 시:분:초로 바꾼다. 값이 없으면 '-'. 다른 스크립트도 쓴다.
function formatClock(iso) {
  return iso ? clockFormat.format(new Date(iso)) : '-';
}

// 팔 목록은 주문 진행 뷰(/api/flow)를 거쳐 config/arms.json에서 온다. 그 응답이 오기 전에는 arm-01 하나로 대신한다.
function manualArms() {
  const arms = flowSummary.flow?.arms;
  return arms && arms.length ? arms : [{ arm_id: 'arm-01', label: '팔 1' }];
}

function workerFor(armId) {
  return manual.workers.find((item) => item.robot_id === armId);
}

// 워커가 지금 long-poll로 기다리는 중이거나 WORKER_LIVE_S초 안에 폴링했으면 붙어 있다고 본다.
function workerLive(worker) {
  if (!worker) return false;
  if (worker.waiting_polls > 0) return true;
  return (Date.now() - new Date(worker.last_poll_at).getTime()) / 1000 < WORKER_LIVE_S;
}

// 워커가 붙은 팔만 고를 수 있다. 어느 팔에도 워커가 없으면 arm-01은 고를 수 있게 남겨 둔다. 곧 시작될 워커를 위해
// 명령이 대기열에서 기다릴 수 있게 하려는 것이다.
function armSelectable(armId) {
  const anyLive = manual.workers.some(workerLive);
  return workerLive(workerFor(armId)) || (!anyLive && armId === 'arm-01');
}

// 팔 선택 버튼. 고른 팔을 더 고를 수 없게 되면 고를 수 있는 첫 팔로 옮긴다. HTML이 같으면 다시 쓰지 않는다
// (1초마다 갱신돼도 누르던 버튼이 바뀌지 않게).
function renderArmPicker() {
  const arms = manualArms();
  if (!armSelectable(manual.armId)) {
    const first = arms.find((arm) => armSelectable(arm.arm_id));
    if (first && first.arm_id !== manual.armId) {
      manual.armId = first.arm_id;
      renderPolicyTiles();
    }
  }
  const html = arms.map((arm) => {
    const selectable = armSelectable(arm.arm_id);
    const chosen = arm.arm_id === manual.armId;
    const live = workerLive(workerFor(arm.arm_id));
    return `<button type="button" role="radio" class="arm-choice${chosen ? ' chosen' : ''}" data-arm-id="${escapeHtml(arm.arm_id)}"
      aria-checked="${chosen}"${selectable ? '' : ' disabled'}>
      <span>${escapeHtml(arm.label)}</span><small>${live ? '워커 연결됨' : selectable ? '워커 기다리는 중' : '연결 전'}</small></button>`;
  }).join('');
  if (armPicker.dataset.html !== html) {
    armPicker.dataset.html = html;
    armPicker.innerHTML = html;
  }
}

// 팔을 바꾸면 그 팔 기준으로 정책 타일·워커 상태·최신 명령을 다시 그린다.
armPicker.addEventListener('click', (event) => {
  const button = event.target.closest('[data-arm-id]');
  if (!button || button.disabled || button.dataset.armId === manual.armId) return;
  manual.armId = button.dataset.armId;
  armCommandMessage.textContent = '';
  renderArmPicker();
  renderPolicyTiles();
  renderArmWorkers();
  renderManualNow();
});

function armLabel(armId) {
  return manualArms().find((arm) => arm.arm_id === armId)?.label || armId;
}

// 고른 팔의 워커 연결 상태 한 줄.
function renderArmWorkers() {
  const worker = workerFor(manual.armId);
  armWorkerState.className = `worker-state${workerLive(worker) ? ' live' : ''}`;
  if (!worker) {
    armWorkerState.textContent = '워커 연결 안 됨';
  } else if (worker.waiting_polls > 0) {
    armWorkerState.textContent = `워커 대기 중 · ${worker.worker_id}`;
  } else {
    const seconds = Math.round((Date.now() - new Date(worker.last_poll_at).getTime()) / 1000);
    armWorkerState.textContent = `워커 ${seconds}초 전 접속 · ${worker.worker_id}`;
  }
}

// 명령이 무엇을 하는지 한 줄(이미 이스케이프된 HTML을 돌려준다).
function commandWhat(command) {
  const target = command.target;
  return { MOVE_TO_SLOT: '칸 앞으로 이동', RETURN_HOME: '원상복귀' }[command.operation]
    || (target.policy ? `${escapeHtml(target.policy.label)} 집기` : `${escapeHtml(target.object_class)} → ${escapeHtml(command.place.slot_id)}`);
}

// 명령의 근거 위치: 칸(정면 캠으로 찾았으면 그 신뢰도), 칸 없는 정책 집기면 '정면 캠 확인 없음', 원상복귀면 'home 자세'.
function commandWhere(command) {
  const target = command.target;
  if (target.pick_slot_id) {
    const camera = command.source?.kind === 'CAMERA'
      ? ` · 정면 캠 ${escapeHtml(command.source.class_name)} ${Math.round(command.source.confidence * 100)}%`
      : '';
    return `칸 ${escapeHtml(target.slot_label || target.pick_slot_id)}${camera}`;
  }
  return target.policy ? '정면 캠 확인 없음' : 'home 자세';
}

// 명령에 대해 지금 누를 수 있는 버튼. 대기·진행 중이면 [취소](이미 요청했으면 '취소 요청됨'), 결과 불명(UNKNOWN)이고
// 아직 현장 확인 기록이 없으면 [현장 확인 완료].
function commandAction(command) {
  const inFlight = ['DISPATCHED', 'ACCEPTED', 'RUNNING'].includes(command.state);
  const id = escapeHtml(command.command_id);
  if (command.state === 'QUEUED' || (inFlight && !command.cancel_requested)) {
    return `<button type="button" class="secondary" data-arm-cancel="${id}">취소</button>`;
  }
  if (inFlight) return '<small>취소 요청됨</small>';
  if (command.state === 'UNKNOWN' && !command.operator_cleared_at) {
    return `<button type="button" class="secondary" data-arm-clear="${id}">현장 확인 완료</button>`;
  }
  return '';
}

// 고른 팔의 가장 최근 명령과, 그에 대해 할 일을 한곳에 보여 준다.
function renderManualNow() {
  manualNowArm.textContent = armLabel(manual.armId);
  const command = manual.commands.find((item) => item.robot_id === manual.armId);
  if (!command) {
    manualNow.innerHTML = '<p class="manual-empty">이 팔로 보낸 명령이 없어요</p>';
    manualNow.dataset.html = '';
    return;
  }
  const [word, tone] = armCommandStateShort[command.state] || [command.state, ''];
  const cleared = command.operator_cleared_at ? ` · 현장 확인 ${escapeHtml(formatClock(command.operator_cleared_at))}` : '';
  const reason = command.reason_code ? `<small class="manual-reason">${escapeHtml(command.reason_code)}</small>` : '';
  const simulated = command.result && command.result.simulated ? '<span class="simulated">모의</span>' : '';
  const html = `
    <div class="manual-now-row">
      <span class="chip ${tone}">${escapeHtml(word)}</span>${simulated}
      <span class="manual-now-action">${commandAction(command)}</span>
    </div>
    <p class="manual-now-what">${commandWhat(command)}</p>
    <p class="manual-now-meta">${commandWhere(command)} · ${escapeHtml(formatClock(command.created_at))} 보냄${cleared}</p>
    ${reason}`;
  if (manualNow.dataset.html !== html) {
    manualNow.dataset.html = html;
    manualNow.innerHTML = html;
  }
}

// 명령 기록 표(접혀 있는 '명령 기록'). 최근 명령부터.
function renderArmCommands(commands) {
  armCommandCount.textContent = commands.length ? `최근 ${commands.length}건` : '';
  if (!commands.length) {
    armCommandRows.innerHTML = '<tr><td colspan="5" class="loading">보낸 명령이 없습니다.</td></tr>';
    return;
  }
  armCommandRows.innerHTML = commands.map((command) => {
    const target = command.target;
    const what = target.policy && !['MOVE_TO_SLOT', 'RETURN_HOME'].includes(command.operation)
      ? `${commandWhat(command)}<small>${escapeHtml(target.policy.model)}</small>`
      : commandWhat(command);
    const simulated = command.result && command.result.simulated ? '<span class="simulated">모의</span>' : '';
    const reason = command.reason_code ? `<small>${escapeHtml(command.reason_code)}</small>` : '';
    const cleared = command.operator_cleared_at ? `<small>현장 확인 ${escapeHtml(formatClock(command.operator_cleared_at))}</small>` : '';
    return `<tr>
      <td>${escapeHtml(command.command_id)}<small>${escapeHtml(armLabel(command.robot_id))} · ${escapeHtml(formatClock(command.created_at))}</small></td>
      <td>${escapeHtml(armCommandStateLabels[command.state] || command.state)}${simulated}${reason}${cleared}</td>
      <td>${what}<small>${commandWhere(command)}</small></td>
      <td>${escapeHtml(command.worker_id || '-')}<small>${escapeHtml(formatClock(command.last_event_at || command.dispatched_at))}</small></td>
      <td>${commandAction(command)}</td>
    </tr>`;
  }).join('');
}

// 취소와 "현장 확인 완료"는 최신 명령 카드와 기록 표에서 똑같이 동작한다(이벤트 위임).
document.querySelector('#view-manual').addEventListener('click', (event) => {
  const cancel = event.target.closest('[data-arm-cancel]');
  if (cancel) {
    postArmCommand(`/api/arm/commands/${cancel.dataset.armCancel}/cancel`, {});
    return;
  }
  const clear = event.target.closest('[data-arm-clear]');
  if (clear) {
    const note = window.prompt('팔·물체 상태를 현장에서 확인한 내용을 적어 주세요. 결과는 결과 불명으로 남고, 새 명령만 다시 보낼 수 있게 됩니다.');
    if (note && note.trim()) postArmCommand(`/api/arm/commands/${clear.dataset.armClear}/clear-unknown`, { note: note.trim() });
  }
});

// GET /api/arm/commands: 워커 접속 상태와 최근 명령. 1초마다 부른다.
async function refreshArmCommands() {
  try {
    const response = await fetch('/api/arm/commands', { cache: 'no-store' });
    if (!response.ok) throw new Error();
    const data = await response.json();
    manual.workers = data.workers;
    manual.commands = data.commands;
    renderArmPicker();
    renderArmWorkers();
    renderManualNow();
    renderArmCommands(data.commands);
  } catch {
    armWorkerState.textContent = '명령 목록을 불러오지 못했어요';
  }
}

// 팔 명령 API에 POST하고 만든 명령을 돌려준다(실패하면 메시지를 보이고 null). 끝나면 항상 목록을 다시 읽는다.
async function postArmCommand(url, body) {
  armCommandMessage.textContent = '';
  try {
    const response = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, '요청에 실패했습니다.'));
    return result.command;
  } catch (error) {
    armCommandMessage.textContent = error.message;
    return null;
  } finally {
    await refreshArmCommands();
  }
}

// 팔을 움직이는 모든 명령이 쓰는 확인 바 하나. 확인 버튼을 눌렀을 때만 true로 끝난다.
const manualConfirm = document.querySelector('#manualConfirm');
const manualConfirmOk = document.querySelector('#manualConfirmOk');
let confirmAnswer = null;

// 이미 떠 있는 질문이 있으면 false로 끝내고 새 질문으로 바꾼다. 답은 Promise로 돌려준다.
function askConfirm({ title, detail, ok }) {
  if (confirmAnswer) confirmAnswer(false);
  document.querySelector('#manualConfirmTitle').textContent = title;
  document.querySelector('#manualConfirmDetail').textContent = detail;
  manualConfirmOk.textContent = ok;
  manualConfirm.hidden = false;
  manualConfirmOk.focus();
  return new Promise((resolve) => {
    confirmAnswer = (answer) => {
      confirmAnswer = null;
      manualConfirm.hidden = true;
      resolve(answer);
    };
  });
}

manualConfirmOk.addEventListener('click', () => confirmAnswer?.(true));
document.querySelector('#manualConfirmCancel').addEventListener('click', () => confirmAnswer?.(false));
manualConfirm.addEventListener('keydown', (event) => {
  if (event.key === 'Escape') confirmAnswer?.(false);
});

const cameraCommandForm = document.querySelector('#cameraCommandForm');
const policyButtons = document.querySelector('#policyButtons');
cameraCommandForm.addEventListener('submit', async (event) => {
  // 서버가 정면 카메라로 칸을 고른다. 물건이 정확히 한 칸에 있을 때만 받아들인다.
  event.preventDefault();
  const button = document.querySelector('#cameraSlotCommand');
  const objectClass = cameraCommandForm.elements.object_class.value.trim();
  button.disabled = true;
  armCommandMessage.textContent = '';
  try {
    const response = await fetch('/api/arm/commands/from-camera', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ object_class: objectClass }),
    });
    const result = await response.json();
    if (!response.ok) {
      const detail = result.details ? ` (${JSON.stringify(result.details)})` : '';
      throw new Error(`${apiErrorMessage(result, '칸을 정하지 못했습니다.')}${detail}`);
    }
    const found = result.selection;
    armCommandMessage.textContent = `${found.class_name} ${Math.round(found.confidence * 100)}% → ${found.slot_label} · ${result.command.command_id} 등록, 워커가 가져갈 때까지 대기`;
  } catch (error) {
    armCommandMessage.textContent = error.message;
  } finally {
    button.disabled = false;
    await refreshArmCommands();
  }
});

// 정면 캠 확인이 이 이유로 거절되면 작업자가 그래도 정책을 실행할 수 있다. 정책은 물건을 스스로 찾기 때문이다.
const cameraCheckErrors = new Set([
  'CAMERA_NOT_FOUND', 'CAMERA_NOT_FRESH', 'DETECTOR_NOT_READY', 'SLOT_ZONES_NOT_READY',
  'TARGET_NOT_FOUND', 'TARGET_AMBIGUOUS', 'TARGET_MULTIPLE',
]);

// 요청을 보내는 동안 정책 타일과 원상복귀 버튼을 막아 두 번 누르지 않게 한다.
function setManualBusy(busy) {
  manual.busy = busy;
  policyButtons.querySelectorAll('button').forEach((button) => { button.disabled = busy; });
  document.querySelector('#armReturnHome').disabled = busy;
}

// 정면 캠 근거 없이 PICK_WITH_POLICY 명령을 바로 대기열에 넣는다. 왜 확인 없이 보냈는지 source.note에 남긴다.
async function sendPolicyWithoutCamera(policy, armId, why) {
  const command = await postArmCommand('/api/arm/commands', {
    robot_id: armId,
    operation: 'PICK_WITH_POLICY',
    valid_for_s: 60,
    target: { object_class: policy.object_class },
    place: { slot_id: 'CART_1' },
    source: { kind: 'OPERATOR_INPUT', note: `정면 캠 확인 없이 실행 (${why})` },
  });
  if (command) armCommandMessage.textContent = `${policy.label} 집기를 보냈어요 · 정면 캠 확인 없음 · 워커가 가져가면 움직여요`;
}

async function pickWithPolicy(policy) {
  // 1) 작업자에게 확인한다: 팔이 움직인다. 2) 서버에 정면 카메라에서 물건을 찾아 달라고 한다(칸이 명령의 근거로
  // 남는다). 3) 카메라로 확인하지 못하면, 확인 없이 보내기 전에 한 번 더 묻는다.
  const armId = manual.armId;
  if (manual.busy) return;
  const go = await askConfirm({
    title: `${armLabel(armId)}로 ${policy.label} 집기`,
    detail: '팔이 움직여요. 팔 주변과 핑키 위가 비어 있는지 확인해 주세요.',
    ok: '집기 시작',
  });
  if (!go) return;
  setManualBusy(true);
  armCommandMessage.textContent = `${policy.label}: 정면 캠에서 확인하는 중`;
  try {
    if (armId !== 'arm-01') {
      // 서버의 정면 캠 확인은 arm-01에만 명령을 보내므로, 다른 팔은 카메라 근거 없이 보낸다.
      await sendPolicyWithoutCamera(policy, armId, '정면 캠 확인은 arm-01만 지원');
      return;
    }
    // arm-01이면 서버가 정면 캠에서 물건을 찾아 칸을 근거로 붙인 PICK_WITH_POLICY 명령을 만든다.
    const response = await fetch('/api/arm/commands/from-camera', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ object_class: policy.object_class, operation: 'PICK_WITH_POLICY', robot_id: robotId }),
    });
    const result = await response.json();
    if (response.ok) {
      const found = result.selection;
      armCommandMessage.textContent = `${policy.label} 집기를 보냈어요 · 정면 캠 ${Math.round(found.confidence * 100)}% @ ${found.slot_label} · 워커가 가져가면 움직여요`;
      return;
    }
    const reason = apiErrorMessage(result, '정면 캠 확인 실패');
    if (!cameraCheckErrors.has(result.error_code)) {
      armCommandMessage.textContent = reason;
      return;
    }
    const anyway = await askConfirm({
      title: `정면 캠에서 ${policy.label}을(를) 확인하지 못했어요`,
      detail: `${reason} 정책은 자기 카메라로 물건을 찾으니 확인 없이 실행할 수도 있어요. 팔이 움직여요.`,
      ok: '확인 없이 집기',
    });
    if (anyway) await sendPolicyWithoutCamera(policy, armId, result.error_code);
    else armCommandMessage.textContent = reason;
  } catch (error) {
    armCommandMessage.textContent = error.message;
  } finally {
    setManualBusy(false);
    await refreshArmCommands();
  }
}

// 학습 정책 목록 요청은 한 번만 보내고, 정책 버튼과 주문의 물체 이름표가 함께 쓴다.
const policyListRequest = fetch('/api/arm/policies', { cache: 'no-store' }).then(async (response) => {
  const result = await response.json();
  if (!response.ok) throw new Error(apiErrorMessage(result, '정책 목록을 불러오지 못했습니다'));
  return result.policies;
});

// 정책은 녹화한 팔에서만 돈다. arm_id가 null이면 한 팔에 묶이지 않은 정책이다.
function renderPolicyTiles() {
  const policies = manual.policies.filter((policy) => !policy.arm_id || policy.arm_id === manual.armId);
  if (!policies.length) {
    const text = manual.policies.length ? `${armLabel(manual.armId)}에서 녹화한 정책이 없어요` : '정책 목록 확인 중';
    policyButtons.innerHTML = `<p class="manual-empty">${escapeHtml(text)}</p>`;
    return;
  }
  policyButtons.replaceChildren(...policies.map((policy) => {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'pick-tile';
    button.disabled = manual.busy;
    button.title = `${policy.model} (${policy.object_class})`;
    const mark = document.createElement('span');
    mark.className = 'pick-mark';
    mark.setAttribute('aria-hidden', 'true');
    mark.textContent = [...policy.label][0] || '?';
    const name = document.createElement('span');
    name.className = 'pick-name';
    name.textContent = policy.label;
    const meta = document.createElement('small');
    meta.textContent = policy.max_run_s ? `최대 ${policy.max_run_s}초` : policy.object_class;
    button.append(mark, name, meta);
    button.addEventListener('click', () => pickWithPolicy(policy));
    return button;
  }));
}

// 정책 목록을 받아 정책 타일을 그린다.
async function loadPolicyButtons() {
  try {
    manual.policies = await policyListRequest;
    renderPolicyTiles();
  } catch (error) {
    policyButtons.innerHTML = `<p class="manual-empty">${escapeHtml(error.message)}</p>`;
  }
}

// ==== 8. 수동 조작(개발용, ?dev=1): 정책 연속 실행 ====
// 고른 물체들의 정책을 순서대로 한 번씩 실행한다(POST /api/arm/sequences). 각 팔이 초기 자세로 돌아오면 다음 단계로 간다.
const sequenceChoices = document.querySelector('#sequenceChoices');
const sequenceOrder = document.querySelector('#sequenceOrder');
const startSequenceButton = document.querySelector('#startSequence');
const sequenceStatus = document.querySelector('#sequenceStatus');
const sequenceRequestMessage = document.querySelector('#sequenceRequestMessage');
const sequenceStepDetails = document.querySelector('#sequenceStepDetails');
const sequenceRunActions = document.querySelector('#sequenceRunActions');
// sequencePolicies: 고를 수 있는 정책 목록, selectedSequencePolicyIds: 체크한 순서대로의 정책 id(최대 4개),
// sequenceRunActive: 마지막 연속 실행이 아직 진행 중인지(그동안 시작 버튼을 막는다).
let sequencePolicies = [];
let selectedSequencePolicyIds = [];
let sequenceRunActive = false;

// 정책 목록이 오면 연속 실행 선택지를 그린다. 파일을 읽을 때 바로 한 번 부른다.
async function loadSequencePolicies() {
  try {
    sequencePolicies = await policyListRequest;
    renderSequenceBuilder();
    if (sequenceStatus) sequenceStatus.textContent = '정책 목록을 불러왔습니다. 실행할 물체를 선택하세요.';
  } catch (error) {
    if (sequenceStatus) sequenceStatus.textContent = error.message;
  }
}

loadSequencePolicies();

function sequenceArmLabel(armId) {
  if (armId === 'arm-02') return '오른팔';
  if (armId === 'arm-01') return '왼팔';
  return '선반 담당 팔';
}

// 체크 상자(물체 선택, 최대 4개)와 실행 순서 목록(↑ ↓로 순서 바꾸기)을 다시 그린다.
// 실행 중이거나 고른 것이 없으면 시작 버튼을 막는다.
function renderSequenceBuilder() {
  if (!sequenceChoices || !sequenceOrder) return;
  sequenceChoices.replaceChildren(...sequencePolicies.map((policy) => {
    const label = document.createElement('label');
    label.className = 'sequence-choice';
    const checkbox = document.createElement('input');
    checkbox.type = 'checkbox';
    checkbox.checked = selectedSequencePolicyIds.includes(policy.policy_id);
    checkbox.addEventListener('change', () => {
      if (checkbox.checked) {
        if (selectedSequencePolicyIds.length >= 4) {
          checkbox.checked = false;
          armCommandMessage.textContent = '한 번에 최대 4개 물체까지 선택할 수 있습니다.';
          return;
        }
        selectedSequencePolicyIds.push(policy.policy_id);
      } else {
        selectedSequencePolicyIds = selectedSequencePolicyIds.filter((id) => id !== policy.policy_id);
      }
      renderSequenceBuilder();
    });
    const text = document.createElement('span');
    text.textContent = `${policy.label} · ${sequenceArmLabel(policy.arm_id)}`;
    label.append(checkbox, text);
    return label;
  }));
  sequenceOrder.replaceChildren(...selectedSequencePolicyIds.map((policyId, index) => {
    const policy = sequencePolicies.find((item) => item.policy_id === policyId);
    if (!policy) return null;
    const item = document.createElement('li');
    const text = document.createElement('span');
    text.textContent = `${policy.label} · ${sequenceArmLabel(policy.arm_id)}`;
    item.append(text);
    for (const [delta, symbol, description] of [[-1, '↑', '앞으로'], [1, '↓', '뒤로']]) {
      const button = document.createElement('button');
      button.type = 'button';
      button.className = 'secondary';
      button.textContent = symbol;
      button.title = `${policy.label} 순서 ${description}`;
      button.setAttribute('aria-label', button.title);
      button.disabled = index + delta < 0 || index + delta >= selectedSequencePolicyIds.length;
      button.addEventListener('click', () => {
        const next = [...selectedSequencePolicyIds];
        [next[index], next[index + delta]] = [next[index + delta], next[index]];
        selectedSequencePolicyIds = next;
        renderSequenceBuilder();
      });
      item.append(button);
    }
    return item;
  }).filter(Boolean));
  if (startSequenceButton) startSequenceButton.disabled = sequenceRunActive || selectedSequencePolicyIds.length === 0;
}

// 고른 순서를 확인 창으로 보여 준 뒤 POST /api/arm/sequences로 연속 실행을 시작한다.
startSequenceButton?.addEventListener('click', async () => {
  const steps = selectedSequencePolicyIds
    .map((id) => sequencePolicies.find((policy) => policy.policy_id === id))
    .filter(Boolean);
  if (!steps.length || sequenceRunActive) return;
  const order = steps.map((policy, index) => `${index + 1}. ${policy.label} (${sequenceArmLabel(policy.arm_id)})`).join('\n');
  if (!window.confirm(`다음 순서로 정책을 실행합니다.\n\n${order}\n\n각 단계에서 팔이 초기 자세로 돌아오면 다음 정책을 실행합니다. 물건 적재 여부는 확인하지 않습니다. 로봇 주변을 비웠는지 확인하세요.`)) return;
  startSequenceButton.disabled = true;
  if (sequenceRequestMessage) sequenceRequestMessage.textContent = '';
  try {
    const response = await fetch('/api/arm/sequences', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ object_classes: steps.map((policy) => policy.object_class) }),
    });
    const payload = await response.json();
    if (!response.ok) {
      const blocker = payload.command ? ` · 차단 명령 ${payload.command.command_id} (${payload.command.state})` : '';
      throw new Error(`${apiErrorMessage(payload, '연속 실행을 시작하지 못했습니다')}${blocker}`);
    }
    armCommandMessage.textContent = `${steps.map((policy) => policy.label).join(' → ')} 시작 · ${payload.sequence.sequence_run_id}`;
    await refreshSequences();
    await refreshArmCommands();
  } catch (error) {
    armCommandMessage.textContent = error.message;
    if (sequenceRequestMessage) sequenceRequestMessage.textContent = error.message;
  } finally {
    renderSequenceBuilder();
  }
});

// GET /api/arm/sequences의 latest(가장 최근 연속 실행)를 단계별로 그린다. 지금 단계의 팔 명령은
// GET /api/arm/commands/{id}로 따로 읽어 상태와 마지막 이벤트의 phase를 보여 준다. 1초마다 부른다.
async function refreshSequences() {
  if (!sequenceStatus) return;
  try {
    const response = await fetch('/api/arm/sequences', { cache: 'no-store' });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, '연속 실행 목록을 불러오지 못했습니다'));
    const run = result.latest;
    sequenceRunActive = Boolean(run && ['QUEUED', 'DISPATCHED', 'RUNNING', 'CANCELING'].includes(run.state));
    if (startSequenceButton) startSequenceButton.disabled = sequenceRunActive || selectedSequencePolicyIds.length === 0;
    sequenceRunActions?.replaceChildren();
    if (run) {
      // 적재(placement_confirmed)를 확인하지 못한 결과가 하나라도 있으면 '적재 여부 미확인'을 덧붙인다.
      const placementUnknown = run.results.some((item) => item.placement_confirmed !== true);
      const runStateLabels = {
        QUEUED: '워커 대기 중', DISPATCHED: '워커에 전달됨', RUNNING: '정책 실행 중',
        CANCELING: '취소 처리 중', COMPLETED: '모든 단계 완료',
        NEEDS_INTERVENTION: '중단됨 · 현장 확인 필요', CANCELED: '취소됨',
      };
      sequenceStatus.textContent = `${run.sequence_id} · ${runStateLabels[run.state] || run.state}${placementUnknown && run.results.length ? ' · 적재 여부 미확인' : ''}`;

      const currentCommand = run.current_command_id
        ? await fetch(`/api/arm/commands/${encodeURIComponent(run.current_command_id)}`, { cache: 'no-store' })
          .then(async (response) => {
            const payload = await response.json();
            if (!response.ok) throw new Error(apiErrorMessage(payload, '현재 팔 명령을 불러오지 못했습니다'));
            return payload.command;
          })
        : null;
      const commandStateLabels = {
        QUEUED: '팔 워커 대기 중', DISPATCHED: '워커가 명령을 가져감', ACCEPTED: '팔 워커가 수락함',
        RUNNING: '정책 실행 중', CANCELING: '정지 요청 처리 중', SUCCEEDED: '완료',
        UNKNOWN: '결과 미확인', FAILED: '실패', REJECTED: '실행 거부', CANCELED: '취소됨', EXPIRED: '명령 만료',
      };
      const phaseLabels = {
        APPROACH_TARGET: '물체 접근 중', RETURNING_HOME: '초기자세 복귀 중',
        VERIFY_TRANSFER: '물체 이동 확인 중',
      };
      // 단계마다: 끝난 단계는 결과(result)로, 지금 단계는 현재 명령 상태로, 나머지는 '완료'/'대기'로 쓴다.
      const stepItems = run.steps.map((step, index) => {
        const item = document.createElement('li');
        const result = run.results.find((entry) => entry.step === index);
        const isCurrent = index === run.current_step && !result;
        const state = result?.state || (isCurrent ? currentCommand?.state : null);
        const stateText = result
          ? result.home_confirmed === true
            ? '초기자세 복귀 확인'
            : commandStateLabels[result.state] || result.state
          : isCurrent
            ? `${commandStateLabels[currentCommand?.state] || runStateLabels[run.state] || run.state}${currentCommand?.events?.at(-1)?.payload?.phase ? ` · ${phaseLabels[currentCommand.events.at(-1).payload.phase] || currentCommand.events.at(-1).payload.phase}` : ''}`
            : index < run.current_step ? '완료' : '대기';
        const heading = document.createElement('strong');
        heading.textContent = `${index + 1}. ${step.label} · ${sequenceArmLabel(step.robot_id || run.robot_id)} · ${stateText}`;
        item.append(heading);

        const detail = [currentCommand?.result?.message, result?.reason_code || currentCommand?.result?.reason_code]
          .filter(Boolean).join(' · ')
          || (result?.home_confirmed === true ? '초기자세 복귀 확인' : '')
          || (result?.placement_confirmed === true ? '카트 적재 확인' : '');
        if (detail) {
          const note = document.createElement('small');
          note.textContent = detail;
          item.append(note);
        }
        item.dataset.state = state || (result ? 'COMPLETED' : isCurrent ? run.state : 'WAITING');
        return item;
      });
      sequenceStepDetails?.replaceChildren(...stepItems);
      // 아직 끝나지 않은 실행이면 취소 버튼을 단다(POST /api/arm/sequences/{id}/cancel).
      if (['QUEUED', 'DISPATCHED', 'RUNNING', 'CANCELING'].includes(run.state)) {
        const cancel = document.createElement('button');
        cancel.type = 'button'; cancel.className = 'secondary'; cancel.textContent = run.state === 'QUEUED' ? '대기 중인 연속 실행 취소' : '연속 실행 취소 요청';
        cancel.addEventListener('click', async () => {
          const response = await fetch(`/api/arm/sequences/${run.sequence_run_id}/cancel`, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
          const payload = await response.json();
          if (!response.ok) armCommandMessage.textContent = apiErrorMessage(payload, '취소 요청 실패');
          await refreshSequences();
        });
        sequenceRunActions?.append(cancel);
      }
    } else {
      sequenceStatus.textContent = '실행 이력 없음';
      sequenceStepDetails?.replaceChildren();
    }
  } catch (error) {
    sequenceStatus.textContent = error.message;
  }
}
refreshSequences();

// ==== 9. 주문 표시 공통(물체 이름·핑키 배지)과 토스트 알림 ====
// ---- 주문 표시 공통 (물체 이름, 핑키 배지) ----
// 핑키 id → 화면 이름(P0~P2). 다른 스크립트도 쓴다.
const pinkyNames = { 10: 'P0', 11: 'P1', 12: 'P2' };
let objectLabels = new Map();  // object_class -> 이름(사과, 당근…), 정책 목록에서 채운다
policyListRequest
  .then((policies) => { objectLabels = new Map(policies.map((policy) => [policy.object_class, policy.label])); })
  .catch(() => {});  // 오류는 정책 버튼이 이미 보여 준다. 이름은 클래스 이름으로 대신한다

// 카메라 검출 클래스의 한국어 이름. 정책 목록의 이름보다 먼저 쓴다.
const cameraObjectLabels = new Map(Object.entries({
  strawberry: '딸기', car: '자동차', ball: '공', pumpkin: '호박', apple: '사과',
  carrot: '당근', fruits: '열매 송이', potato: '감자', watermelon: '수박',
  tissue: '물티슈', kiwi: '키위', pepper: '고추', pinky: '핑키',
}));

// 물체 클래스 → 화면 이름. 카메라 이름표 → 정책 이름표 → 클래스 이름 순으로 찾는다.
function objectLabel(objectClass) {
  return cameraObjectLabels.get(objectClass) || objectLabels.get(objectClass) || objectClass;
}

// 주문 물건 목록을 '사과 2, 당근 1' 같은 한 줄로 만든다.
function orderSummary(items) {
  return (items || []).map((item) => `${objectLabel(item.object_class)} ${item.quantity}`).join(', ');
}

// P0~P2 표시: 핑키가 응답하면 파랑, 응답하지 않으면 회색.
function pinkyBadge(pinkyId, { offline = false } = {}) {
  if (!pinkyId) return '';
  return `<span class="pinky-badge${offline ? ' offline' : ''}">${escapeHtml(pinkyNames[pinkyId] || pinkyId)}</span>`;
}

// 화면 구석에 잠깐 뜨는 알림(새 주문, 시작 결과, 거절된 제어). 저절로 사라진다.
// kind: info | warning | error. detail 안의 줄바꿈은 새 줄로 보인다.
const toastStack = document.querySelector('#toastStack');
const TOAST_MS = 5000;

function showToast(title, detail = '', kind = 'info', durationMs = TOAST_MS) {
  const toast = document.createElement('div');
  toast.className = `toast ${kind}`;
  toast.innerHTML = `<strong>${escapeHtml(title)}</strong>${detail ? `<span>${escapeHtml(detail)}</span>` : ''}`;
  toastStack.append(toast);
  window.setTimeout(() => toast.remove(), durationMs);
}

// ==== 10. 수동 조작: 칸으로 이동(개발용)과 원상복귀 ====
// 워커는 고른 칸에 대해 가르쳐 둔 관절 자세를 다시 재생한다. 좌표는 보내지 않는다.
const armCommandSlot = document.querySelector('#armCommandSlot');

// GET /api/arm/shelf-slots의 활성 칸으로 '칸' 선택지를 채운다. 못 읽으면 선택지를 막고 이유를 title에 둔다.
async function loadShelfSlots() {
  try {
    const response = await fetch('/api/arm/shelf-slots', { cache: 'no-store' });
    const data = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(data, '칸 정의를 불러오지 못했습니다.'));
    data.slot_map.slots.filter((slot) => slot.active).forEach((slot) => {
      const option = document.createElement('option');
      option.value = slot.slot_id;
      option.textContent = slot.label;
      armCommandSlot.append(option);
    });
  } catch (error) {
    armCommandSlot.disabled = true;
    armCommandSlot.title = error.message;
  }
}

// 칸으로 이동(개발용): MOVE_TO_SLOT 명령. 대상 종류와 놓을 칸은 기록만 한다.
armCommandForm.addEventListener('submit', async (event) => {
  event.preventDefault();
  const button = armCommandForm.querySelector('button');
  const formData = new FormData(armCommandForm);
  button.disabled = true;
  const command = await postArmCommand('/api/arm/commands', {
    robot_id: manual.armId,
    operation: 'MOVE_TO_SLOT',
    valid_for_s: Number(formData.get('valid_for_s')),
    target: { object_class: String(formData.get('object_class') || '').trim() || null, pick_slot_id: armCommandSlot.value },
    place: { slot_id: formData.get('slot_id') },
    source: { kind: 'OPERATOR_INPUT' },
  });
  if (command) armCommandMessage.textContent = `${command.command_id} 등록 · 워커가 가져갈 때까지 대기`;
  button.disabled = false;
});

document.querySelector('#armReturnHome').addEventListener('click', async () => {
  // 칸도 놓을 곳도 없다: 워커가 기록해 둔 home 자세로 팔을 되돌린다.
  const armId = manual.armId;
  if (manual.busy) return;
  const go = await askConfirm({
    title: `${armLabel(armId)} 원상복귀`,
    detail: '기록해 둔 home 자세로 팔이 움직여요. 팔이 물건을 잡고 있으면 떨어질 수 있어요.',
    ok: '돌려보내기',
  });
  if (!go) return;
  setManualBusy(true);
  const command = await postArmCommand('/api/arm/commands', {
    robot_id: armId,
    operation: 'RETURN_HOME',
    valid_for_s: 60,
    source: { kind: 'OPERATOR_INPUT' },
  });
  if (command) armCommandMessage.textContent = '원상복귀를 보냈어요 · 워커가 가져가면 움직여요';
  setManualBusy(false);
});

// ==== 11. 작업 이력용 상태·단계·이벤트 이름표 ====
const taskStatusLabels = {
  PENDING: '대기', VALIDATING: '검증 중', RUNNING: '실행 중',
  SUCCEEDED: '성공', FAILED: '실패', CANCELING: '취소 중',
  CANCELED: '취소됨', NEEDS_INTERVENTION: '확인 필요',
};
const stepLabels = {
  VALIDATE_ROBOTS: '로봇 상태 확인', NAVIGATE_TO_DESTINATION: '목적지로 이동',
  VERIFY_ARRIVAL: '도착 확인', EXECUTE_ARM_TASK: '팔 작업',
  VERIFY_ARM_RESULT: '팔 작업 결과 확인', VERIFY_LOAD: '적재 확인', FINALIZE_TASK: '작업 마무리',
};
const executionLabels = {
  COMMAND_SENT: '명령 기록', ACCEPTED: '접수됨', RUNNING: '실행 중',
  SUCCEEDED: '성공', FAILED: '실패', TIMED_OUT: '시간 초과',
  CANCELED: '취소됨', UNKNOWN: '결과 불명', PENDING: '대기',
  QUEUED: '워커 대기열', DISPATCHED: '워커가 가져감', REJECTED: '워커 거부', EXPIRED: '만료',
};
// 단계 결과(result.outcome)의 이름. 운영 화면에서는 모의 완료도 '완료'로 보인다.
const stepOutcomeLabels = {
  SIMULATED_SUCCESS: devMode ? '모의 완료' : '완료',
  CANCEL_CONFIRMED: '취소 확인',
  NOT_EXECUTED_AFTER_CANCEL: '취소 뒤 미실행',
  NOT_EXECUTED_AFTER_FAILURE: '실패 뒤 미실행',
  PARKING_ARRIVED: '주차 판정 도착',
  LOAD_CONFIRMED: '적재 수량 확인',
  ARM_HOME_PLACEMENT_NOT_VERIFIED: '팔 home 복귀 · 적재 미확인',
  ARM_HOME_PLACEMENT_LEFT_TO_VERIFY_LOAD: '적재 판단은 적재 확인 단계로',
};
// 작업 이벤트 이름. 모의 관련 말은 개발자 화면에서만 붙는다(devWord).
const taskEventLabels = {
  MOCK_SCENARIO_FAILURE: `${devWord('모의 ')}실패 시나리오 발생`,
  MOCK_EXECUTION_FAILED: `${devWord('모의 ')}실행 오류`,
  MOCK_TASK_COMPLETED: `${devWord('모의 ')}작업 완료`,
  ARRIVAL_NOT_CONFIRMED: '도착 미확인 · 팔 단계 시작 안 함',
  LOAD_NOT_CONFIRMED: '적재 미확인 · 출발하지 않음',
  TARGET_CONFIRMED: '정면 카메라 물체 확인',
  ARM_COMMAND_CREATED: 'OMX 명령 등록',
  ARM_COMMAND_FINISHED: 'OMX 명령 종료',
  ARM_RESULT_RECORDED: '팔 결과 기록',
  ARM_TASK_NOT_CONFIRMED: '팔 단계 미확인 · 다음 단계 진행 안 함',
};
const loadStatusLabels = {
  LOADED: '다 실림',
  INCOMPLETE: '덜 실림',
  EXCESS: '더 실림',
  NOT_VERIFIED: '확인 불가',
};

// LOAD_OBSERVED 이벤트(적재 확인)를 한 줄로: 상태, 기대 개수, 관측 개수.
function formatLoadObservation(payload) {
  const observation = payload.observation || {};
  const observed = observation.observed_count == null ? '개수 없음' : `${observation.observed_count}개`;
  return `적재 확인 · ${loadStatusLabels[payload.status] || payload.status} · 기대 ${payload.expected_count}개 · 관측 ${observed}${observation.simulated ? devWord(' · 모의 관측(카메라 미사용)') : ''}`;
}
// 확정 주차 판정(status)과 벗어난 정도(deviation_level)의 이름.
const parkingStatusLabels = {
  ARRIVED: '도착',
  MISALIGNED: '어긋남',
  NOT_FOUND: '확인 불가(마커 안 보임)',
  AMBIGUOUS: '확인 불가(같은 마커 여러 개)',
  VIEW_UNVERIFIED: '확인 불가(카메라 시야 미확인)',
};
const parkingDeviationLabels = { WITHIN: '범위 안', NEAR: '조금 벗어남', FAR: '많이 벗어남' };

// PARKING_OBSERVED 이벤트의 판정 결과를 한 줄로. 핑키 기준 축(forward/left)이 있으면 그것을, 없으면 마커 기준 dx/dy를 쓴다.
function formatParkingObservation(observation) {
  const parts = [`주차 판정 · ${observation.spot_id} · ${parkingStatusLabels[observation.status] || observation.status}`];
  const offset = observation.offset;
  if (offset) {
    if (observation.deviation_level) {
      parts.push(parkingDeviationLabels[observation.deviation_level] || observation.deviation_level);
    }
    parts.push(offset.forward_mm != null
      ? `앞 ${Number(offset.forward_mm).toFixed(1)} mm · 왼쪽 ${Number(offset.left_mm).toFixed(1)} mm`
      : `dx ${Number(offset.dx_mm).toFixed(1)} mm · dy ${Number(offset.dy_mm).toFixed(1)} mm`);
    parts.push(`yaw ${Number(offset.dyaw_deg).toFixed(1)}°`);
    if (observation.tolerance) {
      parts.push(`허용 ${observation.tolerance.position_mm} mm / ${observation.tolerance.yaw_deg}°`);
    }
  }
  if (observation.status !== 'ARRIVED' && observation.reason) parts.push(observation.reason);
  parts.push(`시야 확인 ${observation.view_check}`, `프레임 ${observation.frames_used}장`);
  return parts.join(' · ');
}

// ==== 12. 장비·카메라: 핑키 도착 확인(주차 판정) 패널 ====
// 천장 카메라로 핑키 윗면 ArUco 마커를 재서 등록된 주차 위치와 비교한다. 결과는 두 가지다.
//   - 실시간(GET /api/parking-spots/{spot}/live): C++ 주차 노드가 계속 추적하는 상태. 0.5초마다, 이 뷰가 보일 때만 읽는다.
//   - 확정 판정(GET /api/parking-spots/{spot}/check): [확정 판정]을 누르면 새 프레임 여러 장으로 한 번 판정한다.
// 실시간 추적이 되면 패널은 실시간 상태를 보여 주고, 확정 판정은 그 아래 한 줄(parkingConfirm)로만 남는다.
// 이 패널은 로봇에 명령을 보내지 않는다.
//
// parkingBusy: 확정 판정 요청 중, parkingLiveBusy: 실시간 요청 중(겹쳐 보내지 않는다),
// parkingLive: 서버가 실시간 추적을 켰는지(health.parking.live_tracking), parkingStreamCamera: 지금 <img>가 여는 카메라.
let parkingBusy = false;
let parkingLiveBusy = false;
let parkingLive = false;
let parkingStreamCamera = null;
let parkingLiveOk = false;  // 마지막 /live 폴링이 성공해 패널이 실시간 상태를 보여 주는 중
// 판정 단계별 색: 윗면 그림(밝은 배경)용과 카메라 영상 위 그림(어두운 배경)용.
const parkingColors = { within: '#2f8f4e', near: '#c98a1a', far: '#bd4242', moving: '#3478cb', unknown: '#7a838d' };
const parkingOverlayColors = { within: '#3fbf6a', near: '#f0a830', far: '#f05a5a', moving: '#5aa2f0', unknown: '#c3c9cf' };
// 실시간 상태(live.state)의 제목.
const parkingLiveTitles = {
  NOT_IN_VIEW: '화면 밖 · Pinky 마커가 보이지 않음',
  MOVING: '이동 중',
  IN_POSITION: '정상 위치',
  OFF_POSITION: '위치 벗어남',
  AMBIGUOUS: '확인 불가 · 같은 마커가 여러 개',
  VIEW_UNVERIFIED: '확인 불가 · 카메라 시야가 등록 때와 다름',
  CAMERA_UNAVAILABLE: '확인 불가 · 카메라 영상 없음',
  SPOT_INVALID: '확인 불가 · 주차 위치 파일 오류',
  TRACKING_ERROR: '확인 불가 · 추적 프로그램 오류',
  NODE_UNAVAILABLE: '확인 불가 · 주차 판정 노드 연결 안 됨',
};
// 판정하지 못한 이유별로 작업자가 할 일.
const parkingHelp = {
  NOT_IN_VIEW: 'Pinky가 주차 구역 밖에 있거나 윗면 마커가 사람·물건에 가려졌습니다.',
  NOT_FOUND: 'Pinky가 주차 구역에 있는지, 윗면 마커가 사람·물건에 가려졌는지 확인하세요.',
  AMBIGUOUS: '화면에 같은 ID 마커가 둘 이상 있습니다. 다른 Pinky나 인쇄물을 치우세요.',
  VIEW_UNVERIFIED: '영상이 밀린 채 열렸으면 관제 서버를 다시 시작해 카메라를 다시 엽니다. 카메라를 건드렸으면 주차 위치를 다시 등록하세요.',
  CAMERA_UNAVAILABLE: '관제 서버의 Head Camera 연결을 확인하세요.',
  SPOT_INVALID: '주차 위치를 다시 등록하거나 등록 파일을 확인하세요.',
  TRACKING_ERROR: '카메라나 주차 위치 문제가 아니라 관제의 추적 코드 오류입니다. 서버 로그와 사유를 개발자에게 전달하세요.',
  NODE_UNAVAILABLE: 'C++ 주차 판정 노드(/pinky_parking)가 꺼졌거나 다른 ROS domain에 있습니다. ceiling_parking.launch.py 실행 여부를 확인하세요.',
};

function signedNumber(value, digits = 1) {
  const number = Number(value);
  return `${number > 0 ? '+' : ''}${number.toFixed(digits)}`;
}

// 오차 정도를 색 단계(within/near/far)로 바꾼다. 오프셋이 없으면 unknown.
function parkingLevel(data) {
  if (!data?.offset) return 'unknown';
  return { WITHIN: 'within', NEAR: 'near', FAR: 'far' }[data.deviation_level] || 'unknown';
}

// 오프셋을 앞/왼쪽 축으로 꺼낸다. 핑키 기준 값(forward_mm/left_mm)이 없는 예전 응답은 마커 기준 dx/dy로 대신한다.
function parkingAxes(offset) {
  const pinkyAxes = offset.forward_mm != null;
  return {
    pinkyAxes,
    forward: Number(pinkyAxes ? offset.forward_mm : offset.dx_mm),
    left: Number(pinkyAxes ? offset.left_mm : offset.dy_mm),
  };
}

// 핑키 자신의 축으로 본 작은 윗면 그림: 위 = 핑키 앞, 왼쪽 = 핑키 왼쪽, 반시계 yaw면 화살표가 왼쪽으로 돈다.
// 허용 범위와 오프셋이 모두 들어오도록 축척을 정한다(그림 반지름 140).
function drawParkingMap(data, level) {
  const tolerance = Number(data?.tolerance?.position_mm) || 50;
  const offset = data?.offset;
  const { pinkyAxes, forward, left } = offset ? parkingAxes(offset) : { pinkyAxes: true, forward: 0, left: 0 };
  const scale = 140 / Math.max(tolerance * 2.3, Math.abs(forward) * 1.25, Math.abs(left) * 1.25, 40);
  const toleranceRadius = tolerance * scale;
  const nearRadius = 2 * toleranceRadius;
  const color = parkingColors[level] || parkingColors.unknown;
  const arrow = 'M0,-22 L15,16 L0,8 L-15,16 Z';
  const x = -left * scale;
  const y = -forward * scale;
  parkingMap.innerHTML = `
    <line x1="-150" y1="0" x2="150" y2="0" stroke="#e1e5ea"/><line x1="0" y1="-150" x2="0" y2="150" stroke="#e1e5ea"/>
    <text x="0" y="-140" text-anchor="middle" font-size="20" fill="#7a838d">${pinkyAxes ? '앞' : '+x'}</text>
    <text x="-150" y="-8" font-size="20" fill="#7a838d">${pinkyAxes ? '왼쪽' : '+y'}</text>
    ${nearRadius < 150 ? `<circle r="${nearRadius}" fill="none" stroke="#c98a1a" stroke-width="2" stroke-dasharray="6 6" opacity=".55"/>` : ''}
    <circle r="${toleranceRadius}" fill="#2f8f4e" fill-opacity=".08" stroke="#2f8f4e" stroke-width="2" stroke-dasharray="8 6"/>
    <path d="${arrow}" fill="none" stroke="#7a838d" stroke-width="2.5" stroke-dasharray="5 3"/>
    ${offset ? `<line x1="0" y1="0" x2="${x}" y2="${y}" stroke="${color}" stroke-width="2" opacity=".6"/>
    <path d="${arrow}" transform="translate(${x} ${y}) rotate(${-Number(offset.dyaw_deg)})" fill="${color}" fill-opacity=".9" stroke="${color}"/>` : ''}`;
}

// 마커 네 꼭짓점에서 중심과, 핑키 앞쪽(forwardAxis: 마커의 어느 축이 핑키 앞인지)을 가리키는 끝점을 구한다.
function markerGeometry(corners, forwardAxis) {
  const point = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
  const center = [0, 1].map((i) => corners.reduce((sum, corner) => sum + corner[i], 0) / 4);
  const rightMid = point(corners[1], corners[2], 0.5);
  const topMid = point(corners[0], corners[1], 0.5);
  const axisEnd = { '+x': rightMid, '-x': point(rightMid, center, 2), '+y': topMid, '-y': point(topMid, center, 2) }[forwardAxis || '+y'];
  return { center, front: point(center, axisEnd, 2.4) };
}

// 카메라 영상 위 SVG: 고정 마커(점선 원), 등록 위치(흰 점선 사각형과 앞 방향), 지금 마커(색 채움과 화살표), 라벨.
function drawParkingOverlay(data, level, label) {
  const overlay = data?.image_overlay;
  if (!overlay) {
    parkingOverlay.innerHTML = '';
    return;
  }
  const { width, height } = overlay.image_size_px;
  parkingOverlay.setAttribute('viewBox', `0 0 ${width} ${height}`);
  // 오버레이에서 가져온다. 그래야 판정한 것이 없을 때도 등록된 방향을 바르게 그린다.
  const forwardAxis = overlay.marker_forward_axis ?? data.offset?.marker_forward_axis;
  const color = parkingOverlayColors[level] || parkingOverlayColors.unknown;
  const polygon = (corners) => corners.map(([x, y]) => `${x},${y}`).join(' ');
  // 판정은 고정 마커로 잰 카메라 흔들림(드리프트)을 빼고 하므로, 등록 위치도 그만큼 옮겨 그린다.
  const [shiftX, shiftY] = overlay.view_shift_xy_px || [0, 0];
  const moved = (points) => points.map(([x, y]) => [x + shiftX, y + shiftY]);
  const registeredCorners = moved(overlay.registered_corners_px);
  const registered = markerGeometry(registeredCorners, forwardAxis);
  let svg = moved(overlay.anchor_centers_px).map(([x, y]) => `<circle cx="${x}" cy="${y}" r="9" fill="none" stroke="#fff" stroke-width="1.5" stroke-dasharray="3 2"/>`).join('');
  svg += `<polygon points="${polygon(registeredCorners)}" fill="none" stroke="#000" stroke-opacity=".45" stroke-width="5"/>
    <polygon points="${polygon(registeredCorners)}" fill="none" stroke="#fff" stroke-width="2" stroke-dasharray="6 4"/>
    <line x1="${registered.center[0]}" y1="${registered.center[1]}" x2="${registered.front[0]}" y2="${registered.front[1]}" stroke="#fff" stroke-width="2" stroke-dasharray="4 3"/>`;
  if (overlay.marker_corners_px) {
    const current = markerGeometry(overlay.marker_corners_px, forwardAxis);
    svg += `<line x1="${registered.center[0]}" y1="${registered.center[1]}" x2="${current.center[0]}" y2="${current.center[1]}" stroke="${color}" stroke-width="2" stroke-dasharray="3 3"/>
      <polygon points="${polygon(overlay.marker_corners_px)}" fill="${color}" fill-opacity=".28" stroke="${color}" stroke-width="3"/>
      <line x1="${current.center[0]}" y1="${current.center[1]}" x2="${current.front[0]}" y2="${current.front[1]}" stroke="${color}" stroke-width="3" marker-end="url(#parkingArrow)"/>`;
  }
  parkingOverlay.innerHTML = `<defs><marker id="parkingArrow" viewBox="0 0 10 10" refX="6" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 Z" fill="${color}"/></marker></defs>${svg}
    <rect x="${width - 168}" y="8" width="160" height="22" rx="3" fill="rgba(20,23,27,.72)"/>
    <text x="${width - 88}" y="23" text-anchor="middle" font-size="12" fill="#e8ebee">${escapeHtml(label)}</text>
    ${Math.hypot(shiftX, shiftY) >= 1 ? `<rect x="${width - 168}" y="34" width="160" height="20" rx="3" fill="rgba(20,23,27,.72)"/>
    <text x="${width - 88}" y="48" text-anchor="middle" font-size="11" fill="#e8ebee">카메라 흐름 ${Math.hypot(shiftX, shiftY).toFixed(1)} px 보정</text>` : ''}`;
}

// 판정 카메라의 MJPEG 스트림을 연다. 같은 카메라면 다시 열지 않는다.
function showParkingStream(cameraId) {
  if (!cameraId || cameraId === parkingStreamCamera) return;
  parkingStreamCamera = cameraId;
  parkingStream.hidden = false;
  parkingStreamEmpty.hidden = true;
  parkingStream.src = `/streams/${encodeURIComponent(cameraId)}.mjpg`;
}

// 스트림 연결을 끊는다(src를 지워야 브라우저가 MJPEG 연결을 닫는다).
function stopParkingStream() {
  if (!parkingStreamCamera) return;
  parkingStream.removeAttribute('src');
  parkingStream.hidden = true;
  parkingStreamEmpty.hidden = false;
  parkingStreamCamera = null;
}

// 스트림이 끊기면 안내 글로 바꾸고, 다음 showParkingStream 호출이 다시 열 수 있게 카메라 id를 비운다.
parkingStream.addEventListener('error', () => {
  parkingStream.hidden = true;
  parkingStreamEmpty.hidden = false;
  parkingStreamCamera = null;
});

// 오차가 허용치의 몇 배인지 보여 주는 막대. 0~3배를 0~100%에 놓고, 3배 이상은 끝에 붙는다.
function gaugeHtml(label, value, tolerance, ratio, unit) {
  const position = Math.min(Number(ratio), 3) / 3 * 100;
  return `<div class="parking-gauge"><div class="gauge-head"><span>${escapeHtml(label)}</span><strong>${escapeHtml(value)} / 허용 ${escapeHtml(tolerance)}${unit} · ${Number(ratio).toFixed(2)}배</strong></div>
    <div class="gauge-track"><span class="gauge-needle" style="left:${position}%"></span></div>
    <div class="gauge-scale"><span>0</span><span>허용</span><span>2배</span><span>3배+</span></div></div>`;
}

// 등록 위치로 맞추려면 어느 쪽으로 얼마나 옮길지(오차의 반대 방향). 1 mm, 0.5° 미만은 말하지 않는다.
function correctionAdvice(offset) {
  const { pinkyAxes, forward, left } = parkingAxes(offset);
  const steps = [];
  if (Math.abs(forward) >= 1) steps.push(`${pinkyAxes ? (forward > 0 ? '뒤로' : '앞으로') : (forward > 0 ? '마커 -x로' : '마커 +x로')} ${Math.abs(forward).toFixed(0)} mm`);
  if (Math.abs(left) >= 1) steps.push(`${pinkyAxes ? (left > 0 ? '오른쪽으로' : '왼쪽으로') : (left > 0 ? '마커 -y로' : '마커 +y로')} ${Math.abs(left).toFixed(0)} mm`);
  if (Math.abs(Number(offset.dyaw_deg)) >= 0.5) steps.push(`${Number(offset.dyaw_deg) > 0 ? '시계 방향' : '반시계 방향'}으로 ${Math.abs(Number(offset.dyaw_deg)).toFixed(1)}°`);
  return steps.length ? `등록 위치로 맞추려면 ${steps.join(' · ')} 옮겨야 합니다.` : '';
}

// 숫자, 게이지, 윗면 그림, 카메라 오버레이, 세부 항목: 실시간 상태와 확정 판정이 함께 쓴다.
function renderParkingFields(data, level, overlayLabel, extraRows = []) {
  const offset = data.offset;
  if (offset) {
    const { forward, left } = parkingAxes(offset);
    parkingForward.textContent = signedNumber(forward);
    parkingLeft.textContent = signedNumber(left);
    parkingYaw.textContent = signedNumber(offset.dyaw_deg, 1);
    parkingGauges.innerHTML = gaugeHtml('위치 오차', `${Number(offset.distance_mm).toFixed(1)} mm`, data.tolerance.position_mm, data.position_ratio, ' mm')
      + gaugeHtml('회전 오차', `${Math.abs(Number(offset.dyaw_deg)).toFixed(1)}°`, data.tolerance.yaw_deg, data.yaw_ratio, '°');
  } else {
    parkingForward.textContent = '–';
    parkingLeft.textContent = '–';
    parkingYaw.textContent = '–';
    parkingGauges.innerHTML = '';
  }
  const rows = [
    ['시야 확인', data.view_check === 'ON' ? `고정 마커로 확인 · 이동 ${data.view_shift_px ?? '-'} px` : '꺼짐 (영상 밀림 감지 안 함)'],
    ...extraRows,
    ['주차 위치', data.spot_id],
    ['카메라', data.feed_camera_id],
  ];
  parkingDetails.innerHTML = rows.map(([label, value]) => `<div class="camera-detail"><span>${escapeHtml(label)}</span><strong>${escapeHtml(value)}</strong></div>`).join('');
  showParkingStream(data.feed_camera_id);
  drawParkingMap(data, level);
  drawParkingOverlay(data, level, overlayLabel);
  renderParkingPinky(data);
}

// 등록된 주차 위치마다 그 마커를 단 핑키(/api/pinky의 marker.spot_id). 카메라 아래 줄에 쓴다.
// 핑키 이름표는 자주 바뀌지 않으니 30초마다만 다시 읽는다.
let parkingPinkies = new Map();  // spot_id -> { name, markerId }
let parkingPinkiesAt = 0;
const PARKING_PINKIES_REFRESH_MS = 30000;

async function loadParkingPinkies() {
  if (Date.now() - parkingPinkiesAt < PARKING_PINKIES_REFRESH_MS) return;
  parkingPinkiesAt = Date.now();
  try {
    const response = await fetch('/api/pinky', { cache: 'no-store' });
    const result = await response.json();
    if (!response.ok) throw new Error('pinky markers unavailable');
    parkingPinkies = new Map((result.pinkies || []).filter((pinky) => pinky.marker?.spot_id)
      .map((pinky) => [pinky.marker.spot_id, { name: pinky.name, markerId: pinky.marker.marker_id }]));
  } catch {
    parkingPinkiesAt = Date.now() - PARKING_PINKIES_REFRESH_MS + 5000;  // 알던 것은 그대로 두고 5초 뒤에 다시 묻는다
  }
}

// 카메라 아래 줄: 판정한 주차 위치를 등록한 마커의 핑키. 주차 위치 하나는 핑키 한 대가 등록한 것이라, 그 위치에서
// 보이는 마커는 곧 그 핑키다. 마커가 보이지 않으면 누구의 자리를 보고 있는지 말한다.
function renderParkingPinky(data) {
  if (!data?.spot_id) {
    parkingPinky.className = 'parking-pinky absent';
    parkingPinky.innerHTML = '<span>핑키 확인 전</span>';
    return;
  }
  const pinky = parkingPinkies.get(data.spot_id);
  const seen = ['MOVING', 'IN_POSITION', 'OFF_POSITION'].includes(data.state) || ['ARRIVED', 'MISALIGNED'].includes(data.status);
  const marker = pinky ? `마커 ${pinky.markerId}번` : '';
  parkingPinky.className = `parking-pinky${seen && pinky ? '' : ' absent'}`;
  if (seen) {
    parkingPinky.innerHTML = pinky
      ? `<span>지금 보이는 핑키</span><strong>${escapeHtml(pinky.name)}</strong><small>${escapeHtml(marker)} · ${escapeHtml(data.spot_id)}</small>`
      : `<span>지금 보이는 핑키</span><strong>이름을 알 수 없어요</strong><small>${escapeHtml(data.spot_id)} 자리를 쓰는 핑키가 정해져 있지 않아요</small>`;
  } else {
    parkingPinky.innerHTML = pinky
      ? `<span>지금 보이는 핑키</span><strong>없음</strong><small>${escapeHtml(pinky.name)} 자리(${escapeHtml(marker)})를 보고 있어요</small>`
      : `<span>지금 보이는 핑키</span><strong>없음</strong><small>${escapeHtml(data.spot_id)}</small>`;
  }
}

function setParkingAdvice(text) {
  parkingAdvice.hidden = !text;
  parkingAdvice.textContent = text || '';
}

// 위 단계 표시(화면 밖 → 이동 중 → 정지·판정)에서 지금 단계와 지난 단계를 칠한다. null이면 모두 끈다.
function setParkingSteps(state) {
  const order = { NOT_IN_VIEW: 0, MOVING: 1, IN_POSITION: 2, OFF_POSITION: 2 };
  const current = order[state];
  [...parkingSteps.children].forEach((step, index) => {
    step.classList.toggle('current', current === index);
    step.classList.toggle('done', current !== undefined && index < current);
  });
}

// 실시간 상태(GET .../live의 live)로 패널 전체를 그린다. since는 이 상태가 시작된 시각이다.
function renderParkingLive(live) {
  const state = live.state;
  const stopped = state === 'IN_POSITION' || state === 'OFF_POSITION';
  const level = state === 'MOVING' ? 'moving' : stopped ? parkingLevel(live) : 'unknown';
  const seconds = Math.max(0, (Date.parse(live.updated_at) - Date.parse(live.since)) / 1000);
  const duration = Number.isFinite(seconds) ? ` · ${seconds.toFixed(0)}초째` : '';
  parkingBanner.className = `parking-banner ${level}`;
  parkingStatus.textContent = state === 'OFF_POSITION'
    ? `위치 벗어남 · ${parkingDeviationLabels[live.deviation_level] || live.deviation_level}`
    : parkingLiveTitles[state] || state;
  parkingSummary.textContent = {
    IN_POSITION: `멈춘 상태로 허용 범위 안에 있습니다${duration}`,
    OFF_POSITION: `멈췄지만 허용 범위를 벗어났습니다${duration}`,
    MOVING: '마커가 움직이고 있습니다. 1초 이상 멈추면 위치를 판정합니다.',
  }[state] || live.reason;
  setParkingSteps(state);
  setParkingAdvice(state === 'OFF_POSITION' && live.offset ? correctionAdvice(live.offset) : parkingHelp[state]);
  const time = clockFormat.format(new Date(live.updated_at));
  renderParkingFields(live, level, `실시간 ${time}`);
  parkingUpdated.textContent = `실시간 ${time}`;
}

// 확정 판정 결과 한 줄(실시간 패널 아래에 붙는다).
function confirmLine(observation) {
  const title = observation.status === 'MISALIGNED'
    ? `어긋남(${parkingDeviationLabels[observation.deviation_level] || observation.deviation_level})`
    : parkingStatusLabels[observation.status] || observation.status;
  const distance = observation.offset ? ` · 거리 ${Number(observation.offset.distance_mm).toFixed(1)} mm / 허용 ${observation.tolerance.position_mm} mm · 회전 ${signedNumber(observation.offset.dyaw_deg, 1)}°` : ` · ${observation.reason}`;
  return `확정 판정 ${clockFormat.format(new Date(observation.captured_until))} · ${title}${distance} · ${observation.frames_used}프레임`;
}

// 실시간 추적이 없으면(예전 서버이거나 설정된 위치가 없음) 확정 판정이 패널 전체를 채운다.
function renderParkingCheck(observation) {
  const level = parkingLevel(observation);
  parkingBanner.className = `parking-banner ${level}`;
  parkingStatus.textContent = observation.status === 'MISALIGNED'
    ? `어긋남 · ${parkingDeviationLabels[observation.deviation_level] || observation.deviation_level}`
    : parkingStatusLabels[observation.status] || observation.status;
  parkingSummary.textContent = observation.status === 'ARRIVED' ? '등록된 주차 위치의 허용 범위 안에 있습니다.' : observation.reason;
  setParkingSteps(null);
  setParkingAdvice(observation.offset ? (observation.status === 'ARRIVED' ? '' : correctionAdvice(observation.offset)) : parkingHelp[observation.status]);
  const time = clockFormat.format(new Date(observation.captured_until));
  renderParkingFields(observation, level, `판정 ${time} 기준`, [['프레임', `${observation.frames_used}장 판정 / ${observation.frames_examined}장 확인`]]);
  parkingUpdated.textContent = `판정 ${time}`;
}

// 모든 숫자와 그림을 지워, 데이터가 사라진 뒤에 예전 "정상 위치"가 화면에 남지 않게 한다.
function clearParkingPanel(title, message) {
  parkingBanner.className = 'parking-banner unknown';
  parkingStatus.textContent = title;
  parkingSummary.textContent = message;
  setParkingSteps(null);
  parkingForward.textContent = '–';
  parkingLeft.textContent = '–';
  parkingYaw.textContent = '–';
  parkingGauges.innerHTML = '';
  setParkingAdvice('');
  parkingDetails.innerHTML = '';
  drawParkingMap(null, 'unknown');
  drawParkingOverlay(null);
  renderParkingPinky(null);
}

// 확정 판정 실패를 한 줄로 알린다. 실시간 상태가 살아 있으면 패널은 그대로 둔다.
function renderParkingError(message) {
  parkingConfirm.hidden = false;
  parkingConfirm.className = 'parking-confirm';
  parkingConfirm.textContent = `확정 판정 실패 · ${message}`;
  if (!(parkingLive && parkingLiveOk)) clearParkingPanel('판정하지 못함', message);
}

// [확정 판정]: 입력한 주차 위치로 GET .../check를 보낸다. 여러 프레임을 모아 판정하므로 그동안 버튼을 막는다.
async function checkParking() {
  const spotId = parkingSpotInput.value.trim();
  if (parkingBusy) return;
  if (!spotId) {
    renderParkingError('주차 위치 ID를 입력하세요.');
    return;
  }
  parkingBusy = true;
  parkingCheckButton.disabled = true;
  parkingCheckButton.textContent = '판정 중…';
  try {
    await loadParkingPinkies();
    const response = await fetch(`/api/parking-spots/${encodeURIComponent(spotId)}/check`, { cache: 'no-store' });
    const result = await response.json();
    if (!response.ok) {
      renderParkingError(apiErrorMessage(result, '주차 판정 실패'));
      return;
    }
    parkingConfirm.hidden = false;
    parkingConfirm.className = `parking-confirm ${parkingLevel(result.observation)}`;
    parkingConfirm.textContent = confirmLine(result.observation);
    if (!(parkingLive && parkingLiveOk)) renderParkingCheck(result.observation);
  } catch {
    renderParkingError('주차 판정 API에 연결할 수 없습니다.');
  } finally {
    parkingBusy = false;
    parkingCheckButton.disabled = false;
    parkingCheckButton.textContent = '확정 판정';
  }
}

// 0.5초마다 부른다. 장비·카메라 뷰가 보일 때만 실시간 상태를 읽고, 안 보이면 이 패널의 스트림을 끊는다.
async function pollParkingLive() {
  const spotId = parkingSpotInput.value.trim();
  if (document.hidden || document.querySelector('#view-overview').hidden) {
    stopParkingStream();  // 카메라 카드는 자기 스트림을 따로 연다. 이 스트림은 패널이 보일 때만 필요하다
    return;
  }
  if (!parkingLive || parkingLiveBusy || !spotId) return;
  parkingLiveBusy = true;
  try {
    if (parkingPinkies.size) loadParkingPinkies();  // 30초마다 기다리지 않고 뒤에서 새로 읽는다
    else await loadParkingPinkies();  // 카메라 아래 첫 줄은 핑키 이름을 받을 때까지 기다린다
    const liveOf = async (id) => {
      const response = await fetch(`/api/parking-spots/${encodeURIComponent(id)}/live`, { cache: 'no-store' });
      return { ok: response.ok, body: await response.json() };
    };
    let { ok, body: result } = await liveOf(spotId);
    // 핑키마다 적재 자리를 따로 등록해 두었다. 보여 주는 위치의 마커가 화면에 없으면 마커가 보이는 위치로 바꿔 보여 준다:
    // 패널이 지금 거기 서 있는 핑키를 따라간다(작업자가 id를 입력하는 중이면 바꾸지 않는다).
    if (document.activeElement !== parkingSpotInput && !(ok && PARKING_TRACKED_STATES.has(result.live?.state))) {
      for (const other of parkingLiveSpots) {
        if (other === spotId) continue;
        const seen = await liveOf(other);
        if (seen.ok && PARKING_TRACKED_STATES.has(seen.body.live?.state)) {
          parkingSpotInput.value = other;
          ({ ok, body: result } = seen);
          break;
        }
      }
    }
    parkingLiveOk = ok;
    if (ok) renderParkingLive(result.live);
    else clearParkingPanel('실시간 추적 없음', apiErrorMessage(result, '이 주차 위치는 실시간 추적 중이 아닙니다.'));
  } catch {
    parkingLiveOk = false;
    clearParkingPanel('실시간 상태 없음', '관제 서버에서 실시간 상태를 가져오지 못했습니다.');
  } finally {
    parkingLiveBusy = false;
  }
}

// 마커가 보이는(추적 중인) 실시간 상태.
const PARKING_TRACKED_STATES = new Set(['MOVING', 'IN_POSITION', 'OFF_POSITION']);
let parkingLiveSpots = [];  // 실시간으로 따라가는 모든 위치(health.parking.live_spots), 등록된 핑키마다 하나

// GET /api/health의 parking 설정을 반영한다(3초마다 refresh가 부른다). 입력칸이 비어 있으면 기본 위치를 넣는다.
function updateParkingConfig(parking) {
  parkingCameraId = parking?.camera_id || null;
  parkingLive = Boolean(parking?.live_tracking);
  parkingLiveSpots = Array.isArray(parking?.live_spots) ? parking.live_spots : [];
  if (parking?.spot_id && !parkingSpotInput.value) parkingSpotInput.value = parking.spot_id;
  if (parkingCameraId) showParkingStream(parkingCameraId);
  parkingLiveMode.classList.toggle('on', parkingLive);
  parkingLiveMode.textContent = parkingLive ? '실시간 추적 중' : '실시간 추적 꺼짐 · 확정 판정만 가능';
  if (!parkingCameraId && parkingStatus.textContent === '판정 전') {
    parkingSummary.textContent = '주차 판정 카메라가 설정되지 않았습니다 (CONTROL_STATION_PARKING_CAMERA).';
  }
}

parkingCheckButton.addEventListener('click', checkParking);
drawParkingMap(null, 'unknown');

// ==== 13. 작업 이력 화면 ====
// 왼쪽 작업 목록. 고른 작업이 목록에 없으면 첫 작업을 고른다. 항목을 누르면 그 작업으로 다시 읽는다.
function renderTasks(tasks) {
  taskCount.textContent = `${tasks.length}개 작업`;
  if (!tasks.length) {
    taskList.innerHTML = '<p class="loading">등록된 작업이 없습니다.</p>';
    selectedTaskId = null;
    taskDetail.innerHTML = '<p class="loading">작업을 등록하면 단계와 생성 이력을 볼 수 있습니다.</p>';
    return;
  }
  if (!tasks.some((task) => task.id === selectedTaskId)) selectedTaskId = tasks[0].id;
  taskList.innerHTML = tasks.map((task) => `
    <button type="button" class="task-item${task.id === selectedTaskId ? ' selected' : ''}" data-task-id="${escapeHtml(task.id)}">
      <span><strong>${task.order_items ? `${pinkyBadge(task.pinky_id)} ${escapeHtml(orderSummary(task.order_items))}` : escapeHtml(task.destination)}</strong><small>${escapeHtml(task.id)}</small></span>
      <span class="task-status">${escapeHtml(taskStatusLabels[task.status] || task.status)}</span>
    </button>`).join('');
  taskList.querySelectorAll('[data-task-id]').forEach((button) => {
    button.addEventListener('click', () => {
      selectedTaskId = button.dataset.taskId;
      refreshTasks();
    });
  });
}

// 오른쪽 작업 상세: 단계와 이벤트. 시작 버튼은 mock 모드의 대기 작업에만, 취소 버튼은 mock 모드에서 이 서버
// 프로세스가 실행 중인(local_execution_owner가 LOCAL_WORKER_ACTIVE) 작업에만 나온다. 저장된 작업이 활성인데
// 이 프로세스에 실행 주체가 없으면(서버 재시작 등) 그 사실을 알리고, 자동 재개는 하지 않는다.
// 이벤트는 종류별로 사람이 읽을 한 줄(주차 판정, 적재 확인, 진행률, 물체 확인, 팔 명령 등록)로 바꾼다.
function renderTaskDetail(task) {
  const steps = task.steps || [];
  const events = task.events || [];
  const pickRequest = task.pick_request;
  const targetSummary = pickRequest
    ? [pickRequest.category, pickRequest.color, pickRequest.shelf_level ? `${pickRequest.shelf_level}층` : null].filter(Boolean).join(' · ')
    : '미지정';
  const canStart = task.status === 'PENDING' && controlMode === 'mock';
  const canCancel = ['VALIDATING', 'RUNNING'].includes(task.status)
    && controlMode === 'mock'
    && task.local_execution_owner === 'LOCAL_WORKER_ACTIVE';
  const modeLabel = controlModeLabels[controlMode] || controlMode;
  const modeNote = controlMode === 'mock'
    ? devWord('모의 모드 · 단계 성공은 진행 시뮬레이션이며 실제 인식/로봇 명령을 의미하지 않습니다.')
    : `현재 ${escapeHtml(modeLabel)} 모드에서는 이 화면에서 작업을 실행하지 않습니다.`;
  const ownershipNote = task.local_execution_owner === 'NO_LOCAL_WORKER'
    ? '<p class="task-error">저장된 작업은 활성 상태지만 이 서버 프로세스에 실행 worker가 없습니다. 자동 재개나 명령 재전송은 하지 않습니다. 작업 상태를 운영자가 확인해야 합니다.</p>'
    : task.local_execution_owner === 'MODE_NOT_MANAGED'
      ? '<p class="task-error">저장된 작업은 활성 상태지만 현재 모드에는 이 작업을 관리하는 실행 worker가 연결되어 있지 않습니다. 자동 재개나 명령 재전송은 하지 않습니다.</p>'
      : '';
  const action = canStart
    ? '<button type="button" data-task-action="start">작업 시작</button>'
    : canCancel
      ? '<button type="button" class="secondary" data-task-action="cancel">작업 취소</button>'
      : '';
  taskDetail.innerHTML = `
    <header class="task-detail-head"><div><h3>${escapeHtml(task.destination)}</h3><small>${escapeHtml(task.id)}</small></div><span class="task-status">${escapeHtml(taskStatusLabels[task.status] || task.status)}</span></header>
    ${task.error ? `<p class="task-error">${escapeHtml([task.error.code, task.error.message].filter(Boolean).join(' · '))}</p>` : ''}
    ${task.order_items
      ? `<p class="task-arm-job">주문: ${escapeHtml(orderSummary(task.order_items))} · 배정 핑키 ${pinkyBadge(task.pinky_id)}</p>`
      : `<p class="task-arm-job">팔 작업: ${escapeHtml(task.arm_job)}</p>
    <p class="task-arm-job">대상: ${escapeHtml(targetSummary)}${pickRequest?.category ? ' · 팔 단계는 이 물체의 학습 정책을 고릅니다' : ''}</p>`}
    ${ownershipNote}
    <div class="task-actions">${action}<span class="note">${modeNote}</span></div>
    <h4>단계</h4>
    <ol class="step-list">${steps.map((step) => {
      const stepLabel = stepLabels[step.step_type] || step.step_type;
      const robotLabel = step.robot_id === 'mobile-01' ? '주행 로봇' : step.robot_id === 'arm-01' ? '로봇팔' : '';
      return `<li><span>${escapeHtml(stepLabel)}${step.robot_id ? `<small class="step-robot">${robotLabel ? `${robotLabel} · ` : ''}${escapeHtml(step.robot_id)}</small>` : ''}</span><strong>${escapeHtml(executionLabels[step.status] || step.status)}${step.result?.outcome && stepOutcomeLabels[step.result.outcome] ? ` · ${stepOutcomeLabels[step.result.outcome]}` : ''}${step.result?.source === 'mock' ? devWord(' · 모의') : ''}</strong></li>`;
    }).join('')}</ol>
    <h4>이벤트</h4>
    <ol class="event-list">${events.map((event) => {
      const parking = event.event_type === 'PARKING_OBSERVED' ? event.payload?.observation : null;
      const eventName = event.payload?.event || event.event_type;
      const linkedStep = steps.find((step) => step.id === event.step_id);
      const stepContext = linkedStep
        ? ` · 연결 단계 ${stepLabels[linkedStep.step_type] || linkedStep.step_type}`
        : '';
      const detail = parking
        ? formatParkingObservation(parking)
        : event.event_type === 'LOAD_OBSERVED'
        ? formatLoadObservation(event.payload || {})
          : eventName === 'PROGRESS' && event.payload?.arm_state
            ? `OMX ${event.payload.arm_command_id} · ${executionLabels[event.payload.arm_state] || event.payload.arm_state}`
          : eventName === 'PROGRESS'
            ? `${event.payload?.source === 'mock' ? devWord('모의 ') : ''}진행률 ${Number(event.payload?.progress) || 0}% · ${event.payload?.message || '진행 이벤트'}`
          : eventName === 'TARGET_CONFIRMED'
            ? `${taskEventLabels[eventName]} · ${event.payload?.message || ''} · 신뢰도 ${event.payload?.confidence ?? '-'}`
          : eventName === 'ARM_COMMAND_CREATED'
            ? `${taskEventLabels[eventName]} · ${event.payload?.arm_command_id} · 정책 ${event.payload?.policy_id}${event.payload?.slot_id ? ` · 칸 ${event.payload.slot_id}` : ''}`
            : (taskEventLabels[eventName] || eventName);
      const source = event.payload?.source;
      const robotId = event.payload?.robot_id;
      return `<li><time>${escapeHtml(event.received_at)}</time><span>${escapeHtml(detail)}${stepContext}${source ? ` · ${escapeHtml(source)}` : ''}${robotId ? ` · ${escapeHtml(robotId)}` : ''}</span></li>`;
    }).join('') || '<li class="empty-history">기록된 이벤트가 없습니다.</li>'}</ol>`;
  taskDetail.querySelector('[data-task-action]')?.addEventListener('click', async (event) => {
    const button = event.currentTarget;
    button.disabled = true;
    await sendTaskAction(task.id, button.dataset.taskAction);
  });
}

// 모의 작업 시작·취소: POST /api/tasks/{id}/start|cancel.
async function sendTaskAction(taskId, action) {
  taskMessage.textContent = action === 'start' ? '작업을 시작합니다…' : '작업 취소를 요청합니다…';
  try {
    const response = await fetch(`/api/tasks/${encodeURIComponent(taskId)}/${action}`, { method: 'POST' });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, '작업 요청에 실패했습니다.'));
    taskMessage.textContent = `${devMode ? `${result.execution_mode || 'mock'} · ` : ''}${taskStatusLabels[result.task.status] || result.task.status}`;
  } catch (error) {
    taskMessage.textContent = error.message;
  } finally {
    await refreshTasks();
  }
}

// GET /api/tasks로 목록을, 고른 작업이 있으면 GET /api/tasks/{id}로 상세를 읽는다. 1초마다 부른다.
async function refreshTasks() {
  try {
    const response = await fetch('/api/tasks', { cache: 'no-store' });
    if (!response.ok) throw new Error('작업 목록을 불러오지 못했습니다.');
    const { tasks } = await response.json();
    renderTasks(tasks);
    if (!selectedTaskId) return;
    const detailResponse = await fetch(`/api/tasks/${encodeURIComponent(selectedTaskId)}`, { cache: 'no-store' });
    if (!detailResponse.ok) throw new Error('작업 상세를 불러오지 못했습니다.');
    const { task } = await detailResponse.json();
    renderTaskDetail(task);
  } catch (error) {
    taskMessage.textContent = error.message;
  }
}

// 이전 형식 작업 등록(POST /api/tasks). 저장만 하고 인식이나 로봇 명령은 실행하지 않는다.
// 대상 칸 중 하나라도 채우면 pick_request를 만들고, 비어 있는 색상·선반 층은 보내지 않는다.
taskForm.addEventListener('submit', async (event) => {
  event.preventDefault();
  const formData = new FormData(taskForm);
  const button = taskForm.querySelector('button');
  button.disabled = true;
  taskMessage.textContent = '';
  try {
    const targetCategory = String(formData.get('target_category') || '').trim();
    const targetColor = String(formData.get('target_color') || '').trim();
    const shelfLevel = String(formData.get('shelf_level') || '').trim();
    const hasPickRequest = Boolean(targetCategory || targetColor || shelfLevel);
    const response = await fetch('/api/tasks', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        destination: formData.get('destination'),
        arm_job: formData.get('arm_job'),
        pick_request: hasPickRequest ? {
          category: targetCategory,
          ...(targetColor ? { color: targetColor } : {}),
          ...(shelfLevel ? { shelf_level: Number(shelfLevel) } : {}),
        } : null,
      }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '작업 등록에 실패했습니다.');
    selectedTaskId = result.task.id;
    taskForm.reset();
    taskMessage.textContent = '작업 명세를 저장했습니다. 인식과 로봇 명령은 실행되지 않았습니다.';
    await refreshTasks();
  } catch (error) {
    taskMessage.textContent = error.message;
  } finally {
    button.disabled = false;
  }
});

// ==== 14. 서버 상태 주기 갱신(refresh) ====
// 서버 상태, 로봇 인터페이스, 카메라, 팔 프로세스를 한꺼번에 읽는다(3초마다). 하나라도 실패하면 서버 연결 실패로 보고
// 표와 카메라 격자를 오류 글로 바꾼다. 카메라 격자는 다음 성공 때 다시 만들도록 cameraIds를 비운다.
async function refresh() {
  try {
    const [healthResponse, robotsResponse, camerasResponse, armResponse] = await Promise.all([
      fetch('/api/health', { cache: 'no-store' }),
      fetch('/api/robots', { cache: 'no-store' }),
      fetch('/api/cameras', { cache: 'no-store' }),
      fetch('/api/arm/status', { cache: 'no-store' }),
    ]);
    if (!healthResponse.ok || !robotsResponse.ok || !camerasResponse.ok || !armResponse.ok) throw new Error('API unavailable');
    const [health, robotData, cameraData, armData] = await Promise.all([
      healthResponse.json(), robotsResponse.json(), camerasResponse.json(), armResponse.json(),
    ]);
    controlMode = health.mode;
    updateParkingConfig(health.parking);
    serverStatus.className = 'server-status ok';
    const modeWords = health.mode !== 'mock' || devMode ? `<small>${escapeHtml(controlModeLabels[health.mode] || health.mode)} 모드</small>` : '';
    serverStatus.innerHTML = `<span class="dot"></span><span><strong>API 정상</strong>${modeWords}</span>`;
    renderOverviewSummary(health, robotData.robots, cameraData.cameras);
    renderRobots(robotData.robots);
    renderCameras(cameraData.cameras, cameraData.pinky_load, cameraData.head_shelf, cameraData.head_middle);
    renderArm(armData.arm);
    renderPolicyRun(armData.policy_run);
    updatedAt.textContent = `갱신 ${clockFormat.format(new Date())}`;
  } catch {
    serverStatus.className = 'server-status error';
    serverStatus.innerHTML = '<span class="dot"></span><span><strong>서버 연결 실패</strong><small>다시 연결하는 중</small></span>';
    renderOverviewSummary(null);
    robotRows.innerHTML = '<tr><td colspan="3" class="error-row">서버 API에 연결할 수 없습니다.</td></tr>';
    cameraGrid.innerHTML = '<p class="error-row">서버 API에 연결할 수 없습니다.</p>';
    cameraGrid.dataset.cameraIds = '';
  }
}

// ==== 15. 주문 진행 화면: 경보, 로봇 상태, 주문 목록, 모의 세계 ====
// ---- 주문 진행 화면 ----
// GET /api/flow(1초마다)로 경보, 핑키·팔 상태, 시작한 주문의 단계, 적재 자리 주인, 모의 세계를 그린다.
const flowAlerts = document.querySelector('#flowAlerts');
const flowRobots = document.querySelector('#flowRobots');
const flowOrders = document.querySelector('#flowOrders');
const flowUpdated = document.querySelector('#flowUpdated');
const flowStation = document.querySelector('#flowStation');
const mockPanel = document.querySelector('#mockPanel');
const mockStock = document.querySelector('#mockStock');
const mockFaultKind = document.querySelector('#mockFaultKind');
const mockMessage = document.querySelector('#mockMessage');
const orderStatusLabels = {
  PENDING: '시작 대기', VALIDATING: '진행 중', RUNNING: '진행 중', SUCCEEDED: '완료',
  FAILED: '멈춤', CANCELING: '취소 중', CANCELED: '취소됨', NEEDS_INTERVENTION: '확인 필요',
};
// 모의 세계에 넣을 수 있는 사고 종류의 이름(개발자 화면).
const mockFaultLabels = {
  STOCK_SHORT: '선반 재고 없음', PICK_FAILED: '집기 실패(선반에 남음)', ITEM_LOST: '분실(어디에도 없음)',
  EXTRA_ON_PINKY: '핑키에 주문 외 물건', PINKY_MISALIGNED: '핑키 위치 이상', FLEET_REJECT: '핑키 신호 거부',
};
const shelfNames = { shelf_a: '왼쪽 선반', shelf_b: '오른쪽 선반' };

// 시작부터 끝(없으면 지금)까지 걸린 시간을 mm:ss로 만든다.
function elapsed(fromIso, toIso) {
  if (!fromIso) return '';
  const seconds = Math.max(0, Math.round(((toIso ? new Date(toIso) : new Date()) - new Date(fromIso)) / 1000));
  return `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
}

const pickupSending = new Set();  // 수령 확인을 보내는 중인 주문 id(도착 확인은 'arrival:' 접두)
// 마지막으로 그린 경보 HTML. 비우면 다음 갱신 때 반드시 다시 그린다.
let renderedAlerts = '';

// 경보 오른쪽 버튼: 도착 확인 대기면 [도착 확인], 수령 확인 대기면 [수령 확인], 확인할 수 있는 경보면 [확인].
// 보내는 중이면 버튼을 막아 두 번 보내지 않는다.
function alertActionsHtml(alert) {
  const id = escapeHtml(alert.task_id);
  if (alert.arrival) {
    const sending = pickupSending.has(`arrival:${alert.task_id}`);
    return `<span class="alert-actions"><button type="button" class="pickup-confirm" data-arrival-confirm="${id}"${sending ? ' disabled' : ''}>${sending ? '보내는 중…' : '도착 확인'}</button></span>`;
  }
  if (alert.pickup) {
    const sending = pickupSending.has(alert.task_id);
    return `<span class="alert-actions"><button type="button" class="pickup-confirm" data-pickup-confirm="${id}"${sending ? ' disabled' : ''}>${sending ? '보내는 중…' : '수령 확인'}</button></span>`;
  }
  return alert.acknowledgeable ? `<button type="button" class="secondary" data-ack-alert="${id}">확인</button>` : '<span></span>';
}

// 바뀐 것이 있을 때만 다시 쓴다: 주문 진행은 1초마다 갱신하는데, 포인터 아래의 버튼이 바뀌면 클릭이 사라진다.
function renderFlowAlerts(alerts) {
  const html = alerts.map((alert) => `
    <div class="alert-card ${alert.level}" role="alert">
      <span class="alert-icon" aria-hidden="true">!</span>
      <div><strong>${escapeHtml(alert.title || '확인 필요')}</strong>
        <div>${escapeHtml(alert.message || '')}</div>
        <small class="alert-meta">${[alert.pinky_name ? `핑키 ${alert.pinky_name}` : '', alert.task_id, alert.code]
          .filter(Boolean).map((part) => `<span>${escapeHtml(part)}</span>`).join('')}</small></div>
      ${alertActionsHtml(alert)}
    </div>`).join('');
  if (html === renderedAlerts) return;
  renderedAlerts = html;
  flowAlerts.innerHTML = html;
}

// 마커가 없는 핑키가 적재 자리에 제대로 섰음을 작업자가 확인한다. 그러면 적재가 시작된다.
async function confirmArrival(orderId) {
  const key = `arrival:${orderId}`;
  if (pickupSending.has(key)) return;
  pickupSending.add(key);
  // 버튼을 바로 '보내는 중…'으로 다시 그린다.
  renderedAlerts = '';
  await refreshFlow();
  try {
    const response = await fetch(`/api/orders/${encodeURIComponent(orderId)}/arrival-confirm`, { method: 'POST' });
    const result = await response.json().catch(() => ({}));
    if (response.ok) showToast('도착 확인을 보냈어요', `${orderId} · 적재를 시작해요`);
    else showToast('도착 확인을 보내지 못했어요', result.message || `HTTP ${response.status}`, 'error');
  } catch {
    showToast('도착 확인 응답을 받지 못했어요', '주문 상태를 확인한 뒤 다시 눌러 주세요', 'warning');
  } finally {
    pickupSending.delete(key);
    renderedAlerts = '';
    await Promise.all([refreshFlow(), refreshIntake()]);
  }
}

// 고객 대신 작업자가 수령을 확인한다(휴대폰이 없거나 고객이 떠난 경우). 그러면 핑키가 주차장으로 돌아간다.
async function confirmPickup(orderId) {
  if (pickupSending.has(orderId)) return;
  if (!window.confirm(`${orderId} 주문을 손님이 받았다고 확인할까요?\n확인하면 핑키가 주차장으로 돌아갑니다.`)) return;
  pickupSending.add(orderId);
  renderedAlerts = '';
  await refreshFlow();
  try {
    const response = await fetch(`/api/orders/${encodeURIComponent(orderId)}/pickup-confirm`, { method: 'POST' });
    const result = await response.json().catch(() => ({}));
    if (response.ok) showToast('수령 확인을 보냈어요', `${orderId} · 핑키가 주차장으로 돌아갑니다`);
    else showToast('수령 확인을 보내지 못했어요', result.message || `HTTP ${response.status}`, 'error');
  } catch {
    showToast('수령 확인 응답을 받지 못했어요', '주문 상태를 확인한 뒤 다시 눌러 주세요', 'warning');
  } finally {
    pickupSending.delete(orderId);
    renderedAlerts = '';
    await Promise.all([refreshFlow(), refreshIntake()]);
  }
}

// 경보 버튼 클릭(이벤트 위임). [확인]은 POST /api/flow/alerts/{task_id}/ack로 경보를 확인 처리한다.
flowAlerts.addEventListener('click', async (event) => {
  const arrival = event.target.closest('[data-arrival-confirm]');
  if (arrival) {
    confirmArrival(arrival.dataset.arrivalConfirm);
    return;
  }
  const pickup = event.target.closest('[data-pickup-confirm]');
  if (pickup) {
    confirmPickup(pickup.dataset.pickupConfirm);
    return;
  }
  const ack = event.target.closest('[data-ack-alert]');
  if (!ack) return;
  ack.disabled = true;
  await fetch(`/api/flow/alerts/${encodeURIComponent(ack.dataset.ackAlert)}/ack`, {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ note: '화면에서 확인' }),
  });
  renderedAlerts = '';
  await refreshFlow();
});

// 로봇 상태 줄: 핑키 카드(적재 자리 주인이면 '적재 자리' 표시, 멈춘 주문이 붙어 있으면 빨강)와 팔 카드.
function renderFlowRobots(view) {
  const pinkies = view.pinkies.map((pinky) => `
    <article class="robot-card${pinky.reachable ? '' : ' unreachable'}${pinky.stopped_order_id ? ' stopped' : ''}">
      <h3>${pinkyBadge(pinky.pinky_id, { offline: !pinky.reachable })}<span>${pinky.order_id && pinky.order_id === view.station_owner ? '<span class="spot-tag">적재 자리</span> ' : ''}${pinky.simulated ? '<span class="sim-tag">모의</span>' : ''}</span></h3>
      <div class="robot-state">${escapeHtml(pinky.reachable ? pinky.phase_label : '연결 안 됨')}</div>
      ${pinky.message && pinky.message !== pinky.phase_label ? `<p>${escapeHtml(pinky.message)}</p>` : ''}
      <p>${pinky.order_id ? `주문 ${escapeHtml(pinky.order_id)}`
        : pinky.stopped_order_id ? `<strong class="stopped-note">멈춘 주문 ${escapeHtml(pinky.stopped_order_id)}, 현장 확인 필요</strong>` : '배정된 진행 주문 없음'}</p>
    </article>`);
  const arms = view.arms.map((arm) => `
    <article class="robot-card arm${arm.state === 'busy' ? ' busy' : ''}">
      <h3><span>${escapeHtml(arm.label)}</span><small>${escapeHtml(arm.arm_id)}</small></h3>
      <div class="robot-state">${arm.doing ? escapeHtml(arm.doing.text) : '대기'}</div>
      <p>${escapeHtml(shelfNames[arm.shelf_id] || arm.shelf_id)} 담당</p>
      <p>${arm.worker ? `워커 ${escapeHtml(arm.worker.worker_id)}` : '워커 접속 기록 없음'}</p>
    </article>`);
  flowRobots.innerHTML = [...pinkies, ...arms].join('') || '<p class="loading">로봇 설정 없음</p>';
}

// 팔 적재 계획을 칩으로: 실은 개수(loaded)보다 앞은 끝남, 그다음 하나는 진행 중(집기·옮김 확인 단계) 또는 실패.
// 종합 화면(dashboard.js)도 쓴다.
function planChips(order) {
  if (!order.plan) return '';
  return `<div class="plan-chips">${order.plan.map((unit, index) => {
    const state = index < order.loaded ? 'done'
      : index === order.loaded && order.status === 'FAILED' && order.stages.find((stage) => stage.key === 'LOAD').state === 'failed' ? 'failed'
        : index === order.loaded && order.active_step && ['PICK_ITEM', 'VERIFY_TRANSFER'].includes(order.active_step) ? 'doing' : '';
    return `<span class="plan-chip ${state}">${escapeHtml(unit.arm_label)} ${escapeHtml(unit.label)} #${unit.unit}</span>`;
  }).join('<span aria-hidden="true">→</span>')}</div>`;
}

function renderFlowOrders(orders) {
  // 시작 대기 주문은 위 주문 접수에서 시작하거나 거절한다. 이 목록은 시작한 뒤부터 따라간다.
  const started = orders.filter((order) => order.status !== 'PENDING');
  if (!started.length) {
    flowOrders.innerHTML = '<p class="loading">시작한 주문이 없습니다. 접수된 주문은 위 주문 접수에서 시작합니다.</p>';
    return;
  }
  flowOrders.innerHTML = started.map((order) => `
    <article class="order-card${order.status === 'FAILED' ? ' failed' : ''}">
      <div class="order-head">
        ${pinkyBadge(order.pinky_id)} <strong>${escapeHtml(orderSummary(order.order_items))}</strong>
        <span class="status-chip ${escapeHtml(order.status)}">${escapeHtml(orderStatusLabels[order.status] || order.status)}</span>
        ${order.simulated ? '<span class="sim-tag">모의</span>' : ''}
        <span class="order-meta"><span>${escapeHtml(order.id)}</span>${order.started_at ? `<time>${elapsed(order.started_at, order.finished_at)}</time>` : ''}</span>
      </div>
      <ol class="pipeline">${order.stages.map((stage) => `<li class="${stage.state}">${escapeHtml(stage.label)}</li>`).join('')}</ol>
      <p class="order-now"><b>지금</b> ${escapeHtml(order.current)}</p>
      ${planChips(order)}
      <div class="order-actions-row">
        ${['RUNNING', 'VALIDATING'].includes(order.status) ? `<button type="button" class="secondary" data-order-action="cancel" data-order-id="${escapeHtml(order.id)}">멈추기</button>` : ''}
        <button type="button" class="secondary" data-order-detail="${escapeHtml(order.id)}">단계·기록 보기</button>
      </div>
    </article>`).join('');
  // [멈추기]는 POST /api/tasks/{id}/cancel, [단계·기록 보기]는 작업 이력 뷰로 옮겨 그 주문을 연다.
  flowOrders.querySelectorAll('[data-order-action]').forEach((button) => {
    button.addEventListener('click', async () => {
      button.disabled = true;
      await orderControl(button.dataset.orderId, button.dataset.orderAction);
      await refreshFlow();
    });
  });
  flowOrders.querySelectorAll('[data-order-detail]').forEach((button) => {
    button.addEventListener('click', () => {
      selectedTaskId = button.dataset.orderDetail;
      document.querySelector('.nav-item[data-view="tasks"]').click();
      refreshTasks();
    });
  });
}

// 모의 세계 패널(개발자 화면이고 서버가 모의 세계를 줄 때만): 선반·핑키 위 재고와 대기 중인 사고.
function renderMockWorld(world, faultKinds) {
  mockPanel.hidden = !world || !devMode;
  if (!world) return;
  if (!mockFaultKind.options.length) {
    mockFaultKind.innerHTML = faultKinds.map((kind) => `<option value="${escapeHtml(kind)}">${escapeHtml(mockFaultLabels[kind] || kind)}</option>`).join('');
  }
  const counts = (items) => Object.entries(items || {}).filter(([, count]) => count > 0).map(([name, count]) => `${escapeHtml(objectLabel(name))} ${count}`).join(', ') || '비어 있음';
  mockStock.innerHTML = [
    ...Object.entries(world.shelves).map(([shelf, items]) => `<div><strong>${escapeHtml(shelfNames[shelf] || shelf)}</strong><br>${counts(items)}</div>`),
    ...Object.entries(world.pinky_loads).map(([pinky, items]) => `<div><strong>핑키 ${escapeHtml(pinkyNames[pinky] || pinky)} 위</strong><br>${counts(items)}</div>`),
    `<div><strong>대기 중인 사고</strong><br>${world.pending_faults.map(escapeHtml).join(', ') || '없음'}</div>`,
  ].join('');
}

// 진행 중 주문 제어(지금은 멈추기 = cancel만). 주문도 작업(task)이라 /api/tasks 경로를 쓴다.
async function orderControl(taskId, action) {
  try {
    const response = await fetch(`/api/tasks/${encodeURIComponent(taskId)}/${action}`, { method: 'POST' });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, `주문 ${action} 실패`));
  } catch (error) {
    showToast(`${taskId} 요청 실패`, error.message, 'error');
  }
}

// ---- 주문 진행 요약: 시작 대기 주문 수는 주문 접수 폴링에서, 나머지는 /api/flow에서 가져온다 ----
const flowSummaryTitle = document.querySelector('#flowSummaryTitle');
const flowStatWaiting = document.querySelector('#flowStatWaiting');
const flowStatRunning = document.querySelector('#flowStatRunning');
const flowStatStopped = document.querySelector('#flowStatStopped');
const flowStatPinkies = document.querySelector('#flowStatPinkies');
// waiting: 시작 대기 주문 수(refreshIntake가 채움), flow: 마지막 GET /api/flow 응답(refreshFlow가 채움).
// dashboard.js와 수동 조작의 팔 목록도 이 값을 읽는다.
const flowSummary = { waiting: null, flow: null };

// 주문 진행 요약 카드. 문장은 멈춘 주문 → 시작 대기 → 진행 중 순으로 가장 급한 것 하나를 말한다.
function renderFlowSummary() {
  const { waiting, flow } = flowSummary;
  const running = flow ? flow.orders.filter((order) => ['VALIDATING', 'RUNNING', 'CANCELING'].includes(order.status)).length : null;
  const stopped = flow ? flow.alerts.filter((alert) => alert.level === 'error').length : null;
  const reachable = flow ? flow.pinkies.filter((pinky) => pinky.reachable).length : null;
  setStat(flowStatWaiting, waiting, { tone: waiting ? 'attention' : 'zero' });
  setStat(flowStatRunning, running, { tone: running ? 'active' : 'zero' });
  setStat(flowStatStopped, stopped, { tone: stopped ? 'danger' : 'zero' });
  setStat(flowStatPinkies, reachable, { total: flow?.pinkies.length, tone: flow && reachable < flow.pinkies.length ? 'attention' : '' });
  if (stopped) flowSummaryTitle.textContent = `멈춘 주문이 ${stopped}건 있어요. 경보를 확인해 주세요`;
  else if (waiting) flowSummaryTitle.textContent = `주문 ${waiting}건이 시작을 기다리고 있어요`;
  else if (running) flowSummaryTitle.textContent = `주문 ${running}건을 처리하고 있어요`;
  else if (waiting === 0 && flow) flowSummaryTitle.textContent = '기다리는 주문이 없어요';
}

// GET /api/flow를 읽어 주문 진행 화면 전체를 다시 그린다. 1초마다, 그리고 제어 버튼을 누른 직후에 부른다.
async function refreshFlow() {
  try {
    const response = await fetch('/api/flow', { cache: 'no-store' });
    const view = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(view, '진행 상황을 불러오지 못했습니다'));
    flowSummary.flow = view;
    renderFlowSummary();
    renderFlowAlerts(view.alerts);
    renderFlowRobots(view);
    renderFlowOrders(view.orders);
    renderMockWorld(view.mock_world, view.mock_fault_kinds);
    flowStation.textContent = !view.station_owner ? '적재 자리 비어 있음'
      : view.station_held_by_stopped ? `적재 자리 막힘, 멈춘 주문 ${view.station_owner} (경보 확인 필요)` : `적재 자리 사용 중, ${view.station_owner}`;
    flowUpdated.textContent = `갱신 ${clockFormat.format(new Date())}`;
  } catch (error) {
    flowUpdated.textContent = `갱신 실패: ${error.message}`;
  }
}

// 모의 사고 넣기: 다음에 해당 상황이 오면 한 번 일어난다(POST /api/mock-world, action: inject).
document.querySelector('#mockFaultForm').addEventListener('submit', async (event) => {
  event.preventDefault();
  const form = event.currentTarget;
  const response = await fetch('/api/mock-world', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'inject', fault: form.elements.fault.value, argument: form.elements.argument.value.trim() }),
  });
  const result = await response.json();
  mockMessage.textContent = response.ok ? `${mockFaultLabels[form.elements.fault.value]} 대기 중 · 다음 해당 상황에서 한 번 일어납니다.` : apiErrorMessage(result, '사고를 넣지 못했습니다');
  await refreshFlow();
});

// 모의 선반 재고를 처음 상태로 채운다(action: restock).
document.querySelector('#mockRestock').addEventListener('click', async () => {
  await fetch('/api/mock-world', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ action: 'restock' }) });
  mockMessage.textContent = '선반 재고를 처음 상태로 채웠습니다.';
  await refreshFlow();
});

// ==== 16. 주문 접수와 자동 출발 ====
// ---- 주문 접수: 주문 창(/order)에서 들어와 작업자의 시작을 기다리는 주문 ----
const intakeOrders = document.querySelector('#intakeOrders');
const intakeUpdated = document.querySelector('#intakeUpdated');
const intakeBadge = document.querySelector('#intakeBadge');
const pageTitle = document.title;
const intake = {
  orders: [],               // GET /api/orders, 오래된 순
  seenIds: null,            // 지난 폴링의 id 목록. 첫 로드 전에는 null이라 이미 있던 주문은 알림을 띄우지 않는다
  previews: new Map(),      // 주문 id -> GET /api/orders/{id}/assignment의 { result } 또는 { error }
  previewLoading: new Set(),  // 미리보기를 요청 중인 주문 id
  notes: new Map(),         // 주문 id -> { title, text, verdicts? }, 시작·거절이 실패하면 카드에 남기는 안내
  busy: new Map(),          // 주문 id -> 'start' | 'reject', 그 요청이 오가는 동안
  dispatch: null,           // GET /api/orders의 자동 출발 상태: { auto, blocked, skipped, started }
};
const renderedCards = new WeakMap();  // 카드 요소 -> 지금 보여 주는 HTML
// 폴링 요청 번호. 늦게 도착한 옛 응답이 새 응답을 덮지 않게 한다.
let intakeRequestSeq = 0;

function itemCount(items) {
  return (items || []).reduce((sum, item) => sum + Number(item.quantity || 0), 0);
}

// 핑키 후보 판정 목록(순위, 제외 이유). 시작이 PINKY_UNAVAILABLE로 거절됐을 때 카드에 붙인다.
function verdictListHtml(verdicts, chosenId) {
  return `<ul class="verdict-list">${verdicts.map((verdict) => {
    const chosen = verdict.pinky_id === chosenId;
    const label = !verdict.eligible ? '제외' : chosen ? `배정 · ${verdict.rank}순위` : `${verdict.rank}순위`;
    return `<li class="verdict${chosen ? ' chosen' : ''}${verdict.eligible ? '' : ' excluded'}">
      ${pinkyBadge(verdict.pinky_id)}<b>${escapeHtml(label)}</b><span>${escapeHtml(verdict.reason)}</span></li>`;
  }).join('')}</ul>`;
}

// 계획 자체는 intake-plan.js가 그린다(쟁반, 팔 시간표, "자세히" 안의 이유). 그 스크립트가 없으면 핑키 요약 한 줄만 보인다.
function previewHtml(order) {
  const preview = intake.previews.get(order.id);
  if (!preview) return '<p class="preview-state">배정 미리보기를 계산하는 중…</p>';
  if (preview.error) return `<p class="preview-state blocked">배정 미리보기 실패 · ${escapeHtml(preview.error)}</p>`;
  if (window.IntakePlan) return window.IntakePlan.html(order, preview.result);
  return `<p class="preview-state">${escapeHtml(preview.result.pinky?.summary || '')}</p>`;
}

// 접수 카드 하나: 머리(시각, id, 물건, 목적지, 시작·거절 버튼), 배정 미리보기, 자동 출발 안내, 실패 안내.
// 요청이 오가는 동안에는 두 버튼을 모두 막는다.
function intakeCardHtml(order) {
  const id = escapeHtml(order.id);
  const busy = intake.busy.get(order.id);
  const disabled = busy ? ' disabled' : '';
  const note = intake.notes.get(order.id);
  return `
    <header class="intake-head">
      <time datetime="${escapeHtml(order.created_at)}">${escapeHtml(formatClock(order.created_at))}</time>
      <h3 class="intake-id">${id}</h3>
      <span class="intake-items">${escapeHtml(orderSummary(order.order_items))}, 총 ${itemCount(order.order_items)}개</span>
      <span class="intake-zone">${escapeHtml(order.destination)}</span>
      <span class="intake-actions">
        <button type="button" data-intake-action="start" data-order-id="${id}"${disabled}>${busy === 'start' ? '시작 중…' : '시작'}</button>
        <button type="button" class="secondary" data-intake-action="reject" data-order-id="${id}"${disabled}>${busy === 'reject' ? '거절 중…' : '거절'}</button>
      </span>
    </header>
    ${previewHtml(order)}
    ${dispatchNoteHtml(order.id)}
    ${note ? `<div class="intake-note" role="alert"><strong>${escapeHtml(note.title)}</strong> ${escapeHtml(note.text)}
      ${note.verdicts ? verdictListHtml(note.verdicts, null) : ''}</div>` : ''}`;
}

// 자동 출발(orders/dispatcher.py): 스위치, 줄이 무엇을 기다리는지, 카드별 이유.
const dispatchBar = document.querySelector('#dispatchBar');
const dispatchAuto = document.querySelector('#dispatchAuto');
const dispatchState = document.querySelector('#dispatchState');
// 스위치 변경 요청이 오가는 동안 true. 그동안 폴링이 스위치를 되돌리지 않고, 스위치를 막아 둔다.
let dispatchBusy = false;

// 자동 출발이 켜졌을 때 이 주문 카드에 붙일 안내: 건너뛴 이유(skipped) 또는 줄 맨 앞에서 기다리는 이유(blocked).
function dispatchNoteHtml(orderId) {
  const dispatch = intake.dispatch;
  if (!dispatch?.auto) return '';
  const skipped = (dispatch.skipped || []).find((entry) => entry.order_id === orderId);
  if (skipped) return `<div class="dispatch-note skipped"><strong>자동 출발에서 건너뜀</strong> ${escapeHtml(skipped.message || skipped.code)}</div>`;
  if (dispatch.blocked?.order_id === orderId) return `<div class="dispatch-note"><strong>자동 출발 대기</strong> ${escapeHtml(dispatch.blocked.message || dispatch.blocked.code)}</div>`;
  return '';
}

// 자동 출발 막대. 서버가 dispatch 값을 주지 않으면(예전 서버) 숨긴다.
function renderDispatch() {
  const dispatch = intake.dispatch;
  dispatchBar.hidden = !dispatch;
  if (!dispatch) return;
  dispatchBar.classList.toggle('on', dispatch.auto);
  if (!dispatchBusy) dispatchAuto.checked = dispatch.auto;
  dispatchAuto.disabled = dispatchBusy;
  if (!dispatch.auto) dispatchState.textContent = '꺼짐 · 주문마다 [시작]을 눌러요';
  else if (dispatch.blocked) dispatchState.textContent = `켜짐 · ${dispatch.blocked.order_id ? `${dispatch.blocked.order_id} 대기 · ` : ''}${dispatch.blocked.message || dispatch.blocked.code}`;
  else if (!intake.orders.length) dispatchState.textContent = '켜짐 · 기다리는 주문이 없어요';
  else dispatchState.textContent = '켜짐 · 차례대로 출발시키는 중';
}

// 켤 때만 확인 창을 거친다. POST /api/dispatch {auto} 응답의 dispatch로 바로 바꾸고, 실패하면 finally의
// renderDispatch가 스위치를 서버 값으로 되돌린다.
dispatchAuto.addEventListener('change', async () => {
  const auto = dispatchAuto.checked;
  if (auto && !window.confirm(
    `접수 순서대로 자동 출발을 켤까요?\n기다리는 주문 ${intake.orders.length}건부터 [시작]을 누르지 않아도 핑키가 출발합니다.`,
  )) {
    dispatchAuto.checked = false;
    return;
  }
  dispatchBusy = true;
  renderDispatch();
  try {
    const response = await fetch('/api/dispatch', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ auto }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, '자동 출발을 바꾸지 못했습니다'));
    intake.dispatch = result.dispatch;
    showToast(auto ? '자동 출발을 켰어요' : '자동 출발을 껐어요', auto ? '오래된 주문부터 차례로 출발해요' : '이미 출발한 주문은 그대로 진행해요');
  } catch (error) {
    showToast('자동 출발을 바꾸지 못했어요', error.message, 'error');
  } finally {
    dispatchBusy = false;
    renderDispatch();
    refreshIntake();
  }
});

// 접수 목록 전체: 자동 출발 막대, 요약 숫자, 메뉴 배지, 탭 제목의 (n), 카드들.
function renderIntake() {
  const { orders } = intake;
  renderDispatch();
  flowSummary.waiting = orders.length;
  renderFlowSummary();
  intakeBadge.hidden = !orders.length;
  intakeBadge.textContent = String(orders.length);
  intakeBadge.title = `기다리는 주문 ${orders.length}건`;
  document.title = orders.length ? `(${orders.length}) ${pageTitle}` : pageTitle;
  if (!orders.length) {
    intakeOrders.innerHTML = '<p class="intake-empty">기다리는 주문이 없어요.<a href="/order" target="_blank" rel="noopener">주문 창 열기</a></p>';
    return;
  }
  // 카드는 주문 id로 찾아 쓰고 HTML이 바뀐 카드만 다시 쓴다. 그래야 폴링이 포인터 아래의 버튼을 바꿔치지 않는다.
  const cards = new Map([...intakeOrders.querySelectorAll('[data-intake-order]')].map((card) => [card.dataset.intakeOrder, card]));
  // 빈 목록 안내처럼 카드가 아닌 요소는 지운다.
  intakeOrders.querySelectorAll(':scope > :not([data-intake-order])').forEach((node) => node.remove());
  orders.forEach((order, index) => {
    let card = cards.get(order.id);
    cards.delete(order.id);
    if (!card) {
      card = document.createElement('article');
      card.className = 'intake-card';
      card.dataset.intakeOrder = order.id;
    }
    const html = intakeCardHtml(order);
    if (renderedCards.get(card) !== html) {
      card.innerHTML = html;
      renderedCards.set(card, html);
    }
    // 순서가 다르면 그 자리로 옮긴다(새 카드도 여기서 들어간다).
    if (intakeOrders.children[index] !== card) intakeOrders.insertBefore(card, intakeOrders.children[index] || null);
  });
  // 목록에서 사라진(시작·거절된) 주문의 카드를 지운다.
  cards.forEach((card) => card.remove());
}

// 지난 폴링에 없던 주문마다 '새 주문' 알림을 띄운다.
function announceNewOrders(orders) {
  if (intake.seenIds) {
    orders.filter((order) => !intake.seenIds.has(order.id)).forEach((order) => {
      showToast(`새 주문 ${order.id} 접수`, `${orderSummary(order.order_items)}, ${order.destination}`);
    });
  }
  intake.seenIds = new Set(orders.map((order) => order.id));
}

// 더는 기다리지 않는 주문의 미리보기와 안내를 지운다.
function forgetGoneOrders() {
  const waiting = new Set(intake.orders.map((order) => order.id));
  [intake.previews, intake.notes].forEach((byOrder) => {
    [...byOrder.keys()].filter((id) => !waiting.has(id)).forEach((id) => byOrder.delete(id));
  });
}

// 주문 하나의 배정 미리보기(GET /api/orders/{id}/assignment)를 읽는다. 아무것도 예약하지 않는 계산이다.
// 같은 주문을 겹쳐 요청하지 않는다.
async function loadPreview(orderId) {
  if (intake.previewLoading.has(orderId)) return;
  intake.previewLoading.add(orderId);
  let preview;
  try {
    const response = await fetch(`/api/orders/${encodeURIComponent(orderId)}/assignment`, { cache: 'no-store' });
    const result = await response.json();
    preview = response.ok ? { result } : { error: apiErrorMessage(result, '배정 미리보기를 불러오지 못했습니다') };
  } catch {
    preview = { error: '서버에서 배정 미리보기를 받지 못했습니다' };
  } finally {
    intake.previewLoading.delete(orderId);
  }
  if (!intake.orders.some((order) => order.id === orderId)) return;  // 그사이 시작되거나 거절됐다
  intake.previews.set(orderId, preview);
  renderIntake();
}

// 미리보기는 핑키·재고 상태에 따라 바뀌므로 3초마다 모든 대기 주문을 다시 계산한다.
function refreshIntakePreviews() {
  intake.orders.forEach((order) => loadPreview(order.id));
}

// GET /api/orders(2초마다): 시작 대기 주문과 자동 출발 상태. 새 주문에는 미리보기를 바로 요청한다.
async function refreshIntake() {
  const seq = ++intakeRequestSeq;
  try {
    const response = await fetch('/api/orders', { cache: 'no-store' });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, '접수된 주문을 불러오지 못했습니다'));
    if (seq !== intakeRequestSeq) return;  // 더 새 폴링을 이미 보냈다. 그 응답을 쓴다
    announceNewOrders(result.orders);
    intake.orders = result.orders;
    intake.dispatch = result.dispatch || null;
    forgetGoneOrders();
    renderIntake();
    intake.orders.filter((order) => !intake.previews.has(order.id)).forEach((order) => loadPreview(order.id));
    intakeUpdated.textContent = `갱신 ${clockFormat.format(new Date())}`;
  } catch (error) {
    intakeUpdated.textContent = `갱신 실패: ${error.message}`;
  }
}

// [시작]·[거절]: POST /api/orders/{id}/start|reject. 거절되면 이유를 카드에 남기고, 성공하면 알림을 띄운다.
async function controlIntakeOrder(order, action) {
  const verb = action === 'start' ? '시작' : '거절';
  intake.busy.set(order.id, action);
  intake.notes.delete(order.id);
  renderIntake();
  try {
    const response = await fetch(`/api/orders/${encodeURIComponent(order.id)}/${action}`, { method: 'POST' });
    const result = await response.json();
    if (!response.ok) {
      // PINKY_UNAVAILABLE에는 그 순간의 핑키 판정이 들어 있다. 카드에 이유와 나란히 남겨 둔다.
      intake.notes.set(order.id, {
        title: `${verb}하지 못함`,
        text: apiErrorMessage(result, `주문을 ${verb}하지 못했습니다`),
        verdicts: result.assignment?.verdicts,
      });
    } else if (action === 'start') {
      const pinkyId = result.order.pinky_id;
      showToast(`${order.id} 시작`, `${orderSummary(order.order_items)}, 핑키 ${pinkyNames[pinkyId] || pinkyId}`);
    } else {
      showToast(`${order.id} 거절`, orderSummary(order.order_items));
    }
  } catch {
    // 응답이 없다고 거절된 것은 아니다: 서버에서 주문이 바뀌었을 수 있으니 다시 보내지 말고 확인해 달라고 한다.
    intake.notes.set(order.id, { title: '결과 확인 필요', text: `${verb} 요청의 응답을 받지 못했습니다. 주문 목록에서 상태를 확인하세요.` });
  } finally {
    intake.busy.delete(order.id);
    await Promise.all([refreshIntake(), refreshFlow()]);
  }
}

// 카드 버튼 클릭(이벤트 위임). 거절은 되돌릴 수 없으므로 확인 창을 거친다.
intakeOrders.addEventListener('click', (event) => {
  const button = event.target.closest('[data-intake-action]');
  const order = button && intake.orders.find((item) => item.id === button.dataset.orderId);
  if (!order || intake.busy.has(order.id)) return;
  const action = button.dataset.intakeAction;
  if (action === 'reject' && !window.confirm(
    `${order.id} 주문을 거절할까요?\n${orderSummary(order.order_items)}, ${order.destination}\n거절한 주문은 다시 시작할 수 없습니다.`,
  )) return;
  controlIntakeOrder(order, action);
});

// ==== 17. 시작과 폴링 주기 ====
showView(window.location.hash.slice(1));
refresh();
refreshFlow();
refreshIntake();
refreshTasks();
refreshArmCommands();
loadShelfSlots();
// 주기(ms): 서버·장비 3초, 주문 진행 1초, 주문 접수 2초, 배정 미리보기 3초, 작업 이력·팔 명령·연속 실행 1초,
// 주차 실시간 0.5초. pollParkingLive 말고는 뷰가 보이는지 따지지 않으므로, 어느 뷰에서든 새 주문 알림과
// 메뉴 배지가 갱신되고 종합 화면이 flowSummary·intake를 바로 쓸 수 있다.
window.setInterval(refresh, 3000);
window.setInterval(refreshFlow, 1000);
window.setInterval(refreshIntake, 2000);
window.setInterval(refreshIntakePreviews, 3000);
window.setInterval(refreshTasks, 1000);
window.setInterval(refreshArmCommands, 1000);
window.setInterval(refreshSequences, 1000);
window.setInterval(pollParkingLive, 500);
