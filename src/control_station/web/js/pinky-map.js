// 핑키 지도: 위에서 본 작업장을 GET /api/pinky/map으로 받아, 지도 미터 단위로 +y가 위가 되게 그린다. Nav2 점유 격자
// (지나갈 수 있음, 막힘, 모름), 빗금 친 금지 구역(keepout mask), 주차장 구역과 칸, 적재 자리 5와 교차로 7, 기록된
// 웨이포인트, 웨이포인트를 아는 수령 구역, 그리고 연결된 핑키마다 머리 방향을 따르는 화살표.
//
// app.js(escapeHtml, apiErrorMessage, pinkyNames, activeView, openOverlay) 뒤에 로드된다. 같은 그리기 함수가
// 핑키 뷰의 작은 지도, 크게 본 지도, 종합 화면에 함께 쓰이고, 각각 읽을 때마다 보내는 `pinkymap` 이벤트를 듣는다.
// 그중 하나가 화면에 있는 동안 1.5초마다 지도를 읽는다. pinky.js의 pinkyScreen, pinky-detail.js의 candidatesFor·
// previewCandidates·pickCandidate, pinky.js의 sendPinkyCommand도 쓰는데, 모두 이벤트가 일어날 때 부르므로 로드 순서와 무관하다.

const PINKY_MAP_POLL_MS = 1500;
const MAP_PX_PER_M = 200;  // 지도 1 m당 SVG 단위. SVG는 상자 크기에 맞춰 늘고 준다
const MAP_CELL_COLORS = { free: [255, 255, 255], occupied: [78, 89, 104], unknown: [229, 232, 235] };
const MAP_SVG_NS = 'http://www.w3.org/2000/svg';

// data: 마지막 GET /api/pinky/map 응답, loadError: 마지막 읽기 실패 이유, polling: 요청 중(겹쳐 보내지 않음).
// dockState는 여기서 만들지 않고 pinky-test.js가 적재 자리 카메라의 실시간 상태로 채운다(적재 자리 색).
const pinkyMapState = {
  data: null,
  loadError: null,
  polling: false,
  gridImages: new Map(),  // 격자 키 -> PNG data URL, 격자는 한 번만 칠한다
};

const mapViewer = document.querySelector('#mapViewer');

// 탭이 보이고, 핑키·종합 뷰이거나 크게 본 지도가 열려 있을 때만 지도를 읽는다.
function mapOnScreen() {
  return !document.hidden && (['pinky', 'dashboard'].includes(activeView) || !mapViewer.hidden);
}

// GET /api/pinky/map을 읽고(10초 제한) 성공·실패와 상관없이 'pinkymap' 이벤트를 보낸다. force면 화면 조건을 무시한다.
async function refreshPinkyMap({ force = false } = {}) {
  if (!force && (!mapOnScreen() || pinkyMapState.polling)) return;
  pinkyMapState.polling = true;
  try {
    const response = await fetch('/api/pinky/map', { cache: 'no-store', signal: AbortSignal.timeout(10000) });
    const result = await response.json();
    if (response.ok) {
      pinkyMapState.data = result;
      pinkyMapState.loadError = null;
    } else {
      pinkyMapState.loadError = apiErrorMessage(result, '지도를 불러오지 못했습니다');
    }
  } catch {
    pinkyMapState.loadError = '관제 서버에서 지도를 받지 못했습니다';
  } finally {
    pinkyMapState.polling = false;
  }
  document.dispatchEvent(new CustomEvent('pinkymap'));
}

// 지도를 그릴 수 없는 이유, 그릴 수 있으면 null.
function mapUnavailableReason() {
  const { data, loadError } = pinkyMapState;
  if (!data) return loadError || '지도를 불러오는 중…';
  if (!data.available || !data.grid?.cells?.length) return data.reason || '핑키 관제에 아직 지도가 없어요';
  return null;
}

// ---- 좌표 계산 ----

// 지도 미터 좌표 → SVG 좌표 변환. SVG는 y가 아래로 커지므로 지도 위쪽 끝(top)에서 뺀다.
function mapFrame(grid) {
  const width = grid.width * grid.resolution_m;
  const height = grid.height * grid.resolution_m;
  const x0 = grid.origin_x_m;
  const top = grid.origin_y_m + height;
  return {
    width: width * MAP_PX_PER_M,
    height: height * MAP_PX_PER_M,
    x: (xM) => (xM - x0) * MAP_PX_PER_M,
    y: (yM) => (top - yM) * MAP_PX_PER_M,
  };
}

// 격자가 바뀌었는지 알기 위한 키(크기·원점·해상도와 셀 값 체크섬).
function gridKey(grid) {
  if (!grid?.cells) return '';
  let sum = 0;
  grid.cells.forEach((value, index) => { sum = (sum + (value + 2) * (index + 1)) % 1000000007; });
  return `${grid.width}x${grid.height}@${grid.origin_x_m},${grid.origin_y_m},${grid.resolution_m}#${sum}`;
}

// 점유 격자를 그림으로 만든다. 0번 행이 지도 원점(아래쪽)이라 아래에서 위로 칠한다. 음수는 모름, 50 이상은 막힘.
function gridImage(grid) {
  const key = gridKey(grid);
  if (!pinkyMapState.gridImages.has(key)) {
    const canvas = document.createElement('canvas');
    canvas.width = grid.width;
    canvas.height = grid.height;
    const context = canvas.getContext('2d');
    const image = context.createImageData(grid.width, grid.height);
    grid.cells.forEach((value, index) => {
      const row = Math.floor(index / grid.width);
      const column = index % grid.width;
      const color = value < 0 ? MAP_CELL_COLORS.unknown : value >= 50 ? MAP_CELL_COLORS.occupied : MAP_CELL_COLORS.free;
      const offset = ((grid.height - 1 - row) * grid.width + column) * 4;
      image.data.set([...color, 255], offset);
    });
    context.putImageData(image, 0, 0);
    pinkyMapState.gridImages.set(key, canvas.toDataURL('image/png'));
  }
  return pinkyMapState.gridImages.get(key);
}

// 금지 구역 셀(0이 아닌 값)을 행마다 이어진 구간으로 합쳐 이 지도의 SVG 단위로 된 경로 하나로 만든다.
function keepoutPath(keepout, frame) {
  if (!keepout?.cells?.length) return '';
  const size = keepout.resolution_m * MAP_PX_PER_M;
  const parts = [];
  for (let row = 0; row < keepout.height; row += 1) {
    let start = null;
    for (let column = 0; column <= keepout.width; column += 1) {
      const blocked = column < keepout.width && keepout.cells[row * keepout.width + column] !== 0;
      if (blocked && start == null) start = column;
      if (!blocked && start != null) {
        const x = frame.x(keepout.origin_x_m + start * keepout.resolution_m);
        const y = frame.y(keepout.origin_y_m + (row + 1) * keepout.resolution_m);
        parts.push(`M${x.toFixed(1)} ${y.toFixed(1)}h${((column - start) * size).toFixed(1)}v${size.toFixed(1)}h${(-(column - start) * size).toFixed(1)}z`);
        start = null;
      }
    }
  }
  return parts.join('');
}

// ---- 그리기 ----

// 한 번만 만드는 바탕: 금지 구역 빗금 무늬, 격자 그림, 금지 구역, 50 cm 축척 막대, 위에 덮을 빈 층들.
function mapBaseSvg(svg, data, frame) {
  const hatch = `${svg.id}-keepout`;
  return `
    <defs>
      <pattern id="${hatch}" patternUnits="userSpaceOnUse" width="9" height="9" patternTransform="rotate(45)">
        <rect width="9" height="9" class="map-keepout-fill"/><line x1="0" y1="0" x2="0" y2="9" class="map-keepout-line"/>
      </pattern>
    </defs>
    <image href="${gridImage(data.grid)}" x="0" y="0" width="${frame.width}" height="${frame.height}" preserveAspectRatio="none" class="map-grid"/>
    <path d="${keepoutPath(data.keepout, frame)}" fill="url(#${hatch})" class="map-keepout"/>
    <g class="map-marks"></g>
    <g class="map-scale" transform="translate(16 ${frame.height - 18})">
      <rect x="-8" y="-24" width="${0.5 * MAP_PX_PER_M + 16}" height="32" rx="8"/>
      <line x1="0" y1="0" x2="${0.5 * MAP_PX_PER_M}" y2="0"/><text x="${0.25 * MAP_PX_PER_M}" y="-11">50 cm</text>
    </g>
    <g class="map-paths"></g>
    <g class="map-pinkies"></g>
    <g class="map-candidates"></g>
    <g class="map-pick"></g>`;
}

// 글자는 표시의 오른쪽에 둔다. 지도 오른쪽 끝 가까이면 왼쪽에 두어 지도 안에 남게 한다.
function captionSvg(text, x, frame, gap) {
  const left = x > frame.width * 0.62;
  return `<text class="map-caption ${left ? 'end' : 'start'}" x="${left ? -gap : gap}" y="0">${escapeHtml(text)}</text>`;
}

// 고정 표시: 주차장 구역·칸, 웨이포인트, 적재 자리(dock5)·교차로 7, 위치를 아는 수령 구역.
function mapMarksSvg(data, frame) {
  const parts = [];
  const area = data.parking_area;
  if (Array.isArray(area) && area.length === 4) {
    const [x0, x1, y0, y1] = area;
    parts.push(`<rect class="map-parking-area" x="${frame.x(x0)}" y="${frame.y(y1)}" width="${(x1 - x0) * MAP_PX_PER_M}" height="${(y1 - y0) * MAP_PX_PER_M}" rx="10"/>
      <text class="map-caption start" x="${frame.x(x0) + 4}" y="${frame.y(y0) + 14}">주차장</text>`);
  }
  (data.parking || []).forEach((spot) => {
    if (spot.x_m == null) return;
    parts.push(`<g class="map-spot" transform="translate(${frame.x(spot.x_m)} ${frame.y(spot.y_m)})">
      <rect x="-12" y="-12" width="24" height="24" rx="7"/><text>${escapeHtml(spot.name)}</text></g>`);
  });
  const seen = new Set();
  // 수령 구역이나 적재 자리가 그려진 웨이포인트는 두 번 그리지 않는다.
  const marked = new Set([...(data.zones || []).map((zone) => zone.waypoint), '5']);
  Object.values(data.waypoints || {}).flat().forEach((point) => {
    const key = `${point.label}@${point.x_m.toFixed(2)},${point.y_m.toFixed(2)}`;
    if (seen.has(key) || marked.has(point.label)) return;
    seen.add(key);
    parts.push(`<g class="map-waypoint" transform="translate(${frame.x(point.x_m)} ${frame.y(point.y_m)})">
      <circle r="4.5"/>${captionSvg(point.label, frame.x(point.x_m), frame, 8)}</g>`);
  });
  (data.landmarks || []).forEach((mark) => {
    if (mark.x_m == null) return;
    const station = mark.key === 'dock5';
    parts.push(`<g class="map-landmark ${station ? 'station' : 'junction'}" transform="translate(${frame.x(mark.x_m)} ${frame.y(mark.y_m)})">
      <circle r="${station ? 15 : 12}"/><text>${station ? '적' : '7'}</text>
      ${captionSvg(mark.label, frame.x(mark.x_m), frame, station ? 21 : 18)}</g>`);
  });
  (data.zones || []).forEach((zone) => {
    if (zone.x_m == null) return;
    parts.push(`<g class="map-zone" transform="translate(${frame.x(zone.x_m)} ${frame.y(zone.y_m)})">
      <circle r="14"/><text>${escapeHtml(((zone.label || '').match(/[0-9A-Z]/) || ['·'])[0])}</text>
      ${captionSvg(zone.label, frame.x(zone.x_m), frame, 20)}</g>`);
  });
  return parts.join('');
}

// 핑키 요소는 읽기 사이에 그대로 두므로, 화살표가 뛰지 않고 미끄러지며 돈다.
function drawMapPinkies(layer, pinkies, frame, stopping) {
  const seen = new Set();
  pinkies.filter(mapPlaced).forEach((pinky) => {
    seen.add(pinky.pinky_id);
    let mark = layer.querySelector(`[data-pinky="${CSS.escape(pinky.pinky_id)}"]`);
    if (!mark) {
      mark = document.createElementNS(MAP_SVG_NS, 'g');
      mark.dataset.pinky = pinky.pinky_id;
      mark.innerHTML = `<g class="map-pinky-heading"><circle r="15"/><path d="M 13 0 L -8 -9 L -3 0 L -8 9 Z"/></g>
        <text y="-25">${escapeHtml(pinkyNames[pinky.pinky_id] || pinky.name)}</text>`;
      mark.style.setProperty('--pinky-color', MAP_PATH_COLORS[pinky.pinky_id] || '#3182f6');
      layer.append(mark);
    }
    mark.setAttribute('class', `map-pinky${stopping.has(pinky.pinky_id) ? ' stopping' : ''}`);
    mark.style.transform = `translate(${frame.x(pinky.x_m)}px, ${frame.y(pinky.y_m)}px)`;
    // 지도 yaw는 y가 위인 좌표에서 +x부터 반시계로 돈다. 화면은 y가 아래라서 회전을 뒤집는다.
    mark.querySelector('.map-pinky-heading').style.transform = `rotate(${(-(pinky.yaw_rad || 0) * 180) / Math.PI}deg)`;
  });
  layer.querySelectorAll('[data-pinky]').forEach((mark) => { if (!seen.has(mark.dataset.pinky)) mark.remove(); });
}

// 핑키마다 지나온 길(실선)과 Nav2 계획(갈 길, 점선)을 핑키 고유 색으로 그린다.
const MAP_PATH_COLORS = { 10: '#e5487a', 11: '#12a150', 12: '#3182f6' };

function polylinePoints(points, frame) {
  return points.map(([x, y]) => `${frame.x(x).toFixed(1)},${frame.y(y).toFixed(1)}`).join(' ');
}

function drawMapPaths(layer, pinkies, frame) {
  const html = pinkies.filter((pinky) => pinky.connected).map((pinky) => {
    const color = MAP_PATH_COLORS[pinky.pinky_id] || '#3182f6';
    const trail = (pinky.trail || []).length > 1
      ? `<polyline class="map-trail" points="${polylinePoints(pinky.trail, frame)}" stroke="${color}"/>` : '';
    const plan = (pinky.plan || []).length > 1
      ? `<polyline class="map-plan" points="${polylinePoints(pinky.plan, frame)}" stroke="${color}"/>` : '';
    return trail + plan;
  }).join('');
  if (layer.dataset.html !== html) {
    layer.innerHTML = html;
    layer.dataset.html = html;
  }
}

// 라이다가 고를 자리를 여럿 남긴 핑키는 아직 지도 위치가 없다: AMCL이 지도 원점을 알려 주므로, 그런 핑키 둘이
// 같은 점에 그려지게 된다. 그래서 핑키는 그리지 않고 후보 자리를 번호와 함께 그린다.
function mapCandidates(pinkyId) {
  const pinky = (pinkyScreen.view?.pinkies || []).find((item) => item.pinky_id === pinkyId);
  return pinky && pinky.connected && typeof candidatesFor === 'function' ? candidatesFor(pinky) : [];
}

function mapPlaced(pinky) {
  return pinky.connected && pinky.x_m != null && pinky.y_m != null && !mapCandidates(pinky.pinky_id).length;
}

// 라이다가 구별하지 못한 자리들을 핑키 고유 색으로: 번호 원과, 그 자리에서 바라볼 방향의 뾰족한 끝. 크게 본 지도에서
// 누르면 하나를 고른다(상세의 '이 자리' 버튼과 같은 질문, 같은 명령).
function drawMapCandidates(layer, pinkies, frame) {
  // 나란히 선 두 핑키는 후보도 나란히 나온다: 한 핑키의 이름은 원 위에, 다음 핑키의 이름은 원 아래에 쓴다.
  const html = pinkies.filter((pinky) => mapCandidates(pinky.pinky_id).length).map((pinky, order) => mapCandidates(pinky.pinky_id).map((c, i) => `
    <g class="map-candidate" data-candidate-pinky="${escapeHtml(pinky.pinky_id)}" data-candidate-index="${i}"
       transform="translate(${frame.x(c.x_m).toFixed(1)} ${frame.y(c.y_m).toFixed(1)})" style="--pinky-color:${MAP_PATH_COLORS[pinky.pinky_id] || '#3182f6'}">
      <title>${escapeHtml(`${pinkyNames[pinky.pinky_id] || pinky.name} 후보 ${i + 1} · 일치도 ${Number(c.score).toFixed(2)}`)}</title>
      <g transform="rotate(${((-(c.yaw_rad || 0) * 180) / Math.PI).toFixed(1)})"><circle r="14"/><path d="M 27 0 L 16 -7 L 16 7 Z"/></g>
      <text class="map-candidate-number" y="5">${i + 1}</text>
      <text class="map-candidate-name" y="${order % 2 ? 32 : -21}">${escapeHtml(`${pinkyNames[pinky.pinky_id] || pinky.name} 후보 ${i + 1}`)}</text>
    </g>`).join('')).join('');
  if (layer.dataset.html !== html) {
    layer.innerHTML = html;
    layer.dataset.html = html;
  }
}

// 지금 지도를 `svg`에 그린다(지도가 없으면 `empty`에 이유를 보인다). 바탕·고정 표시는 바뀔 때만 다시 만든다.
function drawPinkyMap(svg, empty) {
  const reason = mapUnavailableReason();
  svg.hidden = Boolean(reason);
  empty.hidden = !reason;
  empty.textContent = reason || '';
  if (reason) return;
  const { data } = pinkyMapState;
  const frame = mapFrame(data.grid);
  const baseKey = `${gridKey(data.grid)}|${gridKey(data.keepout)}`;
  if (svg.dataset.baseKey !== baseKey) {
    svg.dataset.baseKey = baseKey;
    svg.dataset.marksKey = '';
    svg.setAttribute('viewBox', `0 0 ${frame.width} ${frame.height}`);
    svg.innerHTML = mapBaseSvg(svg, data, frame);
  }
  const marksKey = JSON.stringify([data.parking_area, data.parking, data.landmarks, data.waypoints, data.zones]);
  if (svg.dataset.marksKey !== marksKey) {
    svg.dataset.marksKey = marksKey;
    svg.querySelector('.map-marks').innerHTML = mapMarksSvg(data, frame);
  }
  const stopping = new Set((pinkyScreen.view?.pinkies || []).filter((pinky) => pinky.stop_pending).map((pinky) => pinky.pinky_id));
  // 천장 카메라가 적재 자리의 핑키를 정상 위치로 보면 초록, 벗어났다고 보면 주황으로 칠한다.
  const station = svg.querySelector('.map-landmark.station');
  station?.classList.toggle('arrived', pinkyMapState.dockState === 'IN_POSITION');
  station?.classList.toggle('off', pinkyMapState.dockState === 'OFF_POSITION');
  drawMapPaths(svg.querySelector('.map-paths'), data.pinkies || [], frame);
  drawMapPinkies(svg.querySelector('.map-pinkies'), data.pinkies || [], frame, stopping);
  drawMapCandidates(svg.querySelector('.map-candidates'), data.pinkies || [], frame);
  const placed = (data.pinkies || []).filter(mapPlaced);
  svg.setAttribute('aria-label', `핑키 지도${data.simulated ? devWord(' (모의)') : ''}: ${placed.length
    ? placed.map((pinky) => `${pinky.name} x ${pinky.x_m.toFixed(2)} m, y ${pinky.y_m.toFixed(2)} m`).join(', ')
    : '위치를 아는 핑키가 없습니다'}`);
}

// 지도의 안내문. 단, 개발자 화면이 아니면 모의 지도의 안내는 뺀다.
function mapNote() {
  const data = pinkyMapState.data;
  return !data?.note || (data.simulated && !devMode) ? '' : data.note;
}

// 웨이포인트가 기록되지 않은 수령 구역은 그릴 수 없으니 지도 옆에 이름만 적는다.
// (위치가 없는 핑키는 핑키 목록과 노선도가 따로 적는다.)
function mapZoneNotesHtml() {
  return (pinkyMapState.data?.zones || []).filter((zone) => zone.x_m == null)
    .map((zone) => `<span class="chip">${escapeHtml(`${zone.label} 위치 미기록 (웨이포인트 ${zone.waypoint || '없음'})`)}</span>`).join('');
}

// ---- 크게 본 지도 ----

const mapViewerSvg = document.querySelector('#pinkyMapLarge');
const mapViewerEmpty = document.querySelector('#pinkyMapLargeEmpty');
const mapViewerTag = document.querySelector('#mapViewerTag');
const mapViewerNote = document.querySelector('#mapViewerNote');
const mapViewerList = document.querySelector('#mapPinkyList');
const mapViewerOff = document.querySelector('#mapViewerOff');

// 크게 본 지도(열려 있을 때만): 지도와, 핑키별 위치 목록과 [후보 다시 보기]·[↗ 위치 다시 잡기] 버튼.
function renderMapViewer() {
  if (mapViewer.hidden) return;
  drawPinkyMap(mapViewerSvg, mapViewerEmpty);
  const data = pinkyMapState.data;
  mapViewerTag.innerHTML = data?.simulated ? '<span class="sim-tag">모의</span>' : '';
  const note = mapNote();
  mapViewerNote.hidden = !note;
  mapViewerNote.textContent = note;
  mapViewerOff.innerHTML = mapZoneNotesHtml();
  mapViewerList.innerHTML = (data?.pinkies || []).map((pinky) => `
    <li class="${mapPlaced(pinky) ? '' : 'muted'}">
      ${pinkyBadge(pinky.pinky_id, { offline: !pinky.connected })}
      <span><strong>${escapeHtml(pinky.phase_label || '단계 모름')}</strong>
      <small>${mapPlaced(pinky)
        ? `x ${pinky.x_m.toFixed(2)} m, y ${pinky.y_m.toFixed(2)} m, ${Math.round(((pinky.yaw_rad || 0) * 180) / Math.PI)}°`
        : !pinky.connected ? '연결 끊김' : mapCandidates(pinky.pinky_id).length
          ? `위치 후보 ${mapCandidates(pinky.pinky_id).length}곳 · 지도에서 번호를 눌러 고르세요` : '위치를 받지 못했어요'}</small></span>
      ${pinky.connected ? `<div class="map-pinky-actions">
        <button type="button" class="secondary small" data-map-candidates="${escapeHtml(pinky.pinky_id)}"
          title="라이다로 맞는 자리를 다시 찾아 지도에 번호로 보여 줘요. 위치가 이미 들어가 있어도 다시 고를 수 있어요">후보 다시 보기</button>
        <button type="button" class="secondary small" data-map-set-pose="${escapeHtml(pinky.pinky_id)}"
          ${mapPick.pinkyId === pinky.pinky_id ? 'aria-pressed="true"' : ''}>↗ 위치 다시 잡기</button></div>` : ''}
    </li>`).join('');
  renderMapPick();
}

function openMapViewer(opener) {
  openOverlay(mapViewer, opener);
  renderMapViewer();
  refreshPinkyMap({ force: true });
}

// ---- 위치 다시 잡기: 작업자가 크게 본 지도에 핑키를 놓는다(RViz의 2D Pose Estimate와 같다) ----
// 핑키는 주행준비 때 라이다로 자기 위치를 찾는다. 지도가 엉뚱한 곳을 보여 주면, 작업자가 실제로 선 곳을 누른 채
// 바라보는 방향으로 끈다. 끌지 않고 누르기만 하면 지금 방향을 그대로 쓴다.
// 자세는 POST /api/pinky/{id}/set-pose로 가고, 라이다는 그 근처(30 cm, 30°)에서만 다듬으므로 대칭인 작업장에서도
// 작업자가 놓은 자리에 머문다. Esc, 취소, 지도 닫기로 이 모드가 끝난다.

const MAP_PICK_CLICK_PX = 8;  // 화면에서 이보다 덜 움직인 누르기는 클릭으로 본다
// toMap: 누르기 시작할 때 잡은 화면→SVG 변환. 끄는 동안 계속 써서, 그사이 지도가 포인터 아래에서 밀려도
// (읽기마다 옆 목록이 다시 그려짐) 방향이 휘지 않게 한다.
const mapPick = { pinkyId: null, start: null, end: null, startPx: null, endPx: null, toMap: null };
const mapPickBar = document.querySelector('#mapPickBar');
const mapPickText = document.querySelector('#mapPickText');

function mapPinky(pinkyId) {
  return (pinkyMapState.data?.pinkies || []).find((pinky) => pinky.pinky_id === pinkyId);
}

// SVG 위 포인터 이벤트 → SVG 단위(viewBox)와 지도 미터. 지도 밖이면 null.
function mapPointAt(event, toMap = mapViewerSvg.getScreenCTM()?.inverse()) {
  const grid = pinkyMapState.data?.grid;
  if (!grid || !toMap) return null;
  const p = new DOMPoint(event.clientX, event.clientY).matrixTransform(toMap);
  const frame = mapFrame(grid);
  if (p.x < 0 || p.y < 0 || p.x > frame.width || p.y > frame.height) return null;
  return {
    svgX: p.x, svgY: p.y,
    x_m: p.x / MAP_PX_PER_M + grid.origin_x_m,
    y_m: grid.origin_y_m + grid.height * grid.resolution_m - p.y / MAP_PX_PER_M,
  };
}

// 지정한 자세: 누른 곳이 위치이고, 끌었으면 끈 방향이 yaw, 아니면 지금 yaw를 그대로 쓴다.
function mapPickPose() {
  const { start, end, startPx, endPx } = mapPick;
  const dragged = endPx && Math.hypot(endPx.x - startPx.x, endPx.y - startPx.y) >= MAP_PICK_CLICK_PX;
  const yaw = dragged ? Math.atan2(end.y_m - start.y_m, end.x_m - start.x_m) : (mapPinky(mapPick.pinkyId)?.yaw_rad ?? 0);
  return { x_m: start.x_m, y_m: start.y_m, yaw_rad: yaw };
}

// 위치 지정 중의 안내 막대와 지도 위 미리보기(원과 방향 화살표).
function renderMapPick() {
  const picking = Boolean(mapPick.pinkyId);
  mapPickBar.hidden = !picking;
  mapViewerSvg.classList.toggle('picking', picking);
  const layer = mapViewerSvg.querySelector('.map-pick');
  if (!picking) {
    if (layer) layer.innerHTML = '';
    return;
  }
  const name = pinkyName(mapPick.pinkyId);
  mapPickText.textContent = mapPick.start
    ? `${name}: 바라보는 방향으로 끌었다가 놓으세요`
    : `${name}이(가) 실제로 있는 곳을 누른 채, 바라보는 방향으로 끌었다가 놓으세요. 누르기만 하면 지금 방향을 그대로 써요.`;
  if (!layer || !mapPick.start) {
    if (layer) layer.innerHTML = '';
    return;
  }
  const pose = mapPickPose();
  const color = MAP_PATH_COLORS[mapPick.pinkyId] || '#3182f6';
  const deg = (-pose.yaw_rad * 180) / Math.PI;
  layer.innerHTML = `<g transform="translate(${mapPick.start.svgX.toFixed(1)} ${mapPick.start.svgY.toFixed(1)})" style="color:${color}">
    <circle class="map-pick-spot" r="18"/>
    <g transform="rotate(${deg.toFixed(1)})"><line class="map-pick-ray" x1="0" y1="0" x2="44" y2="0"/>
      <path class="map-pick-head" d="M 54 0 L 40 -9 L 40 9 Z"/></g></g>`;
}

// 위치 지정 모드 시작(지도가 닫혀 있으면 연다). endMapPick은 모드를 끝낸다.
function startMapPick(pinkyId, opener) {
  mapPick.pinkyId = pinkyId;
  mapPick.start = mapPick.end = mapPick.startPx = mapPick.endPx = mapPick.toMap = null;
  if (mapViewer.hidden) openMapViewer(opener);
  else renderMapViewer();
  renderMapPick();
}

function endMapPick() {
  if (!mapPick.pinkyId) return;
  mapPick.pinkyId = null;
  mapPick.start = mapPick.end = mapPick.startPx = mapPick.endPx = mapPick.toMap = null;
  renderMapViewer();
  renderMapPick();
}

// 지정한 자세를 POST /api/pinky/{id}/set-pose로 보낸다.
function sendMapPick() {
  const pinkyId = mapPick.pinkyId;
  const pose = mapPickPose();
  endMapPick();
  sendPinkyCommand(pinkyId, 'set_pose', { path: 'set-pose', body: { pose }, label: '위치 다시 잡기' });
}

// 누르기 → 끌기 → 놓기로 위치와 방향을 정한다. 포인터를 잡아 두어 지도 밖으로 끌어도 이어진다.
mapViewerSvg.addEventListener('pointerdown', (event) => {
  if (!mapPick.pinkyId || event.button !== 0) return;
  const toMap = mapViewerSvg.getScreenCTM()?.inverse();
  const point = mapPointAt(event, toMap);
  if (!point) return;
  mapPick.toMap = toMap;
  mapPick.start = mapPick.end = point;
  mapPick.startPx = mapPick.endPx = { x: event.clientX, y: event.clientY };
  mapViewerSvg.setPointerCapture(event.pointerId);
  event.preventDefault();
  renderMapPick();
});
mapViewerSvg.addEventListener('pointermove', (event) => {
  if (!mapPick.pinkyId || !mapPick.start) return;
  mapPick.end = mapPointAt(event, mapPick.toMap) || mapPick.end;
  mapPick.endPx = { x: event.clientX, y: event.clientY };
  renderMapPick();
});
mapViewerSvg.addEventListener('pointerup', (event) => {
  if (!mapPick.pinkyId || !mapPick.start) return;
  mapPick.end = mapPointAt(event, mapPick.toMap) || mapPick.end;
  mapPick.endPx = { x: event.clientX, y: event.clientY };
  sendMapPick();
});
mapViewerSvg.addEventListener('pointercancel', () => {
  if (!mapPick.start) return;
  mapPick.start = mapPick.end = mapPick.startPx = mapPick.endPx = mapPick.toMap = null;
  renderMapPick();
});
mapViewerSvg.addEventListener('click', (event) => {
  const mark = event.target.closest('.map-candidate');
  if (!mark || mapPick.pinkyId) return;  // 위치 다시 잡기 중에는 누르기가 핑키를 놓는 동작이다
  pickCandidate(mark.dataset.candidatePinky, Number(mark.dataset.candidateIndex));
});
document.querySelector('#mapPickCancel').addEventListener('click', endMapPick);
mapViewerList.addEventListener('click', (event) => {
  // 후보 다시 보기: 이 핑키를 라이다로 새로 맞춰 고를 자리를 번호로 그린다. 위치가 이미 들어가 있어도 한다
  // (대칭인 작업장에서는 자동 선택이 틀릴 수 있다). 번호를 고르기 전에는 아무것도 바뀌지 않는다.
  const again = event.target.closest('[data-map-candidates]');
  if (again) {
    const pinkyId = again.dataset.mapCandidates;
    again.disabled = true;
    previewCandidates(pinkyId).then(() => {
      const preview = pinkyDetail.preview?.pinkyId === pinkyId ? pinkyDetail.preview : null;
      if (!preview?.candidates.length) showToast(preview?.error || '라이다가 맞는 자리를 찾지 못했어요. ↗ 위치 다시 잡기로 직접 넣어 주세요.');
      renderMapViewer();
    });
    return;
  }
  const button = event.target.closest('[data-map-set-pose]');
  if (!button) return;
  if (mapPick.pinkyId === button.dataset.mapSetPose) endMapPick();
  else startMapPick(button.dataset.mapSetPose, button);
});
// 지도를 닫으면 위치 지정도 끝낸다.
mapViewer.addEventListener('overlayclose', endMapPick);

// 읽을 때마다 다시 그리고, 뷰 전환·탭 복귀 때 바로 읽고, 1.5초마다 읽는다(화면에 없으면 refreshPinkyMap이 건너뛴다).
document.addEventListener('pinkymap', renderMapViewer);
document.addEventListener('viewchange', () => refreshPinkyMap());
document.addEventListener('visibilitychange', () => refreshPinkyMap());
window.setInterval(refreshPinkyMap, PINKY_MAP_POLL_MS);
