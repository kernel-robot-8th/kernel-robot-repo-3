// 핑키 지도: the cell from above, from GET /api/pinky/map, drawn in map metres with +y up. The Nav2 occupancy grid
// (free, occupied, unknown), the keepout mask hatched, the parking area and spots, station 5 and junction 7, recorded
// waypoints, the receiving zones whose waypoint is known, and each connected Pinky as an arrow along its heading.
//
// Loaded after app.js (escapeHtml, apiErrorMessage, pinkyNames, activeView, openOverlay). The same drawing serves
// the small map on the 핑키 view, the enlarged map and the 종합 dashboard; each listens for `pinkymap`, sent after
// every read. The map is read every 1.5 s while one of them is on screen.

const PINKY_MAP_POLL_MS = 1500;
const MAP_PX_PER_M = 200;  // SVG user units per map metre; the SVG scales to its box
const MAP_CELL_COLORS = { free: [255, 255, 255], occupied: [78, 89, 104], unknown: [229, 232, 235] };
const MAP_SVG_NS = 'http://www.w3.org/2000/svg';

const pinkyMapState = {
  data: null,
  loadError: null,
  polling: false,
  gridImages: new Map(),  // grid key -> PNG data URL, so a grid is painted once
};

const mapViewer = document.querySelector('#mapViewer');

function mapOnScreen() {
  return !document.hidden && (['pinky', 'dashboard'].includes(activeView) || !mapViewer.hidden);
}

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

// Why the map cannot be drawn, or null when it can.
function mapUnavailableReason() {
  const { data, loadError } = pinkyMapState;
  if (!data) return loadError || '지도를 불러오는 중…';
  if (!data.available || !data.grid?.cells?.length) return data.reason || '핑키 관제에 아직 지도가 없어요';
  return null;
}

// ---- geometry ----

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

function gridKey(grid) {
  if (!grid?.cells) return '';
  let sum = 0;
  grid.cells.forEach((value, index) => { sum = (sum + (value + 2) * (index + 1)) % 1000000007; });
  return `${grid.width}x${grid.height}@${grid.origin_x_m},${grid.origin_y_m},${grid.resolution_m}#${sum}`;
}

// The occupancy grid as an image: row 0 is the map origin (bottom), so it is painted bottom up.
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

// Keepout cells (nonzero) merged into one path of row runs, in this map's SVG units.
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

// ---- drawing ----

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
    <g class="map-pick"></g>`;
}

// A caption goes right of its mark, or left of it near the right edge so it stays inside the map.
function captionSvg(text, x, frame, gap) {
  const left = x > frame.width * 0.62;
  return `<text class="map-caption ${left ? 'end' : 'start'}" x="${left ? -gap : gap}" y="0">${escapeHtml(text)}</text>`;
}

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
  Object.values(data.waypoints || {}).flat().forEach((point) => {
    const key = `${point.label}@${point.x_m.toFixed(2)},${point.y_m.toFixed(2)}`;
    if (seen.has(key)) return;
    seen.add(key);
    parts.push(`<g class="map-waypoint" transform="translate(${frame.x(point.x_m)} ${frame.y(point.y_m)})">
      <circle r="4.5"/>${captionSvg(point.label, frame.x(point.x_m), frame, 8)}</g>`);
  });
  (data.landmarks || []).forEach((mark) => {
    if (mark.x_m == null) return;
    const station = mark.key === 'dock5';
    parts.push(`<g class="map-landmark ${station ? 'station' : 'junction'}" transform="translate(${frame.x(mark.x_m)} ${frame.y(mark.y_m)})">
      <circle r="${station ? 15 : 12}"/><text>${station ? '5' : '7'}</text>
      ${captionSvg(mark.label, frame.x(mark.x_m), frame, station ? 21 : 18)}</g>`);
  });
  (data.zones || []).forEach((zone) => {
    if (zone.x_m == null) return;
    parts.push(`<g class="map-zone" transform="translate(${frame.x(zone.x_m)} ${frame.y(zone.y_m)})">
      <circle r="14"/><text>${escapeHtml((zone.label || '').slice(-1))}</text>
      ${captionSvg(zone.label, frame.x(zone.x_m), frame, 20)}</g>`);
  });
  return parts.join('');
}

// Pinkys keep their elements between reads, so an arrow slides and turns instead of jumping.
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
      layer.append(mark);
    }
    mark.setAttribute('class', `map-pinky${stopping.has(pinky.pinky_id) ? ' stopping' : ''}`);
    mark.style.transform = `translate(${frame.x(pinky.x_m)}px, ${frame.y(pinky.y_m)}px)`;
    // Map yaw turns counterclockwise from +x with y up; on screen y points down, so the turn is mirrored.
    mark.querySelector('.map-pinky-heading').style.transform = `rotate(${(-(pinky.yaw_rad || 0) * 180) / Math.PI}deg)`;
  });
  layer.querySelectorAll('[data-pinky]').forEach((mark) => { if (!seen.has(mark.dataset.pinky)) mark.remove(); });
}

// Each Pinky's trail (where it has been, solid) and Nav2's plan (where it means to go, dashed), in its own colour.
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

function mapPlaced(pinky) {
  return pinky.connected && pinky.x_m != null && pinky.y_m != null;
}

// Draws the current map into `svg` (and shows `empty` with the reason when there is none).
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
  drawMapPaths(svg.querySelector('.map-paths'), data.pinkies || [], frame);
  drawMapPinkies(svg.querySelector('.map-pinkies'), data.pinkies || [], frame, stopping);
  const placed = (data.pinkies || []).filter(mapPlaced);
  svg.setAttribute('aria-label', `핑키 지도${data.simulated ? devWord(' (모의)') : ''}: ${placed.length
    ? placed.map((pinky) => `${pinky.name} x ${pinky.x_m.toFixed(2)} m, y ${pinky.y_m.toFixed(2)} m`).join(', ')
    : '위치를 아는 핑키가 없습니다'}`);
}

// The map's note, except the mock map's own remark outside dev mode.
function mapNote() {
  const data = pinkyMapState.data;
  return !data?.note || (data.simulated && !devMode) ? '' : data.note;
}

// Receiving zones whose waypoint is not recorded cannot be drawn; they are named beside the map instead.
// (Pinkys without a pose are listed by the Pinky list and the route strip.)
function mapZoneNotesHtml() {
  return (pinkyMapState.data?.zones || []).filter((zone) => zone.x_m == null)
    .map((zone) => `<span class="chip">${escapeHtml(`${zone.label} 위치 미기록 (웨이포인트 ${zone.waypoint || '없음'})`)}</span>`).join('');
}

// ---- enlarged map ----

const mapViewerSvg = document.querySelector('#pinkyMapLarge');
const mapViewerEmpty = document.querySelector('#pinkyMapLargeEmpty');
const mapViewerTag = document.querySelector('#mapViewerTag');
const mapViewerNote = document.querySelector('#mapViewerNote');
const mapViewerList = document.querySelector('#mapPinkyList');
const mapViewerOff = document.querySelector('#mapViewerOff');

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
        : pinky.connected ? '위치를 받지 못했어요' : '연결 끊김'}</small></span>
      ${pinky.connected ? `<button type="button" class="secondary small" data-map-set-pose="${escapeHtml(pinky.pinky_id)}"
        ${mapPick.pinkyId === pinky.pinky_id ? 'aria-pressed="true"' : ''}>↗ 위치 다시 잡기</button>` : ''}
    </li>`).join('');
  renderMapPick();
}

function openMapViewer(opener) {
  openOverlay(mapViewer, opener);
  renderMapViewer();
  refreshPinkyMap({ force: true });
}

// ---- 위치 다시 잡기: the operator puts a Pinky on the enlarged map, like RViz's 2D Pose Estimate ----
// The Pinky finds itself with its LiDAR when it is prepared; when the map shows it in the wrong place, the operator
// presses where it really stands and drags the way it faces. A click without a drag keeps the heading it has now.
// The pose goes to POST /api/pinky/{id}/set-pose; the LiDAR polishes it only nearby (30 cm, 30°), so the answer
// stays where the operator put it even in the symmetric cell. Esc, 취소 or closing the map ends the mode.

const MAP_PICK_CLICK_PX = 8;  // a press that moves less than this on screen is a click
// toMap: the screen-to-SVG transform taken when the press began, kept for the whole drag, so a map that shifts
// under the pointer meanwhile (the side list re-rendered on a read) does not bend the heading.
const mapPick = { pinkyId: null, start: null, end: null, startPx: null, endPx: null, toMap: null };
const mapPickBar = document.querySelector('#mapPickBar');
const mapPickText = document.querySelector('#mapPickText');

function mapPinky(pinkyId) {
  return (pinkyMapState.data?.pinkies || []).find((pinky) => pinky.pinky_id === pinkyId);
}

// A pointer event on the SVG -> SVG units (the viewBox) and map metres, or null outside the map.
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

function mapPickPose() {
  const { start, end, startPx, endPx } = mapPick;
  const dragged = endPx && Math.hypot(endPx.x - startPx.x, endPx.y - startPx.y) >= MAP_PICK_CLICK_PX;
  const yaw = dragged ? Math.atan2(end.y_m - start.y_m, end.x_m - start.x_m) : (mapPinky(mapPick.pinkyId)?.yaw_rad ?? 0);
  return { x_m: start.x_m, y_m: start.y_m, yaw_rad: yaw };
}

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

function sendMapPick() {
  const pinkyId = mapPick.pinkyId;
  const pose = mapPickPose();
  endMapPick();
  sendPinkyCommand(pinkyId, 'set_pose', { path: 'set-pose', body: { pose }, label: '위치 다시 잡기' });
}

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
document.querySelector('#mapPickCancel').addEventListener('click', endMapPick);
mapViewerList.addEventListener('click', (event) => {
  const button = event.target.closest('[data-map-set-pose]');
  if (!button) return;
  if (mapPick.pinkyId === button.dataset.mapSetPose) endMapPick();
  else startMapPick(button.dataset.mapSetPose, button);
});
mapViewer.addEventListener('overlayclose', endMapPick);

document.addEventListener('pinkymap', renderMapViewer);
document.addEventListener('viewchange', () => refreshPinkyMap());
document.addEventListener('visibilitychange', () => refreshPinkyMap());
window.setInterval(refreshPinkyMap, PINKY_MAP_POLL_MS);
