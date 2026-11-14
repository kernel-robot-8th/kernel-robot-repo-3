// 핑키 경로 노선도: the Pinky route as a strip of stops in driving order, with each Pinky's dot at the stop it
// reports. No scale and no map coordinates: it shows order only, and says so on screen.
//
// Loaded after app.js (escapeHtml, pinkyNames) and before pinky.js, which calls renderRouteStrip(view) with every
// GET /api/pinky answer.
//
// Where a dot goes, from the fleet dashboard's own words:
// - not connected: not drawn, listed beside the strip
// - parked, or no route at all: the parking spot P
// - docking_manual / docking_anchor / docked_manual: station 5 (loading); dock_exit: just past 5
// - course_complete: the receiving zone, the last stop
// - otherwise route.current_label: a stop ("7"), a leg between two stops ("3→7:04" on the dense route), "도킹 탈출",
//   or "주차장" at the end of the route (driving home). A stop that appears twice (7, 1) is picked by
//   route.completed / route.total, the share of the dense route already driven.
// Anything else (the mock world's "이동 중") is listed beside the strip instead of guessed.

const ROUTE_FALLBACK_LABELS = ['1', '2', '3', '7', '4', '5', '6', '7', '9', '8', '1'];  // pinky_fleet_dashboard ROUTE_LABELS
const ROUTE_PARKING = '주차장';
const ROUTE_DOCK_EXIT = '도킹 탈출';
const ROUTE_STATION_LABEL = '5';
const ROUTE_STATION_PHASES = new Set(['docking_manual', 'docking_anchor', 'docked_manual']);
const ROUTE_LEG = /^(.+)→(.+):\d+$/;
const routeLayout = { pad: 72, minGap: 58, lineY: 64, dotRise: 34, dotSpread: 32, labelY: 102, returnY: 134, height: 152 };

const routeScroll = document.querySelector('#routeScroll');
const routeSvg = document.querySelector('#routeStrip');
const routeOffStrip = document.querySelector('#routeOffStrip');
const routeSvgNamespace = 'http://www.w3.org/2000/svg';

const routeStrip = {
  labels: ROUTE_FALLBACK_LABELS,  // replaced by the receiving zone's route from GET /api/catalog when it answers
  zoneLabel: '수령 구역',
  catalogRequested: false,
  trackKey: '',                    // labels and spacing the drawn track was built for
  scrolledToDots: false,           // at phone width the strip scrolls once to the first Pinky, then stays put
  lastView: null,
};

async function loadRouteFromCatalog() {
  routeStrip.catalogRequested = true;
  try {
    const response = await fetch('/api/catalog', { cache: 'no-store' });
    const zone = response.ok ? (await response.json()).zones?.[0] : null;
    if (zone?.route_labels?.length) {
      routeStrip.labels = zone.route_labels;
      routeStrip.zoneLabel = zone.label;
    }
  } catch {
    // The fallback route stays; it is the dashboard's fixed course.
  }
  if (routeStrip.lastView) renderRouteStrip(routeStrip.lastView);
}

// P, then the route's stops; the last stop is the receiving zone and the first 5 is the loading station.
function routeStops() {
  const { labels } = routeStrip;
  const station = labels.indexOf(ROUTE_STATION_LABEL);
  return [
    { key: ROUTE_PARKING, kind: 'parking' },
    ...labels.map((label, index) => ({
      key: label,
      kind: index === labels.length - 1 ? 'zone' : index === station ? 'station' : 'stop',
    })),
  ];
}

// Of several matching places, the one closest to the share of the route already driven.
function closestToProgress(candidates, route, stopCount) {
  if (candidates.length < 2 || !route.total) return candidates[0];
  const progress = (route.completed ?? 0) / route.total;
  const distance = (index) => Math.abs(index / stopCount - progress);
  return candidates.reduce((best, index) => (distance(index) < distance(best) ? index : best));
}

// { at: stop index } | { between: index of the stop before } | { returning: true } | { off: why it is not drawn }
function routePlace(pinky, stops) {
  if (!pinky.connected) return { off: '연결 끊김' };
  const route = pinky.route || {};
  const last = stops.length - 1;
  const station = stops.findIndex((stop) => stop.kind === 'station');
  const progress = route.total ? (route.completed ?? 0) / route.total : 0;
  if (ROUTE_STATION_PHASES.has(pinky.phase) && station >= 0) return { at: station };
  if (pinky.phase === 'dock_exit' && station >= 0) return { between: station };
  if (pinky.phase === 'course_complete') return { at: last };
  if (pinky.phase === 'parked' || (!route.status && !route.current_label)) return { at: 0 };
  const label = route.current_label || '';
  if (label === ROUTE_DOCK_EXIT && station >= 0) return { between: station };
  if (label === ROUTE_PARKING) return progress > 0.5 ? { returning: true } : { at: 0 };
  const leg = label.match(ROUTE_LEG);
  if (leg) {
    const [, from, to] = leg;
    if ((from === ROUTE_DOCK_EXIT || to === ROUTE_DOCK_EXIT) && station >= 0) return { between: station };
    if (to === ROUTE_PARKING && progress > 0.5) return { returning: true };
    const legs = stops.slice(0, -1).map((stop, index) => index)
      .filter((index) => stops[index].key === from && stops[index + 1].key === to);
    if (legs.length) return { between: closestToProgress(legs, route, stops.length) };
  }
  const matches = stops.map((stop, index) => index).filter((index) => index > 0 && stops[index].key === label);
  if (matches.length) return { at: closestToProgress(matches, route, stops.length) };
  return { off: '경로 위치 모름' };
}

function stopName(stops, index) {
  const stop = stops[index];
  if (stop.kind === 'parking') return '주차장';
  if (stop.kind === 'station') return `${stop.key}번 적재`;
  if (stop.kind === 'zone') return routeStrip.zoneLabel;
  return `${stop.key}번`;
}

function placeName(stops, place) {
  if (place.returning) return '주차장으로 돌아가는 중';
  if (place.between != null) return `${stopName(stops, place.between)}, ${stopName(stops, place.between + 1)} 사이`;
  return stopName(stops, place.at);
}

// ---- drawing ----

function routeGeometry(stops) {
  const { pad, minGap } = routeLayout;
  const room = routeScroll.clientWidth - 2 * pad;
  const gap = Math.max(minGap, Math.floor(room / (stops.length - 1)));
  return { gap, x: (index) => pad + index * gap, width: 2 * pad + (stops.length - 1) * gap };
}

function stopSvg(stop, cx) {
  const { lineY, labelY } = routeLayout;
  if (stop.kind === 'parking') {
    return `<g class="route-parking"><rect x="${cx - 14}" y="${lineY - 14}" width="28" height="28" rx="9"/>
      <text x="${cx}" y="${lineY}">P</text></g>
      <text class="route-caption" x="${cx}" y="${labelY}">주차장</text>`;
  }
  if (stop.kind === 'station') {
    return `<g class="route-station"><circle cx="${cx}" cy="${lineY}" r="16"/><text x="${cx}" y="${lineY}">${escapeHtml(stop.key)}</text></g>
      <text class="route-caption end" x="${cx - 21}" y="${labelY}">왼쪽 팔</text>
      <text class="route-caption strong station" x="${cx}" y="${labelY}">적재</text>
      <text class="route-caption start" x="${cx + 21}" y="${labelY}">오른쪽 팔</text>`;
  }
  if (stop.kind === 'zone') {
    return `<g class="route-zone"><circle cx="${cx}" cy="${lineY}" r="16"/><text x="${cx}" y="${lineY}">${escapeHtml(stop.key)}</text></g>
      <text class="route-caption strong zone" x="${cx}" y="${labelY}">${escapeHtml(routeStrip.zoneLabel)}</text>`;
  }
  return `<g class="route-stop"><circle cx="${cx}" cy="${lineY}" r="11"/><text x="${cx}" y="${lineY}">${escapeHtml(stop.key)}</text></g>`;
}

// The track (line, stops, captions, the dashed way home) is rebuilt only when the route or the spacing changes,
// so the Pinky dots on top keep their elements and slide when they move.
function drawRouteTrack(stops, geometry) {
  const { lineY, returnY, height } = routeLayout;
  const first = geometry.x(0);
  const last = geometry.x(stops.length - 1);
  const middle = (first + last) / 2;
  routeSvg.setAttribute('viewBox', `0 0 ${geometry.width} ${height}`);
  routeSvg.setAttribute('width', String(geometry.width));
  routeSvg.setAttribute('height', String(height));
  routeSvg.innerHTML = `
    <path class="route-return" d="M ${last + 18} ${lineY} C ${last + 62} ${lineY}, ${last + 62} ${returnY}, ${last + 18} ${returnY}
      L ${first - 18} ${returnY} C ${first - 62} ${returnY}, ${first - 62} ${lineY}, ${first - 18} ${lineY}"/>
    <text class="route-return-label" x="${middle}" y="${returnY}">수령 확인 뒤 주차장으로 돌아가요</text>
    <line class="route-line" x1="${first}" y1="${lineY}" x2="${last}" y2="${lineY}"/>
    ${stops.map((stop, index) => stopSvg(stop, geometry.x(index))).join('')}
    <g class="route-dots"></g>`;
}

function dotPosition(place, geometry, stops) {
  const { lineY, dotRise, returnY } = routeLayout;
  // Driving home: on the dashed way back, toward P and clear of the captions and the way-back label.
  if (place.returning) return { x: geometry.x(0) + (geometry.x(stops.length - 1) - geometry.x(0)) * 0.22, y: returnY - 20 };
  if (place.between != null) return { x: (geometry.x(place.between) + geometry.x(place.between + 1)) / 2, y: lineY - dotRise };
  return { x: geometry.x(place.at), y: lineY - dotRise };
}

// Draws the dots and returns the x of the leftmost one (null when none is drawn).
function drawRouteDots(placed, geometry, stops) {
  const layer = routeSvg.querySelector('.route-dots');
  const byKey = new Map();
  let leftmost = null;
  placed.forEach((item) => {
    const position = dotPosition(item.place, geometry, stops);
    const key = `${position.x},${position.y}`;
    if (!byKey.has(key)) byKey.set(key, []);
    byKey.get(key).push({ ...item, position });
  });
  const seen = new Set();
  byKey.forEach((group) => group.forEach((item, index) => {
    // Pinkys on the same spot sit side by side, centred over it.
    const x = item.position.x + (index - (group.length - 1) / 2) * routeLayout.dotSpread;
    leftmost = leftmost == null ? x : Math.min(leftmost, x);
    const id = item.pinky.pinky_id;
    seen.add(id);
    let dot = layer.querySelector(`[data-pinky="${CSS.escape(id)}"]`);
    if (!dot) {
      dot = document.createElementNS(routeSvgNamespace, 'g');
      dot.dataset.pinky = id;
      dot.innerHTML = `<path d="M -6 11 L 0 19 L 6 11 Z"/><circle r="15"/><text>${escapeHtml(pinkyNames[id] || item.pinky.name)}</text>`;
      dot.style.transform = `translate(${x}px, ${item.position.y}px)`;
      layer.append(dot);
    }
    dot.setAttribute('class', `route-dot${item.pinky.stop_pending ? ' stopping' : ''}`);
    dot.style.transform = `translate(${x}px, ${item.position.y}px)`;
  }));
  layer.querySelectorAll('[data-pinky]').forEach((dot) => { if (!seen.has(dot.dataset.pinky)) dot.remove(); });
  return leftmost;
}

function scrollToFirstDot(leftmost) {
  if (routeStrip.scrolledToDots || leftmost == null || !routeScroll.clientWidth
    || routeScroll.scrollWidth <= routeScroll.clientWidth) return;
  routeStrip.scrolledToDots = true;
  routeScroll.scrollLeft = Math.max(0, leftmost - 64);
}

function renderRouteStrip(view) {
  routeStrip.lastView = view;
  if (!routeStrip.catalogRequested) loadRouteFromCatalog();
  const stops = routeStops();
  const geometry = routeGeometry(stops);
  const trackKey = `${routeStrip.labels.join(',')}|${routeStrip.zoneLabel}|${geometry.gap}`;
  if (trackKey !== routeStrip.trackKey) {
    routeStrip.trackKey = trackKey;
    drawRouteTrack(stops, geometry);
  }
  const placed = [];
  const off = [];
  view.pinkies.forEach((pinky) => {
    const place = routePlace(pinky, stops);
    (place.off ? off : placed).push({ pinky, place });
  });
  scrollToFirstDot(drawRouteDots(placed, geometry, stops));
  const name = (pinky) => pinkyNames[pinky.pinky_id] || pinky.name;
  routeSvg.setAttribute('aria-label', placed.length
    ? `경로 노선도: ${placed.map(({ pinky, place }) => `${name(pinky)} ${placeName(stops, place)}`).join(', ')}`
    : '경로 노선도: 표시할 핑키가 없습니다');
  routeOffStrip.innerHTML = off.map(({ pinky, place }) => `
    <span class="chip${place.off === '연결 끊김' ? ' danger' : ''}">${escapeHtml(name(pinky))} ${escapeHtml(place.off)}${
      place.off === '연결 끊김' ? '' : ` (${escapeHtml(pinky.route?.current_label || pinky.phase_label || '정보 없음')})`}</span>`).join('');
}

window.addEventListener('resize', () => { if (routeStrip.lastView && !routeScroll.closest('[hidden]')) renderRouteStrip(routeStrip.lastView); });
