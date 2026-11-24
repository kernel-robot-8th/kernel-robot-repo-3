// 핑키 경로 노선도: 핑키 경로를 달리는 순서대로 정류장을 늘어놓은 띠로 그리고, 각 핑키가 알려 준 정류장에 점을 찍는다.
// 축척도 지도 좌표도 없다: 순서만 보여 주며, 화면에도 그렇게 적혀 있다.
//
// app.js(escapeHtml, pinkyNames) 뒤, pinky.js 앞에 로드된다. pinky.js가 GET /api/pinky 응답마다
// renderRouteStrip(view)를 부른다.
//
// 점을 어디에 둘지는 핑키 관제(fleet dashboard)가 쓰는 말 그대로 정한다:
// - 연결 안 됨: 그리지 않고 띠 옆에 적는다
// - 주차(parked)했거나 경로가 아예 없음: 주차 자리 P
// - docking_manual / docking_anchor / docked_manual: 5번 정류장(적재), dock_exit: 5번 바로 다음
// - course_complete: 수령 구역, 마지막 정류장
// - 그 밖에는 route.current_label: 정류장("7"), 두 정류장 사이 구간(촘촘한 경로의 "3→7:04"), "도킹 탈출",
//   또는 경로 끝의 "주차장"(집으로 가는 중). 두 번 나오는 정류장(7, 1)은 route.completed / route.total,
//   즉 촘촘한 경로 중 이미 달린 비율로 고른다.
// 그 밖의 것(모의 세계의 "이동 중")은 짐작하지 않고 띠 옆에 적는다.

// 카탈로그를 받기 전에 쓰는 경로: pinky_fleet_dashboard의 ROUTE_LABELS
const ROUTE_FALLBACK_LABELS = ['1', '2', '3', '7', '4', '5', '6', '7', '9', '8', '1'];
const ROUTE_PARKING = '주차장';
const ROUTE_DOCK_EXIT = '도킹 탈출';
const ROUTE_STATION_LABEL = '5';
const ROUTE_STATION_PHASES = new Set(['docking_manual', 'docking_anchor', 'docked_manual']);
// 촘촘한 경로의 구간 표기 '3→7:04'(앞 정류장→다음 정류장:번호)를 나눈다.
const ROUTE_LEG = /^(.+)→(.+):\d+$/;
// 노선도 배치(px): 양옆 여백, 정류장 최소 간격, 선 높이, 점 높이, 같은 자리 점 간격, 글자 높이, 돌아오는 길 높이, 전체 높이.
const routeLayout = { pad: 72, minGap: 58, lineY: 64, dotRise: 34, dotSpread: 32, labelY: 102, returnY: 134, height: 152 };

const routeScroll = document.querySelector('#routeScroll');
const routeSvg = document.querySelector('#routeStrip');
const routeOffStrip = document.querySelector('#routeOffStrip');
const routeSvgNamespace = 'http://www.w3.org/2000/svg';

const routeStrip = {
  labels: ROUTE_FALLBACK_LABELS,  // GET /api/catalog가 답하면 수령 구역의 경로로 바뀐다
  zoneLabel: '수령 구역',
  catalogRequested: false,
  trackKey: '',                    // 지금 그려 둔 선로가 만들어진 정류장·간격
  scrolledToDots: false,           // 휴대폰 폭에서 첫 핑키로 한 번만 스크롤하고 그다음엔 그대로 둔다
  lastView: null,
};

// 수령 구역 경로를 한 번만 읽는다(첫 번째 구역의 route_labels). 받으면 마지막 화면으로 다시 그린다.
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
    // 대체 경로가 그대로 남는다. 핑키 관제의 고정 코스다.
  }
  if (routeStrip.lastView) renderRouteStrip(routeStrip.lastView);
}

// P 다음에 경로의 정류장들. 마지막 정류장은 수령 구역이고, 처음 나오는 5는 적재 자리다.
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

// 맞는 자리가 여럿이면 이미 달린 경로 비율에 가장 가까운 것을 고른다.
function closestToProgress(candidates, route, stopCount) {
  if (candidates.length < 2 || !route.total) return candidates[0];
  const progress = (route.completed ?? 0) / route.total;
  const distance = (index) => Math.abs(index / stopCount - progress);
  return candidates.reduce((best, index) => (distance(index) < distance(best) ? index : best));
}

// { at: 정류장 번호 } | { between: 앞 정류장 번호 } | { returning: true } | { off: 그리지 않는 이유 }
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

// 정류장과 핑키 자리의 화면 이름(접근성 라벨용).
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

// ---- 그리기 ----

// 상자 폭에 맞춰 간격을 정하되 minGap보다 좁히지 않는다(좁은 화면에서는 옆으로 스크롤).
function routeGeometry(stops) {
  const { pad, minGap } = routeLayout;
  const room = routeScroll.clientWidth - 2 * pad;
  const gap = Math.max(minGap, Math.floor(room / (stops.length - 1)));
  return { gap, x: (index) => pad + index * gap, width: 2 * pad + (stops.length - 1) * gap };
}

// 정류장 하나의 SVG: 주차장 P, 적재 자리 5(양쪽 팔 표기), 수령 구역, 보통 정류장.
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

// 선로(선, 정류장, 글자, 점선으로 된 돌아오는 길)는 경로나 간격이 바뀔 때만 다시 만든다.
// 그래서 위에 얹힌 핑키 점은 요소가 그대로 남아, 움직일 때 미끄러지듯 옮겨 간다.
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

// 자리 → 점 좌표. 두 정류장 사이면 그 가운데에 둔다.
function dotPosition(place, geometry, stops) {
  const { lineY, dotRise, returnY } = routeLayout;
  // 집으로 가는 중: 점선으로 된 돌아오는 길 위, P 쪽으로, 글자와 돌아오는 길 안내문을 피해서 둔다.
  if (place.returning) return { x: geometry.x(0) + (geometry.x(stops.length - 1) - geometry.x(0)) * 0.22, y: returnY - 20 };
  if (place.between != null) return { x: (geometry.x(place.between) + geometry.x(place.between + 1)) / 2, y: lineY - dotRise };
  return { x: geometry.x(place.at), y: lineY - dotRise };
}

// 점을 그리고 가장 왼쪽 점의 x를 돌려준다(그린 점이 없으면 null).
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
    // 같은 자리의 핑키들은 그 위에 가운데 맞춰 나란히 놓는다.
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

// 휴대폰 폭에서 노선도가 넘칠 때 첫 핑키가 보이도록 한 번만 스크롤한다.
function scrollToFirstDot(leftmost) {
  if (routeStrip.scrolledToDots || leftmost == null || !routeScroll.clientWidth
    || routeScroll.scrollWidth <= routeScroll.clientWidth) return;
  routeStrip.scrolledToDots = true;
  routeScroll.scrollLeft = Math.max(0, leftmost - 64);
}

// pinky.js가 GET /api/pinky 응답마다 부른다. 선로는 필요할 때만 다시 만들고, 점을 옮기고, 못 그리는 핑키는 칩으로 적는다.
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

// 창 크기가 바뀌면 간격을 다시 계산한다(노선도가 숨어 있으면 건너뛴다).
window.addEventListener('resize', () => { if (routeStrip.lastView && !routeScroll.closest('[hidden]')) renderRouteStrip(routeStrip.lastView); });
