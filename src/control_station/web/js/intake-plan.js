// 접수 카드의 계획(관제 → 주문 접수): GET /api/orders/{id}/assignment가 정한 것을 목록 대신 그림으로 보여 준다.
//
// 한눈에: 요약 한 줄, 핑키마다 쟁반(4칸, 큰 물건은 2칸 차지)과 팔 적재 시간표. "자세히"를 열면 이유가 나온다:
// 순위를 가른 열이 표시된 핑키 비교, 수령 구역 후보, 그리고 물건마다 팔을 고른 규칙과 두 팔의 값을 나란히.
// app.js(먼저 로드됨)의 escapeHtml, pinkyBadge, formatClock을 쓴다. 바깥에 내놓는 것은 window.IntakePlan.html 하나이고,
// app.js의 previewHtml이 3초마다 미리보기를 다시 그릴 때 부른다. 즉시 실행 함수로 감싸 다른 전역과 섞이지 않게 한다.

(() => {
  const RULE_WORDS = {
    only_option: '이 팔만 가능',
    success_rate: '성공률 높은 팔',
    balance: '일이 적은 팔',
    stock: '재고 많은 선반',
    config: '설정 순서',
  };
  const PINKY_KEYS = [  // 순위를 매기는 순서(assignment._preference)
    { key: 'battery', label: '배터리', words: '배터리(10% 단위) 높은 순' },
    { key: 'trips', label: '오늘 운행', words: '오늘 운행 적은 순' },
    { key: 'rested', label: '마지막 완료', words: '오래 쉰 순' },
    { key: 'id', label: '번호', words: '번호 순' },
  ];
  const ZONE_KEYS = [
    { key: 'uses', label: '오늘 사용', words: '오늘 덜 쓴 곳' },
    { key: 'route', label: '경로', words: '경로가 짧은 곳' },
    { key: 'config', label: '설정 순서', words: '설정 순서' },
  ];

  const openDetails = new Set();  // "자세히"가 열린 주문 id. 카드를 다시 그려도 유지한다
  document.addEventListener('toggle', (event) => {
    const details = event.target;
    if (!(details instanceof HTMLDetailsElement) || !details.dataset.planDetail) return;
    if (details.open) openDetails.add(details.dataset.planDetail);
    else openDetails.delete(details.dataset.planDetail);
  }, true);  // toggle은 버블링되지 않지만 캡처 단계에서는 보인다

  // esc는 null/undefined를 빈 문자열로 바꾼 뒤 이스케이프하고, seconds는 초를 반올림해 'N초'로 쓴다.
  const esc = (value) => escapeHtml(value == null ? '' : String(value));
  const seconds = (value) => `${Math.round(Number(value) || 0)}초`;

  // ---- 어느 값이 순위를 갈랐나 --------------------------------------------------------------------------

  // 핑키 판정의 '배터리…' 점검 항목에서 배터리 값을 꺼낸다.
  function batteryOf(verdict) {
    const check = (verdict.checks || []).find((item) => String(item.rule).startsWith('배터리'));
    return check?.detail || null;  // "92%", 로봇이 알려 주지 않으면 null
  }

  // 1·2순위 핑키를 비교해 처음으로 값이 달라지는 기준(battery → trips → rested → id)을 돌려준다. 후보가 하나면 'only'.
  function pinkyDecider(verdicts) {
    const ranked = verdicts.filter((v) => v.eligible).sort((a, b) => a.rank - b.rank);
    if (ranked.length < 2) return ranked.length ? 'only' : null;
    const [first, second] = ranked.map((v) => v.keys || {});
    if (first.battery_band !== second.battery_band) return 'battery';
    if (first.trips_today !== second.trips_today) return 'trips';
    if ((first.last_finished_at || '') !== (second.last_finished_at || '')) return 'rested';
    return 'id';
  }

  // 수령 구역도 같은 방식: 오늘 사용 횟수 → 경로 길이 → 설정 순서.
  function zoneDecider(verdicts) {
    const ranked = verdicts.filter((v) => v.eligible).sort((a, b) => a.rank - b.rank);
    if (ranked.length < 2) return ranked.length ? 'only' : null;
    const [first, second] = ranked.map((v) => v.keys || {});
    if (first.uses_today !== second.uses_today) return 'uses';
    if (first.route_length !== second.route_length) return 'route';
    return 'config';
  }

  // ---- 한눈에 보기 ---------------------------------------------------------------------------------------

  function chip(text, kind = '') {
    return `<span class="plan-chip ${kind}">${esc(text)}</span>`;
  }

  // 요약 한 줄: 핑키(주문 때 정했으면 그것, 여러 운송이면 첫 핑키), 수령 구역, 팔 요약, 예상 적재 시간.
  function summaryHtml(order, plan) {
    const chips = [];
    const trips = plan.fleet?.trips || [];
    if (order.pinky_id) chips.push(`<span class="plan-chip">${pinkyBadge(order.pinky_id)} 주문할 때 정한 핑키</span>`);
    else if (trips.length > 1) {
      // 운송분은 하나씩 차례로 진행한다. 뒤 운송분은 시작할 때 핑키를 고른다(trips에 적힌 핑키는 추정이다).
      chips.push(chip(`운송 ${trips.length}번으로 나눠 차례로`, 'strong'));
      if (trips[0].pinky_id) chips.push(`<span class="plan-chip">${pinkyBadge(trips[0].pinky_id)} 먼저 출발</span>`);
    }
    else if (plan.pinky?.pinky_id) chips.push(`<span class="plan-chip strong">${pinkyBadge(plan.pinky.pinky_id)} 배정</span>`);
    else chips.push(chip('배정할 핑키 없음', 'bad'));
    if (plan.zone) chips.push(chip(plan.zone.zone_id ? plan.zone.summary.split(' · ')[0] : '수령 구역 모두 사용 중', plan.zone.zone_id ? '' : 'bad'));
    const first = trips[0];
    if (trips.length === 1 && first.arms && !first.arms.error) chips.push(chip(first.arms.summary));
    const total = trips.reduce((sum, trip) => sum + (trip.schedule?.total_s || 0), 0);
    if (total) chips.push(chip(`적재 약 ${seconds(total)}`));
    return `<p class="plan-summary">${chips.join('')}</p>`;
  }

  // 쟁반 그림: 물건 하나가 size칸을 차지하고 남은 칸은 점선으로 둔다.
  function trayHtml(trip, capacity) {
    const cells = [];
    let used = 0;
    trip.units.forEach((unit) => {
      const size = Math.max(1, Number(unit.size) || 1);
      used += size;
      cells.push(`<span class="tray-cell filled${size > 1 ? ' big' : ''}" style="grid-column: span ${size}"
        title="${esc(`${unit.label} ${unit.unit}/${unit.of} · ${size}칸`)}"><b>${esc(unit.label)}</b>${unit.of > 1 ? `<small>${esc(unit.unit)}/${esc(unit.of)}</small>` : ''}</span>`);
    });
    for (let slot = used; slot < capacity; slot += 1) cells.push('<span class="tray-cell empty" aria-hidden="true"></span>');
    const who = trip.pinky_id ? pinkyBadge(trip.pinky_id) : '<span class="plan-chip bad">빈 핑키 기다림</span>';
    return `<div class="tray-box">
        <div class="tray-head">${who}<span class="tray-load">쟁반 ${esc(trip.load)}/${esc(capacity)}칸</span></div>
        <div class="tray" style="grid-template-columns: repeat(${capacity}, minmax(0, 1fr))" role="img"
          aria-label="${esc(`쟁반 ${capacity}칸 중 ${trip.load}칸: ${trip.units.map((u) => u.label).join(', ')}`)}">${cells.join('')}</div>
      </div>`;
  }

  // 시간표의 줄(팔) 목록: 배정 후보 순서가 있으면 그것을, 없으면 시간표에 나온 팔을 쓴다.
  function armLanes(trip) {
    const options = trip.arms?.picks?.[0]?.options;
    if (options?.length) return options.map((option) => ({ id: option.arm_id, label: option.arm_label }));
    const seen = new Map();
    (trip.schedule?.picks || []).forEach((pick) => seen.set(pick.arm_id, pick.arm_label));
    return [...seen].map(([id, label]) => ({ id, label }));
  }

  // 눈금이 6개 이하가 되는 간격(초).
  function tickStep(total) {
    return [5, 10, 15, 30, 60, 120].find((step) => total / step <= 6) || 300;
  }

  // 팔별 적재 시간표: 선반에서 집기, 다른 팔이 핑키 위에 있어 기다림, 핑키 위에 놓기를 초 단위 막대로 놓는다.
  function timelineHtml(trip) {
    if (trip.arms?.error) {
      return `<div class="timeline-box"><p class="plan-blocked">${esc(trip.arms.error.message)}</p></div>`;
    }
    const schedule = trip.schedule;
    if (!schedule?.picks?.length) return '<div class="timeline-box"><p class="plan-muted">적재 시간표 없음</p></div>';
    const total = Math.max(schedule.total_s, 1);
    const at = (value) => `${((value / total) * 100).toFixed(2)}%`;
    const span = (from, to) => `left:${at(from)};width:${at(Math.max(0, to - from))}`;
    const lanes = armLanes(trip).map((arm) => {
      const bars = schedule.picks.filter((pick) => pick.arm_id === arm.id).map((pick) => {
        const shelfEnd = pick.place_start_s - (pick.waited_s || 0);
        const name = `${pick.label}${pick.of > 1 ? ` ${pick.unit}` : ''}`;
        return `<span class="tl-bar shelf" style="${span(pick.shelf_start_s, shelfEnd)}" title="${esc(`${name} 선반에서 집기 ${seconds(pick.shelf_start_s)}~${seconds(shelfEnd)}`)}">${esc(name)}</span>
          ${pick.waited_s > 0 ? `<span class="tl-bar wait" style="${span(shelfEnd, pick.place_start_s)}" title="${esc(`다른 팔이 핑키 위에 있어 기다림 ${seconds(pick.waited_s)}`)}"></span>` : ''}
          <span class="tl-bar place" style="${span(pick.place_start_s, pick.place_end_s)}" title="${esc(`${name} 핑키 위에 놓기 ${seconds(pick.place_start_s)}~${seconds(pick.place_end_s)}`)}"></span>`;
      }).join('');
      return `<div class="tl-lane"><span class="tl-arm">${esc(arm.label)}</span><div class="tl-track">${bars || '<span class="tl-idle">이번엔 쉼</span>'}</div></div>`;
    }).join('');
    const step = tickStep(total);
    const ticks = [];
    for (let value = 0; value <= total + 0.01; value += step) ticks.push(`<span style="left:${at(value)}">${value}초</span>`);
    const mode = schedule.overlap
      ? `두 팔 겹치기 · 한 팔씩이면 ${seconds(schedule.one_arm_at_a_time_s)}`
      : '한 팔이 끝나면 다음 팔 (정책이 집기→놓기→대기를 한 번에 함)';
    return `<div class="timeline-box">
        <div class="tl-head"><span>팔 적재 순서</span><b>${seconds(schedule.total_s)}</b></div>
        <div class="timeline">${lanes}<div class="tl-axis"><span class="tl-arm"></span><div class="tl-ticks">${ticks.join('')}</div></div></div>
        <p class="tl-legend"><i class="shelf"></i>선반에서 집기<i class="place"></i>핑키 위에 놓기${schedule.picks.some((p) => p.waited_s > 0) ? '<i class="wait"></i>기다림' : ''}<span>${esc(mode)}</span></p>
      </div>`;
  }

  // 운송분마다 쟁반과 시간표를 한 줄로. fleet.error면 나눌 수 없는 이유를 보여 준다.
  function tripsHtml(plan) {
    const fleet = plan.fleet;
    if (!fleet) return '';
    if (fleet.error) return `<p class="plan-blocked">${esc(fleet.error.message)}</p>`;
    const capacity = fleet.capacity || 4;
    return `<div class="plan-trips">${fleet.trips.map((trip, index) => `
        <div class="plan-trip">
          ${fleet.trips.length > 1 ? `<span class="trip-index">운송 ${index + 1}</span>` : ''}
          ${trayHtml(trip, capacity)}
          ${timelineHtml(trip)}
        </div>`).join('')}</div>`;
  }

  // ---- 자세히 -------------------------------------------------------------------------------------------

  // 제외 조건 점검 결과를 점(통과 초록, 실패 빨강)으로. 마우스를 올리면 규칙과 값이 보인다.
  function checksHtml(checks) {
    return `<span class="check-dots">${(checks || []).map((check) => `<i class="${check.ok ? 'ok' : 'no'}"
      title="${esc(`${check.rule}: ${check.ok ? '통과' : '안 됨'}${check.detail ? ` (${check.detail})` : ''}`)}"></i>`).join('')}</span>`;
  }

  // 핑키 후보 표: 결과·순위, 배터리·오늘 운행·마지막 완료, 조건. 순위를 가른 열 머리에 '결정'을 붙인다.
  function pinkyTableHtml(plan) {
    const verdicts = plan.pinky?.verdicts || [];
    if (!verdicts.length) return '';
    const trips = plan.fleet?.trips || [];
    const tripOf = new Map();  // 핑키 한 대가 운송분 여러 개를 맡을 수 있다(차례로 진행)
    trips.forEach((trip, index) => { if (trip.pinky_id) tripOf.set(trip.pinky_id, [...(tripOf.get(trip.pinky_id) || []), index + 1]); });
    const decider = pinkyDecider(verdicts);
    const head = (key, label) => `<th class="${decider === key ? 'decider' : ''}">${esc(label)}${decider === key ? '<em>결정</em>' : ''}</th>`;
    const rows = [...verdicts].sort((a, b) => (a.rank ?? 99) - (b.rank ?? 99)).map((verdict) => {
      const keys = verdict.keys || {};
      const battery = batteryOf(verdict);
      const chosen = trips.length > 1 ? tripOf.has(verdict.pinky_id) : verdict.pinky_id === plan.pinky.pinky_id;
      const result = !verdict.eligible ? `<span class="bad">${esc(verdict.reason)}</span>`
        : chosen ? `<b class="good">${trips.length > 1 ? `운송 ${tripOf.get(verdict.pinky_id).join('·')}` : '배정'}</b> · ${verdict.rank}순위`
          : `${verdict.rank}순위`;
      return `<tr class="${chosen ? 'chosen' : ''}${verdict.eligible ? '' : ' excluded'}">
          <td>${pinkyBadge(verdict.pinky_id)}</td><td>${result}</td>
          <td class="num">${battery ? esc(battery) : '모름'}${keys.battery_band != null ? `<small>${esc(keys.battery_band)}%대</small>` : ''}</td>
          <td class="num">${esc(keys.trips_today ?? 0)}회</td>
          <td class="num">${keys.last_finished_at ? esc(formatClock(keys.last_finished_at)) : '없음'}</td>
          <td>${checksHtml(verdict.checks)}</td></tr>`;
    }).join('');
    const rule = PINKY_KEYS.map((item) => item.words).join(' → ');
    const note = decider === 'only' ? '보낼 수 있는 핑키가 하나뿐이에요' : decider ? `<b>${esc(PINKY_KEYS.find((k) => k.key === decider).label)}</b>에서 순위가 갈렸어요` : '보낼 수 있는 핑키가 없어요';
    return `<section class="plan-section"><h5>핑키 후보</h5>
        <div class="plan-table-wrap"><table class="plan-table"><thead><tr><th>핑키</th><th>결과</th>${head('battery', '배터리')}${head('trips', '오늘 운행')}${head('rested', '마지막 완료')}<th>조건</th></tr></thead>
        <tbody>${rows}</tbody></table></div>
        <p class="plan-rule">제외 조건: 진행 중 주문 없음 · 관제 연결 · 주차장 대기 · 운행 준비 · 배터리 30% 이상. 순위: ${esc(rule)}. ${note}</p></section>`;
  }

  // 수령 구역 후보 표. 작업자가 지정한 구역이면 비었는지만 확인한 것이다.
  function zoneTableHtml(plan) {
    const zone = plan.zone;
    if (!zone?.verdicts?.length) return '';
    const decider = zoneDecider(zone.verdicts);
    const head = (key, label) => `<th class="${decider === key ? 'decider' : ''}">${esc(label)}${decider === key ? '<em>결정</em>' : ''}</th>`;
    const rows = [...zone.verdicts].sort((a, b) => (a.rank ?? 99) - (b.rank ?? 99)).map((verdict) => {
      const keys = verdict.keys || {};
      const chosen = verdict.zone_id === zone.zone_id;
      const result = !verdict.eligible ? `<span class="bad">${esc(verdict.reason)}</span>` : chosen ? `<b class="good">배정</b> · ${verdict.rank}순위` : `${verdict.rank}순위`;
      return `<tr class="${chosen ? 'chosen' : ''}${verdict.eligible ? '' : ' excluded'}"><td><b>${esc(verdict.label)}</b></td><td>${result}</td>
          <td class="num">${esc(keys.uses_today ?? 0)}회</td><td class="num">${esc(keys.route_length ?? '-')}곳</td><td class="num">${esc(keys.config_order ?? '-')}</td></tr>`;
    }).join('');
    const note = zone.operator_chosen ? '작업자가 정한 구역이라 비어 있는지만 확인했어요.'
      : `비어 있는 곳 중 ${ZONE_KEYS.map((item) => item.words).join(' → ')}.`;
    return `<section class="plan-section"><h5>수령 구역</h5>
        <div class="plan-table-wrap"><table class="plan-table"><thead><tr><th>구역</th><th>결과</th>${head('uses', '오늘 사용')}${head('route', '경로')}${head('config', '설정 순서')}</tr></thead>
        <tbody>${rows}</tbody></table></div><p class="plan-rule">${esc(note)}</p></section>`;
  }

  // 물건별 팔 고르기 표.
  function armDecisionsHtml(plan) {
    const trips = (plan.fleet?.trips || []).filter((trip) => trip.arms?.picks?.length);
    if (!trips.length) return '';
    // 팔이 집는 순서대로 물건 하나에 한 줄. 팔마다의 열은 그 물건을 배정할 때 규칙이 비교한 값이다
    // (성공률 · 이미 맡은 개수 · 그 선반에 남은 수).
    const blocks = trips.map((trip, index) => {
      const arms = armLanes(trip);
      const rows = trip.arms.picks.map((pick, position) => {
        const cells = arms.map((arm) => {
          const option = (pick.options || []).find((item) => item.arm_id === arm.id);
          if (!option) return '<td>-</td>';
          if (!option.possible) return `<td><span class="bad">${esc(option.why)}</span></td>`;
          const rate = option.success_rate == null ? '-' : `${Math.round(option.success_rate * 100)}%`;
          return `<td class="num${option.chosen ? ' picked' : ''}">${esc(rate)} · ${esc(option.assigned)}개 · 재고 ${esc(option.remaining)}</td>`;
        }).join('');
        return `<tr><td class="num">${position + 1}</td>
            <td><b>${esc(pick.label)}</b>${pick.of > 1 ? `<small>${esc(pick.unit)}/${esc(pick.of)}</small>` : ''}</td>
            <td><b class="good">${esc(pick.arm_label)}</b></td>
            <td>${chip(RULE_WORDS[pick.rule] || pick.rule || '규칙', 'rule')}<small>${esc(pick.reason)}</small></td>${cells}</tr>`;
      }).join('');
      return `${trips.length > 1 ? `<h6>운송 ${index + 1}${trip.pinky_name ? ` · ${esc(trip.pinky_name)}` : ''}</h6>` : ''}
        <div class="plan-table-wrap"><table class="plan-table arms"><thead><tr><th>순서</th><th>물건</th><th>고른 팔</th><th>정한 이유</th>
          ${arms.map((arm) => `<th>${esc(arm.label)}<small>성공률 · 맡은 수 · 재고</small></th>`).join('')}</tr></thead><tbody>${rows}</tbody></table></div>`;
    }).join('');
    return `<section class="plan-section"><h5>물건별 팔 고르기</h5>${blocks}
        <p class="plan-rule">선반에 있고 정책이 있는 팔만 후보. 하나뿐이면 그 팔, 둘이면 성공률 → 맡은 개수 → 남은 재고 → 설정 순서. 더 바쁜 팔이 먼저 시작해요.</p></section>`;
  }

  // '자세히'를 접힌 <details>로 만든다. 열림 상태는 openDetails에 기억해 다시 그려도 유지한다.
  function detailsHtml(order, plan) {
    const body = [pinkyTableHtml(plan), zoneTableHtml(plan), armDecisionsHtml(plan)].join('');
    if (!body) return '';
    return `<details class="plan-detail" data-plan-detail="${esc(order.id)}"${openDetails.has(order.id) ? ' open' : ''}>
        <summary>자세히 · 왜 이렇게 정했는지</summary>${body}</details>`;
  }

  // app.js의 previewHtml이 부르는 공개 함수: 접수 카드에 넣을 계획 HTML.
  window.IntakePlan = {
    html(order, plan) {
      return `<div class="plan">${summaryHtml(order, plan)}${tripsHtml(plan)}${detailsHtml(order, plan)}</div>`;
    },
  };
})();
