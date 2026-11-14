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
const commandParseForm = document.querySelector('#commandParseForm');
const commandParseMessage = document.querySelector('#commandParseMessage');
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
const navItems = [...document.querySelectorAll('.nav-item[data-view]')];
const views = [...document.querySelectorAll('.view[id^="view-"]')];
let selectedTaskId = null;
let parkingCameraId = null;
let controlMode = 'unknown';

const viewNames = { flow: '주문 진행', overview: '현황', manual: '수동 조작', tasks: '작업 이력', api: 'API' };
const controlModeLabels = {
  mock: '모의',
  ros2: 'ROS 2 상태 관측',
};

navItems.forEach((item) => {
  item.addEventListener('click', () => {
    const selectedView = item.dataset.view;
    views.forEach((view) => { view.hidden = view.id !== `view-${selectedView}`; });
    navItems.forEach((navItem) => {
      const active = navItem === item;
      navItem.classList.toggle('active', active);
      if (active) navItem.setAttribute('aria-current', 'page');
      else navItem.removeAttribute('aria-current');
    });
    viewTitle.textContent = viewNames[selectedView] || '현황';
  });
});

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (character) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
  })[character]);
}

function apiErrorMessage(result, fallback) {
  const message = result.message || result.error || fallback;
  return result.error_code ? `${result.error_code} · ${message}` : message;
}

const clockFormat = new Intl.DateTimeFormat('ko-KR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });

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

function renderCameras(cameras) {
  const names = { HEAD: 'Head Camera', JOINT: 'Joint Camera', FRONT: 'Front Camera', SHELF: '선반 카메라 (정면)' };
  const connectionLabel = {
    CONNECTED: '연결됨', DISCONNECTED: '연결 끊김', UNKNOWN: '확인 필요',
  };
  const streamLabel = {
    STREAMING: '수신 중', NO_FRAMES: '프레임 미수신', STOPPED: '중지',
    ERROR: '오류', UNKNOWN: '확인 필요',
  };
  const freshnessLabel = { FRESH: '최신', STALE: '지연', UNKNOWN: '확인 필요' };
  const cameraIds = cameras.map((camera) => camera.id).join(',');
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
    return option;
  }));
  if (cameraGrid.dataset.cameraIds !== cameraIds) {
    cameraGrid.dataset.cameraIds = cameraIds;
    cameraGrid.innerHTML = cameras.map((camera) => `
      <article class="camera-card" data-camera-id="${escapeHtml(camera.id)}">
        <header class="camera-head"><h3>${escapeHtml(names[camera.position] || camera.id)}</h3><span class="camera-position">${escapeHtml(camera.position)}</span></header>
        <div class="camera-preview" data-field="preview"><span>영상 스트림 미연결</span></div>
        <div class="camera-details">
          <div class="camera-detail"><span>연결</span><strong data-field="connection"></strong></div>
          <div class="camera-detail"><span>스트림</span><strong data-field="stream"></strong></div>
          <div class="camera-detail"><span>프레임 상태</span><strong data-field="freshness"></strong></div>
          <div class="camera-detail"><span>마지막 프레임</span><strong data-field="last-frame"></strong></div>
          <div class="camera-detail"><span>프레임 ID / 해상도</span><strong data-field="frame-info"></strong></div>
          <div class="camera-detail"><span>프레임률</span><strong data-field="frame-rate"></strong></div>
          <div class="camera-detail" data-field="detections-row" hidden><span>YOLO 검출</span><strong data-field="detections"></strong></div>
          <div class="camera-detail" data-field="slot-zones-row" hidden><span>선반 칸 · 표시 전용</span><strong data-field="slot-zones"></strong></div>
          <div class="camera-detail" data-field="detector-provenance-row" hidden><span>YOLO 설정 / 모델</span><strong data-field="detector-provenance"></strong></div>
        </div>
        <p class="note" data-field="error"></p>
      </article>`).join('');
  }
  cameras.forEach((camera) => {
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
    card.querySelector('[data-field="detections-row"]').hidden = !camera.detector_state;
    card.querySelector('[data-field="detections-row"] span').textContent = camera.detector_model_type === 'YOLO_WORLD_OPEN_VOCAB'
      ? 'YOLO-World 후보 · 관측 전용'
      : 'YOLO 검출 · 관측 전용';
    card.querySelector('[data-field="detections"]').textContent = camera.detector_state === 'ready'
      ? (camera.detections.map((item) => `${item.class_name} ${Math.round(item.confidence * 100)}% @ (${item.center_px.join(', ')})`).join(', ') || '없음')
      : (detectorLabel[camera.detector_state] || '');
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
    const preview = card.querySelector('[data-field="preview"]');
    if (camera.stream_url && !preview.querySelector('img')) {
      const image = document.createElement('img');
      image.src = camera.stream_url;
      image.alt = `${names[camera.position] || camera.id} 영상`;
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

const slotZoneStatusLabels = {
  READY: '보정 적용', INVALID: '보정 파일 오류', SLOT_MAP_MISMATCH: '칸 정의 버전 다름',
  SLOT_MAP_UNAVAILABLE: '칸 정의 없음', IMAGE_MISMATCH: '카메라 설정 다름',
};
const slotAssignmentLabels = { OUTSIDE: '칸 밖', AMBIGUOUS: '칸 경계·애매', NO_BBOX: 'bbox 없음' };
const svgNamespace = 'http://www.w3.org/2000/svg';

function renderSlotZones(card, preview, camera) {
  // Display only: zone outlines on the stream and the slot each detection's bottom-center falls in.
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
    // The filled shape is the anchor zone that decides the slot; the dashed outline is the whole slot space.
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

function renderArm(arm) {
  const labels = {
    DISABLED: '비활성화', STOPPED: '중지', STARTING: '시작 중',
    RUNNING: '프로세스 실행 중', STOPPING: '종료 중', ERROR: '오류',
  };
  armState.textContent = labels[arm.state] || arm.state;
  armDetail.textContent = arm.detail;
  armStart.disabled = !arm.enabled || arm.process_alive;
  armStop.disabled = !arm.process_alive;
}

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
}

async function sendArmAction(action) {
  armStart.disabled = true;
  armStop.disabled = true;
  try {
    const response = await fetch(`/api/arm/${action}`, { method: 'POST' });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '요청에 실패했습니다.');
    renderArm(result.arm);
  } catch (error) {
    armDetail.textContent = error.message;
  } finally {
    await refresh();
  }
}

armStart.addEventListener('click', () => sendArmAction('start'));
armStop.addEventListener('click', () => sendArmAction('stop'));

async function sendPolicyRunAction(action) {
  policyRunStart.disabled = true;
  policyRunStop.disabled = true;
  try {
    const profileQuery = action === 'start'
      ? `?profile_id=${encodeURIComponent(policyProfile.value)}`
      : '';
    const response = await fetch(`/api/arm/policy/${action}${profileQuery}`, { method: 'POST' });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '정책 실행 요청에 실패했습니다.');
    renderPolicyRun(result.policy_run);
  } catch (error) {
    policyRunDetail.textContent = error.message;
  } finally {
    await refresh();
  }
}

policyRunStart.addEventListener('click', () => sendPolicyRunAction('start'));
policyRunStop.addEventListener('click', () => sendPolicyRunAction('stop'));

const armCommandForm = document.querySelector('#armCommandForm');
const armCommandMessage = document.querySelector('#armCommandMessage');
const armCommandRows = document.querySelector('#armCommandRows');
const armWorkerState = document.querySelector('#armWorkerState');
const armCommandStateLabels = {
  QUEUED: '대기 (워커 미수신)', DISPATCHED: '워커 수신 · 수락 대기', ACCEPTED: '수락',
  RUNNING: '실행 중', SUCCEEDED: '성공', FAILED: '실패', REJECTED: '워커 거부',
  CANCELED: '취소됨', EXPIRED: '만료', UNKNOWN: '결과 불명 · 현장 확인 필요',
};

function formatClock(iso) {
  return iso ? clockFormat.format(new Date(iso)) : '-';
}

function renderArmWorkers(workers) {
  const worker = workers.find((item) => item.robot_id === 'arm-01');
  if (!worker) {
    armWorkerState.textContent = '워커 접속 기록 없음';
  } else if (worker.waiting_polls > 0) {
    armWorkerState.textContent = `워커 대기 중 · ${worker.worker_id}`;
  } else {
    const seconds = Math.round((Date.now() - new Date(worker.last_poll_at).getTime()) / 1000);
    armWorkerState.textContent = `워커 마지막 접속 ${seconds}초 전 · ${worker.worker_id}`;
  }
}

function renderArmCommands(commands) {
  if (!commands.length) {
    armCommandRows.innerHTML = '<tr><td colspan="5" class="loading">보낸 명령이 없습니다.</td></tr>';
    return;
  }
  armCommandRows.innerHTML = commands.map((command) => {
    const target = command.target;
    const camera = command.source?.kind === 'CAMERA'
      ? ` · 정면 캠 ${escapeHtml(command.source.class_name)} ${Math.round(command.source.confidence * 100)}%`
      : '';
    let where = target.policy ? '카메라 위치 근거 없음' : 'home 자세';
    if (target.pick_slot_id) {
      where = `칸 ${escapeHtml(target.slot_label || target.pick_slot_id)} (${escapeHtml(target.pick_slot_id)})${camera}`;
    }
    const what = { MOVE_TO_SLOT: '칸 앞으로 이동', RETURN_HOME: '원상복귀' }[command.operation]
      || (target.policy
        ? `학습 정책 집기: ${escapeHtml(target.policy.label)}<small>${escapeHtml(target.policy.model)}</small>`
        : `${escapeHtml(target.object_class)} → ${escapeHtml(command.place.slot_id)}`);
    const simulated = command.result && command.result.simulated ? '<span class="simulated">모의</span>' : '';
    const reason = command.reason_code ? `<small>${escapeHtml(command.reason_code)}</small>` : '';
    const cleared = command.operator_cleared_at ? `<small>현장 확인 ${escapeHtml(formatClock(command.operator_cleared_at))}</small>` : '';
    const inFlight = ['DISPATCHED', 'ACCEPTED', 'RUNNING'].includes(command.state);
    let action = '';
    if (command.state === 'QUEUED' || (inFlight && !command.cancel_requested)) {
      action = `<button type="button" class="secondary" data-arm-cancel="${escapeHtml(command.command_id)}">취소</button>`;
    } else if (inFlight) {
      action = '<small>취소 요청됨</small>';
    } else if (command.state === 'UNKNOWN' && !command.operator_cleared_at) {
      action = `<button type="button" class="secondary" data-arm-clear="${escapeHtml(command.command_id)}">현장 확인 완료</button>`;
    }
    return `<tr>
      <td>${escapeHtml(command.command_id)}<small>${escapeHtml(formatClock(command.created_at))}</small></td>
      <td>${escapeHtml(armCommandStateLabels[command.state] || command.state)}${simulated}${reason}${cleared}</td>
      <td>${what}<small>${where}</small></td>
      <td>${escapeHtml(command.worker_id || '-')}<small>${escapeHtml(formatClock(command.last_event_at || command.dispatched_at))}</small></td>
      <td>${action}</td>
    </tr>`;
  }).join('');
  armCommandRows.querySelectorAll('[data-arm-cancel]').forEach((button) => {
    button.addEventListener('click', () => postArmCommand(`/api/arm/commands/${button.dataset.armCancel}/cancel`, {}));
  });
  armCommandRows.querySelectorAll('[data-arm-clear]').forEach((button) => {
    button.addEventListener('click', () => {
      const note = window.prompt('팔·물체 상태를 현장에서 확인한 내용을 적어 주세요. 결과는 UNKNOWN으로 남고, 새 명령만 다시 보낼 수 있게 됩니다.');
      if (note && note.trim()) postArmCommand(`/api/arm/commands/${button.dataset.armClear}/clear-unknown`, { note: note.trim() });
    });
  });
}

async function refreshArmCommands() {
  try {
    const response = await fetch('/api/arm/commands', { cache: 'no-store' });
    if (!response.ok) throw new Error();
    const data = await response.json();
    renderArmWorkers(data.workers);
    renderArmCommands(data.commands);
  } catch {
    armWorkerState.textContent = '명령 목록을 불러오지 못했습니다';
  }
}

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

const cameraCommandForm = document.querySelector('#cameraCommandForm');
const policyButtons = document.querySelector('#policyButtons');

cameraCommandForm.addEventListener('submit', async (event) => {
  // The server picks the slot from the front camera; it refuses unless the object is in exactly one slot.
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

// Front-camera refusals after which the operator may still run the policy; the policy finds the object itself.
const cameraCheckErrors = new Set([
  'CAMERA_NOT_FOUND', 'CAMERA_NOT_FRESH', 'DETECTOR_NOT_READY', 'SLOT_ZONES_NOT_READY',
  'TARGET_NOT_FOUND', 'TARGET_AMBIGUOUS', 'TARGET_MULTIPLE',
]);

async function pickWithPolicy(policy) {
  // 1) Ask the server to confirm the object in the front camera (the slot becomes evidence in the command).
  // 2) If the camera cannot confirm it, the operator may send the command without camera evidence.
  const buttons = policyButtons.querySelectorAll('button');
  buttons.forEach((button) => { button.disabled = true; });
  armCommandMessage.textContent = `${policy.label}: 정면 캠에서 확인 중`;
  try {
    const response = await fetch('/api/arm/commands/from-camera', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ object_class: policy.object_class, operation: 'PICK_WITH_POLICY' }),
    });
    const result = await response.json();
    if (response.ok) {
      const found = result.selection;
      armCommandMessage.textContent = `${policy.label} 집기 등록 (${result.command.command_id}) · 정면 캠 ${Math.round(found.confidence * 100)}% @ ${found.slot_label} · 워커가 가져갈 때까지 대기`;
      return;
    }
    const reason = apiErrorMessage(result, '정면 캠 확인 실패');
    if (!cameraCheckErrors.has(result.error_code)
      || !window.confirm(`${reason}\n\n정면 캠 확인 없이 ${policy.label} 집기 정책을 실행할까요? 팔이 움직입니다.`)) {
      armCommandMessage.textContent = reason;
      return;
    }
    const command = await postArmCommand('/api/arm/commands', {
      robot_id: 'arm-01',
      operation: 'PICK_WITH_POLICY',
      valid_for_s: 60,
      target: { object_class: policy.object_class },
      place: { slot_id: 'CART_1' },
      source: { kind: 'OPERATOR_INPUT', note: `정면 캠 확인 없이 실행 (${result.error_code})` },
    });
    if (command) armCommandMessage.textContent = `${policy.label} 집기 등록 (${command.command_id}) · 정면 캠 확인 없음 · 워커가 가져갈 때까지 대기`;
  } catch (error) {
    armCommandMessage.textContent = error.message;
  } finally {
    buttons.forEach((button) => { button.disabled = false; });
    await refreshArmCommands();
  }
}

// One request for the trained-policy list, shared by the policy buttons and the order form.
const policyListRequest = fetch('/api/arm/policies', { cache: 'no-store' }).then(async (response) => {
  const result = await response.json();
  if (!response.ok) throw new Error(apiErrorMessage(result, '정책 목록을 불러오지 못했습니다'));
  return result.policies;
});

async function loadPolicyButtons() {
  try {
    const policies = await policyListRequest;
    policyButtons.replaceChildren(...policies.map((policy) => {
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = `${policy.label} 집기`;
      button.title = `${policy.model} (${policy.object_class})`;
      button.addEventListener('click', () => pickWithPolicy(policy));
      return button;
    }));
  } catch (error) {
    policyButtons.innerHTML = `<span class="note">${escapeHtml(error.message)}</span>`;
  }
}
loadPolicyButtons();

// ---- 주문 (여러 물체·수량, 핑키 배정) ----
const orderForm = document.querySelector('#orderForm');
const orderItems = document.querySelector('#orderItems');
const orderMessage = document.querySelector('#orderMessage');
const pinkyNames = { 10: 'P0', 11: 'P1', 12: 'P2' };
let orderPolicies = [];

function objectLabel(objectClass) {
  return orderPolicies.find((policy) => policy.object_class === objectClass)?.label || objectClass;
}

function orderSummary(items) {
  return (items || []).map((item) => `${objectLabel(item.object_class)} ${item.quantity}`).join(' · ');
}

function pinkyBadge(pinkyId) {
  return pinkyId ? `<span class="pinky-badge pinky-${escapeHtml(pinkyId)}">${escapeHtml(pinkyNames[pinkyId] || pinkyId)}</span>` : '';
}

function addOrderRow(objectClass = '', quantity = 1) {
  const row = document.createElement('div');
  row.className = 'order-item';
  const objects = [...new Map(orderPolicies.map((policy) => [policy.object_class, policy.label])).entries()];
  row.innerHTML = `
    <label>물체<select name="object_class" required>${objects.map(([value, label]) => `<option value="${escapeHtml(value)}">${escapeHtml(label)} (${escapeHtml(value)})</option>`).join('')}</select></label>
    <label>수량<input name="quantity" type="number" min="1" max="5" step="1" required value="${Number(quantity) || 1}"></label>
    <button type="button" class="secondary" data-remove-order-item>삭제</button>`;
  if (objectClass) row.querySelector('select').value = objectClass;
  row.querySelector('[data-remove-order-item]').addEventListener('click', () => {
    row.remove();
    if (!orderItems.children.length) addOrderRow();
  });
  orderItems.append(row);
}

function setOrderRows(items) {
  orderItems.replaceChildren();
  (items.length ? items : [{ object_class: '', quantity: 1 }]).forEach((item) => addOrderRow(item.object_class, item.quantity));
}

async function loadOrderPolicies() {
  try {
    orderPolicies = await policyListRequest;
    setOrderRows([]);
  } catch (error) {
    orderMessage.textContent = `주문할 물체 목록을 불러오지 못했습니다: ${error.message}`;
  }
}
loadOrderPolicies();

document.querySelector('#orderAddItem').addEventListener('click', () => addOrderRow());

orderForm.addEventListener('submit', async (event) => {
  event.preventDefault();
  const button = orderForm.querySelector('button[type="submit"]');
  const items = [...orderItems.querySelectorAll('.order-item')].map((row) => ({
    object_class: row.querySelector('select').value,
    quantity: Number(row.querySelector('input').value),
  }));
  const pinkyId = orderForm.elements.pinky_id.value;
  button.disabled = true;
  try {
    const response = await fetch('/api/tasks', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ order_items: items, ...(pinkyId ? { pinky_id: pinkyId } : {}) }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, '주문을 등록하지 못했습니다.'));
    selectedTaskId = result.task.id;
    const startNow = orderForm.elements.start_now.checked;
    setOrderRows([]);
    orderForm.elements.pinky_id.value = '';
    orderMessage.textContent = `${result.task.id} 등록 · ${orderSummary(result.task.order_items)} · 핑키 ${pinkyNames[result.task.pinky_id] || result.task.pinky_id} 배정.`;
    if (startNow) await orderControl(result.task.id, 'start');
    await Promise.all([refreshTasks(), refreshFlow()]);
  } catch (error) {
    orderMessage.textContent = error.message;
  } finally {
    button.disabled = false;
  }
});

// The worker replays the joint pose it taught for the chosen slot; no coordinates are sent.
const armCommandSlot = document.querySelector('#armCommandSlot');

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

armCommandForm.addEventListener('submit', async (event) => {
  event.preventDefault();
  const button = armCommandForm.querySelector('button');
  const formData = new FormData(armCommandForm);
  button.disabled = true;
  const command = await postArmCommand('/api/arm/commands', {
    robot_id: 'arm-01',
    operation: 'MOVE_TO_SLOT',
    valid_for_s: Number(formData.get('valid_for_s')),
    target: { object_class: String(formData.get('object_class') || '').trim() || null, pick_slot_id: armCommandSlot.value },
    place: { slot_id: formData.get('slot_id') },
    source: { kind: 'OPERATOR_INPUT' },
  });
  if (command) armCommandMessage.textContent = `${command.command_id} 등록 · 워커가 가져갈 때까지 대기`;
  button.disabled = false;
});

document.querySelector('#armReturnHome').addEventListener('click', async (event) => {
  // No slot or place: the worker moves the arm back to the home pose it recorded.
  const button = event.currentTarget;
  button.disabled = true;
  const command = await postArmCommand('/api/arm/commands', {
    robot_id: 'arm-01',
    operation: 'RETURN_HOME',
    valid_for_s: Number(armCommandForm.elements.valid_for_s.value),
    source: { kind: 'OPERATOR_INPUT' },
  });
  if (command) armCommandMessage.textContent = `${command.command_id} 원상복귀 등록 · 워커가 가져갈 때까지 대기`;
  button.disabled = false;
});

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
const stepOutcomeLabels = {
  SIMULATED_SUCCESS: '모의 완료',
  CANCEL_CONFIRMED: '취소 확인',
  NOT_EXECUTED_AFTER_CANCEL: '취소 뒤 미실행',
  NOT_EXECUTED_AFTER_FAILURE: '실패 뒤 미실행',
  PARKING_ARRIVED: '주차 판정 도착',
  LOAD_CONFIRMED: '적재 수량 확인',
  ARM_HOME_PLACEMENT_NOT_VERIFIED: '팔 home 복귀 · 적재 미확인',
  ARM_HOME_PLACEMENT_LEFT_TO_VERIFY_LOAD: '적재 판단은 적재 확인 단계로',
};
const taskEventLabels = {
  MOCK_SCENARIO_FAILURE: '모의 실패 시나리오 발생',
  MOCK_EXECUTION_FAILED: '모의 실행 오류',
  MOCK_TASK_COMPLETED: '모의 작업 완료',
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

function formatLoadObservation(payload) {
  const observation = payload.observation || {};
  const observed = observation.observed_count == null ? '개수 없음' : `${observation.observed_count}개`;
  return `적재 확인 · ${loadStatusLabels[payload.status] || payload.status} · 기대 ${payload.expected_count}개 · 관측 ${observed}${observation.simulated ? ' · 모의 관측(카메라 미사용)' : ''}`;
}
const parkingStatusLabels = {
  ARRIVED: '도착',
  MISALIGNED: '어긋남',
  NOT_FOUND: '확인 불가(마커 안 보임)',
  AMBIGUOUS: '확인 불가(같은 마커 여러 개)',
  VIEW_UNVERIFIED: '확인 불가(카메라 시야 미확인)',
};
const parkingDeviationLabels = { WITHIN: '범위 안', NEAR: '조금 벗어남', FAR: '많이 벗어남' };

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

let parkingBusy = false;
let parkingLiveBusy = false;
let parkingLive = false;
let parkingStreamCamera = null;
let parkingLiveOk = false;  // the last /live poll succeeded, so the panel shows live state
const parkingColors = { within: '#2f8f4e', near: '#c98a1a', far: '#bd4242', moving: '#3478cb', unknown: '#7a838d' };
const parkingOverlayColors = { within: '#3fbf6a', near: '#f0a830', far: '#f05a5a', moving: '#5aa2f0', unknown: '#c3c9cf' };
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

function parkingLevel(data) {
  if (!data?.offset) return 'unknown';
  return { WITHIN: 'within', NEAR: 'near', FAR: 'far' }[data.deviation_level] || 'unknown';
}

function parkingAxes(offset) {
  const pinkyAxes = offset.forward_mm != null;
  return {
    pinkyAxes,
    forward: Number(pinkyAxes ? offset.forward_mm : offset.dx_mm),
    left: Number(pinkyAxes ? offset.left_mm : offset.dy_mm),
  };
}

// Small top view in Pinky's own axes: up = Pinky front, left = Pinky left, counterclockwise yaw turns the arrow left.
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

function markerGeometry(corners, forwardAxis) {
  const point = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
  const center = [0, 1].map((i) => corners.reduce((sum, corner) => sum + corner[i], 0) / 4);
  const rightMid = point(corners[1], corners[2], 0.5);
  const topMid = point(corners[0], corners[1], 0.5);
  const axisEnd = { '+x': rightMid, '-x': point(rightMid, center, 2), '+y': topMid, '-y': point(topMid, center, 2) }[forwardAxis || '+y'];
  return { center, front: point(center, axisEnd, 2.4) };
}

function drawParkingOverlay(data, level, label) {
  const overlay = data?.image_overlay;
  if (!overlay) {
    parkingOverlay.innerHTML = '';
    return;
  }
  const { width, height } = overlay.image_size_px;
  parkingOverlay.setAttribute('viewBox', `0 0 ${width} ${height}`);
  // Taken from the overlay so the registered heading is drawn right even when nothing was judged.
  const forwardAxis = overlay.marker_forward_axis ?? data.offset?.marker_forward_axis;
  const color = parkingOverlayColors[level] || parkingOverlayColors.unknown;
  const polygon = (corners) => corners.map(([x, y]) => `${x},${y}`).join(' ');
  // The judgment subtracts the camera drift measured on the fixed markers, so draw the registered spot moved by it too.
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

function showParkingStream(cameraId) {
  if (!cameraId || cameraId === parkingStreamCamera) return;
  parkingStreamCamera = cameraId;
  parkingStream.hidden = false;
  parkingStreamEmpty.hidden = true;
  parkingStream.src = `/streams/${encodeURIComponent(cameraId)}.mjpg`;
}

function stopParkingStream() {
  if (!parkingStreamCamera) return;
  parkingStream.removeAttribute('src');
  parkingStream.hidden = true;
  parkingStreamEmpty.hidden = false;
  parkingStreamCamera = null;
}

parkingStream.addEventListener('error', () => {
  parkingStream.hidden = true;
  parkingStreamEmpty.hidden = false;
  parkingStreamCamera = null;
});

function gaugeHtml(label, value, tolerance, ratio, unit) {
  const position = Math.min(Number(ratio), 3) / 3 * 100;
  return `<div class="parking-gauge"><div class="gauge-head"><span>${escapeHtml(label)}</span><strong>${escapeHtml(value)} / 허용 ${escapeHtml(tolerance)}${unit} · ${Number(ratio).toFixed(2)}배</strong></div>
    <div class="gauge-track"><span class="gauge-needle" style="left:${position}%"></span></div>
    <div class="gauge-scale"><span>0</span><span>허용</span><span>2배</span><span>3배+</span></div></div>`;
}

function correctionAdvice(offset) {
  const { pinkyAxes, forward, left } = parkingAxes(offset);
  const steps = [];
  if (Math.abs(forward) >= 1) steps.push(`${pinkyAxes ? (forward > 0 ? '뒤로' : '앞으로') : (forward > 0 ? '마커 -x로' : '마커 +x로')} ${Math.abs(forward).toFixed(0)} mm`);
  if (Math.abs(left) >= 1) steps.push(`${pinkyAxes ? (left > 0 ? '오른쪽으로' : '왼쪽으로') : (left > 0 ? '마커 -y로' : '마커 +y로')} ${Math.abs(left).toFixed(0)} mm`);
  if (Math.abs(Number(offset.dyaw_deg)) >= 0.5) steps.push(`${Number(offset.dyaw_deg) > 0 ? '시계 방향' : '반시계 방향'}으로 ${Math.abs(Number(offset.dyaw_deg)).toFixed(1)}°`);
  return steps.length ? `등록 위치로 맞추려면 ${steps.join(' · ')} 옮겨야 합니다.` : '';
}

// Numbers, gauges, top view, camera overlay and details shared by live states and confirmed checks.
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
}

function setParkingAdvice(text) {
  parkingAdvice.hidden = !text;
  parkingAdvice.textContent = text || '';
}

function setParkingSteps(state) {
  const order = { NOT_IN_VIEW: 0, MOVING: 1, IN_POSITION: 2, OFF_POSITION: 2 };
  const current = order[state];
  [...parkingSteps.children].forEach((step, index) => {
    step.classList.toggle('current', current === index);
    step.classList.toggle('done', current !== undefined && index < current);
  });
}

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

function confirmLine(observation) {
  const title = observation.status === 'MISALIGNED'
    ? `어긋남(${parkingDeviationLabels[observation.deviation_level] || observation.deviation_level})`
    : parkingStatusLabels[observation.status] || observation.status;
  const distance = observation.offset ? ` · 거리 ${Number(observation.offset.distance_mm).toFixed(1)} mm / 허용 ${observation.tolerance.position_mm} mm · 회전 ${signedNumber(observation.offset.dyaw_deg, 1)}°` : ` · ${observation.reason}`;
  return `확정 판정 ${clockFormat.format(new Date(observation.captured_until))} · ${title}${distance} · ${observation.frames_used}프레임`;
}

// Without live tracking (older server or no configured spot) the confirmed check fills the whole panel.
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

// Clears every number and drawing so an earlier "정상 위치" never stays on screen after the data is gone.
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
}

function renderParkingError(message) {
  parkingConfirm.hidden = false;
  parkingConfirm.className = 'parking-confirm';
  parkingConfirm.textContent = `확정 판정 실패 · ${message}`;
  if (!(parkingLive && parkingLiveOk)) clearParkingPanel('판정하지 못함', message);
}

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

async function pollParkingLive() {
  const spotId = parkingSpotInput.value.trim();
  if (document.hidden || document.querySelector('#view-overview').hidden) {
    stopParkingStream();  // the camera card keeps its own stream; this one only matters while the panel is visible
    return;
  }
  if (!parkingLive || parkingLiveBusy || !spotId) return;
  parkingLiveBusy = true;
  try {
    const response = await fetch(`/api/parking-spots/${encodeURIComponent(spotId)}/live`, { cache: 'no-store' });
    const result = await response.json();
    parkingLiveOk = response.ok;
    if (response.ok) renderParkingLive(result.live);
    else clearParkingPanel('실시간 추적 없음', apiErrorMessage(result, '이 주차 위치는 실시간 추적 중이 아닙니다.'));
  } catch {
    parkingLiveOk = false;
    clearParkingPanel('실시간 상태 없음', '관제 서버에서 실시간 상태를 가져오지 못했습니다.');
  } finally {
    parkingLiveBusy = false;
  }
}

function updateParkingConfig(parking) {
  parkingCameraId = parking?.camera_id || null;
  parkingLive = Boolean(parking?.live_tracking);
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
    ? '모의 모드 · 단계 성공은 진행 시뮬레이션이며 실제 인식/로봇 명령을 의미하지 않습니다.'
    : `현재 ${escapeHtml(modeLabel)} 모드 · 작업 실행은 모의 모드에서만 제공됩니다.`;
  const ownershipNote = task.local_execution_owner === 'NO_LOCAL_WORKER'
    ? '<p class="task-error">저장된 작업은 활성 상태지만 이 서버 프로세스에 실행 worker가 없습니다. 자동 재개나 명령 재전송은 하지 않습니다. 작업 상태를 운영자가 확인해야 합니다.</p>'
    : task.local_execution_owner === 'MODE_NOT_MANAGED'
      ? '<p class="task-error">저장된 작업은 활성 상태지만 현재 모드에는 이 작업을 관리하는 실행 worker가 연결되어 있지 않습니다. 자동 재개나 명령 재전송은 하지 않습니다.</p>'
      : '';
  const action = canStart
    ? '<button type="button" data-task-action="start">모의 작업 시작</button>'
    : canCancel
      ? '<button type="button" class="secondary" data-task-action="cancel">모의 작업 취소</button>'
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
      return `<li><span>${escapeHtml(stepLabel)}${step.robot_id ? `<small class="step-robot">${robotLabel ? `${robotLabel} · ` : ''}${escapeHtml(step.robot_id)}</small>` : ''}</span><strong>${escapeHtml(executionLabels[step.status] || step.status)}${step.result?.outcome && stepOutcomeLabels[step.result.outcome] ? ` · ${stepOutcomeLabels[step.result.outcome]}` : ''}${step.result?.source === 'mock' ? ' · 모의' : ''}</strong></li>`;
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
            ? `${event.payload?.source === 'mock' ? '모의 ' : ''}진행률 ${Number(event.payload?.progress) || 0}% · ${event.payload?.message || '진행 이벤트'}`
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

async function sendTaskAction(taskId, action) {
  taskMessage.textContent = action === 'start' ? '모의 작업을 시작합니다…' : '모의 작업 취소를 요청합니다…';
  try {
    const response = await fetch(`/api/tasks/${encodeURIComponent(taskId)}/${action}`, { method: 'POST' });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, '작업 요청에 실패했습니다.'));
    taskMessage.textContent = `${result.execution_mode || 'mock'} · ${taskStatusLabels[result.task.status] || result.task.status}`;
  } catch (error) {
    taskMessage.textContent = error.message;
  } finally {
    await refreshTasks();
  }
}

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

commandParseForm.addEventListener('submit', async (event) => {
  event.preventDefault();
  const text = String(new FormData(commandParseForm).get('text') || '').trim();
  const button = commandParseForm.querySelector('button');
  button.disabled = true;
  commandParseMessage.textContent = '명령 초안을 해석하는 중…';
  try {
    const response = await fetch('/api/tasks/parse-command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ text }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '명령 초안을 해석하지 못했습니다.');
    const draft = result.draft;
    const parsed = draft.parsed;
    const order = result.order;
    if (order?.order_items?.length) setOrderRows(order.order_items);
    taskForm.elements.target_category.value = parsed.pick_request?.category || '';
    taskForm.elements.target_color.value = parsed.pick_request?.color || '';
    taskForm.elements.shelf_level.value = parsed.pick_request?.shelf_level || '';
    taskForm.elements.destination.value = parsed.destination || '';
    taskForm.elements.arm_job.value = parsed.arm_job || '';
    commandParseMessage.textContent = order?.order_items?.length
      ? `주문 초안: ${orderSummary(order.order_items)}${order.warnings.length ? ` · ${order.warnings.join(' · ')}` : ''} · 확인한 뒤 주문 등록을 누르세요.`
      : `주문 물체를 찾지 못했습니다${order?.warnings?.length ? ` · ${order.warnings.join(' · ')}` : ''}.`;
  } catch (error) {
    commandParseMessage.textContent = error.message;
  } finally {
    button.disabled = false;
  }
});

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
    serverStatus.innerHTML = `<span class="dot"></span> API 정상 · ${escapeHtml(controlModeLabels[health.mode] || health.mode)} 모드`;
    renderRobots(robotData.robots);
    renderCameras(cameraData.cameras);
    renderArm(armData.arm);
    renderPolicyRun(armData.policy_run);
    updatedAt.textContent = `갱신 ${clockFormat.format(new Date())}`;
  } catch {
    serverStatus.className = 'server-status error';
    serverStatus.innerHTML = '<span class="dot"></span> 서버 연결 실패';
    robotRows.innerHTML = '<tr><td colspan="3" class="error-row">서버 API에 연결할 수 없습니다.</td></tr>';
    cameraGrid.innerHTML = '<p class="error-row">서버 API에 연결할 수 없습니다.</p>';
    cameraGrid.dataset.cameraIds = '';
  }
}

// ---- 주문 진행 화면 ----
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
const mockFaultLabels = {
  STOCK_SHORT: '선반 재고 없음', PICK_FAILED: '집기 실패(선반에 남음)', ITEM_LOST: '분실(어디에도 없음)',
  EXTRA_ON_PINKY: '핑키에 주문 외 물건', PINKY_MISALIGNED: '핑키 위치 이상', FLEET_REJECT: '핑키 신호 거부',
};
const shelfNames = { shelf_a: '왼쪽 선반', shelf_b: '오른쪽 선반' };

function elapsed(fromIso, toIso) {
  if (!fromIso) return '';
  const seconds = Math.max(0, Math.round(((toIso ? new Date(toIso) : new Date()) - new Date(fromIso)) / 1000));
  return `${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
}

function renderFlowAlerts(alerts) {
  flowAlerts.innerHTML = alerts.map((alert) => `
    <div class="alert-card ${alert.level}" role="alert">
      <span class="alert-icon" aria-hidden="true">${alert.level === 'error' ? '⛔' : '⚠️'}</span>
      <div><strong>${escapeHtml(alert.title || '확인 필요')}</strong>
        <div>${escapeHtml(alert.message || '')}</div>
        <small>${alert.pinky_name ? `핑키 ${escapeHtml(alert.pinky_name)} · ` : ''}${escapeHtml(alert.task_id)}${alert.code ? ` · ${escapeHtml(alert.code)}` : ''}</small></div>
      ${alert.acknowledgeable ? `<button type="button" class="secondary" data-ack-alert="${escapeHtml(alert.task_id)}">확인</button>` : '<span></span>'}
    </div>`).join('');
  flowAlerts.querySelectorAll('[data-ack-alert]').forEach((button) => {
    button.addEventListener('click', async () => {
      button.disabled = true;
      await fetch(`/api/flow/alerts/${encodeURIComponent(button.dataset.ackAlert)}/ack`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ note: '화면에서 확인' }),
      });
      await refreshFlow();
    });
  });
}

function renderFlowRobots(view) {
  const pinkies = view.pinkies.map((pinky) => `
    <article class="robot-card pinky-${escapeHtml(pinky.pinky_id)}${pinky.reachable ? '' : ' unreachable'}${pinky.stopped_order_id ? ' stopped' : ''}">
      <h3><span>핑키 ${escapeHtml(pinky.name)}</span><span>${pinky.order_id && pinky.order_id === view.station_owner ? '<span class="spot-tag">적재 자리</span> ' : ''}${pinky.simulated ? '<span class="sim-tag">모의</span>' : ''}</span></h3>
      <div class="robot-state">${escapeHtml(pinky.reachable ? pinky.phase_label : '연결 안 됨')}</div>
      <p>${escapeHtml(pinky.message || '')}</p>
      <p>${pinky.order_id ? `주문 ${escapeHtml(pinky.order_id)}`
        : pinky.stopped_order_id ? `<strong class="stopped-note">멈춘 주문 ${escapeHtml(pinky.stopped_order_id)} · 현장 확인 필요</strong>` : '배정된 진행 주문 없음'}</p>
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

function planChips(order) {
  if (!order.plan) return '';
  return `<div class="plan-chips">${order.plan.map((unit, index) => {
    const state = index < order.loaded ? 'done'
      : index === order.loaded && order.status === 'FAILED' && order.stages.find((stage) => stage.key === 'LOAD').state === 'failed' ? 'failed'
        : index === order.loaded && order.active_step && ['PICK_ITEM', 'VERIFY_TRANSFER'].includes(order.active_step) ? 'doing' : '';
    return `<span class="plan-chip ${state}">${escapeHtml(unit.arm_label)} · ${escapeHtml(unit.label)} #${unit.unit}</span>`;
  }).join('<span aria-hidden="true">→</span>')}</div>`;
}

function renderFlowOrders(orders) {
  if (!orders.length) {
    flowOrders.innerHTML = '<p class="loading">아직 주문이 없습니다. 위에서 주문을 등록하세요.</p>';
    return;
  }
  flowOrders.innerHTML = orders.map((order) => `
    <article class="order-card pinky-${escapeHtml(order.pinky_id || '')}${order.status === 'FAILED' ? ' failed' : ''}">
      <div class="order-head">
        ${pinkyBadge(order.pinky_id)} <strong>${escapeHtml(orderSummary(order.order_items))}</strong>
        <span class="status-chip ${escapeHtml(order.status)}">${escapeHtml(orderStatusLabels[order.status] || order.status)}</span>
        ${order.simulated ? '<span class="sim-tag">모의</span>' : ''}
        <span class="order-meta">${escapeHtml(order.id)}${order.started_at ? ` · ${elapsed(order.started_at, order.finished_at)}` : ''}</span>
      </div>
      <ol class="pipeline">${order.stages.map((stage) => `<li class="${stage.state}">${escapeHtml(stage.label)}</li>`).join('')}</ol>
      <p class="order-now"><b>지금</b> ${escapeHtml(order.current)}</p>
      ${planChips(order)}
      <div class="order-actions-row">
        ${order.status === 'PENDING' ? `<button type="button" data-order-action="start" data-order-id="${escapeHtml(order.id)}">주문 시작</button>
          <button type="button" class="secondary" data-order-action="cancel" data-order-id="${escapeHtml(order.id)}">주문 취소</button>` : ''}
        ${['RUNNING', 'VALIDATING'].includes(order.status) ? `<button type="button" class="secondary" data-order-action="cancel" data-order-id="${escapeHtml(order.id)}">멈추기</button>` : ''}
        <button type="button" class="secondary" data-order-detail="${escapeHtml(order.id)}">단계·기록 보기</button>
      </div>
    </article>`).join('');
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

function renderMockWorld(world, faultKinds) {
  mockPanel.hidden = !world;
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

async function orderControl(taskId, action) {
  try {
    const response = await fetch(`/api/tasks/${encodeURIComponent(taskId)}/${action}`, { method: 'POST' });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, `주문 ${action} 실패`));
  } catch (error) {
    orderMessage.textContent = error.message;
  }
}

async function refreshFlow() {
  try {
    const response = await fetch('/api/flow', { cache: 'no-store' });
    const view = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(view, '진행 상황을 불러오지 못했습니다'));
    renderFlowAlerts(view.alerts);
    renderFlowRobots(view);
    renderFlowOrders(view.orders);
    renderMockWorld(view.mock_world, view.mock_fault_kinds);
    flowStation.textContent = !view.station_owner ? '적재 자리 비어 있음'
      : view.station_held_by_stopped ? `적재 자리 막힘 · 멈춘 주문 ${view.station_owner} (경보 확인 필요)` : `적재 자리 사용 중 · ${view.station_owner}`;
    flowUpdated.textContent = `갱신 ${clockFormat.format(new Date())}`;
  } catch (error) {
    flowUpdated.textContent = `갱신 실패 · ${error.message}`;
  }
}

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

document.querySelector('#mockRestock').addEventListener('click', async () => {
  await fetch('/api/mock-world', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ action: 'restock' }) });
  mockMessage.textContent = '선반 재고를 처음 상태로 채웠습니다.';
  await refreshFlow();
});

refresh();
refreshFlow();
refreshTasks();
refreshArmCommands();
loadShelfSlots();
window.setInterval(refresh, 3000);
window.setInterval(refreshFlow, 1000);
window.setInterval(refreshTasks, 1000);
window.setInterval(refreshArmCommands, 1000);
window.setInterval(pollParkingLive, 500);
