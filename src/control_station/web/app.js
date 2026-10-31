const serverStatus = document.querySelector('#serverStatus');
const robotRows = document.querySelector('#robotRows');
const cameraGrid = document.querySelector('#cameraGrid');
const updatedAt = document.querySelector('#updatedAt');
const armState = document.querySelector('#armState');
const armDetail = document.querySelector('#armDetail');
const armStart = document.querySelector('#armStart');
const armStop = document.querySelector('#armStop');
const coordinateTargetForm = document.querySelector('#coordinateTargetForm');
const coordinateTargetMessage = document.querySelector('#coordinateTargetMessage');
const coordinateTargetResult = document.querySelector('#coordinateTargetResult');
const targetObservationForm = document.querySelector('#targetObservationForm');
const detectorClassOptions = document.querySelector('#detectorClassOptions');
const taskForm = document.querySelector('#taskForm');
const commandParseForm = document.querySelector('#commandParseForm');
const commandParseMessage = document.querySelector('#commandParseMessage');
const taskList = document.querySelector('#taskList');
const taskDetail = document.querySelector('#taskDetail');
const taskCount = document.querySelector('#taskCount');
const taskMessage = document.querySelector('#taskMessage');
const viewTitle = document.querySelector('#viewTitle');
const navItems = [...document.querySelectorAll('.nav-item[data-view]')];
const views = [...document.querySelectorAll('.view[id^="view-"]')];
let selectedTaskId = null;
let controlMode = 'unknown';
let targetObservationClass = '';
let targetObservationColor = '';
let targetObservationShelfLevel = '';
let targetObservationRequest = 0;
let taskObservationRequest = 0;
let lastTaskObservation = { key: '', message: '' };

const viewNames = { overview: '현황', manual: '수동 조작', tasks: '작업 이력', api: 'API' };
const controlModeLabels = {
  mock: '모의',
  ros2: 'ROS 2 상태 관측',
  mujoco: 'MuJoCo 시뮬레이션 관측',
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
  const names = { HEAD: 'Head Camera', JOINT: 'Joint Camera', FIXED: 'MuJoCo Fixed Camera' };
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
          <div class="camera-detail" data-field="source-info-row" hidden><span>ROS source frame / stamp</span><strong data-field="source-info"></strong></div>
          <div class="camera-detail" data-field="depth-info-row" hidden><span>Depth · 관측 전용</span><strong data-field="depth-info"></strong></div>
          <div class="camera-detail" data-field="camera-info-row" hidden><span>CameraInfo · 보정 검토 필요</span><strong data-field="camera-info"></strong></div>
          <div class="camera-detail" data-field="tf-info-row" hidden><span>카메라 → base TF · 시뮬레이션</span><strong data-field="tf-info"></strong></div>
          <div class="camera-detail" data-field="point-cloud-info-row" hidden><span>XYZ PointCloud2 · 관측 전용</span><strong data-field="point-cloud-info"></strong></div>
          <div class="camera-detail"><span>프레임률</span><strong data-field="frame-rate"></strong></div>
          <div class="camera-detail" data-field="detections-row" hidden><span>YOLO 검출</span><strong data-field="detections"></strong></div>
          <div class="camera-detail" data-field="detector-provenance-row" hidden><span>YOLO 설정 / 모델</span><strong data-field="detector-provenance"></strong></div>
          <div class="camera-detail" data-field="target-observation" hidden><span>대상 판정 · 관측 전용</span><strong data-field="target-status"></strong></div>
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
    const sourceInfo = camera.source_frame_id || camera.source_stamp
      ? `${camera.source_frame_id || 'frame ID 없음'}${camera.source_stamp ? ` · ROS ${camera.source_stamp.sec}.${String(camera.source_stamp.nanosec).padStart(9, '0')}` : ''}`
      : '—';
    card.querySelector('[data-field="source-info-row"]').hidden = sourceInfo === '—';
    card.querySelector('[data-field="source-info"]').textContent = sourceInfo;
    const depth = camera.depth_observation;
    const depthSummary = depth?.state === 'OBSERVED'
      ? `${depth.state} · ${depth.encoding} · ${depth.width}×${depth.height} · RGB ${depth.rgb_header_dimension_match || '미비교'} · ${depth.freshness || '신선도 미확인'}`
      : (depth?.state || '미제공');
    card.querySelector('[data-field="depth-info-row"]').hidden = !depth;
    card.querySelector('[data-field="depth-info"]').textContent = depthSummary;
    const cameraInfo = camera.camera_info_observation;
    const cameraInfoSummary = cameraInfo?.state === 'OBSERVED'
      ? `${cameraInfo.state} · ${cameraInfo.width}×${cameraInfo.height} · ${cameraInfo.distortion_model || '모델 미지정'} · RGB ${cameraInfo.rgb_header_dimension_match || '미비교'} · ${cameraInfo.freshness || '신선도 미확인'} · 검증 전`
      : (cameraInfo?.state || '미제공');
    card.querySelector('[data-field="camera-info-row"]').hidden = !cameraInfo;
    card.querySelector('[data-field="camera-info"]').textContent = cameraInfoSummary;
    const tf = camera.tf_observation;
    const tfSummary = tf?.state === 'AVAILABLE'
      ? `${tf.state} · (${tf.translation_xyz_m.map((value) => Number(value).toFixed(3)).join(', ')}) m · placeholder`
      : (tf?.state || '미제공');
    card.querySelector('[data-field="tf-info-row"]').hidden = !tf;
    card.querySelector('[data-field="tf-info"]').textContent = tfSummary;
    const pointCloud = camera.point_cloud_observation;
    const pointCloudSummary = pointCloud?.state === 'OBSERVED'
      ? `${pointCloud.state} · ${pointCloud.width}×${pointCloud.height} · ${pointCloud.source_frame_id} · 좌표 미해석 · RGB ${pointCloud.rgb_header_dimension_match || '미비교'} · ${pointCloud.freshness || '신선도 미확인'}`
      : (pointCloud?.state || '미제공');
    card.querySelector('[data-field="point-cloud-info-row"]').hidden = !pointCloud;
    card.querySelector('[data-field="point-cloud-info"]').textContent = pointCloudSummary;
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
  });
  if (targetObservationClass) {
    const requestId = ++targetObservationRequest;
    cameras
      .filter((camera) => camera.position === 'HEAD' || camera.position === 'JOINT')
      .forEach((camera) => refreshTargetObservation(camera, requestId));
  }
}

async function refreshTargetObservation(camera, requestId) {
  const row = cameraGrid.querySelector(`[data-camera-id="${camera.id}"] [data-field="target-observation"]`);
  const output = row?.querySelector('[data-field="target-status"]');
  if (!row || !output) return;
  row.hidden = false;
  output.textContent = `${targetObservationClass} 관측 중…`;
  try {
    const query = new URLSearchParams({ class_name: targetObservationClass });
    if (targetObservationColor) query.set('color', targetObservationColor);
    if (targetObservationShelfLevel) query.set('shelf_level', targetObservationShelfLevel);
    const response = await fetch(`/api/cameras/${encodeURIComponent(camera.id)}/target-observation?${query}`, { cache: 'no-store' });
    const result = await response.json();
    if (requestId !== targetObservationRequest) return;
    if (!response.ok) throw new Error(apiErrorMessage(result, '관측 요청 실패'));
    const observation = result.observation;
    const labels = {
      DETECTOR_NOT_CONFIGURED: '검출 모델 미설정', DETECTOR_LOADING: '검출 모델 준비 중',
      DETECTOR_ERROR: '검출 모델 오류', DETECTOR_NOT_READY: '검출기 사용 불가',
      REQUEST_PROMPT_INFERENCE_ERROR: '요청 대상 prompt 추론 오류',
      CLASS_UNKNOWN: '모델에 없는 클래스', PIXEL_TRANSFORM_UNKNOWN: '픽셀 변환 출처 미확인',
      CAMERA_DISCONNECTED: '카메라 연결 끊김', CAMERA_STREAM_UNAVAILABLE: '카메라 스트림 오류',
      FRAME_PROVENANCE_UNKNOWN: '프레임 fingerprint 미확인',
      FRAME_STALE: '프레임 지연 · 결과 폐기', FRAME_UNAVAILABLE: '최신 프레임 없음',
      TARGET_NOT_FOUND: '대상 후보 없음', SINGLE_CANDIDATE: '단일 후보', AMBIGUOUS: '복수 후보 · 모호',
    };
    const candidates = observation.candidates.map(formatDetectionCandidate).join(', ');
    const colorEvaluation = formatColorEvaluation(observation.color_evaluation);
    const shelfEvaluation = formatShelfLevelEvaluation(observation.shelf_level_evaluation);
    output.textContent = `${labels[observation.status] || observation.status}${observation.candidate_count == null ? '' : ` · ${observation.candidate_count}개`}${colorEvaluation ? ` · ${colorEvaluation}` : ''}${shelfEvaluation ? ` · ${shelfEvaluation}` : ''}${candidates ? ` · ${candidates}` : ''}`;
  } catch (error) {
    if (requestId === targetObservationRequest) output.textContent = `관측 API 오류 · ${error.message}`;
  }
}

function formatDetectionCandidate(item) {
  const colorLabels = { red: '빨강', blue: '파랑', green: '초록', yellow: '노랑', black: '검정', white: '흰색' };
  const hint = item.color_hint;
  const hintStatusLabels = {
    ESTIMATED: '색상 추정',
    LOW_COLOR_COVERAGE: '색상 픽셀 비율 낮음',
    MIXED_COLOR: '색상 혼합',
    UNAVAILABLE: '색상 힌트 사용 불가',
  };
  const hintVersion = typeof hint?.algorithm_version === 'string' && /^[a-z0-9-]{1,80}$/.test(hint.algorithm_version)
    ? ` · ${hint.algorithm_version}`
    : '';
  const hintFraction = Number.isFinite(hint?.pixel_fraction)
    ? ` · 픽셀 ${Math.round(hint.pixel_fraction * 100)}%`
    : '';
  const hintValue = hint?.status === 'ESTIMATED' && colorLabels[hint.color]
    ? ` ${colorLabels[hint.color]}`
    : '';
  const colorHint = hint && hintStatusLabels[hint.status]
    ? ` · ${hintStatusLabels[hint.status]}${hintValue}${hintFraction}${hintVersion}`
    : '';
  return `(${item.center_px.join(', ')}) ${Math.round(item.confidence * 100)}%${colorHint}`;
}

function formatColorEvaluation(evaluation) {
  if (!evaluation) return '';
  const indices = Array.isArray(evaluation.candidate_indices)
    ? evaluation.candidate_indices.map((index) => `#${index + 1}`).join(', ')
    : '';
  const labels = {
    UNSUPPORTED_REQUEST_COLOR: '지원하지 않는 색상 조건',
    NO_CANDIDATES: '색상 비교 후보 없음',
    NO_ESTIMATED_COLOR_HINTS: '색상 힌트 판정 불가',
    NO_COLOR_HINT_MATCH: '색상 힌트 일치 0개',
    SINGLE_COLOR_HINT_MATCH: `색상 힌트 일치 ${indices || '1개'}`,
    MULTIPLE_COLOR_HINT_MATCHES: `색상 힌트 일치 ${indices || `${evaluation.candidate_indices?.length || 0}개`}`,
  };
  const algorithm = typeof evaluation.algorithm_version === 'string' && /^[a-z0-9-]{1,80}$/.test(evaluation.algorithm_version)
    ? ` · 기준 ${evaluation.algorithm_version}`
    : '';
  return `${labels[evaluation.status] || evaluation.status}${algorithm} · 검토 필요 · 자동 선택 아님`;
}

function formatShelfLevelEvaluation(evaluation) {
  if (!evaluation) return '';
  const indices = Array.isArray(evaluation.candidate_indices)
    ? evaluation.candidate_indices.map((index) => `#${index + 1}`).join(', ')
    : '';
  const assignmentLabels = {
    AMBIGUOUS_REGION_ASSIGNMENT: '층 영역 모호',
    OUTSIDE_REGIONS: '미분류',
    INVALID_CENTER_PX: '중심 좌표 오류',
    INVALID_CANDIDATE: '후보 오류',
  };
  const assignments = Array.isArray(evaluation.candidate_assignments)
    ? evaluation.candidate_assignments.map((assignment) => {
      const candidate = `#${assignment.candidate_index + 1}`;
      if (Number.isInteger(assignment.shelf_level)) return `${candidate}→${assignment.shelf_level}층`;
      if (assignment.status === 'AMBIGUOUS_REGION_ASSIGNMENT' && assignment.shelf_levels?.length) {
        return `${candidate}→모호(${assignment.shelf_levels.join('/')})`;
      }
      return `${candidate}→${assignmentLabels[assignment.status] || assignment.status}`;
    }).join(', ')
    : '';
  const labels = {
    CALIBRATION_NOT_CONFIGURED: '층 영역 초안 미설정',
    CALIBRATION_UNREADABLE: '층 영역 초안 읽기 실패',
    CALIBRATION_INVALID: '층 영역 초안 무효',
    CALIBRATION_CAMERA_MISMATCH: '층 영역 카메라 불일치',
    CALIBRATION_PIXEL_TRANSFORM_MISMATCH: '픽셀 변환 불일치',
    CALIBRATION_RESOLUTION_MISMATCH: '영상 해상도 불일치',
    REQUESTED_LEVEL_NOT_CONFIGURED: '요청 층 영역 미설정',
    NO_CANDIDATES: '층 비교 후보 없음',
    NO_REGION_MATCH: '등록 영역 안 후보 없음',
    NO_SHELF_LEVEL_MATCH: '요청 층 힌트 일치 없음',
    AMBIGUOUS_REGION_ASSIGNMENT: '층 영역 겹침/경계로 모호',
    INVALID_CANDIDATE_POSITION: '후보 중심 좌표 오류',
    SINGLE_SHELF_LEVEL_MATCH: `층 힌트 일치 ${indices || '1개'}`,
    MULTIPLE_SHELF_LEVEL_MATCHES: `층 힌트 일치 ${indices || '복수'}`,
  };
  const artifactHash = /^[0-9a-f]{64}$/.test(evaluation.calibration_artifact_sha256 || '')
    ? ` · ROI SHA-256 ${evaluation.calibration_artifact_sha256}`
    : '';
  return `${labels[evaluation.status] || evaluation.status}${assignments ? ` (${assignments})` : ''}${artifactHash} · 검토 필요 · 자동 선택 아님`;
}

targetObservationForm.addEventListener('submit', (event) => {
  event.preventDefault();
  const formData = new FormData(targetObservationForm);
  targetObservationClass = String(formData.get('class_name') || '').trim();
  targetObservationColor = String(formData.get('color') || '').trim();
  targetObservationShelfLevel = String(formData.get('shelf_level') || '').trim();
  refresh();
});

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

coordinateTargetForm.addEventListener('submit', async (event) => {
  event.preventDefault();
  const button = coordinateTargetForm.querySelector('button');
  const formData = new FormData(coordinateTargetForm);
  button.disabled = true;
  coordinateTargetMessage.textContent = '';
  coordinateTargetResult.hidden = true;
  try {
    const response = await fetch('/api/arm/coordinate-target-drafts', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        frame_id: formData.get('frame_id'),
        unit: formData.get('unit'),
        position: {
          x: Number(formData.get('x')),
          y: Number(formData.get('y')),
          z: Number(formData.get('z')),
        },
      }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '좌표 초안을 저장하지 못했습니다.');
    coordinateTargetMessage.textContent = `미실행 초안 저장 완료 · ${result.draft.file}`;
    coordinateTargetResult.textContent = JSON.stringify(result.draft, null, 2);
    coordinateTargetResult.hidden = false;
  } catch (error) {
    coordinateTargetMessage.textContent = error.message;
  } finally {
    button.disabled = false;
  }
});

const taskStatusLabels = {
  PENDING: '대기', VALIDATING: '검증 중', RUNNING: '실행 중',
  SUCCEEDED: '성공', FAILED: '실패', CANCELING: '취소 중',
  CANCELED: '취소됨', NEEDS_INTERVENTION: '확인 필요',
};
const stepLabels = {
  VALIDATE_ROBOTS: '로봇 상태 확인', NAVIGATE_TO_DESTINATION: '목적지로 이동',
  VERIFY_ARRIVAL: '도착 확인', EXECUTE_ARM_TASK: '팔 작업',
  VERIFY_ARM_RESULT: '팔 작업 결과 확인', FINALIZE_TASK: '작업 마무리',
  VALIDATE_REQUEST: '요청 조건 확인', DETECT_TARGETS: '대상 후보 관측',
  SELECT_TARGET: '대상 선택', ESTIMATE_TARGET_POSE: '대상 위치 추정',
  VALIDATE_GRASP: '파지 가능성 확인', APPROACH_TARGET: '대상 접근',
  REFINE_TARGET: '손목 카메라 재관측', GRASP_TARGET: '대상 파지',
  VERIFY_GRASP: '파지 확인', LIFT_TARGET: '대상 들어 올리기',
  MOVE_TO_DESTINATION: '목적지 이동', PLACE_TARGET: '대상 놓기',
  VERIFY_PLACEMENT: '놓기 확인',
};
const executionLabels = {
  COMMAND_SENT: '명령 기록', ACCEPTED: '접수됨', RUNNING: '실행 중',
  SUCCEEDED: '성공', FAILED: '실패', TIMED_OUT: '시간 초과',
  CANCELED: '취소됨', UNKNOWN: '결과 불명', PENDING: '대기',
};
const stepOutcomeLabels = {
  SIMULATED_SUCCESS: '모의 완료',
  CANCEL_CONFIRMED: '취소 확인',
  NOT_EXECUTED_AFTER_CANCEL: '취소 뒤 미실행',
  NOT_EXECUTED_AFTER_FAILURE: '실패 뒤 미실행',
};
const taskEventLabels = {
  MOCK_SCENARIO_FAILURE: '모의 실패 시나리오 발생',
  MOCK_EXECUTION_FAILED: '모의 실행 오류',
  MOCK_TASK_COMPLETED: '모의 작업 완료',
  TARGET_PLANE_PROJECTION_RECORDED: '선반 평면 좌표 초안 기록',
};

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
      <span><strong>${escapeHtml(task.destination)}</strong><small>${escapeHtml(task.id)}</small></span>
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
  const observationKey = `${task.id}:${pickRequest?.category || ''}`;
  const observationMessage = lastTaskObservation.key === observationKey
    ? lastTaskObservation.message
    : '아직 관측하지 않았습니다.';
  const canRecordObservation = task.status === 'PENDING';
  const canStart = task.status === 'PENDING' && controlMode === 'mock';
  const canCancel = ['VALIDATING', 'RUNNING'].includes(task.status)
    && controlMode === 'mock'
    && task.local_execution_owner === 'LOCAL_WORKER_ACTIVE';
  const modeLabel = controlModeLabels[controlMode] || controlMode;
  const modeNote = controlMode === 'mock'
    ? '모의 모드 · 단계 성공은 진행 시뮬레이션이며 실제 인식/로봇 명령을 의미하지 않습니다.'
    : controlMode === 'mujoco'
      ? 'MuJoCo 시뮬레이션 관측 모드 · 작업 start/cancel과 실물 팔 제어는 비활성화되어 있습니다.'
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
    <p class="task-arm-job">흐름: ${task.workflow === 'PICK_AND_PLACE' ? 'Pick & Place' : '이동·팔 작업'}</p>
    <p class="task-arm-job">팔 작업: ${escapeHtml(task.arm_job)}</p>
    <p class="task-arm-job">대상 요청: ${escapeHtml(targetSummary)} · 저장된 명세이며 아직 인식/집기에는 연결되지 않음</p>
    ${ownershipNote}
    ${pickRequest?.category ? `<div class="task-observation"><button type="button" class="secondary" data-task-observe${canRecordObservation ? '' : ' disabled'}>카메라에서 대상 관측</button><label class="observation-image-option"><input type="checkbox" data-save-observation-frame> 원본 RGB 프레임 PNG도 저장</label><p class="note">Head Camera 선반 탐색 관측의 선택 기록 버튼은 원본 PNG가 저장되고 hash가 확인된 경우에만 나타납니다. 선택은 snapshot에 대한 감사 정보이며 task/step 진행이나 팔 명령을 바꾸지 않습니다.</p>${canRecordObservation ? '' : '<p class="note">관측·선택 이벤트는 대기 작업에서만 기록할 수 있습니다.</p>'}<p class="note" data-task-observation-result>${escapeHtml(observationMessage)}</p></div>` : ''}
    <div class="task-actions">${action}<span class="note">${modeNote}</span></div>
    <h4>단계</h4>
    <ol class="step-list">${steps.map((step) => {
      const stepLabel = stepLabels[step.step_type] || step.step_type;
      const robotLabel = step.robot_id === 'mobile-01' ? '주행 로봇' : step.robot_id === 'arm-01' ? '로봇팔' : '';
      return `<li><span>${escapeHtml(stepLabel)}${step.robot_id ? `<small class="step-robot">${robotLabel ? `${robotLabel} · ` : ''}${escapeHtml(step.robot_id)}</small>` : ''}</span><strong>${escapeHtml(executionLabels[step.status] || step.status)}${step.result?.outcome && stepOutcomeLabels[step.result.outcome] ? ` · ${stepOutcomeLabels[step.result.outcome]}` : ''}${step.result?.source === 'mock' ? ' · 모의' : ''}</strong></li>`;
    }).join('')}</ol>
    <h4>이벤트</h4>
    <ol class="event-list">${events.map((event) => {
      const projection = event.event_type === 'TARGET_PLANE_PROJECTION_RECORDED' ? event.payload : null;
      const observation = event.event_type === 'TARGET_OBSERVED' ? event.payload?.observation : null;
      const selection = event.event_type === 'TARGET_SELECTED' ? event.payload : null;
      const candidateCount = observation?.candidate_count == null ? '후보 수 미확정' : `후보 ${observation.candidate_count}개`;
      const candidateHints = observation?.candidates?.map(formatDetectionCandidate).join('; ');
      const colorEvaluation = formatColorEvaluation(observation?.color_evaluation);
      const shelfEvaluation = formatShelfLevelEvaluation(observation?.shelf_level_evaluation);
      const depthRegion = observation?.depth_region_observation;
      const depthRegionText = depthRegion
        ? ` · bbox raw-depth ${depthRegion.state === 'SUMMARIZED' ? (depthRegion.candidates || []).map((sample) => {
            const median = Number(sample.raw_value_median);
            return sample.state === 'POSITIVE_FINITE_SAMPLES' && Number.isFinite(median)
              ? `후보 #${Number(sample.candidate_index) + 1} ${depthRegion.depth_encoding} 원시 중앙값 ${median.toFixed(3)} (단위 미해석)`
              : `후보 #${Number(sample.candidate_index) + 1} ${sample.state || '표본 없음'}`;
          }).join(', ') : depthRegion.state} · 위치/선택에 미사용`
        : '';
      const sourceStamp = observation?.source_stamp
        ? ` · ROS source ${observation.source_frame_id || 'frame ID 없음'} ${observation.source_stamp.sec}.${String(observation.source_stamp.nanosec).padStart(9, '0')}`
        : '';
      const requestPrompt = observation?.detector_provenance?.prompt_scope === 'REQUEST_CLASS_ONE_SHOT'
        ? ` · 요청 prompt ${observation.detector_provenance.requested_prompt || observation.target_class}`
        : '';
      const eventName = event.payload?.event || event.event_type;
      const selectionCandidate = selection?.candidate;
      const linkedStep = steps.find((step) => step.id === event.step_id);
      const stepContext = linkedStep
        ? ` · 연결 단계 ${stepLabels[linkedStep.step_type] || linkedStep.step_type}`
        : '';
      const detail = projection
        ? `2D 선반 평면 좌표 초안 · ${projection.plane_frame?.frame_id || 'frame 미지정'} · x=${Number(projection.plane_xy?.x).toFixed(3)}, y=${Number(projection.plane_xy?.y).toFixed(3)} ${projection.plane_frame?.unit || ''} · bbox 하단 중앙 proxy · 후보 #${Number(projection.candidate_index) + 1} · 로봇 목표로 사용 불가`
        : observation
        ? `대상 관측 · ${event.payload?.pick_request?.category || observation.target_class} · ${observation.camera_id} · ${observation.status}${requestPrompt} · ${candidateCount}${colorEvaluation ? ` · ${colorEvaluation}` : ''}${shelfEvaluation ? ` · ${shelfEvaluation}` : ''}${depthRegionText} · frame ${observation.frame_sequence}${sourceStamp}${candidateHints ? ` · ${candidateHints}` : ''}`
        : selection
          ? `운영자 후보 선택 기록 · ${selectionCandidate?.class_name || '대상'} #${Number(selection.candidate_index) + 1} · 관측 ${selection.observation_event_id} · frame ${selection.frame_sequence}`
          : eventName === 'PROGRESS'
            ? `${event.payload?.source === 'mock' ? '모의 ' : ''}진행률 ${Number(event.payload?.progress) || 0}% · ${event.payload?.message || '진행 이벤트'}`
            : (taskEventLabels[eventName] || eventName);
      const source = event.payload?.source;
      const robotId = event.payload?.robot_id;
      const imageLink = event.event_type === 'TARGET_OBSERVED' && event.payload?.frame_artifact?.path
        ? `<a href="/api/tasks/${encodeURIComponent(task.id)}/observations/${encodeURIComponent(event.id)}/image" target="_blank" rel="noopener">프레임 보기</a>`
        : '';
      const projectionArtifactLink = projection
        ? `<a href="/api/tasks/${encodeURIComponent(task.id)}/projection-drafts/${encodeURIComponent(event.id)}/artifact" target="_blank" rel="noopener">2D 초안 JSON 보기</a>`
        : '';
      const selectionControls = canRecordObservation
        && linkedStep?.step_type === 'DETECT_TARGETS'
        && event.payload?.frame_artifact?.path
        && observation?.candidates?.length
        ? `<div class="candidate-selection-controls">${observation.candidates.map((candidate, index) => `<button type="button" class="secondary" data-candidate-selection data-observation-event-id="${escapeHtml(event.id)}" data-candidate-index="${index}">후보 #${index + 1} 선택 기록</button>`).join('')}</div>`
        : '';
      return `<li><time>${escapeHtml(event.received_at)}</time><span>${escapeHtml(detail)}${stepContext}${source ? ` · ${escapeHtml(source)}` : ''}${robotId ? ` · ${escapeHtml(robotId)}` : ''}</span>${imageLink}${projectionArtifactLink}${selectionControls}</li>`;
    }).join('') || '<li class="empty-history">기록된 이벤트가 없습니다.</li>'}</ol>`;
  taskDetail.querySelector('[data-task-action]')?.addEventListener('click', async (event) => {
    const button = event.currentTarget;
    button.disabled = true;
    await sendTaskAction(task.id, button.dataset.taskAction);
  });
  taskDetail.querySelector('[data-task-observe]')?.addEventListener('click', async (event) => {
    const button = event.currentTarget;
    button.disabled = true;
    await observeTaskTarget(task.id, pickRequest, button);
  });
  taskDetail.querySelectorAll('[data-candidate-selection]').forEach((button) => {
    button.addEventListener('click', async (event) => {
      const selected = event.currentTarget;
      selected.disabled = true;
      await recordTargetSelection(
        task.id,
        selected.dataset.observationEventId,
        Number(selected.dataset.candidateIndex),
      );
    });
  });
}

async function recordTargetSelection(taskId, observationEventId, candidateIndex) {
  taskMessage.textContent = '관측 snapshot의 후보 선택을 감사 기록에 남깁니다…';
  try {
    const response = await fetch(`/api/tasks/${encodeURIComponent(taskId)}/target-selection`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ observation_event_id: observationEventId, candidate_index: candidateIndex }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(apiErrorMessage(result, '후보 선택을 기록하지 못했습니다.'));
    const recordedMessage = result.already_recorded
      ? `기존 선택 기록 ${result.selection_event_id} 재사용`
      : `선택 기록 ${result.selection_event_id}`;
    taskMessage.textContent = `후보 #${candidateIndex + 1} ${recordedMessage} (관측 ${observationEventId}). 작업 상태와 로봇 명령은 변경되지 않았습니다.`;
  } catch (error) {
    taskMessage.textContent = `후보 선택 기록 오류: ${error.message}`;
  } finally {
    await refreshTasks();
  }
}

async function observeTaskTarget(taskId, pickRequest, button) {
  const category = pickRequest.category;
  const saveFrame = taskDetail.querySelector('[data-save-observation-frame]')?.checked === true;
  const requestId = ++taskObservationRequest;
  const observationKey = `${taskId}:${category}`;
  const output = taskDetail.querySelector('[data-task-observation-result]');
  if (output) output.textContent = `${category} · 카메라 관측 요청 중…`;
  try {
    const camerasResponse = await fetch('/api/cameras', { cache: 'no-store' });
    const camerasResult = await camerasResponse.json();
    if (!camerasResponse.ok) throw new Error(camerasResult.error || '카메라 목록을 불러오지 못했습니다.');
    const observations = [];
    let recordedCount = 0;
    for (const camera of camerasResult.cameras || []) {
      const response = await fetch(`/api/tasks/${encodeURIComponent(taskId)}/target-observation`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ camera_id: camera.id, save_frame: saveFrame }),
      });
      const result = await response.json();
      if (!response.ok) {
        observations.push(`${camera.position || camera.id}: 기록 실패 (${apiErrorMessage(result, response.status)})`);
        continue;
      }
      recordedCount += 1;
      const observation = result.observation;
      const labels = {
        DETECTOR_NOT_CONFIGURED: '검출기 미설정', DETECTOR_LOADING: '검출기 준비 중',
        DETECTOR_ERROR: '검출기 오류', DETECTOR_NOT_READY: '검출기 사용 불가',
        DETECTOR_INFERENCE_ERROR: '고정 카메라 추론 오류',
        DETECTOR_NOT_RUN: '이 프레임 추론 안 함',
        REQUEST_PROMPT_INFERENCE_ERROR: '요청 대상 prompt 추론 오류',
        CLASS_UNKNOWN: '모델에 없는 클래스', PIXEL_TRANSFORM_UNKNOWN: '픽셀 변환 출처 미확인',
        CAMERA_DISCONNECTED: '카메라 연결 끊김', CAMERA_STREAM_UNAVAILABLE: '카메라 스트림 오류',
        FRAME_PROVENANCE_UNKNOWN: '프레임 fingerprint 미확인', FRAME_STALE: '프레임 지연',
        FRAME_UNAVAILABLE: '최신 프레임 없음', TARGET_NOT_FOUND: '후보 없음',
        SINGLE_CANDIDATE: '단일 후보', AMBIGUOUS: '복수 후보 · 모호',
      };
      const position = camera.position || camera.id;
      const candidates = (observation.candidates || []).map(formatDetectionCandidate).join(', ');
      const colorEvaluation = formatColorEvaluation(observation.color_evaluation);
      const shelfEvaluation = formatShelfLevelEvaluation(observation.shelf_level_evaluation);
      const requestPrompt = observation.detector_provenance?.prompt_scope === 'REQUEST_CLASS_ONE_SHOT'
        ? ` · 요청 prompt ${observation.detector_provenance.requested_prompt || observation.target_class}`
        : '';
      const savedImage = result.frame_artifact?.path;
      observations.push(`${position}: ${labels[observation.status] || observation.status}${requestPrompt}${observation.candidate_count == null ? '' : ` · ${observation.candidate_count}개`}${colorEvaluation ? ` · ${colorEvaluation}` : ''}${shelfEvaluation ? ` · ${shelfEvaluation}` : ''}${candidates ? ` · ${candidates}` : ''}${savedImage ? ` · 이미지 ${savedImage}` : ''}`);
    }
    if (!observations.length) observations.push('카메라 설정 없음');
    if (requestId !== taskObservationRequest) return;
    lastTaskObservation = {
      key: observationKey,
      message: `${observations.join(' / ')} · ${recordedCount ? `${recordedCount}개 task 관측 이벤트를 기록했습니다.` : '기록된 관측 이벤트가 없습니다.'} 위치 추정이나 집기 명령은 수행하지 않았습니다.`,
    };
  } catch (error) {
    if (requestId === taskObservationRequest) {
      lastTaskObservation = {
        key: observationKey,
        message: `관측 오류: ${error.message} · 로봇 명령은 전송하지 않았습니다.`,
      };
    }
  } finally {
    if (requestId === taskObservationRequest) {
      const currentOutput = taskDetail.querySelector('[data-task-observation-result]');
      if (currentOutput && selectedTaskId === taskId) currentOutput.textContent = lastTaskObservation.message;
      button.disabled = false;
    }
  }
}

async function sendTaskAction(taskId, action) {
  taskMessage.textContent = action === 'start' ? '모의 작업을 시작합니다…' : '모의 작업 취소를 요청합니다…';
  try {
    const response = await fetch(`/api/tasks/${encodeURIComponent(taskId)}/${action}`, { method: 'POST' });
    const result = await response.json();
    if (!response.ok) {
      const message = result.message || result.error || '작업 요청에 실패했습니다.';
      throw new Error(result.error_code ? `${result.error_code} · ${message}` : message);
    }
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
    taskForm.elements.target_category.value = parsed.pick_request?.category || '';
    taskForm.elements.target_color.value = parsed.pick_request?.color || '';
    taskForm.elements.shelf_level.value = parsed.pick_request?.shelf_level || '';
    taskForm.elements.destination.value = parsed.destination || '';
    taskForm.elements.arm_job.value = parsed.arm_job || '';
    const warnings = [...draft.warnings, ...draft.missing_fields.map((field) => `필수 확인: ${field}`)];
    commandParseMessage.textContent = `검토 필요 · ${warnings.length ? warnings.join(' · ') : '추출된 값을 확인한 뒤 작업 등록을 누르세요.'}`;
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
    serverStatus.className = 'server-status ok';
    serverStatus.innerHTML = `<span class="dot"></span> API 정상 · ${escapeHtml(controlModeLabels[health.mode] || health.mode)} 모드`;
    renderRobots(robotData.robots);
    renderCameras(cameraData.cameras);
    renderArm(armData.arm);
    updatedAt.textContent = `갱신 ${new Intl.DateTimeFormat('ko-KR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false }).format(new Date())}`;
  } catch {
    serverStatus.className = 'server-status error';
    serverStatus.innerHTML = '<span class="dot"></span> 서버 연결 실패';
    robotRows.innerHTML = '<tr><td colspan="3" class="error-row">서버 API에 연결할 수 없습니다.</td></tr>';
    cameraGrid.innerHTML = '<p class="error-row">서버 API에 연결할 수 없습니다.</p>';
    cameraGrid.dataset.cameraIds = '';
  }
}

refresh();
refreshTasks();
window.setInterval(refresh, 3000);
window.setInterval(refreshTasks, 1000);
