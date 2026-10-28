const serverStatus = document.querySelector('#serverStatus');
const robotRows = document.querySelector('#robotRows');
const cameraGrid = document.querySelector('#cameraGrid');
const updatedAt = document.querySelector('#updatedAt');
const armState = document.querySelector('#armState');
const armDetail = document.querySelector('#armDetail');
const armStart = document.querySelector('#armStart');
const armStop = document.querySelector('#armStop');
const taskForm = document.querySelector('#taskForm');
const taskList = document.querySelector('#taskList');
const taskDetail = document.querySelector('#taskDetail');
const taskCount = document.querySelector('#taskCount');
const taskMessage = document.querySelector('#taskMessage');
let selectedTaskId = null;

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (character) => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
  })[character]);
}

function renderRobots(robots) {
  if (!robots.length) {
    robotRows.innerHTML = '<tr><td colspan="3" class="loading">등록된 로봇이 없습니다.</td></tr>';
    return;
  }
  robotRows.innerHTML = robots.map((robot) => {
    const stateClass = robot.state === 'available' ? 'available' : robot.state === 'waiting' ? 'waiting' : '';
    return `<tr><td>${escapeHtml(robot.name)}</td><td><span class="state ${stateClass}">${escapeHtml(robot.communication)}</span></td><td>${escapeHtml(robot.detail)}</td></tr>`;
  }).join('');
}

function renderCameras(cameras) {
  const names = { HEAD: 'Head Camera', JOINT: 'Joint Camera' };
  const connectionLabel = {
    CONNECTED: '연결됨', DISCONNECTED: '연결 끊김', UNKNOWN: '확인 필요',
  };
  const streamLabel = {
    STREAMING: '수신 중', NO_FRAMES: '프레임 미수신', STOPPED: '중지',
    ERROR: '오류', UNKNOWN: '확인 필요',
  };
  const freshnessLabel = { FRESH: '최신', STALE: '지연', UNKNOWN: '확인 필요' };
  const cameraIds = cameras.map((camera) => camera.id).join(',');
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
          <div class="camera-detail"><span>프레임률</span><strong data-field="frame-rate"></strong></div>
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
    card.querySelector('[data-field="frame-rate"]').textContent = camera.frame_rate == null ? '—' : `${camera.frame_rate} fps`;
    card.querySelector('[data-field="error"]').textContent = camera.error_message || '';
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

const taskStatusLabels = {
  PENDING: '대기', VALIDATING: '검증 중', RUNNING: '실행 중',
  SUCCEEDED: '성공', FAILED: '실패', CANCELING: '취소 중',
  CANCELED: '취소됨', NEEDS_INTERVENTION: '확인 필요',
};
const stepLabels = {
  VALIDATE_ROBOTS: '로봇 상태 확인', NAVIGATE_TO_DESTINATION: '목적지로 이동',
  VERIFY_ARRIVAL: '도착 확인', EXECUTE_ARM_TASK: '팔 작업',
  VERIFY_ARM_RESULT: '팔 작업 결과 확인', FINALIZE_TASK: '작업 마무리',
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
  taskDetail.innerHTML = `
    <header class="task-detail-head"><div><h3>${escapeHtml(task.destination)}</h3><small>${escapeHtml(task.id)}</small></div><span class="task-status">${escapeHtml(taskStatusLabels[task.status] || task.status)}</span></header>
    <p class="task-arm-job">팔 작업: ${escapeHtml(task.arm_job)}</p>
    <h4>단계</h4>
    <ol class="step-list">${steps.map((step) => `<li><span>${escapeHtml(stepLabels[step.step_type] || step.step_type)}</span><strong>${escapeHtml(taskStatusLabels[step.status] || step.status)}</strong></li>`).join('')}</ol>
    <h4>이벤트</h4>
    <ol class="event-list">${events.map((event) => `<li><time>${escapeHtml(event.received_at)}</time><span>${escapeHtml(event.event_type)}</span></li>`).join('') || '<li class="empty-history">기록된 이벤트가 없습니다.</li>'}</ol>`;
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

taskForm.addEventListener('submit', async (event) => {
  event.preventDefault();
  const formData = new FormData(taskForm);
  const button = taskForm.querySelector('button');
  button.disabled = true;
  taskMessage.textContent = '';
  try {
    const response = await fetch('/api/tasks', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ destination: formData.get('destination'), arm_job: formData.get('arm_job') }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '작업 등록에 실패했습니다.');
    selectedTaskId = result.task.id;
    taskForm.reset();
    taskMessage.textContent = '작업 이력을 저장했습니다. 로봇 명령은 전송되지 않았습니다.';
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
    serverStatus.className = 'server-status ok';
    serverStatus.innerHTML = `<span class="dot"></span> API 정상 · ${health.mode === 'ros2' ? 'ROS 2' : '모의'} 모드`;
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
window.setInterval(refreshTasks, 3000);
