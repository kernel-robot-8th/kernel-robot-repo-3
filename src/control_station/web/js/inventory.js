// 재고 화면: 물건별로 선반에 있는 수, 열린 주문이 잡아 둔 수, 아직 팔 수 있는 수, 그리고 물건마다 작업자가
// 덮어쓰는 설정(판매 중지, 직접 센 수량으로 팔기, 카메라 값으로 되돌리기).
//
// app.js 뒤에 로드되고 그 도우미 escapeHtml, apiErrorMessage, formatClock, showToast, setStat을 쓴다.
//
// GET /api/inventory는 이 뷰가 화면에 있는 동안 2초마다 읽는다. POST /api/inventory/{object}는 새 재고 전체를
// 답하고, 그것을 바로 그린다. 모르는 개수는 "?"로 보이며 절대 0으로 보이지 않는다: 서버는 그런 물건의 주문을
// 닫고, 화면도 그렇게 말한다.

const INVENTORY_POLL_MS = 2000;

const inventoryView = document.querySelector('#view-inventory');
const inventoryProducts = document.querySelector('#inventoryProducts');
const inventoryEvents = document.querySelector('#inventoryEvents');
const inventoryBanner = document.querySelector('#inventoryBanner');
const inventorySummaryTitle = document.querySelector('#inventorySummaryTitle');
const inventorySummaryDetail = document.querySelector('#inventorySummaryDetail');
const inventoryStatOpen = document.querySelector('#inventoryStatOpen');
const inventoryStatSoldOut = document.querySelector('#inventoryStatSoldOut');
const inventoryStatUnknown = document.querySelector('#inventoryStatUnknown');
const inventoryStatReserved = document.querySelector('#inventoryStatReserved');

const inventoryScreen = {
  data: null,           // 마지막 GET(또는 POST) 응답
  loadError: null,      // 마지막 GET이 실패한 이유. 마지막 응답은 화면에 남는다
  polling: false,       // GET 요청 중. 타이머는 요청을 쌓지 않고 건너뛴다
  version: 0,           // POST 응답을 그릴 때 올린다. 그 전에 보낸 GET이 옛 상태를 그리지 못하게
  busy: new Set(),      // 수정 요청이 오가는 중인 물체 클래스
  messages: new Map(),  // 물체 클래스 -> 마지막 수정이 거절된 이유
};

// 물건 상태와 수량 출처마다 [말, 칩 색].
const inventoryStatusChips = {
  available: ['있음', 'good'], sold_out: ['품절', 'danger'], unknown: ['모름', 'attention'], closed: ['판매 중지', 'dark'],
};
const inventorySourceChips = {
  camera: ['카메라', 'active'], manual: ['직접 수량', 'attention'], closed: ['판매 중지', 'dark'],
};
// 두 팔이 각각 선반 하나를 맡는다(config/arms.json). 응답에 그 밖의 선반이 있으면 id로 보인다.
const inventoryShelves = [['shelf_a', '선반 A', '왼쪽 팔'], ['shelf_b', '선반 B', '오른쪽 팔']];
// 최근 변화 종류의 이름과 색.
const inventoryEventWords = {
  RESTOCKED: ['채움', 'good'], REMOVED_OUTSIDE: ['관제 밖 감소', 'attention'], OVERRIDE: ['작업자 수정', ''],
};

function inventoryLabel(objectClass) {
  return inventoryScreen.data?.products.find((product) => product.object_class === objectClass)?.label || objectClass;
}

function chipHtml([word, tone]) {
  return `<span class="chip${tone ? ` ${tone}` : ''}">${escapeHtml(word)}</span>`;
}

// 수량 출처 칩. 모의 세계 값은 모의 태그로 보인다.
function sourceChipHtml(source) {
  if (source === 'mock_world') return '<span class="sim-tag">모의</span>';
  return chipHtml(inventorySourceChips[source] || [source, '']);
}

// ---- 요약 ----

// 요약 카드와 경고 띠. data.state: live(카메라로 세는 중), held(팔이 일하는 동안 마지막 확인값 유지), unknown(모름).
function renderInventorySummary() {
  const { data, loadError } = inventoryScreen;
  if (loadError) {
    inventorySummaryTitle.textContent = '재고를 불러오지 못했어요';
    inventorySummaryDetail.textContent = data ? `${loadError}. 아래는 ${formatClock(data.checked_at)}에 받은 값이에요` : loadError;
  }
  if (!data) return;
  const { products } = data;
  const unknown = products.filter((product) => product.status === 'unknown');
  const soldOut = products.filter((product) => product.status === 'sold_out').length;
  const open = products.filter((product) => product.status === 'available').length;
  const reserved = products.reduce((sum, product) => sum + product.reserved, 0);
  setStat(inventoryStatOpen, open, { total: products.length, tone: open < products.length ? 'attention' : '' });
  setStat(inventoryStatSoldOut, soldOut, { tone: soldOut ? 'danger' : 'zero' });
  setStat(inventoryStatUnknown, unknown.length, { tone: unknown.length ? 'attention' : 'zero' });
  setStat(inventoryStatReserved, reserved, { tone: reserved ? 'active' : 'zero' });
  if (!loadError) {
    inventorySummaryTitle.innerHTML = {
      live: !data.simulated ? '카메라로 재고를 확인하고 있어요'
        : devMode ? '모의 세계에서 센 재고예요 <span class="sim-tag">모의</span>' : '재고를 확인하고 있어요',
      held: '팔이 작업 중이라 마지막 확인값을 보여 줘요',
      unknown: '재고를 확인하지 못했어요',
    }[data.state] || escapeHtml(data.state);
    inventorySummaryDetail.textContent = `${formatClock(data.checked_at)}에 확인했어요`;
  }
  // 재고를 모르면 그 물건의 주문을 닫는다. 사람이 선반 카메라를 봐야 한다.
  inventoryBanner.hidden = data.state !== 'unknown';
  if (data.state === 'unknown') {
    const names = unknown.map((product) => product.label).join(', ');
    inventoryBanner.innerHTML = `<strong>${names ? `${escapeHtml(names)}의 주문을 닫았어요` : '카메라 재고를 쓸 수 없어요'}</strong>
      <span>${escapeHtml(data.reason || '선반 카메라의 새 값이 없습니다')}${names ? '' : '. 직접 수량을 정한 물건만 주문을 받아요'}</span>`;
  }
}

// ---- 물건별 줄: 물건마다 한 번 만들고 그 자리에서 값만 바꾼다. 그래서 개수를 입력하는 중에 끊기지 않는다 ----

// 줄 뼈대: 이름과 칩, 카메라 재고, 선반별·합계·주문 가능·예약 수, [판매 중지]·[카메라 기준으로], 직접 수량 입력.
function inventoryRowSkeleton(product) {
  const id = escapeHtml(product.object_class);
  const shelves = inventoryShelves.map(([shelfId, shelf, arm]) => `
    <div class="stat"><dt>${shelf} <small>${arm}</small></dt><dd data-shelf="${shelfId}">–</dd></div>`).join('');
  return `
    <div class="inventory-head">
      <div class="inventory-name">
        <h3>${escapeHtml(product.label)}</h3>
        <span class="inventory-chips" data-field="chips"></span>
      </div>
      <dl class="inventory-available"><div class="stat"><dt>카메라 재고</dt><dd data-field="camera-stock">–</dd></div></dl>
    </div>
    <dl class="inventory-counts">
      ${shelves}
      <div class="stat"><dt>선반 합계</dt><dd data-field="total">–</dd></div>
      <div class="stat"><dt>주문 가능</dt><dd data-field="available">–</dd></div>
      <div class="stat"><dt>주문이 잡아 둔 개수</dt><dd data-field="reserved">–</dd></div>
    </dl>
    <p class="inventory-hint" data-field="hint" hidden></p>
    <div class="inventory-controls">
      <div class="inventory-modes">
        <button type="button" class="secondary" data-inventory-mode="closed" data-object="${id}">판매 중지</button>
        <button type="button" class="secondary" data-inventory-mode="auto" data-object="${id}">카메라 기준으로</button>
      </div>
      <form class="inventory-manual" data-object="${id}" novalidate>
        <label class="visually-hidden" for="inventoryCount-${id}">${escapeHtml(product.label)} 직접 센 개수</label>
        <input id="inventoryCount-${id}" name="count" type="number" inputmode="numeric" min="0" max="99" step="1" placeholder="센 개수" required>
        <button type="submit">직접 수량 적용</button>
      </form>
    </div>
    <p class="inventory-message" data-field="message" role="status" hidden></p>`;
}

function shelfCount(product, shelfId) {
  // 카메라 수가 모름인 동안 선반별 마지막 숫자는 오래된 값이니 그것도 모름으로 보인다.
  if (product.status === 'unknown') return '?';
  return String(product.per_shelf[shelfId] || 0);
}

// 줄 값 갱신. [판매 중지]는 이미 중지면, [카메라 기준으로]는 작업자 수정이 없으면 막는다. 요청 중이면 모두 막는다.
function updateInventoryRow(row, product) {
  const field = (name) => row.querySelector(`[data-field="${name}"]`);
  const busy = inventoryScreen.busy.has(product.object_class);
  row.className = `inventory-row ${product.status}`;
  field('chips').innerHTML = chipHtml(inventoryStatusChips[product.status] || [product.status, ''])
    + (product.source === 'closed' ? '' : sourceChipHtml(product.source));
  field('available').textContent = product.available == null ? '?' : String(product.available);
  field('total').textContent = product.on_shelves == null ? '?' : String(product.on_shelves);
  field('reserved').textContent = String(product.reserved);
  row.querySelectorAll('[data-shelf]').forEach((cell) => { cell.textContent = shelfCount(product, cell.dataset.shelf); });
  const hint = field('hint');
  hint.hidden = product.source !== 'manual';
  if (product.source === 'manual') {
    hint.textContent = `카메라 값과 상관없이 직접 센 ${product.on_shelves}개로 팔고 있어요. 선반별 수는 카메라 값이에요.`;
  }
  row.querySelector('[data-inventory-mode="closed"]').disabled = busy || product.source === 'closed';
  row.querySelector('[data-inventory-mode="auto"]').disabled = busy || !product.override;
  row.querySelector('.inventory-manual button').disabled = busy;
  const message = inventoryScreen.messages.get(product.object_class);
  field('message').hidden = !message;
  field('message').textContent = message || '';
}

// 카메라 재고(camera_stock) 순서로 줄을 만든다. 주문 상품(products)에 없는 물건은 카메라 재고만 보인다.
// 물건 구성이 바뀔 때만 다시 만든다.
function renderInventoryProducts() {
  const data = inventoryScreen.data;
  if (!data) return;
  const rows = (data.camera_stock || []).map((stock) => ({
    stock, product: data.products.find((item) => item.object_class === stock.object_class),
  }));
  const keys = rows.map(({ stock, product }) => `${stock.object_class}:${Boolean(product)}`).join('|');
  if (inventoryProducts.dataset.keys !== keys) {
    inventoryProducts.dataset.keys = keys;
    inventoryProducts.innerHTML = rows.map(({ stock, product }) => `
      <article class="inventory-row" data-object-class="${escapeHtml(stock.object_class)}" aria-label="${escapeHtml(stock.label)} 재고">
        ${product ? inventoryRowSkeleton(product) : `
          <div class="inventory-head">
            <div class="inventory-name"><h3>${escapeHtml(stock.label)}</h3></div>
            <dl class="inventory-available"><div class="stat"><dt>카메라 재고</dt><dd data-field="camera-stock">–</dd></div></dl>
          </div>`}
      </article>`).join('') || '<p class="loading">카메라 재고를 불러오는 중…</p>';
  }
  rows.forEach(({ stock, product }, index) => {
    const row = inventoryProducts.children[index];
    if (product) updateInventoryRow(row, product);
    row.querySelector('[data-field="camera-stock"]').textContent = stock.count == null ? '?' : String(stock.count);
  });
}

// ---- 최근 변화 ----

// 변화 한 줄: 작업자 수정이면 무엇으로 바꿨는지, 아니면 어느 선반에서 몇 개에서 몇 개로 바뀌었는지.
function inventoryEventText(event) {
  const label = inventoryLabel(event.object_class);
  if (event.kind === 'OVERRIDE') {
    if (event.mode === 'closed') return `${label} 판매 중지`;
    if (event.mode === 'manual') return `${label} 직접 수량 ${event.count}개`;
    return `${label} 카메라 기준으로 되돌림`;
  }
  const shelf = inventoryShelves.find(([shelfId]) => shelfId === event.shelf_id);
  return `${label}, ${shelf ? `${shelf[1]} ${shelf[2]}` : event.shelf_id} ${event.from}개에서 ${event.to}개로`;
}

function renderInventoryEvents() {
  const events = inventoryScreen.data?.events;
  if (!events) return;
  inventoryEvents.innerHTML = events.length
    ? events.map((event) => `
      <li>
        <time datetime="${escapeHtml(event.at)}">${escapeHtml(formatClock(event.at))}</time>
        ${chipHtml(inventoryEventWords[event.kind] || [event.kind, ''])}
        <span>${escapeHtml(inventoryEventText(event))}</span>
      </li>`).join('')
    : '<li class="inventory-events-empty">아직 바뀐 것이 없어요.</li>';
}

function renderInventory() {
  renderInventorySummary();
  renderInventoryProducts();
  renderInventoryEvents();
}

// ---- 읽기 ----

// GET /api/inventory(2초마다, 재고 뷰가 보일 때만). 그사이 수정 응답을 그렸으면 이 응답은 버린다.
async function refreshInventory({ force = false } = {}) {
  if (!force && (document.hidden || inventoryView.hidden || inventoryScreen.polling)) return;
  const version = inventoryScreen.version;
  inventoryScreen.polling = true;
  try {
    const response = await fetch('/api/inventory', { cache: 'no-store' });
    const result = await response.json();
    if (version !== inventoryScreen.version) return;  // 그사이 수정 응답을 그렸다. 그쪽이 더 새 값이다
    if (response.ok) {
      inventoryScreen.data = result;
      inventoryScreen.loadError = null;
    } else {
      inventoryScreen.loadError = apiErrorMessage(result, '재고를 불러오지 못했습니다');
    }
  } catch {
    if (version === inventoryScreen.version) inventoryScreen.loadError = '관제 서버에서 재고를 받지 못했습니다';
  } finally {
    inventoryScreen.polling = false;
  }
  renderInventory();
}

// ---- 작업자 수정 ----

// POST /api/inventory/{object} {mode: auto | closed | manual, count}. 성공 응답은 새 재고 전체라 바로 그린다.
async function setInventoryOverride(objectClass, body, doneText) {
  inventoryScreen.busy.add(objectClass);
  inventoryScreen.messages.delete(objectClass);
  renderInventoryProducts();
  try {
    const response = await fetch(`/api/inventory/${encodeURIComponent(objectClass)}`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
    });
    const result = await response.json();
    if (response.ok) {
      inventoryScreen.version += 1;
      inventoryScreen.data = result;
      inventoryScreen.loadError = null;
      showToast(doneText);
      return true;
    }
    inventoryScreen.messages.set(objectClass, apiErrorMessage(result, '재고를 바꾸지 못했습니다'));
  } catch {
    // 응답이 없다고 거절된 것은 아니다: 바뀐 값이 저장됐을 수 있으니, 다음 읽기가 지금 값을 보여 준다.
    inventoryScreen.messages.set(objectClass, '관제 서버의 답을 받지 못했어요. 잠시 뒤 목록에서 지금 값을 확인하세요.');
  } finally {
    inventoryScreen.busy.delete(objectClass);
    renderInventory();
  }
  return false;
}

// [판매 중지]·[카메라 기준으로] 버튼(이벤트 위임).
inventoryProducts.addEventListener('click', (event) => {
  const button = event.target.closest('[data-inventory-mode]');
  if (!button) return;
  const objectClass = button.dataset.object;
  const label = inventoryLabel(objectClass);
  const mode = button.dataset.inventoryMode;
  setInventoryOverride(objectClass, { mode }, mode === 'closed' ? `${label} 판매를 멈췄어요` : `${label} 재고를 카메라 기준으로 되돌렸어요`);
});

// 직접 수량: 0~99 정수만 보낸다. 적용되면 입력칸을 비운다.
inventoryProducts.addEventListener('submit', async (event) => {
  event.preventDefault();
  const form = event.target;
  const objectClass = form.dataset.object;
  const input = form.elements.count;
  const count = Number(input.value);
  if (input.value === '' || !Number.isInteger(count) || count < 0 || count > 99) {
    inventoryScreen.messages.set(objectClass, '직접 센 개수는 0에서 99 사이의 정수로 넣어 주세요.');
    renderInventoryProducts();
    return;
  }
  if (await setInventoryOverride(objectClass, { mode: 'manual', count }, `${inventoryLabel(objectClass)} 직접 수량 ${count}개를 적용했어요`)) {
    input.value = '';
  }
});

// 재고 뷰로 오면 바로 읽고, 그 뒤로는 2초마다 읽는다(뷰가 숨어 있으면 refreshInventory가 건너뛴다).
document.addEventListener('viewchange', (event) => { if (event.detail.view === 'inventory') refreshInventory(); });
document.addEventListener('visibilitychange', () => refreshInventory());
window.setInterval(refreshInventory, INVENTORY_POLL_MS);
refreshInventory();
