// 재고 화면: what the shelves hold per object, what open orders have claimed, what can still be sold, and the
// operator's override per object (stop selling, sell from a hand count, back to the camera).
//
// Loaded after app.js and uses its helpers: escapeHtml, apiErrorMessage, formatClock, showToast, setStat.
//
// GET /api/inventory is read every 2 s while this view is on screen. POST /api/inventory/{object} answers with the
// new snapshot, which is drawn at once. An unknown count is shown as "?" and never as 0: the server closes those
// objects for ordering, and the screen says so.

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
  data: null,           // last GET (or POST) answer
  loadError: null,      // why the last GET failed; the last answer stays on screen
  polling: false,       // a GET is in flight; the timer skips instead of piling up requests
  version: 0,           // bumped when a POST answer is drawn, so a GET sent before it cannot draw an older state
  busy: new Set(),      // object classes with an override request in flight
  messages: new Map(),  // object class -> why its last override was refused
};

// [word, chip tone] for each product status and count source.
const inventoryStatusChips = {
  available: ['있음', 'good'], sold_out: ['품절', 'danger'], unknown: ['모름', 'attention'], closed: ['판매 중지', 'dark'],
};
const inventorySourceChips = {
  camera: ['카메라', 'active'], manual: ['직접 수량', 'attention'], closed: ['판매 중지', 'dark'],
};
// The two arms each serve one shelf (config/arms.json); any other shelf in the answer is shown by its id.
const inventoryShelves = [['shelf_a', '선반 A', '왼쪽 팔'], ['shelf_b', '선반 B', '오른쪽 팔']];
const inventoryEventWords = {
  RESTOCKED: ['채움', 'good'], REMOVED_OUTSIDE: ['관제 밖 감소', 'attention'], OVERRIDE: ['작업자 수정', ''],
};

function inventoryLabel(objectClass) {
  return inventoryScreen.data?.products.find((product) => product.object_class === objectClass)?.label || objectClass;
}

function chipHtml([word, tone]) {
  return `<span class="chip${tone ? ` ${tone}` : ''}">${escapeHtml(word)}</span>`;
}

function sourceChipHtml(source) {
  if (source === 'mock_world') return '<span class="sim-tag">모의</span>';
  return chipHtml(inventorySourceChips[source] || [source, '']);
}

// ---- summary ----

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
      live: data.simulated ? '모의 세계에서 센 재고예요 <span class="sim-tag">모의</span>' : '카메라로 재고를 확인하고 있어요',
      held: '팔이 작업 중이라 마지막 확인값을 보여 줘요',
      unknown: '재고를 확인하지 못했어요',
    }[data.state] || escapeHtml(data.state);
    inventorySummaryDetail.textContent = `${formatClock(data.checked_at)}에 확인했어요`;
  }
  // Unknown stock closes those objects for ordering; a person should look at the shelf camera.
  inventoryBanner.hidden = data.state !== 'unknown';
  if (data.state === 'unknown') {
    const names = unknown.map((product) => product.label).join(', ');
    inventoryBanner.innerHTML = `<strong>${names ? `${escapeHtml(names)}의 주문을 닫았어요` : '카메라 재고를 쓸 수 없어요'}</strong>
      <span>${escapeHtml(data.reason || '선반 카메라의 새 값이 없습니다')}${names ? '' : '. 직접 수량을 정한 물건만 주문을 받아요'}</span>`;
  }
}

// ---- product rows: built once per product, then updated in place so typing in a count is never interrupted ----

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
      <dl class="inventory-available"><div class="stat"><dt>주문 가능</dt><dd data-field="available">–</dd></div></dl>
    </div>
    <dl class="inventory-counts">
      ${shelves}
      <div class="stat"><dt>선반 합계</dt><dd data-field="total">–</dd></div>
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
  // While the camera count is unknown the last per-shelf numbers are stale; show them as unknown too.
  if (product.status === 'unknown') return '?';
  return String(product.per_shelf[shelfId] || 0);
}

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

function renderInventoryProducts() {
  const products = inventoryScreen.data?.products;
  if (!products) return;
  if (!products.length) {
    inventoryProducts.dataset.keys = '';
    inventoryProducts.innerHTML = '<p class="loading">주문할 수 있는 물건이 없어요. 정책 정의 파일을 확인하세요.</p>';
    return;
  }
  const keys = products.map((product) => `${product.object_class}:${product.label}`).join('|');
  if (inventoryProducts.dataset.keys !== keys) {
    inventoryProducts.dataset.keys = keys;
    inventoryProducts.innerHTML = products.map((product) => `
      <article class="inventory-row" data-object-class="${escapeHtml(product.object_class)}" aria-label="${escapeHtml(product.label)} 재고">
        ${inventoryRowSkeleton(product)}
      </article>`).join('');
  }
  products.forEach((product, index) => updateInventoryRow(inventoryProducts.children[index], product));
}

// ---- recent changes ----

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

// ---- reading ----

async function refreshInventory({ force = false } = {}) {
  if (!force && (document.hidden || inventoryView.hidden || inventoryScreen.polling)) return;
  const version = inventoryScreen.version;
  inventoryScreen.polling = true;
  try {
    const response = await fetch('/api/inventory', { cache: 'no-store' });
    const result = await response.json();
    if (version !== inventoryScreen.version) return;  // an override answer was drawn meanwhile; it is newer
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

// ---- operator overrides ----

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
    // No answer is not a refusal: the change may have been stored, so the next read shows what holds.
    inventoryScreen.messages.set(objectClass, '관제 서버의 답을 받지 못했어요. 잠시 뒤 목록에서 지금 값을 확인하세요.');
  } finally {
    inventoryScreen.busy.delete(objectClass);
    renderInventory();
  }
  return false;
}

inventoryProducts.addEventListener('click', (event) => {
  const button = event.target.closest('[data-inventory-mode]');
  if (!button) return;
  const objectClass = button.dataset.object;
  const label = inventoryLabel(objectClass);
  const mode = button.dataset.inventoryMode;
  setInventoryOverride(objectClass, { mode }, mode === 'closed' ? `${label} 판매를 멈췄어요` : `${label} 재고를 카메라 기준으로 되돌렸어요`);
});

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

document.addEventListener('viewchange', (event) => { if (event.detail.view === 'inventory') refreshInventory(); });
document.addEventListener('visibilitychange', () => refreshInventory());
window.setInterval(refreshInventory, INVENTORY_POLL_MS);
refreshInventory();
