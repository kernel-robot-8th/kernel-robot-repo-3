// 주문 앱(/order, order.html): 고객 쪽 화면. 주문할 수 있는 것을 GET /api/catalog로 읽고, POST /api/orders로
// 작업자의 주문 접수에 보낸 뒤, 고객이 수령 구역에서 [받았어요]를 누를 때까지 GET /api/orders/{id}/track으로
// 따라간다. 여기서는 로봇을 출발시키지 않는다. 그것은 작업자가 한다.
// 관제 화면 스크립트와 따로 동작하는 단독 스크립트이고(app.js를 쓰지 않음), 서비스 워커 order-sw.js를 등록한다.

const MAX_PER_PRODUCT = 5;  // 서버는 물건마다 1~5개를 받는다
// 카탈로그(남은 수량)는 5초마다, 주문 추적은 2초마다 읽는다.
const CATALOG_REFRESH_MS = 5000;
const TRACK_REFRESH_MS = 2000;
const ORDER_KEY = 'robot-order:last';  // 이 휴대폰이 넣은 주문. 새로고침해도 그 진행 화면으로 돌아온다
// 추적을 멈추는 단계.
const FINISHED_STAGES = ['done', 'canceled'];

const orderForm = document.querySelector('#orderForm');
const productList = document.querySelector('#products');
const stockNotice = document.querySelector('#stockNotice');
const cartNote = document.querySelector('#cartNote');
const cartSummary = document.querySelector('#cartSummary');
const submitButton = document.querySelector('#submitOrder');
const formError = document.querySelector('#formError');
const tracking = document.querySelector('#tracking');
const myOrderLink = document.querySelector('#myOrderLink');
const pickupButton = document.querySelector('#pickupButton');
const trackError = document.querySelector('#trackError');

const state = {
  catalogLoaded: false,
  catalogFailed: false,  // 마지막 새로 읽기가 실패했다. 마지막으로 받은 카탈로그가 화면에 남는다
  products: [],          // 카탈로그 순서의 [{object_class, label, available, status, notice}]
  notice: null,          // 지금 일부 물건을 주문할 수 없는 이유(재고 모름), 카탈로그에서 옴
  zones: [],             // [{zone_id, label}], 서버가 주문 시작 때 고르므로 앱은 하나라도 있는지만 본다
  cart: new Map(),       // object_class -> 수량, 0보다 큰 것만
  adjusted: new Map(),   // object class -> 새로 읽기로 'lowered'(남은 수량으로 줄임) | 'removed'(닫힘·품절로 뺌)
  sending: false,        // 주문을 보내는 중(버튼과 수량 조절을 막는다)
  orderKey: null,        // 보내는 주문의 Idempotency-Key. 응답을 못 받았으면 남겨 두어, 다시 보낼 때
                         // 두 번째 주문을 만들지 않고 첫 시도가 만들었을지 모를 주문을 돌려받게 한다
  orderId: null,         // 추적 화면에서 따라가는 주문
  track: null,           // 마지막 GET /api/orders/{id}/track 응답
  trackTimer: null,      // 추적 setInterval id
  confirming: false,     // [받았어요]를 보내는 중
};

// localStorage는 없거나 예외를 던질 수 있다(사생활 보호 모드, 사이트 데이터 차단). 그러면 앱은 주문을 기억하지 않을 뿐이다.
const saved = {
  get() { try { return window.localStorage.getItem(ORDER_KEY); } catch { return null; } },
  set(orderId) { try { window.localStorage.setItem(ORDER_KEY, orderId); } catch { /* not kept */ } },
  clear() { try { window.localStorage.removeItem(ORDER_KEY); } catch { /* nothing kept */ } },
};

// 요소를 만든다. 문자열 자식은 텍스트 노드가 되므로 카탈로그 글이 HTML로 해석되는 일이 없다.
function element(tag, attributes = {}, ...children) {
  const node = document.createElement(tag);
  Object.entries(attributes).forEach(([name, value]) => node.setAttribute(name, value));
  node.append(...children);
  return node;
}

// 지금 주문할 수 없는 물건의 상태 말. 카탈로그의 notice가 더 많은 것을 말하면 그것도 보인다.
const closedStatus = {
  sold_out: { chip: '품절', className: 'sold-out' },
  unknown: { chip: '주문 닫힘', className: 'unknown' },
  closed: { chip: '판매 중지', className: 'closed' },
};

function orderable(product) {
  return !product.status || product.status === 'available';  // 예전 서버는 status를 보내지 않는다
}

// 담을 수 있는 최대 수량: 주문 불가면 0, 남은 수량을 모르면 5, 아니면 남은 수량과 5 중 작은 값.
function maxQuantity(product) {
  if (!orderable(product)) return 0;
  return product.available == null ? MAX_PER_PRODUCT : Math.min(MAX_PER_PRODUCT, product.available);
}

function cartItems() {
  return state.products
    .filter((product) => state.cart.has(product.object_class))
    .map((product) => ({ object_class: product.object_class, label: product.label, quantity: state.cart.get(product.object_class) }));
}

// 128비트 난수를 16진수로. crypto.randomUUID는 보안 컨텍스트가 필요한데 앱은 가게 네트워크에서 일반 http로도
// 열리므로, 어디서나 되는 getRandomValues를 쓴다.
function newOrderKey() {
  const bytes = new Uint8Array(16);
  window.crypto.getRandomValues(bytes);
  return Array.from(bytes, (byte) => byte.toString(16).padStart(2, '0')).join('');
}

function setQuantity(product, quantity) {
  state.orderKey = null;  // 장바구니가 달라지면 다른 주문이다
  const next = Math.max(0, Math.min(quantity, maxQuantity(product)));
  if (next) state.cart.set(product.object_class, next);
  else state.cart.delete(product.object_class);
  state.adjusted.delete(product.object_class);
}

// ---- 카탈로그 ---------------------------------------------------------------------------------------------

// 카탈로그를 반영한다. notice가 없으면 예전 필드 stock_error를 쓴다.
function applyCatalog(catalog) {
  state.products = catalog.products;
  state.notice = catalog.notice ?? catalog.stock_error ?? null;
  state.zones = catalog.zones || [];
  // 고객이 고르는 사이 재고가 줄거나 닫힐 수 있다: 장바구니를 남은 만큼으로 줄이고 카드에 그렇게 적는다.
  for (const [objectClass, quantity] of state.cart) {
    const product = state.products.find((item) => item.object_class === objectClass);
    const max = product ? maxQuantity(product) : 0;
    if (quantity <= max) continue;
    if (max) state.cart.set(objectClass, max);
    else state.cart.delete(objectClass);
    state.adjusted.set(objectClass, max ? 'lowered' : 'removed');
    state.orderKey = null;
  }
}

// GET /api/catalog(5초마다). 탭이 숨어 있으면 첫 로드 뒤에는 건너뛴다.
async function loadCatalog() {
  if (document.hidden && state.catalogLoaded) return;
  try {
    const response = await fetch('/api/catalog', { cache: 'no-store' });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    applyCatalog(await response.json());
    state.catalogLoaded = true;
    state.catalogFailed = false;
  } catch {
    state.catalogFailed = true;
  }
  render();
}

// ---- 그리기 -----------------------------------------------------------------------------------------------

// 위 안내 띠: 카탈로그를 못 읽음, 또는 일부·전부 주문할 수 없는 이유.
function renderNotice() {
  let message = null;
  let detail = null;
  if (state.catalogFailed) {
    message = state.catalogLoaded
      ? '남은 수량을 새로 불러오지 못했어요. 잠시 뒤 다시 확인합니다.'
      : '물건 목록을 불러오지 못했어요. 잠시 뒤 다시 시도합니다.';
  } else if (state.notice) {
    message = state.products.length ? state.notice : '지금은 주문을 받을 수 없어요.';
    detail = state.products.length ? null : state.notice;
  }
  stockNotice.hidden = !message;
  stockNotice.classList.toggle('error', state.catalogFailed || !state.products.length);
  stockNotice.replaceChildren(...(message ? [message] : []), ...(detail ? [element('br'), element('small', {}, detail)] : []));
}

// 물건 카드 뼈대: 첫 글자 표시, 이름, 재고 줄, 조정 안내, −/+ 수량 조절.
function productCard(product) {
  const name = product.label;
  return element('article', { class: 'product', 'data-object-class': product.object_class },
    element('span', { class: 'product-mark', 'aria-hidden': 'true' }, [...name][0] || '?'),
    element('div', { class: 'product-info' },
      element('h3', { class: 'product-name' }, name),
      element('p', { class: 'product-stock' }),
      element('p', { class: 'product-adjusted', hidden: '' })),
    element('div', { class: 'stepper', role: 'group', 'aria-label': `${name} 수량` },
      element('button', { type: 'button', class: 'step', 'data-step': '-1', 'aria-label': `${name} 하나 빼기` }, '−'),
      element('output', { class: 'step-value', 'aria-live': 'polite' }, '0'),
      element('button', { type: 'button', class: 'step', 'data-step': '1', 'aria-label': `${name} 하나 더하기` }, '+')));
}

// 카드 값 갱신: 재고 표시, 선택·주문 불가 표시, 수량. 보내는 중이거나 0·최대면 −/+를 막는다.
function updateProductCard(card, product) {
  const quantity = state.cart.get(product.object_class) || 0;
  const max = maxQuantity(product);
  const stock = card.querySelector('.product-stock');
  const closed = orderable(product) ? null : (closedStatus[product.status] || { chip: '주문 닫힘', className: 'unknown' });
  if (closed) {
    const notice = product.notice && product.notice !== closed.chip ? product.notice : '';
    stock.replaceChildren(element('span', { class: `status-chip ${closed.className}` }, closed.chip), ...(notice ? [notice] : []));
    stock.className = 'product-stock closed';
  } else if (product.available == null) {
    stock.textContent = '남은 수량은 작업자가 확인해요';
  } else if (product.available === 0) {
    stock.replaceChildren(element('span', { class: 'status-chip sold-out' }, '품절'));
    stock.className = 'product-stock closed';
  } else {
    stock.textContent = `${product.available}개 남았어요`;
    stock.className = 'product-stock';
  }
  card.classList.toggle('selected', quantity > 0);
  card.classList.toggle('unavailable', max === 0);
  const adjusted = state.adjusted.get(product.object_class);
  const adjustedNote = card.querySelector('.product-adjusted');
  adjustedNote.hidden = !adjusted;
  adjustedNote.textContent = adjusted === 'removed' ? '지금은 주문할 수 없어서 담은 물건에서 뺐어요' : '남은 수량에 맞춰 줄였어요';
  card.querySelector('.step-value').textContent = String(quantity);
  card.querySelector('[data-step="-1"]').disabled = state.sending || quantity === 0;
  card.querySelector('[data-step="1"]').disabled = state.sending || quantity >= max;
}

function renderProducts() {
  if (!state.catalogLoaded || !state.products.length) {
    const text = !state.catalogLoaded
      ? (state.catalogFailed ? '물건 목록을 불러오지 못했어요' : '물건 목록을 불러오는 중…')
      : '지금 주문할 수 있는 물건이 없어요';
    productList.dataset.keys = '';
    productList.replaceChildren(element('p', { class: 'placeholder' }, text));
    return;
  }
  // 카드는 물건 구성마다 한 번 만들고 그다음엔 그 자리에서 바꾼다. 그래서 새로 읽어도 포커스를 빼앗지 않는다.
  const keys = state.products.map((product) => `${product.object_class}:${product.label}`).join('|');
  if (productList.dataset.keys !== keys) {
    productList.dataset.keys = keys;
    productList.replaceChildren(...state.products.map(productCard));
  }
  state.products.forEach((product, index) => updateProductCard(productList.children[index], product));
}

// 장바구니 줄과 [주문하기]. 서버가 수령 구역을 하나도 못 읽었으면 주문을 막는다.
function renderCart() {
  const items = cartItems();
  const total = items.reduce((sum, item) => sum + item.quantity, 0);
  let summary = '담은 물건이 없어요';
  if (total) {
    summary = `${items.map((item) => `${item.label} ${item.quantity}개`).join(', ')}, 모두 ${total}개`;
  }
  cartSummary.textContent = summary;
  cartSummary.classList.toggle('filled', total > 0);
  const noZone = state.catalogLoaded && !state.zones.length;  // 서버가 수령 구역을 읽지 못했다
  cartNote.textContent = noZone ? '받을 곳이 정해지지 않아 지금은 주문할 수 없어요' : '받을 곳은 로봇이 출발할 때 알려 드려요';
  cartNote.classList.toggle('error', noZone);
  submitButton.disabled = state.sending || !total || noZone;
  submitButton.textContent = state.sending ? '보내는 중…' : '주문하기';
}

function render() {
  renderNotice();
  renderProducts();
  renderCart();
}

function showError(message) {
  formError.textContent = message;
  formError.hidden = false;
}

function clearError() {
  formError.hidden = true;
  formError.replaceChildren();
}

// ---- 주문하기 (제출 처리는 아래 orderForm의 submit) -----------------------------------------------------

// ---- 주문 따라가기 ----------------------------------------------------------------------------------------

// 주문 화면('order')과 추적 화면('track')을 바꾼다. 따라갈 주문이 있으면 주문 화면에 [내 주문 보기]를 보인다.
function showScreen(name) {
  orderForm.hidden = name !== 'order';
  tracking.hidden = name !== 'track';
  myOrderLink.hidden = name !== 'order' || !state.orderId;
  document.querySelector('#pageTitle').textContent = name === 'track' ? '내 주문' : '무엇을 가져다 드릴까요?';
  document.querySelector('#pageLead').hidden = name === 'track';
}

// 추적 제목 위 작은 줄: 배달 로봇(핑키) 이름이나 배달 완료.
function eyebrow(track) {
  if (track.stage === 'canceled' || track.stage === 'problem') return '';
  if (track.stage === 'done') return '배달 완료';
  return track.pinky ? `배달 로봇 · 핑키 ${track.pinky}` : '배달 로봇 배정 전';
}

// 추적 화면: 제목·설명, 단계 줄, 물건, 운송분(큰 주문), 받을 곳, 주문 번호, [받았어요](수령 확인할 수 있을 때만).
function renderTrack() {
  const track = state.track;
  if (!track) return;
  document.querySelector('#track-title').textContent = track.title;
  document.querySelector('#trackDetail').textContent = track.detail;
  document.querySelector('#trackPinky').textContent = eyebrow(track);
  tracking.dataset.stage = track.stage;
  // 취소·문제가 생긴 주문은 단계 줄에 자리가 없다. 단계는 그대로 두고 제목이 이유를 말한다.
  document.querySelector('#trackStages').replaceChildren(...track.stages.map((stage) => element('li',
    { class: `track-stage ${stage.state}`, ...(stage.state === 'current' ? { 'aria-current': 'step' } : {}) },
    element('span', { class: 'track-dot', 'aria-hidden': 'true' }), element('span', { class: 'track-label' }, stage.label))));
  document.querySelector('#trackItems').replaceChildren(
    ...track.items.map((item) => element('li', {}, `${item.label} ${item.quantity}개`)));
  // 큰 주문은 여러 번에 나눠 차례로 온다. 운송분마다 따로 받는다.
  const split = (track.shipments || 1) > 1 && !['canceled', 'problem'].includes(track.stage);
  const shipmentChip = document.querySelector('#trackShipment');
  shipmentChip.hidden = !split;
  shipmentChip.textContent = split ? `운송 ${track.shipment}/${track.shipments}` : '';
  document.querySelector('#trackShipmentRow').hidden = !split || track.stage === 'done' || !track.shipment_items;
  document.querySelector('#trackShipmentItems').replaceChildren(
    ...(split && track.shipment_items ? track.shipment_items : []).map((item) => element('li', {}, `${item.label} ${item.quantity}개`)));
  document.querySelector('#trackItemsLabel').textContent = split ? '주문 전체' : '물건';
  const trackZone = document.querySelector('#trackZone');
  trackZone.textContent = track.zone || '출발할 때 정해져요';
  trackZone.classList.toggle('pending', !track.zone);
  document.querySelector('#trackId').textContent = track.order_id;
  pickupButton.hidden = !track.can_confirm_pickup;
  pickupButton.disabled = state.confirming;
  pickupButton.textContent = state.confirming ? '보내는 중…' : '받았어요';
  const finished = FINISHED_STAGES.includes(track.stage);
  document.querySelector('#newOrder').classList.toggle('primary', finished);
  document.querySelector('#newOrder').classList.toggle('secondary', !finished);
}

function showTrackError(message) {
  trackError.textContent = message;
  trackError.hidden = !message;
}

// GET /api/orders/{id}/track(2초마다). 끝난 단계면 추적을 멈춘다.
async function loadTrack() {
  if (!state.orderId || (document.hidden && state.track)) return;
  const orderId = state.orderId;
  try {
    const response = await fetch(`/api/orders/${encodeURIComponent(orderId)}/track`, { cache: 'no-store' });
    if (orderId !== state.orderId) return;  // 응답을 기다리는 사이 고객이 다른 화면으로 갔다
    if (response.status === 404) {
      forgetOrder();  // 관제 데이터베이스에 더는 없다(초기화). 계속 돌지 말고 처음부터 시작한다
      return;
    }
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    state.track = await response.json();
    showTrackError('');
    renderTrack();
    if (FINISHED_STAGES.includes(state.track.stage)) stopTracking();
  } catch {
    showTrackError('진행 상황을 새로 불러오지 못했어요. 잠시 뒤 다시 확인합니다.');
  }
}

function stopTracking() {
  window.clearInterval(state.trackTimer);
  state.trackTimer = null;
}

// 주문 하나를 따라가기 시작한다: 기억해 두고, 추적 화면으로 바꾸고, 바로 읽은 뒤 2초마다 읽는다.
function followOrder(orderId, { focus = true } = {}) {
  state.orderId = orderId;
  state.track = null;
  saved.set(orderId);
  showTrackError('');
  document.querySelector('#track-title').textContent = '주문을 불러오는 중…';
  document.querySelector('#trackDetail').textContent = '';
  showScreen('track');
  stopTracking();
  loadTrack();
  state.trackTimer = window.setInterval(loadTrack, TRACK_REFRESH_MS);
  if (focus) document.querySelector('#track-title').focus();
}

// 따라가던 주문을 잊고 주문 화면으로 돌아간다.
function forgetOrder() {
  stopTracking();
  saved.clear();
  state.orderId = null;
  state.track = null;
  showScreen('order');
  render();
}

// [받았어요]: POST /api/orders/{id}/pickup-confirm. 그러면 핑키가 주차장으로 돌아간다.
pickupButton.addEventListener('click', async () => {
  if (!state.orderId || state.confirming) return;
  state.confirming = true;
  renderTrack();
  try {
    const response = await fetch(`/api/orders/${encodeURIComponent(state.orderId)}/pickup-confirm`, { method: 'POST' });
    const result = await response.json().catch(() => ({}));
    if (!response.ok) showTrackError(result.message || '수령 확인을 보내지 못했어요. 다시 눌러 주세요.');
  } catch {
    showTrackError('수령 확인을 보내지 못했어요. 다시 눌러 주세요.');
  } finally {
    state.confirming = false;
    await loadTrack();
    renderTrack();
  }
});

// 수량 −/+ 버튼(이벤트 위임).
productList.addEventListener('click', (event) => {
  const button = event.target.closest('[data-step]');
  if (!button) return;
  const objectClass = button.closest('[data-object-class]').dataset.objectClass;
  const product = state.products.find((item) => item.object_class === objectClass);
  if (!product) return;
  setQuantity(product, (state.cart.get(objectClass) || 0) + Number(button.dataset.step));
  clearError();
  render();
});

// [주문하기]: POST /api/orders에 Idempotency-Key 머리글을 붙여 보낸다. 같은 키로 다시 보내면 서버는 이미 만든
// 주문을 돌려주므로(200 replayed) 응답을 못 받아 다시 눌러도 두 번 접수되지 않는다.
orderForm.addEventListener('submit', async (event) => {
  event.preventDefault();
  const items = cartItems().map(({ object_class, quantity }) => ({ object_class, quantity }));
  if (!items.length || !state.zones.length || state.sending) return;
  state.sending = true;
  state.orderKey = state.orderKey || newOrderKey();
  clearError();
  render();
  try {
    const response = await fetch('/api/orders', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', 'Idempotency-Key': state.orderKey },
      body: JSON.stringify({ items }),  // 수령 구역은 서버가 주문을 시작할 때 고른다
    });
    const result = await response.json().catch(() => ({}));
    if (!response.ok) {
      state.orderKey = null;  // 서버가 답했다: 주문은 만들어지지 않았으니 다음 시도는 새 요청이다
      // OUT_OF_STOCK과 ITEM_CLOSED는 거절된 물건을 모두 한 메시지에 담는다. 카드가 맞도록 카탈로그를 새로 읽는다.
      showError(result.message || result.error || '주문을 접수하지 못했어요.');
      if (['OUT_OF_STOCK', 'ITEM_CLOSED'].includes(result.error_code)) loadCatalog();
      return;
    }
    state.cart.clear();
    state.adjusted.clear();
    state.orderKey = null;
    followOrder(result.order.id);
  } catch {
    // 요청이 서버에 닿았을 수 있다. 키를 남겨 두므로 다시 누르면 두 번째 주문이 아니라 그 주문을 찾는다.
    showError('응답을 받지 못했어요. 주문하기를 다시 누르면 같은 주문인지 확인해요 — 두 번 접수되지 않아요.');
  } finally {
    state.sending = false;
    render();
  }
});

document.querySelector('#newOrder').addEventListener('click', () => {
  // 진행 중인 주문은 계속된다. 고객은 "내 주문 보기"로 그 주문에 돌아올 수 있다.
  const running = state.track && !FINISHED_STAGES.includes(state.track.stage);
  if (running) {
    stopTracking();
    showScreen('order');
  } else {
    forgetOrder();
  }
  render();
  productList.querySelector('[data-step="1"]:not(:disabled)')?.focus();
});

myOrderLink.addEventListener('click', () => {
  if (state.orderId) followOrder(state.orderId);
});

// 탭으로 돌아오면 카탈로그와 (추적 중이면) 주문을 바로 읽는다.
document.addEventListener('visibilitychange', () => {
  if (document.hidden) return;
  loadCatalog();
  if (!tracking.hidden) loadTrack();
});

// /order?order=<id>는 그 주문의 진행을 연다(고객에게 건넨 링크나 QR). 아니면 이 휴대폰의 마지막 주문을 연다.
const linkedOrder = new URLSearchParams(window.location.search).get('order');
const lastOrder = linkedOrder || saved.get();
if (lastOrder) followOrder(lastOrder, { focus: false });
loadCatalog();
window.setInterval(loadCatalog, CATALOG_REFRESH_MS);

// 서비스 워커(order-sw.js)는 보안 컨텍스트(https·localhost)에서만 등록할 수 있다.
if ('serviceWorker' in navigator && window.isSecureContext) {
  navigator.serviceWorker.register('/order-sw.js', { scope: '/order' }).catch(() => { /* 없어도 앱은 동작한다 */ });
}
