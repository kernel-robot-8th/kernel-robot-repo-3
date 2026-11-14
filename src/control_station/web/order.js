// Order app (/order): the customer side. It reads what can be ordered from GET /api/catalog, sends the order to
// the operator's intake with POST /api/orders, then follows it with GET /api/orders/{id}/track until the customer
// presses [받았어요] at the receiving zone. Nothing here starts a robot; an operator does that.

const MAX_PER_PRODUCT = 5;  // the server accepts 1 to 5 of each object
const CATALOG_REFRESH_MS = 5000;
const TRACK_REFRESH_MS = 2000;
const ORDER_KEY = 'robot-order:last';  // the order this phone placed, so a reload returns to its progress
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
  catalogFailed: false,  // the latest refresh failed; the last good catalog stays on screen
  products: [],          // [{object_class, label, available, status, notice}] in catalog order
  notice: null,          // why some objects cannot be ordered now (stock unknown), from the catalog
  zones: [],             // [{zone_id, label}]; the server picks one when the order starts, the app only needs one to exist
  cart: new Map(),       // object_class -> quantity, only quantities above 0
  adjusted: new Map(),   // object class -> 'lowered' (to the stock left) | 'removed' (closed or sold out) by a refresh
  sending: false,
  orderKey: null,        // Idempotency-Key of the order being sent; kept when no answer came back, so a resend
                         // returns the order the first try may have made instead of a second one
  orderId: null,         // the order being followed on the tracking screen
  track: null,           // the latest GET /api/orders/{id}/track answer
  trackTimer: null,
  confirming: false,
};

// localStorage can be missing or throw (private mode, blocked site data); the app then just forgets the order.
const saved = {
  get() { try { return window.localStorage.getItem(ORDER_KEY); } catch { return null; } },
  set(orderId) { try { window.localStorage.setItem(ORDER_KEY, orderId); } catch { /* not kept */ } },
  clear() { try { window.localStorage.removeItem(ORDER_KEY); } catch { /* nothing kept */ } },
};

// Builds an element; string children become text nodes, so catalog text is never parsed as HTML.
function element(tag, attributes = {}, ...children) {
  const node = document.createElement(tag);
  Object.entries(attributes).forEach(([name, value]) => node.setAttribute(name, value));
  node.append(...children);
  return node;
}

// Status words for objects that cannot be ordered now. The catalog's own notice is shown when it says more.
const closedStatus = {
  sold_out: { chip: '품절', className: 'sold-out' },
  unknown: { chip: '주문 닫힘', className: 'unknown' },
  closed: { chip: '판매 중지', className: 'closed' },
};

function orderable(product) {
  return !product.status || product.status === 'available';  // an older server sends no status
}

function maxQuantity(product) {
  if (!orderable(product)) return 0;
  return product.available == null ? MAX_PER_PRODUCT : Math.min(MAX_PER_PRODUCT, product.available);
}

function cartItems() {
  return state.products
    .filter((product) => state.cart.has(product.object_class))
    .map((product) => ({ object_class: product.object_class, label: product.label, quantity: state.cart.get(product.object_class) }));
}

// 128 random bits as hex. crypto.randomUUID needs a secure context and the app is also opened over plain http
// on the shop's network, so this uses getRandomValues, which works everywhere.
function newOrderKey() {
  const bytes = new Uint8Array(16);
  window.crypto.getRandomValues(bytes);
  return Array.from(bytes, (byte) => byte.toString(16).padStart(2, '0')).join('');
}

function setQuantity(product, quantity) {
  state.orderKey = null;  // a different cart is a different order
  const next = Math.max(0, Math.min(quantity, maxQuantity(product)));
  if (next) state.cart.set(product.object_class, next);
  else state.cart.delete(product.object_class);
  state.adjusted.delete(product.object_class);
}

// ---- catalog ----------------------------------------------------------------------------------------------

function applyCatalog(catalog) {
  state.products = catalog.products;
  state.notice = catalog.notice ?? catalog.stock_error ?? null;
  state.zones = catalog.zones || [];
  // Stock can drop or close while the customer is choosing: lower the cart to what is left and say so on the card.
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

// ---- rendering --------------------------------------------------------------------------------------------

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
  // Cards are built once per product set and then updated in place, so a refresh never steals the focus.
  const keys = state.products.map((product) => `${product.object_class}:${product.label}`).join('|');
  if (productList.dataset.keys !== keys) {
    productList.dataset.keys = keys;
    productList.replaceChildren(...state.products.map(productCard));
  }
  state.products.forEach((product, index) => updateProductCard(productList.children[index], product));
}

function renderCart() {
  const items = cartItems();
  const total = items.reduce((sum, item) => sum + item.quantity, 0);
  let summary = '담은 물건이 없어요';
  if (total) {
    summary = `${items.map((item) => `${item.label} ${item.quantity}개`).join(', ')}, 모두 ${total}개`;
  }
  cartSummary.textContent = summary;
  cartSummary.classList.toggle('filled', total > 0);
  const noZone = state.catalogLoaded && !state.zones.length;  // the server could not read its receiving zones
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

// ---- ordering ---------------------------------------------------------------------------------------------

// ---- following an order -----------------------------------------------------------------------------------

function showScreen(name) {
  orderForm.hidden = name !== 'order';
  tracking.hidden = name !== 'track';
  myOrderLink.hidden = name !== 'order' || !state.orderId;
  document.querySelector('#pageTitle').textContent = name === 'track' ? '내 주문' : '무엇을 가져다 드릴까요?';
  document.querySelector('#pageLead').hidden = name === 'track';
}

function eyebrow(track) {
  if (track.stage === 'canceled' || track.stage === 'problem') return '';
  if (track.stage === 'done') return '배달 완료';
  return track.pinky ? `배달 로봇 · 핑키 ${track.pinky}` : '배달 로봇 배정 전';
}

function renderTrack() {
  const track = state.track;
  if (!track) return;
  document.querySelector('#track-title').textContent = track.title;
  document.querySelector('#trackDetail').textContent = track.detail;
  document.querySelector('#trackPinky').textContent = eyebrow(track);
  tracking.dataset.stage = track.stage;
  // canceled / problem orders have no place on the line; the stages stay as they were, the title says why
  document.querySelector('#trackStages').replaceChildren(...track.stages.map((stage) => element('li',
    { class: `track-stage ${stage.state}`, ...(stage.state === 'current' ? { 'aria-current': 'step' } : {}) },
    element('span', { class: 'track-dot', 'aria-hidden': 'true' }), element('span', { class: 'track-label' }, stage.label))));
  document.querySelector('#trackItems').replaceChildren(
    ...track.items.map((item) => element('li', {}, `${item.label} ${item.quantity}개`)));
  // A larger order comes in several loads, one after another; each one is picked up on its own.
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

async function loadTrack() {
  if (!state.orderId || (document.hidden && state.track)) return;
  const orderId = state.orderId;
  try {
    const response = await fetch(`/api/orders/${encodeURIComponent(orderId)}/track`, { cache: 'no-store' });
    if (orderId !== state.orderId) return;  // the customer moved on while this was in flight
    if (response.status === 404) {
      forgetOrder();  // the operator's database no longer has it (reset); start fresh instead of spinning
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

function forgetOrder() {
  stopTracking();
  saved.clear();
  state.orderId = null;
  state.track = null;
  showScreen('order');
  render();
}

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
      body: JSON.stringify({ items }),  // the receiving zone is the server's choice, made when the order starts
    });
    const result = await response.json().catch(() => ({}));
    if (!response.ok) {
      state.orderKey = null;  // the server answered: no order was made, the next try is a new request
      // OUT_OF_STOCK and ITEM_CLOSED name every refused object in one message; refresh so the cards agree.
      showError(result.message || result.error || '주문을 접수하지 못했어요.');
      if (['OUT_OF_STOCK', 'ITEM_CLOSED'].includes(result.error_code)) loadCatalog();
      return;
    }
    state.cart.clear();
    state.adjusted.clear();
    state.orderKey = null;
    followOrder(result.order.id);
  } catch {
    // The request may have reached the server. The key is kept, so pressing again finds that order, not a second.
    showError('응답을 받지 못했어요. 주문하기를 다시 누르면 같은 주문인지 확인해요 — 두 번 접수되지 않아요.');
  } finally {
    state.sending = false;
    render();
  }
});

document.querySelector('#newOrder').addEventListener('click', () => {
  // A running order keeps going; the customer can come back to it with "내 주문 보기".
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

document.addEventListener('visibilitychange', () => {
  if (document.hidden) return;
  loadCatalog();
  if (!tracking.hidden) loadTrack();
});

// /order?order=<id> opens that order's progress (a link or QR handed to the customer); otherwise this phone's last.
const linkedOrder = new URLSearchParams(window.location.search).get('order');
const lastOrder = linkedOrder || saved.get();
if (lastOrder) followOrder(lastOrder, { focus: false });
loadCatalog();
window.setInterval(loadCatalog, CATALOG_REFRESH_MS);

if ('serviceWorker' in navigator && window.isSecureContext) {
  navigator.serviceWorker.register('/order-sw.js', { scope: '/order' }).catch(() => { /* the app works without it */ });
}
