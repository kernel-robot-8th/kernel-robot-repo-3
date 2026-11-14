// Order window (/order): a customer-side mock. It reads what can be ordered from GET /api/catalog and sends the
// order to the operator's intake with POST /api/orders. Nothing here starts a robot; an operator does that.

const MAX_PER_PRODUCT = 5;  // the server accepts 1 to 5 of each object
const CATALOG_REFRESH_MS = 5000;

const orderForm = document.querySelector('#orderForm');
const productList = document.querySelector('#products');
const stockNotice = document.querySelector('#stockNotice');
const zoneList = document.querySelector('#zones');
const cartSummary = document.querySelector('#cartSummary');
const submitButton = document.querySelector('#submitOrder');
const formError = document.querySelector('#formError');
const orderDone = document.querySelector('#orderDone');

const state = {
  catalogLoaded: false,
  catalogFailed: false,  // the latest refresh failed; the last good catalog stays on screen
  products: [],          // [{object_class, label, available, status, notice}] in catalog order
  notice: null,          // why some objects cannot be ordered now (stock unknown), from the catalog
  zones: [],             // [{zone_id, label}]
  cart: new Map(),       // object_class -> quantity, only quantities above 0
  adjusted: new Map(),   // object class -> 'lowered' (to the stock left) | 'removed' (closed or sold out) by a refresh
  zoneId: null,
  sending: false,
};

const clockFormat = new Intl.DateTimeFormat('ko-KR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });

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

function labelFor(objectClass) {
  return state.products.find((product) => product.object_class === objectClass)?.label || objectClass;
}

function cartItems() {
  return state.products
    .filter((product) => state.cart.has(product.object_class))
    .map((product) => ({ object_class: product.object_class, label: product.label, quantity: state.cart.get(product.object_class) }));
}

function setQuantity(product, quantity) {
  const next = Math.max(0, Math.min(quantity, maxQuantity(product)));
  if (next) state.cart.set(product.object_class, next);
  else state.cart.delete(product.object_class);
  state.adjusted.delete(product.object_class);
}

// ---- catalog ----------------------------------------------------------------------------------------------

function applyCatalog(catalog) {
  state.products = catalog.products;
  state.notice = catalog.notice ?? catalog.stock_error ?? null;
  state.zones = catalog.zones;
  // Stock can drop or close while the customer is choosing: lower the cart to what is left and say so on the card.
  for (const [objectClass, quantity] of state.cart) {
    const product = state.products.find((item) => item.object_class === objectClass);
    const max = product ? maxQuantity(product) : 0;
    if (quantity <= max) continue;
    if (max) state.cart.set(objectClass, max);
    else state.cart.delete(objectClass);
    state.adjusted.set(objectClass, max ? 'lowered' : 'removed');
  }
  if (!state.zones.some((zone) => zone.zone_id === state.zoneId)) {
    state.zoneId = state.zones.length === 1 ? state.zones[0].zone_id : null;
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

function zoneChoices() {
  if (!state.catalogLoaded) return [];
  if (!state.zones.length) {
    return [element('p', { class: 'notice error' }, '받을 곳이 정해지지 않아 지금은 주문할 수 없어요')];
  }
  if (state.zones.length === 1) {
    return [element('span', { class: 'zone-chip fixed' }, state.zones[0].label),
      element('span', { class: 'zone-fixed-note' }, '지금은 이곳에서만 받을 수 있어요')];
  }
  return state.zones.map((zone) => element('label', { class: 'zone-chip' },
    element('input', { type: 'radio', name: 'receiving_zone', value: zone.zone_id }), zone.label));
}

function renderZones() {
  const keys = `${state.catalogLoaded}|${state.zones.map((zone) => `${zone.zone_id}:${zone.label}`).join('|')}`;
  if (zoneList.dataset.keys !== keys) {
    zoneList.dataset.keys = keys;
    zoneList.replaceChildren(...zoneChoices());
  }
  zoneList.querySelectorAll('input[name="receiving_zone"]').forEach((input) => {
    input.checked = input.value === state.zoneId;
    input.disabled = state.sending;
    input.closest('.zone-chip').classList.toggle('selected', input.checked);
  });
}

function renderCart() {
  const items = cartItems();
  const total = items.reduce((sum, item) => sum + item.quantity, 0);
  let summary = '담은 물건이 없어요';
  if (total) {
    summary = `${items.map((item) => `${item.label} ${item.quantity}개`).join(', ')}, 모두 ${total}개`;
    if (!state.zoneId && state.zones.length) summary += '. 받을 곳을 골라 주세요';
  }
  cartSummary.textContent = summary;
  cartSummary.classList.toggle('filled', total > 0);
  submitButton.disabled = state.sending || !total || !state.zoneId;
  submitButton.textContent = state.sending ? '보내는 중…' : '주문하기';
}

function render() {
  renderNotice();
  renderProducts();
  renderZones();
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

function showConfirmation(order) {
  const zone = state.zones.find((item) => item.zone_id === order.receiving_zone);
  document.querySelector('#doneId').textContent = order.id;
  document.querySelector('#doneItems').replaceChildren(
    ...order.order_items.map((item) => element('li', {}, `${labelFor(item.object_class)} ${item.quantity}개`)));
  document.querySelector('#doneZone').textContent = zone?.label || order.destination;
  document.querySelector('#doneTime').textContent = clockFormat.format(new Date(order.created_at));
  orderForm.hidden = true;
  orderDone.hidden = false;
  document.querySelector('#done-title').focus();
}

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

zoneList.addEventListener('change', (event) => {
  if (event.target.name !== 'receiving_zone') return;
  state.zoneId = event.target.value;
  clearError();
  render();
});

orderForm.addEventListener('submit', async (event) => {
  event.preventDefault();
  const items = cartItems().map(({ object_class, quantity }) => ({ object_class, quantity }));
  if (!items.length || !state.zoneId || state.sending) return;
  state.sending = true;
  clearError();
  render();
  try {
    const response = await fetch('/api/orders', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ items, receiving_zone: state.zoneId }),
    });
    const result = await response.json().catch(() => ({}));
    if (!response.ok) {
      // OUT_OF_STOCK and ITEM_CLOSED name every refused object in one message; refresh so the cards agree.
      showError(result.message || result.error || '주문을 접수하지 못했어요.');
      if (['OUT_OF_STOCK', 'ITEM_CLOSED'].includes(result.error_code)) loadCatalog();
      return;
    }
    state.cart.clear();
    state.adjusted.clear();
    showConfirmation(result.order);
  } catch {
    // The request may have reached the server, so do not invite a second order blindly.
    showError('응답을 받지 못했어요. 주문이 접수됐을 수 있으니 작업자에게 확인한 뒤 다시 주문해 주세요.');
  } finally {
    state.sending = false;
    render();
  }
});

document.querySelector('#newOrder').addEventListener('click', () => {
  orderDone.hidden = true;
  orderForm.hidden = false;
  render();
  productList.querySelector('[data-step="1"]:not(:disabled)')?.focus();
});

document.addEventListener('visibilitychange', () => {
  if (!document.hidden) loadCatalog();
});

loadCatalog();
window.setInterval(loadCatalog, CATALOG_REFRESH_MS);
