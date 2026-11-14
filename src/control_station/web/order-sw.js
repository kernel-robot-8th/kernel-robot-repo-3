// Service worker for the order app (/order). It keeps the app's own files so the screen still opens when the
// network blips, always asking the server first. Orders, stock and tracking (/api/...) are never cached: a stale
// answer there would show stock or progress that is no longer true.

const CACHE = 'robot-order-v1';
const SHELL = [
  '/order', '/order.css', '/order.js', '/order-manifest.json', '/icons/order-192.png',
  '/fonts/Pretendard-Regular.subset.woff2', '/fonts/Pretendard-Medium.subset.woff2',
  '/fonts/Pretendard-SemiBold.subset.woff2', '/fonts/Pretendard-Bold.subset.woff2',
];

self.addEventListener('install', (event) => {
  event.waitUntil(caches.open(CACHE).then((cache) => cache.addAll(SHELL)).then(() => self.skipWaiting()));
});

self.addEventListener('activate', (event) => {
  event.waitUntil(caches.keys()
    .then((keys) => Promise.all(keys.filter((key) => key !== CACHE).map((key) => caches.delete(key))))
    .then(() => self.clients.claim()));
});

self.addEventListener('fetch', (event) => {
  const url = new URL(event.request.url);
  if (event.request.method !== 'GET' || url.origin !== self.location.origin || url.pathname.startsWith('/api/')) return;
  event.respondWith(fetch(event.request)
    .then((response) => {
      if (response.ok && SHELL.includes(url.pathname)) {
        const copy = response.clone();
        caches.open(CACHE).then((cache) => cache.put(event.request, copy));
      }
      return response;
    })
    .catch(() => caches.match(event.request, { ignoreSearch: true })
      .then((cached) => cached || Response.error())));
});
