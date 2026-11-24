// 주문 앱(/order)의 서비스 워커. order.js가 보안 컨텍스트에서만 scope '/order'로 등록한다.
// 앱 자신의 파일을 보관해 네트워크가 잠깐 끊겨도 화면이 열리게 하되, 언제나 서버에 먼저 묻는다(네트워크 우선).
// 주문·재고·추적(/api/...)은 절대 캐시하지 않는다: 거기서 오래된 응답은 이미 사실이 아닌 재고나 진행을 보여 준다.

const CACHE = 'robot-order-v2';  // v2: css/, js/ 폴더로 옮김 (옛 캐시는 activate에서 지움)
// 캐시해 둘 앱 껍데기 파일. fetch 처리에서 이 경로의 성공 응답만 캐시를 새로 쓴다.
const SHELL = [
  '/order', '/css/order.css', '/js/order.js', '/order-manifest.json', '/icons/order-192.png',
  '/fonts/Pretendard-Regular.subset.woff2', '/fonts/Pretendard-Medium.subset.woff2',
  '/fonts/Pretendard-SemiBold.subset.woff2', '/fonts/Pretendard-Bold.subset.woff2',
];

// 설치: SHELL 파일을 미리 받아 두고, 기다리지 않고 바로 새 워커로 넘어간다(skipWaiting).
self.addEventListener('install', (event) => {
  event.waitUntil(caches.open(CACHE).then((cache) => cache.addAll(SHELL)).then(() => self.skipWaiting()));
});

// 활성화: 이름이 다른(옛 버전) 캐시를 모두 지우고, 열려 있는 페이지도 바로 이 워커가 맡는다(clients.claim).
self.addEventListener('activate', (event) => {
  event.waitUntil(caches.keys()
    .then((keys) => Promise.all(keys.filter((key) => key !== CACHE).map((key) => caches.delete(key))))
    .then(() => self.clients.claim()));
});

// 요청 가로채기: GET이 아니거나 다른 출처이거나 /api/ 요청이면 손대지 않는다(브라우저 기본 동작).
self.addEventListener('fetch', (event) => {
  const url = new URL(event.request.url);
  if (event.request.method !== 'GET' || url.origin !== self.location.origin || url.pathname.startsWith('/api/')) return;
  // 네트워크 우선: 성공한 SHELL 응답은 캐시에 복사해 두고, 네트워크가 실패하면 캐시(쿼리 무시)에서 꺼낸다.
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
