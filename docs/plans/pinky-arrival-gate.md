# Pinky 도착 판정과 로봇 진행 계획 (지난 계획)

2026-11-02의 이 계획(천장 카메라 주차 판정이 `ARRIVED`일 때만 팔 단계를 시작하고, 어긋나면 자동 재주행 없이 운영자에게 보이기)은 구현됐습니다. 지금 흐름은 [주문 흐름](../implementation/order-flow.md)의 `WAIT_PINKY_ARRIVAL`·`CONFIRM_DOCKING`에 있습니다.

결과 형식과 아직 정하지 않은 것(어긋났을 때 처리, 허용 오차, 시야 확인 없는 판정)은 [주차 정렬 관측 계약](../spec/interface-contracts.md#pinky-주차-정렬-관측-초안)에 있습니다.
