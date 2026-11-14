# Pinky 주차 판정 C++ ROS 2 노드 — 남은 계획

천장 카메라 주차 판정을 C++ ROS 2 노드로 옮기는 계획의 순서 1~5(인터페이스, 라이브러리, 카메라 캡처, 판정 컴포넌트, 관제 연결)는 2026-11-03에 끝났습니다. 지금 구성과 검증 기록은 [cpp/README.md](../../cpp/README.md), 운영 절차는 [실행 방법](../implementation/runbook.md#c-주차-판정-노드로-관제-실행)에 있습니다. 남은 것은 아래뿐입니다.

## 순서 6: YOLO 노드 분리 (미착수)

지금은 관제 안의 YOLO 검출기가 `ros:/ceiling_camera/image_raw/compressed`로 받은 영상에 직접 돕니다. 계획은 YOLO를 별도 노드로 옮겨 `vision_msgs/Detection2DArray`(예: `/ceiling_camera/detections`)를 발행하고, 관제는 박스만 받아 주차 패널처럼 영상 위에 그리는 것입니다. 나중에 C++ 추론(ONNX Runtime 또는 TensorRT)으로 바꿀 수 있습니다.

## 남은 확인 사항

- 노드 rosbag 검증과 실물 위치 시험(Pinky를 정답 위치, 앞으로 5 cm, 옆으로 2 cm, 10° 돌린 상태로 두고 화면·판정 확인)
- 자체 캡처 노드에서 96 px 영상 밀림이 다시 나는지 ([cpp/README.md](../../cpp/README.md)의 현재 상태 표)
- 관제와 노드를 다른 PC에서 돌릴 때 압축 영상 대역폭
- 선반 카메라도 같은 구조(`/shelf_camera/...`)로 옮길 시점
