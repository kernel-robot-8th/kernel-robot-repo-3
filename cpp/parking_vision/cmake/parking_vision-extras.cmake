# ament CONFIG_EXTRAS 파일이다. 다른 패키지가 find_package(parking_vision)을 하면 이 내용이 실행된다.
# 의존 패키지(parking_vision_ros)의 테스트가 쓰도록 설치한 비교용 기대값(주차 위치 파일, 프레임)의 위치를 알려준다.
# parking_vision_DIR은 share/parking_vision/cmake이므로 그 옆의 share/parking_vision/fixtures를 가리킨다.
set(parking_vision_FIXTURE_DIR "${parking_vision_DIR}/../fixtures")
