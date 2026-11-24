// 비교용 기대값(fixtures)의 프레임으로 observe_frame 한 번에 걸리는 시간(프레임당 지연)을 잰다.
// Python 쪽 측정 도구는 2026-11-03에 Python 판정과 함께 지웠고, 마지막 측정값은 cpp/README.md에 남아 있다.
//
// 위치: parking_vision 패키지의 측정용 실행 파일(설치하지 않음). 결과는 판정이 10 Hz 추적·30 fps 영상에 비해
// 충분히 빠른지 확인하는 용도다. 2026-11-02 측정에서 C++과 Python(같은 OpenCV 4.6)의 차이는 프레임당 약 0.1 ms였다.
//
//   parking_vision_benchmark <fixture dir> [iterations] [opencv threads]
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "parking_vision/parking_check.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <fixture dir> [iterations] [opencv threads]\n", argv[0]);
    return 2;
  }
  const std::filesystem::path fixtures = argv[1];
  const int iterations = argc > 2 ? std::atoi(argv[2]) : 500;
  // README의 측정은 OpenCV 스레드 1개로 했다. 스레드 수가 결과에 영향을 주므로 인자로 고정할 수 있게 한다.
  if (argc > 3) cv::setNumThreads(std::atoi(argv[3]));
  const parking_vision::ParkingSpot spot = parking_vision::load_spot(fixtures / "spot" / "spot.json", fixtures / "spot");

  std::printf("C++ OpenCV %s, %d threads, %d iterations\n", CV_VERSION, cv::getNumThreads(), iterations);
  // 렌더링한 등록 위치, 블러·노이즈를 넣은 등록 위치, 26 mm 이동, 96 px 밀림(시야 확인에서 거부) 프레임.
  for (const char* name : {"registered", "camera_like_registered", "moved", "wrapped_96"}) {
    cv::Mat frame;
    cv::cvtColor(cv::imread((fixtures / "frames" / (std::string(name) + ".png")).string(), cv::IMREAD_GRAYSCALE),
                 frame, cv::COLOR_GRAY2BGR);  // feed는 BGR로 전달한다(흑백 변환 비용까지 재기 위해)
    // 처음 몇 번은 캐시·메모리 할당 때문에 느리므로 50번은 재지 않고 먼저 돌린다(warm-up).
    for (int i = 0; i < 50; ++i) parking_vision::observe_frame(frame, spot);
    std::vector<double> ms;
    for (int i = 0; i < iterations; ++i) {
      const auto start = std::chrono::steady_clock::now();
      parking_vision::observe_frame(frame, spot);
      ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
    }
    // 정렬해서 중앙값과 95 백분위수를 낸다(평균은 가끔 튀는 값에 끌려가므로 쓰지 않는다).
    std::sort(ms.begin(), ms.end());
    std::printf("  %-24s median %.3f ms  p95 %.3f ms\n", name, ms[ms.size() / 2], ms[ms.size() * 95 / 100]);
  }
  return 0;
}
