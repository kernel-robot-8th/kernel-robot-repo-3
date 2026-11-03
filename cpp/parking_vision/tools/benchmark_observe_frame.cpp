// Per-frame latency of observe_frame on the parity fixtures; tools/benchmark_observe_frame.py measures the
// Python module on the same frames and spot.
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
  if (argc > 3) cv::setNumThreads(std::atoi(argv[3]));
  const parking_vision::ParkingSpot spot = parking_vision::load_spot(fixtures / "spot" / "spot.json", fixtures / "spot");

  std::printf("C++ OpenCV %s, %d threads, %d iterations\n", CV_VERSION, cv::getNumThreads(), iterations);
  for (const char* name : {"registered", "camera_like_registered", "moved", "wrapped_96"}) {
    cv::Mat frame;
    cv::cvtColor(cv::imread((fixtures / "frames" / (std::string(name) + ".png")).string(), cv::IMREAD_GRAYSCALE),
                 frame, cv::COLOR_GRAY2BGR);  // the feed delivers BGR
    for (int i = 0; i < 50; ++i) parking_vision::observe_frame(frame, spot);
    std::vector<double> ms;
    for (int i = 0; i < iterations; ++i) {
      const auto start = std::chrono::steady_clock::now();
      parking_vision::observe_frame(frame, spot);
      ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
    }
    std::sort(ms.begin(), ms.end());
    std::printf("  %-24s median %.3f ms  p95 %.3f ms\n", name, ms[ms.size() / 2], ms[ms.size() * 95 / 100]);
  }
  return 0;
}
