// Register Pinky's parking spot and check whether Pinky stopped on it, using the ceiling camera.
//
// C++ port of the former scripts/pinky_parking.py; the same arguments, spot files and exit codes.
//   register: Pinky를 올바른 주차 위치에 세워 두고 Space를 누르면 마커와 바닥 고정 마커 위치를 기준으로 저장한다.
//             --headless는 창 없이 바로 모은다.
//   check:    저장한 기준과 지금 마커 위치를 비교해 ARRIVED / MISALIGNED / NOT_FOUND / AMBIGUOUS / VIEW_UNVERIFIED를
//             판정한다. --once는 창 없이 한 번 판정하고 JSON을 출력한다 (종료 코드 0 도착, 1 어긋남, 2 판정 불가).
// 결과는 관측 초안일 뿐이며 Pinky나 로봇팔에 명령을 보내지 않는다. 관제가 같은 카메라를 열고 있으면 먼저 끈다.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "parking_vision/parking_check.hpp"
#include "parking_vision/registration.hpp"

namespace fs = std::filesystem;
using nlohmann::ordered_json;
using namespace parking_vision;

namespace {

const char* const kUsage =
    "사용법: pinky_parking {register|check} --spot-id ID [옵션]\n"
    "  --camera PATH            천장 카메라 장치 (기본 CONTROL_STATION_CEILING_CAMERA; by-path 경로 권장)\n"
    "  --repository-root DIR    주차 위치·보정 파일 기준 경로 (기본 현재 폴더)\n"
    "  --camera-id ID           보정 camera_id (기본 ceiling_camera)\n"
    "  --intrinsics FILE        내부 보정 JSON (기본 .control-station/camera-calibrations/ceiling_camera.json)\n"
    "  --spot-dir DIR           주차 위치 폴더 (기본 .control-station/parking-spots)\n"
    "  --frames N               기준 등록·판정에 모을 프레임 수 (기본 15)\n"
    "  register: --marker-id N --marker-side-mm MM [--dictionary DICT_4X4_50] [--anchor-ids 1,3]\n"
    "            [--marker-forward-axis +x|-x|+y|-y] [--trust-single-anchor | --no-anchors]\n"
    "            [--tolerance-mm 30] [--tolerance-deg 5] [--headless]\n"
    "  check:    [--once] [--timeout 3] [--tolerance-mm MM] [--tolerance-deg DEG]\n";

struct Arguments {
  std::string command;
  std::string spot_id;
  std::string camera;
  fs::path repository_root = ".";
  std::string camera_id = "ceiling_camera";
  std::optional<fs::path> intrinsics;
  std::optional<fs::path> spot_dir;
  std::string dictionary = "DICT_4X4_50";
  std::optional<int> marker_id;
  std::optional<double> marker_side_mm;
  std::optional<std::vector<int>> anchor_ids;
  std::optional<std::string> marker_forward_axis;
  bool trust_single_anchor = false;
  bool no_anchors = false;
  std::optional<double> tolerance_mm;
  std::optional<double> tolerance_deg;
  int frames = 15;
  bool once = false;
  bool headless = false;
  double timeout = 3.0;
};

[[noreturn]] void usage_error(const std::string& message) {
  std::cerr << kUsage << "오류: " << message << "\n";
  std::exit(2);
}

Arguments parse(int argc, char** argv) {
  Arguments args;
  if (const char* camera = std::getenv("CONTROL_STATION_CEILING_CAMERA")) args.camera = camera;
  std::vector<std::string> words(argv + 1, argv + argc);
  if (words.empty()) usage_error("register 또는 check가 필요합니다");
  if (words[0] == "-h" || words[0] == "--help") {
    std::cout << kUsage;
    std::exit(0);
  }
  args.command = words[0];
  if (args.command != "register" && args.command != "check") usage_error("register 또는 check가 필요합니다");
  for (std::size_t i = 1; i < words.size(); ++i) {
    const std::string& flag = words[i];
    const auto value = [&]() -> std::string {
      if (i + 1 >= words.size()) usage_error(flag + "에 값이 필요합니다");
      return words[++i];
    };
    if (flag == "--spot-id") args.spot_id = value();
    else if (flag == "--camera") args.camera = value();
    else if (flag == "--repository-root") args.repository_root = value();
    else if (flag == "--camera-id") args.camera_id = value();
    else if (flag == "--intrinsics") args.intrinsics = fs::path(value());
    else if (flag == "--spot-dir") args.spot_dir = fs::path(value());
    else if (flag == "--dictionary") args.dictionary = value();
    else if (flag == "--marker-id") args.marker_id = std::stoi(value());
    else if (flag == "--marker-side-mm") args.marker_side_mm = std::stod(value());
    else if (flag == "--anchor-ids") {
      std::vector<int> ids;
      std::stringstream text(value());
      for (std::string part; std::getline(text, part, ',');) ids.push_back(std::stoi(part));
      args.anchor_ids = ids;
    } else if (flag == "--marker-forward-axis") {
      args.marker_forward_axis = value();
      if (*args.marker_forward_axis != "+x" && *args.marker_forward_axis != "-x" && *args.marker_forward_axis != "+y" &&
          *args.marker_forward_axis != "-y") {
        usage_error("--marker-forward-axis는 +x, -x, +y, -y 중 하나여야 합니다");
      }
    } else if (flag == "--trust-single-anchor") args.trust_single_anchor = true;
    else if (flag == "--no-anchors") args.no_anchors = true;
    else if (flag == "--tolerance-mm") args.tolerance_mm = std::stod(value());
    else if (flag == "--tolerance-deg") args.tolerance_deg = std::stod(value());
    else if (flag == "--frames") args.frames = std::stoi(value());
    else if (flag == "--once") args.once = true;
    else if (flag == "--headless") args.headless = true;
    else if (flag == "--timeout") args.timeout = std::stod(value());
    else if (flag == "-h" || flag == "--help") {
      std::cout << kUsage;
      std::exit(0);
    } else usage_error("알 수 없는 옵션: " + flag);
  }
  if (args.spot_id.empty()) usage_error("--spot-id가 필요합니다");
  if (args.camera.empty()) {
    usage_error("--camera 또는 CONTROL_STATION_CEILING_CAMERA가 필요합니다 (같은 모델 카메라가 여럿이면 /dev/v4l/by-path 경로)");
  }
  if (args.command == "register") {
    if (!args.marker_id || !args.marker_side_mm) usage_error("register에는 --marker-id와 --marker-side-mm가 필요합니다");
    if (args.anchor_ids && std::find(args.anchor_ids->begin(), args.anchor_ids->end(), *args.marker_id) != args.anchor_ids->end()) {
      usage_error("--anchor-ids에 Pinky 마커 ID를 넣을 수 없습니다");
    }
    if (args.anchor_ids && args.no_anchors) usage_error("--anchor-ids와 --no-anchors는 함께 쓸 수 없습니다");
    if (args.trust_single_anchor && args.no_anchors) usage_error("--trust-single-anchor와 --no-anchors는 함께 쓸 수 없습니다");
  }
  return args;
}

fs::path spot_dir(const Arguments& args) {
  return args.spot_dir.value_or(args.repository_root / ".control-station" / "parking-spots");
}

// A /dev/videoN device opens by index through V4L2 (resolving by-path links); any other path opens as a file.
cv::VideoCapture open_camera(const std::string& device, int width, int height) {
  std::error_code missing;
  const fs::path resolved = fs::canonical(device, missing);
  if (missing) throw std::runtime_error("카메라 장치 경로가 없습니다: " + device);
  static const std::regex video_index("/dev/video(\\d+)");
  std::smatch match;
  const std::string path = resolved.string();
  cv::VideoCapture camera;
  if (std::regex_match(path, match, video_index)) {
    camera.open(std::stoi(match[1]), cv::CAP_V4L2);
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
  } else {
    camera.open(path, cv::CAP_ANY);
  }
  if (!camera.isOpened()) throw std::runtime_error("카메라를 열 수 없습니다: " + device + " (다른 프로그램이 쓰는지 fuser -v 로 확인)");
  return camera;
}

std::optional<cv::Mat> read_frame(cv::VideoCapture& camera, int width, int height) {
  cv::Mat frame;
  if (!camera.read(frame) || frame.empty()) return std::nullopt;
  if (frame.cols != width || frame.rows != height) {  // 다른 해상도로 돌면 내부 보정값이 맞지 않는다
    throw std::runtime_error("프레임 해상도 " + std::to_string(frame.cols) + "x" + std::to_string(frame.rows) +
                             "가 보정 해상도 " + std::to_string(width) + "x" + std::to_string(height) + "와 다릅니다");
  }
  return frame;
}

std::vector<cv::Point> points(const Corners& corners) {
  return {corners[0], corners[1], corners[2], corners[3]};
}

void draw_lines(cv::Mat& view, const std::vector<std::pair<std::string, cv::Scalar>>& lines) {
  for (std::size_t row = 0; row < lines.size(); ++row) {
    const cv::Point at(10, 26 + 24 * static_cast<int>(row));
    cv::putText(view, lines[row].first, at, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 4);
    cv::putText(view, lines[row].first, at, cv::FONT_HERSHEY_SIMPLEX, 0.6, lines[row].second, 1);
  }
}

double round_to(double value, int digits) {
  const double scale = std::pow(10.0, digits);
  return std::round(value * scale) / scale;
}

// ParkingResult.to_dict() plus Pinky-relative forward/left when the marker mounting direction is recorded.
ordered_json result_payload(const ParkingResult& result, const ParkingSpot& spot) {
  ordered_json payload = {{"status", to_string(result.status)}, {"reason", result.reason}};
  if (result.offset) {
    payload["dx_mm"] = round_to(result.offset->dx_mm, 1);
    payload["dy_mm"] = round_to(result.offset->dy_mm, 1);
    payload["dyaw_deg"] = round_to(result.offset->dyaw_deg, 2);
    payload["distance_mm"] = round_to(result.offset->distance_mm(), 1);
  }
  if (result.deviation_level) {
    payload["deviation_level"] = to_string(*result.deviation_level);
    payload["position_ratio"] = round_to(result.position_ratio.value_or(0.0), 2);
    payload["yaw_ratio"] = round_to(result.yaw_ratio.value_or(0.0), 2);
  }
  if (result.view_shift_px) payload["view_shift_px"] = round_to(*result.view_shift_px, 1);
  if (result.offset && spot.marker_forward_axis) {
    const auto [forward, left] = pinky_frame_offset(*result.offset, *spot.marker_forward_axis);
    payload["forward_mm"] = round_to(forward, 1);
    payload["left_mm"] = round_to(left, 1);
  }
  return payload;
}

std::string fmt(const char* pattern, double a, double b = 0.0, double c = 0.0) {
  char text[160];
  std::snprintf(text, sizeof(text), pattern, a, b, c);
  return text;
}

int run_register(const Arguments& args) {
  const fs::path intrinsics_path =
      args.intrinsics.value_or(args.repository_root / ".control-station" / "camera-calibrations" / "ceiling_camera.json");
  const Intrinsics intrinsics = load_intrinsics(intrinsics_path, args.camera_id);
  const fs::path output = spot_dir(args) / (args.spot_id + ".json");
  if (fs::exists(output)) {
    std::cerr << "이미 등록된 주차 위치가 있습니다: " << output.string()
              << " (다른 --spot-id를 쓰거나 기존 파일을 직접 옮기세요)\n";
    return 1;
  }
  RegistrationOptions options;
  options.spot_id = args.spot_id;
  options.camera_id = args.camera_id;
  options.dictionary = args.dictionary;
  options.marker_id = *args.marker_id;
  options.marker_side_mm = *args.marker_side_mm;
  options.anchor_ids = args.anchor_ids;
  options.marker_forward_axis = args.marker_forward_axis;
  options.trust_single_anchor = args.trust_single_anchor;
  options.no_anchors = args.no_anchors;
  options.tolerance_mm = args.tolerance_mm.value_or(30.0);
  options.tolerance_deg = args.tolerance_deg.value_or(5.0);

  cv::VideoCapture camera = open_camera(args.camera, intrinsics.width, intrinsics.height);
  const std::string window = "register parking spot " + args.spot_id;
  std::vector<CollectedFrame> collected;
  bool collecting = args.headless;
  std::string warning;
  ordered_json record;
  std::vector<int> skipped;
  if (args.headless) {
    std::cout << "창 없이 바로 기준 프레임을 모읍니다. Pinky가 정답 위치에 서 있어야 합니다." << std::endl;
  } else {
    std::cout << "Pinky를 올바른 주차 위치에 세운 뒤 Space. q: 종료" << std::endl;
    std::cout << "화면 왼쪽 가장자리에 세로 이음선이 보이면 영상이 밀린 상태이니 종료 후 다시 실행하세요." << std::endl;
  }
  int empty_reads = 0;
  while (true) {
    const std::optional<cv::Mat> frame = read_frame(camera, intrinsics.width, intrinsics.height);
    if (!frame) {
      if (args.headless && ++empty_reads > 100) throw std::runtime_error("카메라에서 프레임이 들어오지 않습니다: " + args.camera);
      continue;
    }
    empty_reads = 0;
    const MarkerDetections found = detect_markers(*frame, args.dictionary);
    const auto markers = found.find(*args.marker_id);
    const std::size_t seen = markers == found.end() ? 0 : markers->second.size();
    if (collecting && seen == 1) collected.emplace_back(markers->second[0], found);
    if (!args.headless) {
      cv::Mat view = frame->clone();
      std::vector<int> others;
      for (const auto& [id, detections] : found) {
        for (const auto& corners : detections) {
          cv::polylines(view, points(corners), true,
                        id == *args.marker_id ? cv::Scalar(255, 200, 0) : cv::Scalar(160, 160, 160), 2);
        }
        if (id != *args.marker_id) others.push_back(id);
      }
      std::string other_text = "[";
      for (std::size_t i = 0; i < others.size(); ++i) other_text += (i ? ", " : "") + std::to_string(others[i]);
      other_text += "]";
      std::vector<std::pair<std::string, cv::Scalar>> lines = {
          {"marker " + std::to_string(*args.marker_id) + " (" + args.dictionary + ", " +
               fmt("%g", *args.marker_side_mm) + " mm)",
           cv::Scalar(255, 255, 255)}};
      if (seen == 1) {
        lines.push_back({"found; fixed markers " + other_text, cv::Scalar(0, 200, 0)});
      } else {
        lines.push_back({"NOT FOUND (seen " + std::to_string(seen) + " times)", cv::Scalar(0, 0, 255)});
      }
      lines.push_back({collecting ? "registering... " + std::to_string(collected.size()) + "/" + std::to_string(args.frames)
                                  : "[Space] register this spot  [q] quit",
                       cv::Scalar(0, 220, 255)});
      if (!warning.empty()) lines.push_back({warning, cv::Scalar(0, 0, 255)});
      draw_lines(view, lines);
      cv::imshow(window, view);
      const int key = cv::waitKey(1) & 0xFF;
      if (key == 'q' || key == 27) {
        std::cout << "저장하지 않고 종료합니다." << std::endl;
        return 1;
      }
      if (key == ' ' && !collecting) {
        collecting = true;
        collected.clear();
        warning.clear();
      }
    }
    if (collecting && static_cast<int>(collected.size()) >= args.frames) {
      try {
        skipped.clear();
        record = build_registration(collected, options, intrinsics, args.repository_root, &skipped);
        break;
      } catch (const RegistrationRejected& error) {
        std::cout << "등록하지 않았습니다: " << error.what() << (args.headless ? "" : ". 다시 Space") << std::endl;
        if (args.headless) return 1;
        collecting = false;
        warning = std::string(error.what()) + " - press Space again";
      }
    }
  }
  camera.release();
  if (!args.headless) cv::destroyAllWindows();
  for (const int id : skipped) std::cout << "마커 " << id << "는 모든 프레임에서 고정돼 보이지 않아 시야 확인 기준에서 뺍니다." << std::endl;
  fs::create_directories(output.parent_path());
  FILE* file = std::fopen(output.c_str(), "wx");  // 기존 파일은 덮어쓰지 않는다
  if (!file) throw std::runtime_error("주차 위치 파일을 만들 수 없습니다: " + output.string());
  const std::string text = record.dump(2) + "\n";
  std::fwrite(text.data(), 1, text.size(), file);
  std::fclose(file);
  std::cout << "주차 위치 저장: " << output.string() << std::endl;
  std::cout << "기준 프레임 " << collected.size() << "장, 흔들림 최대 "
            << fmt("%.1f mm / %.2f°", record["reference_jitter"]["position_mm"].get<double>(),
                   record["reference_jitter"]["yaw_deg"].get<double>())
            << std::endl;
  if (record["anchors"].empty()) {
    std::cout << "시야 확인 꺼짐: 영상 밀림이나 카메라 이동을 감지하지 않습니다." << std::endl;
  } else {
    std::string ids;
    for (const auto& anchor : record["anchors"]) ids += (ids.empty() ? "" : ", ") + std::to_string(anchor["marker_id"].get<int>());
    std::cout << "시야 확인 고정 마커: [" << ids << "]" << std::endl;
  }
  return 0;
}

int run_check(const Arguments& args) {
  ParkingSpot spot = load_spot(spot_dir(args) / (args.spot_id + ".json"), args.repository_root);
  const ParkingTolerance tolerance{args.tolerance_mm.value_or(spot.tolerance.position_mm),
                                   args.tolerance_deg.value_or(spot.tolerance.yaw_deg)};
  const std::string view_check = spot.anchors.empty() ? "OFF" : "ON";
  cv::VideoCapture camera = open_camera(args.camera, spot.intrinsics.width, spot.intrinsics.height);
  const std::string window = "parking check " + args.spot_id;
  ParkingJudge judgment(tolerance, args.frames);
  ParkingResult result = judgment.result();
  std::optional<ParkingStatus> last_status;
  const auto started = std::chrono::steady_clock::now();
  const std::map<ParkingStatus, cv::Scalar> colors = {
      {ParkingStatus::Arrived, {0, 200, 0}},       {ParkingStatus::Misaligned, {0, 160, 255}},
      {ParkingStatus::NotFound, {0, 0, 255}},      {ParkingStatus::Ambiguous, {0, 0, 255}},
      {ParkingStatus::ViewUnverified, {0, 0, 255}}};
  while (true) {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const std::optional<cv::Mat> frame = read_frame(camera, spot.intrinsics.width, spot.intrinsics.height);
    if (!frame) {
      if (args.once && elapsed > args.timeout) break;
      continue;
    }
    const FrameObservation observation = observe_frame(*frame, spot);
    result = judgment.add(observation);
    if (args.once) {
      if (static_cast<int>(judgment.recent().size()) >= args.frames || elapsed > args.timeout) break;
      continue;
    }
    if (result.status != last_status) {
      ordered_json line = {{"spot_id", args.spot_id}, {"view_check", view_check}};
      line.update(result_payload(result, spot));
      std::cout << line.dump() << std::endl;
      last_status = result.status;
    }
    cv::Mat view = frame->clone();
    cv::polylines(view, points(spot.reference_corners_px), true, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    const cv::Point2d center = (spot.reference_corners_px[0] + spot.reference_corners_px[1] +
                                spot.reference_corners_px[2] + spot.reference_corners_px[3]) / 4.0;
    cv::line(view, center, (spot.reference_corners_px[0] + spot.reference_corners_px[1]) / 2.0,
             cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    for (const auto& [id, anchor] : spot.anchors) cv::circle(view, anchor, 6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    if (observation.corners) cv::polylines(view, points(*observation.corners), true, cv::Scalar(255, 200, 0), 2);
    const cv::Scalar color = colors.at(result.status);
    std::vector<std::pair<std::string, cv::Scalar>> lines = {{to_string(result.status), color}};
    if (result.offset) {
      const PlanarOffset& o = *result.offset;
      lines.push_back({fmt("dx %+.0f mm  dy %+.0f mm  dyaw %+.1f deg  ", o.dx_mm, o.dy_mm, o.dyaw_deg) +
                           (result.deviation_level ? to_string(*result.deviation_level) : ""),
                       color});
      if (spot.marker_forward_axis) {
        const auto [forward, left] = pinky_frame_offset(o, *spot.marker_forward_axis);
        lines.push_back({fmt("Pinky forward %+.0f mm  left %+.0f mm", forward, left), color});
      }
    }
    lines.push_back({result.reason, color});
    lines.push_back({fmt("tolerance %g mm / %g deg   view check ", tolerance.position_mm, tolerance.yaw_deg) +
                         view_check + "   white = registered   [q] quit",
                     cv::Scalar(220, 220, 220)});
    draw_lines(view, lines);
    cv::rectangle(view, cv::Point(0, 0), cv::Point(view.cols - 1, view.rows - 1), color, 6);
    cv::imshow(window, view);
    const int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break;
  }
  camera.release();
  if (!args.once) cv::destroyAllWindows();  // --once never opens a window
  if (args.once) {
    ordered_json line = {{"spot_id", args.spot_id},
                         {"frames", judgment.recent().size()},
                         {"view_check", view_check}};
    line.update(result_payload(result, spot));
    std::cout << line.dump() << std::endl;
  }
  return result.status == ParkingStatus::Arrived ? 0 : result.status == ParkingStatus::Misaligned ? 1 : 2;
}

}  // namespace

int main(int argc, char** argv) {
  const Arguments args = parse(argc, argv);
  try {
    return args.command == "register" ? run_register(args) : run_check(args);
  } catch (const std::exception& error) {
    // 등록 파일·카메라 문제로 판정하지 못한 경우도 1(어긋남)이 아닌 2(판정 불가)로 끝낸다
    std::cerr << error.what() << "\n";
    return args.command == "check" ? 2 : 1;
  }
}
