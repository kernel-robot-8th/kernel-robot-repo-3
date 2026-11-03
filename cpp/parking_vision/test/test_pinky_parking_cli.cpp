// Spot registration: build_registration() against the former Python CLI on the same frames, and the pinky_parking
// CLI end to end with a video file standing in for the ceiling camera.
#include <sys/wait.h>

#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "parking_vision/registration.hpp"

namespace parking_vision {
namespace {

namespace fs = std::filesystem;
using nlohmann::json;

const fs::path kFixtures = PARKING_VISION_FIXTURE_DIR;

json parity() {
  std::ifstream stream(kFixtures / "parity.json");
  return json::parse(stream);
}

TEST(RegistrationParity, SameRecordAsThePythonCli) {
  const Intrinsics intrinsics = load_intrinsics(kFixtures / "spot" / "ceiling_camera.json", "ceiling_camera");
  for (const json& c : parity().at("registration_cases")) {
    const std::string where = c.at("name");
    std::vector<CollectedFrame> collected;
    for (const json& frame : c.at("frames")) {
      const cv::Mat image = cv::imread((kFixtures / frame.get<std::string>()).string(), cv::IMREAD_UNCHANGED);
      const MarkerDetections found = detect_markers(image, "DICT_4X4_50");
      collected.emplace_back(found.at(21).front(), found);
    }
    const json& o = c.at("options");
    RegistrationOptions options;
    options.spot_id = o.at("spot_id");
    options.camera_id = o.at("camera_id");
    options.dictionary = o.at("dictionary");
    options.marker_id = o.at("marker_id");
    options.marker_side_mm = o.at("marker_side_mm");
    if (!o.at("anchor_ids").is_null()) options.anchor_ids = o.at("anchor_ids").get<std::vector<int>>();
    if (!o.at("marker_forward_axis").is_null()) options.marker_forward_axis = o.at("marker_forward_axis");
    options.trust_single_anchor = o.at("trust_single_anchor");
    options.no_anchors = o.at("no_anchors");
    options.tolerance_mm = o.at("tolerance_mm");
    options.tolerance_deg = o.at("tolerance_deg");
    const json& expected = c.at("expected");
    try {
      const json record = build_registration(collected, options, intrinsics, kFixtures);
      ASSERT_FALSE(expected.contains("rejected")) << where;
      std::vector<std::string> keys;
      for (const auto& item : record.items()) keys.push_back(item.key());
      EXPECT_EQ(keys, expected.at("keys").get<std::vector<std::string>>()) << where;
      EXPECT_EQ(record.at("intrinsic_calibration").at("sha256"), expected.at("intrinsic_calibration_sha256")) << where;
      for (const char* key : {"schema_version", "spot_id", "camera_id", "pixel_transform_id", "image_size_px",
                              "aruco_dictionary", "marker_id", "marker_side_mm", "marker_forward_axis",
                              "position_method", "reference_frame_count", "reference_jitter",
                              "reference_marker_side_on_plane_mm", "anchors", "view_check", "tolerance", "axes",
                              "status", "review_required", "usable_for_robot_control"}) {
        EXPECT_EQ(record.at(key), expected.at(key)) << where << " " << key;
      }
      for (int r = 0; r < 3; ++r) {
        for (int k = 0; k < 3; ++k) {
          EXPECT_NEAR(record["reference_pose_in_camera"]["rotation"][r][k].get<double>(),
                      expected["reference_pose_in_camera"]["rotation"][r][k].get<double>(), 1e-6)
              << where;
        }
        EXPECT_NEAR(record["reference_pose_in_camera"]["translation_mm"][r].get<double>(),
                    expected["reference_pose_in_camera"]["translation_mm"][r].get<double>(), 1e-4)
            << where;
      }
      for (int i = 0; i < 4; ++i) {
        for (int k = 0; k < 2; ++k) {
          EXPECT_NEAR(record["reference_corners_px"][i][k].get<double>(),
                      expected["reference_corners_px"][i][k].get<double>(), 1e-9)
              << where;
        }
      }
    } catch (const RegistrationRejected& error) {
      ASSERT_TRUE(expected.contains("rejected")) << where << ": " << error.what();
      EXPECT_EQ(std::string(error.what()), expected.at("rejected").get<std::string>()) << where;
    }
  }
}

struct CliRun {
  int exit_code;
  std::string output;
};

CliRun run(const std::string& arguments) {
  const std::string command = std::string(PINKY_PARKING_CLI) + " " + arguments + " 2>&1";
  std::unique_ptr<FILE, int (*)(FILE*)> pipe(popen(command.c_str(), "r"), pclose);
  std::string output;
  std::array<char, 512> buffer{};
  while (fgets(buffer.data(), buffer.size(), pipe.get())) output += buffer.data();
  const int status = pclose(pipe.release());
  return {WIFEXITED(status) ? WEXITSTATUS(status) : -1, output};
}

// An MJPEG AVI cycling through fixture frames, the format the camera delivers.
void write_video(const fs::path& path, const std::vector<std::string>& frame_names, int frames = 60) {
  cv::VideoWriter writer(path.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(640, 480));
  ASSERT_TRUE(writer.isOpened());
  for (int i = 0; i < frames; ++i) {
    cv::Mat color;
    cv::cvtColor(cv::imread((kFixtures / "frames" / (frame_names[i % frame_names.size()] + ".png")).string(),
                            cv::IMREAD_GRAYSCALE),
                 color, cv::COLOR_GRAY2BGR);
    writer.write(color);
  }
}

class PinkyParkingCli : public ::testing::Test {
 protected:
  void SetUp() override {
    root = fs::temp_directory_path() /
           ("pinky_parking_cli_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    fs::remove_all(root);
    fs::create_directories(root);
    fs::copy_file(kFixtures / "spot" / "ceiling_camera.json", root / "ceiling_camera.json");
  }
  void TearDown() override { fs::remove_all(root); }

  std::string common() const {
    return "--spot-id dock --repository-root " + root.string() + " --intrinsics " + (root / "ceiling_camera.json").string();
  }

  fs::path root;
};

TEST_F(PinkyParkingCli, RegisterThenCheck) {
  write_video(root / "parked.avi", {"reg_0", "reg_1", "reg_2", "reg_3", "reg_4"});
  const CliRun registered = run("register " + common() + " --camera " + (root / "parked.avi").string() +
                             " --marker-id 21 --marker-side-mm 70 --marker-forward-axis -x --headless");
  ASSERT_EQ(registered.exit_code, 0) << registered.output;
  const fs::path spot_file = root / ".control-station" / "parking-spots" / "dock.json";
  ASSERT_TRUE(fs::exists(spot_file));
  const ParkingSpot spot = load_spot(spot_file, root);
  EXPECT_EQ(spot.anchors.size(), 2u);
  EXPECT_EQ(spot.marker_forward_axis, "-x");
  EXPECT_EQ(spot.tolerance.position_mm, 30.0);

  write_video(root / "same.avi", {"registered"});
  const CliRun arrived = run("check " + common() + " --camera " + (root / "same.avi").string() + " --once");
  EXPECT_EQ(arrived.exit_code, 0) << arrived.output;
  const json line = json::parse(arrived.output.substr(arrived.output.find('{')));
  EXPECT_EQ(line.at("status"), "ARRIVED");
  EXPECT_EQ(line.at("frames"), 15);
  EXPECT_EQ(line.at("view_check"), "ON");
  EXPECT_TRUE(line.contains("forward_mm"));

  write_video(root / "moved.avi", {"moved"});
  const CliRun misaligned =
      run("check " + common() + " --camera " + (root / "moved.avi").string() + " --once --tolerance-mm 20");
  EXPECT_EQ(misaligned.exit_code, 1) << misaligned.output;
  EXPECT_NE(misaligned.output.find("\"MISALIGNED\""), std::string::npos);

  // Never overwrites a registered spot.
  const CliRun again = run("register " + common() + " --camera " + (root / "parked.avi").string() +
                        " --marker-id 21 --marker-side-mm 70 --headless");
  EXPECT_EQ(again.exit_code, 1);
  EXPECT_NE(again.output.find("이미 등록된 주차 위치"), std::string::npos);
}

TEST_F(PinkyParkingCli, RegistrationRejectedWhenPinkyMoves) {
  write_video(root / "moving.avi", {"reg_0", "moved"});
  const CliRun result = run("register " + common() + " --camera " + (root / "moving.avi").string() +
                         " --marker-id 21 --marker-side-mm 70 --headless");
  EXPECT_EQ(result.exit_code, 1);
  EXPECT_NE(result.output.find("moved while registering"), std::string::npos) << result.output;
  EXPECT_FALSE(fs::exists(root / ".control-station" / "parking-spots" / "dock.json"));
}

TEST_F(PinkyParkingCli, CheckWithoutSpotOrCameraIsNotJudged) {
  EXPECT_EQ(run("check " + common() + " --camera " + (root / "none.avi").string() + " --once").exit_code, 2);
}

}  // namespace
}  // namespace parking_vision
