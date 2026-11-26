#include "../src/calibration.hpp"

#include <cmath>
#include <limits>
#include <cstdlib>
#include <unistd.h>
#include <stdexcept>
#include <opencv2/core.hpp>

namespace fs = std::filesystem;
using atd_orbslam3::Calibration;
using atd_orbslam3::MapCamera;

void check(bool condition) { if (!condition) { throw std::runtime_error("calibration test failed"); } }

int main()
{
  sensor_msgs::msg::CameraInfo left;
  left.width = 640; left.height = 480;
  left.p = {400., 0., 320., 0., 0., 400., 240., 0., 0., 0., 1., 0.};
  left.r = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
  auto right = left;
  right.p[3] = -38.;
  const auto stereo = Calibration::from(left, right);
  check(std::abs(stereo.baseline - .095) < 1e-12);
  right.p[2] = 321.;
  bool rejected = false;
  try { Calibration::from(left, right); } catch (const std::runtime_error &) { rejected = true; }
  check(rejected);

  auto pattern = (fs::temp_directory_path() / "atd-calibration-test-XXXXXX").string();
  check(mkdtemp(pattern.data()) != nullptr);
  const auto dir = fs::path(pattern);
  const auto extrinsics = Eigen::Translation3d(.1, .2, .3) * Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitY());
  const MapCamera original{stereo, extrinsics};
  original.save(dir / "camera.yml");
  const auto restored = MapCamera::load(dir / "camera.yml");
  check(restored.calibration.matches(stereo));
  check(restored.map_from_atlas.matrix().isApprox(original.map_from_atlas.matrix()));

  const auto invalid = std::numeric_limits<float>::quiet_NaN();
  original.save_cloud(dir / "cloud.json", {{invalid, 0, 0}, {1, 2, 3}, {0, 0, 0}});
  cv::FileStorage cloud((dir / "cloud.json").string(), cv::FileStorage::READ);
  check(cloud.isOpened() && int(cloud["source_count"]) == 2 && cloud["points"].size() == 2);
  const Eigen::Vector3d expected = extrinsics * Eigen::Vector3d(1., 2., 3.);
  for (int i = 0; i < 3; ++i) {
    check(std::abs(double(cloud["points"][0][i]) - expected[i]) < 1.e-7);
  }

  atd_orbslam3::Inertial imu{"imu", extrinsics, 400., .001, .01, .0001, .0002};
  imu.validate();
  atd_orbslam3::write_settings(dir / "settings.yml", stereo, 90, &imu, {}, false);
  cv::FileStorage settings((dir / "settings.yml").string(), cv::FileStorage::READ);
  check(settings.isOpened() && settings["Camera1.fx"].isReal() && settings["Stereo.b"].isReal());
  check(settings["Camera.fps"].isInt() && int(settings["Camera.fps"]) == 90);
  check(settings["IMU.Frequency"].isReal() && double(settings["IMU.Frequency"]) == 400.);
  cv::Mat t;
  settings["IMU.T_b_c1"] >> t;
  check(t.rows == 4 && t.cols == 4 && t.type() == CV_32F && std::abs(t.at<float>(0, 3) - .1) < 1e-6);
  atd_orbslam3::write_settings(dir / "fixed.yml", stereo, 30, nullptr, dir / "atlas", true);
  cv::FileStorage fixed((dir / "fixed.yml").string(), cv::FileStorage::READ);
  check(std::string(fixed["System.LoadAtlasFromFile"]) == (dir / "atlas").string());
  check(fixed["System.SaveAtlasToFile"].empty());
  atd_orbslam3::write_settings(dir / "vio.yml", stereo, 90, &imu, {}, false, false);
  cv::FileStorage vio((dir / "vio.yml").string(), cv::FileStorage::READ);
  check(vio["loopClosing"].isInt() && int(vio["loopClosing"]) == 0);
  check(vio["System.SaveAtlasToFile"].empty() && vio["System.LoadAtlasFromFile"].empty());
  check(settings["loopClosing"].empty());
  fs::remove_all(dir);
}
