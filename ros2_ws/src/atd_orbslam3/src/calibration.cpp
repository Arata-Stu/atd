#include "calibration.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <stdexcept>
#include <opencv2/core.hpp>

namespace atd_orbslam3 {
namespace {
bool close(double a, double b) { return std::abs(a - b) <= 1e-5 * std::max({1., std::abs(a), std::abs(b)}); }

void check_projection(const sensor_msgs::msg::CameraInfo & info)
{
  const auto & p = info.p;
  if (!info.width || !info.height || info.width > 65535 || info.height > 65535 ||
      info.binning_x > 1 || info.binning_y > 1 || info.roi.x_offset || info.roi.y_offset ||
      !std::all_of(p.begin(), p.end(), [](double x) { return std::isfinite(x); }) ||
      p[0] <= 0 || p[5] <= 0 || p[2] <= 0 || p[2] >= info.width || p[6] <= 0 || p[6] >= info.height ||
      !close(p[1], 0) || !close(p[4], 0) || !close(p[8], 0) || !close(p[9], 0) ||
      !close(p[10], 1) || !close(p[11], 0) || std::abs(p[7] / p[5]) > .001) {
    throw std::runtime_error("invalid rectified CameraInfo projection");
  }
  // The static optical frames of the supported D455 / Gazebo rig are the rectified axes.
  for (std::size_t i = 0; i < info.r.size(); ++i) {
    if (!close(info.r[i], i % 4 == 0 ? 1. : 0.)) {
      throw std::runtime_error("non-identity rectification requires a separate rectified optical frame");
    }
  }
}

constexpr const char * settings_defaults = R"(Camera.RGB: 0
Stereo.ThDepth: 40.0
ORBextractor.nFeatures: 1250
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -3.5
Viewer.ViewpointF: 500.0
)";
}  // namespace

Calibration Calibration::from(const sensor_msgs::msg::CameraInfo & left,
                              const sensor_msgs::msg::CameraInfo & right)
{
  check_projection(left);
  check_projection(right);
  const auto & l = left.p;
  const auto & r = right.p;
  if (left.width != right.width || left.height != right.height || !close(l[3], 0.) ||
      !close(l[0], r[0]) || !close(l[5], r[5]) || !close(l[2], r[2]) || !close(l[6], r[6]) ||
      r[3] >= 0.) {
    throw std::runtime_error("ORB Rectified requires a horizontal pair with common rectified intrinsics and positive baseline");
  }
  return {static_cast<int>(left.width), static_cast<int>(left.height), {l[0], l[5], l[2], l[6]}, -r[3] / r[0]};
}

bool Calibration::matches(const Calibration & other) const
{
  return width == other.width && height == other.height && close(baseline, other.baseline) &&
    std::equal(intrinsics.begin(), intrinsics.end(), other.intrinsics.begin(), close);
}

void Inertial::validate() const
{
  for (double value : {hz, gyro_noise, accel_noise, gyro_walk, accel_walk}) {
    if (!std::isfinite(value) || value <= 0.) {
      throw std::invalid_argument("VIO requires positive IMU frequency, noise densities and random walks");
    }
  }
  if (frame.empty()) { throw std::invalid_argument("VIO requires the combined IMU frame"); }
}

void MapCamera::save(const std::filesystem::path & path) const
{
  cv::FileStorage file(path.string(), cv::FileStorage::WRITE);
  if (!file.isOpened()) { throw std::runtime_error("cannot write camera.yml"); }
  cv::Mat matrix(4, 4, CV_64F);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) { matrix.at<double>(i, j) = map_from_atlas.matrix()(i, j); }
  }
  file << "version" << 1 << "width" << calibration.width << "height" << calibration.height;
  file << "intrinsics" << "["; for (double value : calibration.intrinsics) { file << value; }
  file << "]" << "baseline" << calibration.baseline << "map_from_atlas" << matrix;
  file.release();
}

void MapCamera::save_cloud(const std::filesystem::path & path,
                           const std::vector<Eigen::Vector3f> & points) const
{
  const std::size_t count = std::count_if(points.begin(), points.end(),
    [](const Eigen::Vector3f & point) { return point.allFinite(); });
  if (!count) { throw std::runtime_error("saved Atlas has no finite landmarks"); }
  constexpr std::size_t max_points = 250000;
  const auto stride = 1 + (count - 1) / max_points;
  std::ofstream out;
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out.open(path);
  out.imbue(std::locale::classic());
  out << "{\"frame\":\"map\",\"source_count\":" << count << ",\"display_stride\":" << stride
      << ",\"provenance\":{\"backend\":\"orbslam3\",\"source\":\"atlas\"},\"points\":["
      << std::setprecision(9);
  std::size_t index = 0;
  for (const auto & point : points) {
    if (!point.allFinite()) { continue; }
    const auto current = index++;
    if (current % stride) { continue; }
    const Eigen::Vector3d mapped = map_from_atlas * point.cast<double>();
    if (current) { out << ','; }
    out << '[' << mapped.x() << ',' << mapped.y() << ',' << mapped.z() << ']';
  }
  out << "]}\n";
  out.close();
}

MapCamera MapCamera::load(const std::filesystem::path & path)
{
  cv::FileStorage file(path.string(), cv::FileStorage::READ);
  if (!file.isOpened() || int(file["version"]) != 1) { throw std::runtime_error("invalid camera.yml"); }
  const auto k = file["intrinsics"];
  cv::Mat m;
  file["map_from_atlas"] >> m;
  if (!k.isSeq() || k.size() != 4 || m.type() != CV_64F || m.rows != 4 || m.cols != 4) {
    throw std::runtime_error("missing map calibration or frame transform");
  }
  MapCamera camera{{int(file["width"]), int(file["height"]), {}, double(file["baseline"])},
                   Eigen::Isometry3d::Identity()};
  for (int i = 0; i < 4; ++i) {
    camera.calibration.intrinsics[i] = double(k[i]);
    for (int j = 0; j < 4; ++j) { camera.map_from_atlas.matrix()(i, j) = m.at<double>(i, j); }
  }
  const auto & transform = camera.map_from_atlas;
  if (!transform.matrix().allFinite() || !transform.linear().isUnitary(1e-7) ||
      std::abs(transform.linear().determinant() - 1.) > 1e-7 ||
      !transform.matrix().row(3).isApprox(Eigen::RowVector4d(0., 0., 0., 1.), 1e-7)) {
    throw std::runtime_error("invalid rigid transform in camera.yml");
  }
  return camera;
}

void write_settings(const std::filesystem::path & path, const Calibration & camera, int fps,
                    const Inertial * imu, const std::filesystem::path & atlas_stem, bool fixed, bool loop_closing)
{
  std::ofstream out;
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out.open(path);
  out.imbue(std::locale::classic());
  out << "%YAML:1.0\nFile.version: \"1.0\"\nCamera.type: \"Rectified\"\n"
      << std::showpoint << std::setprecision(17)
      << "Camera1.fx: " << camera.intrinsics[0] << "\nCamera1.fy: " << camera.intrinsics[1]
      << "\nCamera1.cx: " << camera.intrinsics[2] << "\nCamera1.cy: " << camera.intrinsics[3]
      << "\nStereo.b: " << camera.baseline << "\nCamera.width: " << camera.width
      << "\nCamera.height: " << camera.height << "\nCamera.fps: " << fps << '\n' << settings_defaults;
  if (!loop_closing) { out << "loopClosing: 0\n"; }
  if (!atlas_stem.empty()) {
    const auto stem = atlas_stem.string();
    if (stem.find_first_of("\r\n\"\\") != std::string::npos) { throw std::invalid_argument("invalid atlas path"); }
    out << (fixed ? "System.LoadAtlasFromFile" : "System.SaveAtlasToFile") << ": \"" << stem << "\"\n";
  }
  if (imu) {
    out << "IMU.Frequency: " << imu->hz << "\nIMU.NoiseGyro: " << imu->gyro_noise
        << "\nIMU.NoiseAcc: " << imu->accel_noise << "\nIMU.GyroWalk: " << imu->gyro_walk
        << "\nIMU.AccWalk: " << imu->accel_walk << "\nIMU.InsertKFsWhenLost: 0\n"
        << "IMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: [";
    for (int i = 0; i < 16; ++i) {
      if (i) { out << ", "; }
      out << imu->imu_from_camera.matrix()(i / 4, i % 4);
    }
    out << "]\n";
  }
  out.close();
}
}  // namespace atd_orbslam3
