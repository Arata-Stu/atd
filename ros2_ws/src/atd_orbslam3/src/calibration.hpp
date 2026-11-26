#pragma once

#include <array>
#include <vector>
#include <filesystem>
#include <string>
#include <Eigen/Geometry>
#include <sensor_msgs/msg/camera_info.hpp>

namespace atd_orbslam3 {

struct Calibration {
  int width, height;
  std::array<double, 4> intrinsics;  // fx, fy, cx, cy of the rectified pair
  double baseline;
  static Calibration from(const sensor_msgs::msg::CameraInfo & left,
                          const sensor_msgs::msg::CameraInfo & right);
  bool matches(const Calibration & other) const;
};

struct Inertial {
  std::string frame;
  Eigen::Isometry3d imu_from_camera = Eigen::Isometry3d::Identity();
  double hz, gyro_noise, accel_noise, gyro_walk, accel_walk;
  void validate() const;
};

struct MapCamera {
  Calibration calibration;
  Eigen::Isometry3d map_from_atlas;
  void save(const std::filesystem::path & path) const;
  void save_cloud(const std::filesystem::path & path, const std::vector<Eigen::Vector3f> & points) const;
  static MapCamera load(const std::filesystem::path & path);
};

void write_settings(const std::filesystem::path & path, const Calibration & camera, int fps,
                    const Inertial * imu, const std::filesystem::path & atlas_stem, bool fixed,
                    bool loop_closing = true);
}  // namespace atd_orbslam3
