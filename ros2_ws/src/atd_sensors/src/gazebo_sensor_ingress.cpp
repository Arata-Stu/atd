#include <array>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace atd_sensors {
namespace {
using namespace std::chrono_literals;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Imu = sensor_msgs::msg::Imu;

struct FileDescriptor {
  int fd{-1};
  ~FileDescriptor() { reset(); }
  void reset(int next = -1) {
    if (fd >= 0) { ::close(fd); }
    fd = next;
  }
  FileDescriptor() = default;
  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor &operator=(const FileDescriptor &) = delete;
};

template<typename Message>
std::unique_ptr<Message> decode(const uint8_t * bytes, size_t length) {
  rclcpp::SerializedMessage serialized(length);
  auto & buffer = serialized.get_rcl_serialized_message();
  std::memcpy(buffer.buffer, bytes, length);
  buffer.buffer_length = length;
  auto message = std::make_unique<Message>();
  rclcpp::Serialization<Message>().deserialize_message(&serialized, message.get());
  return message;
}
}  // namespace

class GazeboSensorIngress : public rclcpp::Node {
public:
  explicit GazeboSensorIngress(const rclcpp::NodeOptions & options)
  : Node("gazebo_sensor_ingress", options) {
    const auto * domain = std::getenv("ROS_DOMAIN_ID");
    endpoint_ = "atd-gazebo-sensors-v1-" + std::to_string(domain ? std::stoul(domain) : 0);
    timer_ = create_wall_timer(1ms, [this] { receive(); });
  }

private:
  void connect_to_gazebo() {
    next_attempt_ = std::chrono::steady_clock::now() + 250ms;
    int socket_type = SOCK_SEQPACKET;
#ifdef __linux__
    socket_type |= SOCK_NONBLOCK | SOCK_CLOEXEC;
#endif
    connection_.reset(::socket(AF_UNIX, socket_type, 0));
    if (connection_.fd < 0) { return; }
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    // The leading NUL selects a Linux abstract socket in the host network namespace.
    std::memcpy(address.sun_path + 1, endpoint_.data(), endpoint_.size());
    auto size = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + endpoint_.size());
    if (::connect(connection_.fd, reinterpret_cast<sockaddr *>(&address), size) != 0) {
      connection_.reset();
      return;
    }
#ifdef __linux__
    ucred peer{};
    socklen_t length = sizeof(peer);
    if (getsockopt(connection_.fd, SOL_SOCKET, SO_PEERCRED, &peer, &length) || peer.uid != geteuid()) {
      connection_.reset();
    }
#endif
  }

  void advertise(bool inertial) {
    const auto qos = rclcpp::SensorDataQoS();
    for (size_t side = 0; side < images_.size(); ++side) {
      auto root = "/camera/camera/infra" + std::to_string(side + 1);
      images_[side] = create_publisher<Image>(root + "/image_rect_raw", qos);
      cameras_[side] = create_publisher<CameraInfo>(root + "/camera_info", qos);
    }
    if (inertial) {
      const char * topics[] = {"/camera/camera/imu", "/camera/camera/gyro/sample", "/camera/camera/accel/sample"};
      for (size_t i = 0; i < imu_.size(); ++i) { imu_[i] = create_publisher<Imu>(topics[i], qos); }
    }
    RCLCPP_INFO(get_logger(), "Gazebo の共有メモリに接続しました (stereo%s)", inertial ? " + IMU" : "");
  }

  void disconnect() {
    images_ = {};
    cameras_ = {};
    imu_ = {};
    connection_.reset();
    next_attempt_ = std::chrono::steady_clock::now() + 250ms;
  }

  void publish(uint8_t channel, const uint8_t * bytes, size_t length, int fd) {
    if (channel == 255 && length == 1 && !images_[0] && bytes[0] <= 1 && fd < 0) {
      advertise(bytes[0] != 0);
      return;
    }
    if (!images_[0]) { throw std::runtime_error("センサー構成を受け取っていません"); }
    if (channel < 2) {
      auto image = decode<Image>(bytes, length);
      const uint64_t size = static_cast<uint64_t>(image->step) * image->height;
      struct stat info{};
      if (fd < 0 || fstat(fd, &info) || !S_ISREG(info.st_mode) || size == 0 || size > 16 * 1024 * 1024 ||
          info.st_size != static_cast<off_t>(size) || image->width == 0 || image->step < image->width ||
          image->encoding != "mono8" || !image->data.empty()) {
        throw std::runtime_error("共有メモリの画像と Image header が一致しません");
      }
      image->data.resize(size);
      if (pread(fd, image->data.data(), size, 0) != static_cast<ssize_t>(size)) {
        throw std::runtime_error("共有メモリの画像を読み込めません");
      }
      images_[channel]->publish(std::move(image));
    } else if (channel < 4 && fd < 0) {
      cameras_[channel - 2]->publish(decode<CameraInfo>(bytes, length));
    } else if (channel >= 4 && channel < 7 && fd < 0 && imu_[channel - 4]) {
      imu_[channel - 4]->publish(decode<Imu>(bytes, length));
    } else {
      throw std::runtime_error("Gazebo センサー channel またはファイル記述子が不正です");
    }
  }

  void receive() {
    if (connection_.fd < 0) {
      if (std::chrono::steady_clock::now() >= next_attempt_) { connect_to_gazebo(); }
      return;
    }
    for (int i = 0; i < 64; ++i) {
      std::array<uint8_t, 4096> data;
      alignas(cmsghdr) unsigned char control[CMSG_SPACE(sizeof(int))]{};
      iovec buffer{data.data(), data.size()};
      msghdr message{};
      message.msg_iov = &buffer;
      message.msg_iovlen = 1;
      message.msg_control = control;
      message.msg_controllen = sizeof(control);
      int flags = MSG_DONTWAIT;
#ifdef __linux__
      flags |= MSG_CMSG_CLOEXEC;
#endif
      ssize_t count = recvmsg(connection_.fd, &message, flags);
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) { return; }
      if (count <= 0) { disconnect(); return; }
      FileDescriptor pixels;
      for (auto * header = CMSG_FIRSTHDR(&message); header; header = CMSG_NXTHDR(&message, header)) {
        if (header->cmsg_level == SOL_SOCKET && header->cmsg_type == SCM_RIGHTS) {
          size_t fds = (header->cmsg_len - CMSG_LEN(0)) / sizeof(int);
          for (size_t index = 0; index < fds; ++index) {
            int fd;
            std::memcpy(&fd, CMSG_DATA(header) + index * sizeof(int), sizeof(fd));
            pixels.reset(fd);
          }
        }
      }
      try {
        if (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) { throw std::runtime_error("パケットが長すぎます"); }
        publish(data[0], data.data() + 1, static_cast<size_t>(count) - 1, pixels.fd);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Gazebo センサーの受信に失敗しました: %s", error.what());
        disconnect();
        return;
      }
    }
  }

  std::string endpoint_;
  FileDescriptor connection_;
  std::chrono::steady_clock::time_point next_attempt_{};
  std::array<rclcpp::Publisher<Image>::SharedPtr, 2> images_;
  std::array<rclcpp::Publisher<CameraInfo>::SharedPtr, 2> cameras_;
  std::array<rclcpp::Publisher<Imu>::SharedPtr, 3> imu_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace atd_sensors
RCLCPP_COMPONENTS_REGISTER_NODE(atd_sensors::GazeboSensorIngress)
