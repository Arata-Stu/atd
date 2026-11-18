#pragma once

#include <algorithm>
#include <array>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace atd_vehicle_interface
{
struct Device
{
  std::string path, serial;
};

inline std::string read_attribute(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  std::string value;
  std::getline(stream, value);
  return value;
}

// Linux sysfsでデバイスとinterfaceを照合し、tty番号を識別子にしない。
inline std::vector<Device> discover(
  const std::filesystem::path & sysfs, const std::filesystem::path & dev,
  const std::string & serial_filter = "", const std::string & path_filter = "")
{
  std::vector<Device> devices;
  std::error_code error;
  const auto wanted = path_filter.empty() ? std::filesystem::path{} : std::filesystem::canonical(path_filter, error);
  if (!path_filter.empty() && error) {return devices;}
  for (const auto & entry : std::filesystem::directory_iterator(sysfs, error)) {
    if (error) {break;}
    const auto name = entry.path().filename().string();
    if (!name.starts_with("ttyACM")) {continue;}
    auto ancestor = std::filesystem::canonical(entry.path() / "device", error);
    if (error) {error.clear(); continue;}
    std::string interface;
    for (; !ancestor.empty() && ancestor != ancestor.root_path(); ancestor = ancestor.parent_path()) {
      if (interface.empty()) {interface = read_attribute(ancestor / "bInterfaceNumber");}
      const auto vendor = read_attribute(ancestor / "idVendor");
      if (vendor.empty()) {continue;}
      const auto product = read_attribute(ancestor / "idProduct");
      const auto serial = read_attribute(ancestor / "serial");
      const auto label = read_attribute(ancestor / "product");
      const bool valid_serial = serial.size() == 16 && std::all_of(serial.begin(), serial.end(), [](unsigned char c) {
          return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
        });
      if ((vendor == "cafe" || vendor == "CAFE") && product == "4002" && interface == "02" &&
        label == "JPBB Rev6 Pico" && valid_serial && (serial_filter.empty() || serial_filter == serial))
      {
        auto path = std::filesystem::canonical(dev / name, error);
        if (!error && (path_filter.empty() || path == wanted)) {devices.push_back({path.string(), serial});}
        error.clear();
      }
      break;
    }
  }
  return devices;
}

class Serial
{
public:
  Serial() = default;
  Serial(const Serial &) = delete;
  Serial & operator=(const Serial &) = delete;
  ~Serial() {close();}
  void open(const std::string & path)
  {
    close();
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {fail("serial open");}
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {fail("serial already locked");}
    if (::ioctl(fd_, TIOCEXCL) != 0) {fail("serial exclusive access");}
    termios settings{};
    if (::tcgetattr(fd_, &settings) != 0) {fail("serial tcgetattr");}
    ::cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
#ifdef CRTSCTS
    settings.c_cflag &= ~CRTSCTS;
#endif
    ::cfsetispeed(&settings, B115200);
    ::cfsetospeed(&settings, B115200);
    if (::tcsetattr(fd_, TCSANOW, &settings) != 0 || ::tcflush(fd_, TCIOFLUSH) != 0) {fail("serial raw setup");}
    pending_ = 0;
  }
  void close()
  {
    if (fd_ >= 0) {
#ifdef TIOCNXCL
      ::ioctl(fd_, TIOCNXCL);
#endif
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
      fd_ = -1;
    }
  }
  bool connected() const {return fd_ >= 0;}
  void request_stream()
  {
    static constexpr char request[] = "INFO\r\nSTREAM ON\r\n";
    while (pending_ < sizeof(request) - 1) {
      const auto written = ::write(fd_, request + pending_, sizeof(request) - 1 - pending_);
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {return;}
      if (written <= 0) {fail("serial stream request");}
      pending_ += static_cast<size_t>(written);
    }
  }
  size_t read(char * bytes, size_t capacity)
  {
    pollfd descriptor{fd_, POLLIN, 0};
    const int result = ::poll(&descriptor, 1, 0);
    if (result < 0 && errno == EINTR) {return 0;}
    if (result < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {fail("serial disconnected");}
    if (!result || !(descriptor.revents & POLLIN)) {return 0;}
    const auto count = ::read(fd_, bytes, capacity);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {return 0;}
    if (count <= 0) {fail("serial read");}
    return static_cast<size_t>(count);
  }
private:
  [[noreturn]] void fail(const char * message)
  {
    const int error = errno;
    close();
    throw std::system_error(error ? error : EIO, std::generic_category(), message);
  }
  int fd_{-1};
  size_t pending_{};
};
}  // namespace atd_vehicle_interface
