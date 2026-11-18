#include "atd_vehicle_interface/serial.hpp"

#include <cstdlib>
#include <iostream>

using namespace atd_vehicle_interface;

void check(bool condition, const char * message)
{
  if (!condition) {std::cerr << message << '\n'; std::exit(1);}
}

void attribute(const std::filesystem::path & path, const char * value)
{
  std::ofstream(path) << value << '\n';
}

int main()
{
  char folder[] = "/tmp/atd-pico-serial-XXXXXX";
  const char * created = ::mkdtemp(folder);
  check(created != nullptr, "temporary folder");
  const std::filesystem::path root(created);
  const auto sys = root / "class";
  const auto dev = root / "dev";
  const auto board = root / "usb";
  std::filesystem::create_directories(sys / "ttyACM7");
  std::filesystem::create_directories(dev);
  std::filesystem::create_directories(board / "1-1:1.2");
  attribute(board / "idVendor", "cafe");
  attribute(board / "idProduct", "4002");
  attribute(board / "product", "JPBB Rev6 Pico");
  attribute(board / "serial", "0123456789ABCDEF");
  attribute(board / "1-1:1.2/bInterfaceNumber", "02");
  std::filesystem::create_directory_symlink(board / "1-1:1.2", sys / "ttyACM7/device");
  attribute(dev / "ttyACM7", "fixture");
  auto devices = discover(sys, dev);
  check(devices.size() == 1 && devices[0].serial == "0123456789ABCDEF", "USB identity and interface discovery");
  check(discover(sys, dev, "different").empty(), "serial filter");
  check(discover(sys, dev, "", (dev / "missing").string()).empty(), "nonexistent path filter");
  std::filesystem::create_symlink(dev / "ttyACM7", dev / "by-id");
  check(discover(sys, dev, "", (dev / "by-id").string()).size() == 1, "by-id symlink supported");
  attribute(board / "1-1:1.2/bInterfaceNumber", "00");
  check(discover(sys, dev).empty(), "control interface never selected");
  attribute(board / "1-1:1.2/bInterfaceNumber", "02");
  attribute(board / "product", "other TinyUSB example");
  check(discover(sys, dev).empty(), "shared development VID PID alone is insufficient");
  attribute(board / "product", "JPBB Rev6 Pico");
  std::filesystem::create_directories(sys / "ttyACM9");
  std::filesystem::create_directory_symlink(board / "1-1:1.2", sys / "ttyACM9/device");
  attribute(dev / "ttyACM9", "fixture");
  check(discover(sys, dev).size() == 2, "ambiguous matches remain visible for caller to reject");
  std::filesystem::remove_all(root);

  const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  check(master >= 0 && ::grantpt(master) == 0 && ::unlockpt(master) == 0, "PTY allocation");
  const char * slave_name = ::ptsname(master);
  check(slave_name != nullptr, "PTY slave");
  Serial serial;
  serial.open(slave_name);
  check(serial.connected(), "serial opened");
  char bytes[64]{};
  check(serial.read(bytes, sizeof(bytes)) == 0, "nonblocking empty read");
  serial.request_stream();
  auto count = ::read(master, bytes, sizeof(bytes));
  check(count == 17 && std::string(bytes, static_cast<size_t>(count)) == "INFO\r\nSTREAM ON\r\n", "only read-only monitor commands sent");
  serial.request_stream();
  check(::read(master, bytes, sizeof(bytes)) < 0 && (errno == EAGAIN || errno == EWOULDBLOCK), "request is not repeatedly sent");
  bool exclusive{};
  try {Serial another; another.open(slave_name);} catch (const std::system_error &) {exclusive = true;}
  check(exclusive, "exclusive serial access");
  check(::write(master, "sample\r\n", 8) == 8, "PTY write sample");
  count = static_cast<ssize_t>(serial.read(bytes, sizeof(bytes)));
  check(count == 8 && std::string(bytes, static_cast<size_t>(count)) == "sample\r\n", "raw input bytes");
  ::close(master);
  bool disconnected{};
  try {serial.read(bytes, sizeof(bytes));} catch (const std::system_error &) {disconnected = true;}
  check(disconnected && !serial.connected(), "disconnect closes FD");
  std::cout << "pico USB discovery, exclusive PTY transport and disconnect tests passed\n";
}
