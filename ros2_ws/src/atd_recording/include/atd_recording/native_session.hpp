#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/context.hpp"

namespace atd_recording
{
struct Options
{
  std::string profile, root;
  std::vector<std::string> topics, best_effort, transient_local;
  std::map<std::string, std::string> required;
  uint64_t cache_bytes{}, min_free_bytes{};
  double start_timeout_s{}, storage_check_period_s{};
};
struct ClosedBag
{
  bool success{};
  std::string reason;
  uint64_t size_bytes{}, message_count{};
  bool have_size{}, have_count{};
};

// Uses the public non-auto-start Recorder constructor. All calls except health()
// are made by one manager worker. Own executor services recorder subscriptions.
class NativeSession
{
public:
  NativeSession(const Options &, const std::string & path, const std::string & instance,
    rclcpp::Context::SharedPtr context);
  ~NativeSession();
  NativeSession(const NativeSession &) = delete;
  NativeSession & operator=(const NativeSession &) = delete;
  void start();
  void close();
  std::string health() const;
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
ClosedBag check_closed_bag(const std::string & path, const Options & options);
}  // namespace atd_recording
