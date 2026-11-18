#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace atd_recording
{
enum class State {idle, starting, recording, stopping, error};
inline const char * name(State state)
{
  switch (state) {
    case State::idle: return "idle";
    case State::starting: return "starting";
    case State::recording: return "recording";
    case State::stopping: return "stopping";
    case State::error: return "error";
  }
  return "error";
}
struct Request
{
  std::string source;
  uint8_t action{};
  int64_t stamp_ns{};
  bool stamp_valid{true};
};
struct Decision
{
  std::string result, reason;
  bool start{}, stop{};
};
struct LastRequest
{
  Request request;
  Decision decision;
};

// Caller holds its own mutex. Only the worker performs Recorder / filesystem I/O.
class Core
{
public:
  explicit Core(int64_t timeout_ns = 1000000000LL) : timeout_ns_(timeout_ns)
  {
    if (timeout_ns <= 0) {throw std::invalid_argument("event timeout must be positive");}
  }
  Decision request(const Request & request, const std::string & expected_source, int64_t now_ns)
  {
    auto complete = [&](std::string result, std::string reason, bool start = false, bool stop = false) {
        Decision value{std::move(result), std::move(reason), start, stop};
        if (request.action == 1U || request.action == 2U) {last_ = LastRequest{request, value};}
        return value;
      };
    if ((expected_source != "joy" && expected_source != "propo") || request.source != expected_source) {
      return complete("rejected", "invalid_source");
    }
    if (request.action != 1U && request.action != 2U) {return complete("rejected", "invalid_action");}
    if (!request.stamp_valid) {return complete("rejected", "invalid_stamp");}
    if (request.stamp_ns <= 0) {return complete("rejected", "zero_stamp");}
    if (request.stamp_ns > now_ns) {return complete("rejected", "future_request");}
    if (now_ns - request.stamp_ns > timeout_ns_) {return complete("rejected", "stale_request");}
    auto & seen = seen_[request.source];
    if (request.stamp_ns < seen.stamp) {return complete("rejected", "out_of_order_request");}
    if (request.stamp_ns == seen.stamp) {
      if (request.action == seen.action) {return complete("rejected", "duplicate_request");}
      if (seen.action == 2U) {return complete("rejected", "stop_has_priority");}
    }
    seen = {request.stamp_ns, request.action};
    if (shutting_down_) {return complete("rejected", "shutting_down");}
    if (request.action == 1U) {
      if (request.stamp_ns <= last_stop_stamp_) {return complete("rejected", "stop_has_priority");}
      if (state_ == State::stopping) {return complete("rejected", "busy_stopping");}
      if (state_ == State::starting) {return complete("ignored", "already_starting");}
      if (state_ == State::recording) {return complete("ignored", "already_recording");}
      state_ = State::starting;
      cancel_ = false;
      return complete("accepted", "start_accepted", true);
    }
    last_stop_stamp_ = std::max(last_stop_stamp_, request.stamp_ns);
    if (state_ == State::idle || state_ == State::error) {return complete("ignored", "already_idle");}
    if (state_ == State::stopping) {return complete("ignored", "already_stopping");}
    cancel_ = true;
    state_ = State::stopping;
    return complete("accepted", "stop_accepted", false, true);
  }
  bool started()
  {
    if (state_ != State::starting || cancel_) {return false;}
    state_ = State::recording;
    return true;
  }
  void stopping() {state_ = State::stopping;}
  void finish(bool error) {state_ = error ? State::error : State::idle; cancel_ = false;}
  void shutdown()
  {
    shutting_down_ = true;
    cancel_ = true;
    if (state_ == State::starting || state_ == State::recording) {state_ = State::stopping;}
  }
  State state() const {return state_;}
  bool cancelled() const {return cancel_;}
  const std::optional<LastRequest> & last_request() const {return last_;}
private:
  struct Seen {int64_t stamp{}; uint8_t action{};};
  std::map<std::string, Seen> seen_;
  std::optional<LastRequest> last_;
  int64_t timeout_ns_;
  int64_t last_stop_stamp_{};
  State state_{State::idle};
  bool cancel_{}, shutting_down_{};
};

struct TopicCount {std::string name, type; uint64_t count{};};
struct Metadata
{
  std::string storage;
  uint64_t message_count{};
  std::vector<TopicCount> topics;
};
inline std::string validate_metadata(
  const Metadata & metadata, const std::vector<std::string> & allowed,
  const std::map<std::string, std::string> & required)
{
  if (metadata.storage != "mcap") {return "metadata_invalid";}
  if (!metadata.message_count) {return "empty_bag";}
  std::map<std::string, TopicCount> topics;
  uint64_t sum{};
  for (const auto & topic : metadata.topics) {
    if (std::find(allowed.begin(), allowed.end(), topic.name) == allowed.end() ||
      !topics.emplace(topic.name, topic).second || topic.type.empty() ||
      sum > UINT64_MAX - topic.count) {return "metadata_invalid";}
    sum += topic.count;
  }
  if (sum != metadata.message_count) {return "metadata_invalid";}
  for (const auto & [name, type] : required) {
    const auto found = topics.find(name);
    if (found == topics.end()) {return "required_topic_missing";}
    if (found->second.type != type) {return "required_type_mismatch";}
    if (!found->second.count) {return "required_topic_empty";}
  }
  return "";
}
}  // namespace atd_recording
