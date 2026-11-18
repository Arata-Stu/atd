#include "atd_recording/core.hpp"
#include "atd_recording/native_session.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include "atd_interfaces/msg/operator_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"

namespace atd_recording
{
namespace
{
using Clock = std::chrono::steady_clock;

// Held until the worker has closed its last Writer. Never remove the lock file:
// replacing its inode would let another manager lock a different file.
class RootLock
{
public:
  explicit RootLock(const std::string & root)
  {
    std::filesystem::create_directories(root);
    const auto file = std::filesystem::path(root) / ".atd_recording.lock";
    fd_ = ::open(file.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (fd_ < 0) {throw std::runtime_error("cannot open recording root lock");}
    if (::flock(fd_, LOCK_EX | LOCK_NB)) {
      ::close(fd_); fd_ = -1;
      throw std::runtime_error("another recording manager holds this record_root");
    }
  }
  ~RootLock() {if (fd_ >= 0) {::close(fd_);}}
  RootLock(const RootLock &) = delete;
  RootLock & operator=(const RootLock &) = delete;
private:
  int fd_{-1};
};

std::string new_instance()
{
  // Only ROS namespace-safe characters; namespace isolates native Recorder services.
  std::ostringstream text;
  text << std::hex << std::chrono::system_clock::now().time_since_epoch().count()
       << '_' << ::getpid() << '_' << Clock::now().time_since_epoch().count();
  return text.str();
}
double seconds(Clock::duration duration) {return std::chrono::duration<double>(duration).count();}

bool valid_topic(const std::string & value)
{
  if (value.size() < 2 || value.front() != '/' || value.back() == '/') {return false;}
  bool first = true;
  for (size_t i = 1; i < value.size(); ++i) {
    const char ch = value[i];
    if (ch == '/') {if (first) {return false;} first = true; continue;}
    if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_' ||
      (!first && ch >= '0' && ch <= '9'))) {return false;}
    first = false;
  }
  return true;
}

std::vector<std::string> checked_topics(const std::vector<std::string> & values)
{
  std::set<std::string> unique;
  for (const auto & value : values) {
    if (!valid_topic(value) || !unique.insert(value).second || value == "/recording/status") {
      throw std::invalid_argument("topics must be unique absolute names; /recording/status is excluded");
    }
  }
  return values;
}

struct LastResult
{
  std::string path;
  ClosedBag bag;
  double elapsed{}, closed_at{};
};
}  // namespace

class RecordingManager : public rclcpp::Node
{
public:
  explicit RecordingManager(const rclcpp::NodeOptions & node_options)
  : Node("atd_recording_node", node_options), instance_(new_instance())
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    options_.profile = declare_parameter<std::string>("profile_name", "mapping", fixed);
    options_.root = declare_parameter<std::string>("record_root", "/data/record/rosbags", fixed);
    options_.topics = checked_topics(declare_parameter<std::vector<std::string>>("topics", {}, fixed));
    const auto required = checked_topics(declare_parameter<std::vector<std::string>>("required_topics", {}, fixed));
    const auto types = declare_parameter<std::vector<std::string>>("required_types", {}, fixed);
    options_.best_effort = checked_topics(declare_parameter<std::vector<std::string>>("best_effort_topics", {}, fixed));
    options_.transient_local = checked_topics(declare_parameter<std::vector<std::string>>("transient_local_topics", {}, fixed));
    const auto cache = declare_parameter<int64_t>("max_cache_bytes", 64 * 1024 * 1024, fixed);
    const auto free = declare_parameter<int64_t>("min_free_bytes", 1024LL * 1024 * 1024, fixed);
    options_.start_timeout_s = declare_parameter<double>("start_timeout_s", 10.0, fixed);
    options_.storage_check_period_s = declare_parameter<double>("storage_check_period_s", 0.5, fixed);
    const double event_timeout = declare_parameter<double>("event_timeout_s", 1.0, fixed);
    if ((options_.profile != "mapping" && options_.profile != "inputs" && options_.profile != "calibration") ||
      !std::filesystem::path(options_.root).is_absolute() || options_.topics.empty() ||
      required.size() != types.size() || cache <= 0 || cache > 1024LL * 1024 * 1024 || free <= 0 ||
      !std::isfinite(event_timeout) || event_timeout <= 0.0 || event_timeout > 60.0 ||
      !std::isfinite(options_.start_timeout_s) || options_.start_timeout_s <= 0.0 || options_.start_timeout_s > 300.0 ||
      !std::isfinite(options_.storage_check_period_s) || options_.storage_check_period_s < 0.1 || options_.storage_check_period_s > 10.0) {
      throw std::invalid_argument("invalid recording configuration");
    }
    if (has_parameter("use_sim_time") && get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("live recording manager requires use_sim_time=false");
    }
    const auto selected = [this](const std::string & topic) {
        return std::find(options_.topics.begin(), options_.topics.end(), topic) != options_.topics.end();
      };
    for (size_t i = 0; i < required.size(); ++i) {
      if (!selected(required[i]) || types[i].empty()) {throw std::invalid_argument("required topics/types must be selected");}
      options_.required.emplace(required[i], types[i]);
    }
    for (const auto & topic : options_.best_effort) {
      if (!selected(topic)) {throw std::invalid_argument("QoS override topic is not selected");}
    }
    for (const auto & topic : options_.transient_local) {
      if (!selected(topic) || std::find(options_.best_effort.begin(), options_.best_effort.end(), topic) != options_.best_effort.end()) {
        throw std::invalid_argument("transient-local QoS must be selected and not best effort");
      }
    }
    options_.cache_bytes = static_cast<uint64_t>(cache);
    options_.min_free_bytes = static_cast<uint64_t>(free);
    core_ = Core(static_cast<int64_t>(event_timeout * 1e9));
    root_lock_ = std::make_unique<RootLock>(options_.root);
    context_ = get_node_base_interface()->get_context();
    status_pub_ = create_publisher<std_msgs::msg::String>("/recording/status", rclcpp::QoS(1).reliable());
    const auto event_qos = rclcpp::QoS(10).reliable().durability_volatile();
    joy_sub_ = create_subscription<atd_interfaces::msg::OperatorEvent>("/teleop/events", event_qos,
      [this](atd_interfaces::msg::OperatorEvent::ConstSharedPtr event) {on_event(*event, "joy");});
    rc_sub_ = create_subscription<atd_interfaces::msg::OperatorEvent>("/rc/events", event_qos,
      [this](atd_interfaces::msg::OperatorEvent::ConstSharedPtr event) {on_event(*event, "propo");});
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {publish_status();});
    worker_ = std::thread([this]() {work();});
    RCLCPP_INFO(get_logger(), "Recording manager ready (%s); recording starts only on fresh operator event", options_.profile.c_str());
  }
  ~RecordingManager() override
  {
    shutting_down_.store(true);
    {std::lock_guard lock(mutex_); core_.shutdown();}
    condition_.notify_all();
    if (worker_.joinable()) {worker_.join();}
  }
private:
  void on_event(const atd_interfaces::msg::OperatorEvent & event, const std::string & expected)
  {
    const auto stamp = static_cast<int64_t>(event.header.stamp.sec) * 1000000000LL + event.header.stamp.nanosec;
    std::lock_guard lock(mutex_);
    const bool valid_stamp = event.header.stamp.sec >= 0 && event.header.stamp.nanosec < 1000000000U;
    const auto decision = core_.request({event.source, event.action, valid_stamp ? stamp : 0, valid_stamp}, expected, now().nanoseconds());
    if (decision.result == "rejected") {message_ = "操作要求を拒否しました: " + decision.reason;}
    if (decision.start) {pending_start_ = true; message_ = "記録の準備中";}
    if (decision.stop) {message_ = "記録を停止しています";}
    condition_.notify_all();
  }
  bool cancelled()
  {
    if (shutting_down_.load() || !rclcpp::ok(context_)) {return true;}
    std::lock_guard lock(mutex_);
    return core_.cancelled();
  }
  bool wait_for_cancel(double duration)
  {
    std::unique_lock lock(mutex_);
    condition_.wait_for(lock, std::chrono::duration<double>(duration), [this]() {
        return core_.cancelled() || shutting_down_.load() || !rclcpp::ok(context_);
      });
    return core_.cancelled() || shutting_down_.load() || !rclcpp::ok(context_);
  }
  std::string storage_problem() const
  {
    std::error_code error;
    const auto space = std::filesystem::space(options_.root, error);
    if (error) {return "storage_unavailable";}
    return space.available < options_.min_free_bytes ? "low_disk_space" : "";
  }
  std::string unavailable_topics() const
  {
    std::string missing;
    const auto graph = get_topic_names_and_types();
    for (const auto & [topic, type] : options_.required) {
      const auto found = graph.find(topic);
      if (found == graph.end() || found->second.size() != 1 || found->second.front() != type ||
        count_publishers(topic) == 0) {
        if (!missing.empty()) {missing += ", ";}
        missing += topic;
      }
    }
    return missing;
  }
  std::string unique_path()
  {
    // Recorder creates this directory itself. Existing paths are never reused.
    for (;;) {
      std::ostringstream id;
      id << "bag_" << instance_ << '_' << ++session_number_;
      const auto path = std::filesystem::path(options_.root) / id.str();
      if (!std::filesystem::exists(path)) {return path.string();}
    }
  }
  void run_session()
  {
    std::string path, failure, detail;
    bool opened = false, active = false;
    Clock::time_point started{};
    std::unique_ptr<NativeSession> session;
    ClosedBag closed;
    try {
      path = unique_path();
      {std::lock_guard lock(mutex_); active_path_ = path;}
      const auto deadline = Clock::now() + std::chrono::duration<double>(options_.start_timeout_s);
      while (!cancelled()) {
        failure = storage_problem();
        if (!failure.empty()) {break;}
        detail = unavailable_topics();
        if (detail.empty()) {break;}
        if (Clock::now() >= deadline) {failure = "missing_required_topics"; break;}
        wait_for_cancel(0.1);
      }
      if (cancelled()) {failure = "start_cancelled";}
      if (failure.empty()) {
        session = std::make_unique<NativeSession>(options_, path, instance_, context_);
        session->start();
        opened = true;
        started = Clock::now();
        std::lock_guard lock(mutex_);
        active = core_.started();
        if (active) {started_at_ = now().seconds(); started_steady_ = started; message_ = "記録中";}
        else {failure = "start_cancelled";}
      }
      while (active && !cancelled()) {
        if (!(detail = session->health()).empty()) {failure = "write_failed"; break;}
        failure = storage_problem();
        if (!failure.empty()) {break;}
        detail = unavailable_topics();
        if (!detail.empty()) {failure = "required_publishers_lost"; break;}
        if (wait_for_cancel(options_.storage_check_period_s)) {break;}
      }
    } catch (const std::exception & error) {
      failure = opened ? "write_failed" : "start_failed";
      detail = error.what();
    } catch (...) {failure = opened ? "write_failed" : "start_failed"; detail = "unknown recorder error";}
    const double elapsed = opened ? seconds(Clock::now() - started) : 0.0;
    {
      std::lock_guard lock(mutex_); core_.stopping(); message_ = "記録を閉じています";
      if (started_steady_) {stopped_elapsed_ = elapsed;}
    }
    if (session) {
      try {session->close();} catch (const std::exception & error) {failure = "write_failed"; detail = error.what();}
      catch (...) {failure = "write_failed"; detail = "unknown close error";}
      session.reset();
      try {closed = check_closed_bag(path, options_);}
      catch (const std::exception & error) {closed.reason = "metadata_invalid"; detail = error.what();}
    }
    if (shutting_down_.load() || !rclcpp::ok(context_)) {failure = "manager_shutdown";}
    if (!failure.empty()) {closed.success = false; closed.reason = failure;}
    if (closed.reason.empty()) {closed.reason = "start_failed";}
    const bool error = !closed.success && closed.reason != "start_cancelled" && closed.reason != "manager_shutdown";
    {
      std::lock_guard lock(mutex_);
      last_result_ = LastResult{path, closed, elapsed, now().seconds()};
      core_.finish(error);
      active_path_.clear(); started_at_.reset(); started_steady_.reset(); stopped_elapsed_.reset();
      message_ = closed.success ? "記録を保存しました" : closed.reason == "start_cancelled" ? "記録開始を取り消しました" :
        closed.reason == "manager_shutdown" ? "終了時に記録を閉じました" : "記録に失敗しました: " + closed.reason;
      if (!detail.empty()) {message_ += " (" + detail + ")";}
    }
  }
  void work() noexcept
  {
    try {
      for (;;) {
        std::unique_lock lock(mutex_);
        condition_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
            return pending_start_ || shutting_down_.load() || !rclcpp::ok(context_);
          });
        if (shutting_down_.load() || !rclcpp::ok(context_)) {break;}
        if (!pending_start_) {continue;}
        pending_start_ = false;
        lock.unlock();
        run_session();
      }
    } catch (const std::exception & error) {
      std::lock_guard lock(mutex_); core_.finish(true); message_ = "記録 worker エラー: " + std::string(error.what());
    } catch (...) {std::lock_guard lock(mutex_); core_.finish(true); message_ = "記録 worker エラー";}
  }
  void publish_status()
  {
    YAML::Node status;
    {
      std::lock_guard lock(mutex_);
      status["schema_version"] = 1; status["instance_id"] = instance_; status["stamp"] = now().seconds();
      status["state"] = name(core_.state()); status["profile"] = options_.profile;
      status["bag_path"] = active_path_; status["message"] = message_;
      status["elapsed_s"] = stopped_elapsed_ ? YAML::Node(*stopped_elapsed_) : started_steady_ ?
        YAML::Node(seconds(Clock::now() - *started_steady_)) : YAML::Node(YAML::NodeType::Null);
      status["started_at"] = started_at_ ? YAML::Node(*started_at_) : YAML::Node(YAML::NodeType::Null);
      status["last_request"] = YAML::Node(YAML::NodeType::Null);
      if (core_.last_request()) {
        const auto & last = *core_.last_request(); YAML::Node request;
        request["source"] = last.request.source; request["action"] = last.request.action == 1U ? "start" : "stop";
        request["stamp"] = static_cast<double>(last.request.stamp_ns) / 1e9;
        request["result"] = last.decision.result; request["reason"] = last.decision.reason;
        status["last_request"] = request;
      }
      status["last_result"] = YAML::Node(YAML::NodeType::Null);
      if (last_result_) {
        const auto & last = *last_result_; YAML::Node result;
        result["bag_path"] = last.path; result["success"] = last.bag.success;
        result["reason"] = last.bag.reason; result["elapsed_s"] = last.elapsed; result["closed_at"] = last.closed_at;
        result["size_bytes"] = last.bag.have_size ? YAML::Node(last.bag.size_bytes) : YAML::Node(YAML::NodeType::Null);
        result["message_count"] = last.bag.have_count ? YAML::Node(last.bag.message_count) : YAML::Node(YAML::NodeType::Null);
        status["last_result"] = result;
      }
    }
    YAML::Emitter output; output << status;
    std_msgs::msg::String message; message.data = output.c_str(); status_pub_->publish(message);
  }
  Options options_;
  std::string instance_, active_path_, message_{"待機中"};
  std::unique_ptr<RootLock> root_lock_;
  rclcpp::Context::SharedPtr context_;
  Core core_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::atomic_bool shutting_down_{false};
  bool pending_start_{};
  uint64_t session_number_{};
  std::optional<double> started_at_;
  std::optional<double> stopped_elapsed_;
  std::optional<Clock::time_point> started_steady_;
  std::optional<LastResult> last_result_;
  std::thread worker_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<atd_interfaces::msg::OperatorEvent>::SharedPtr joy_sub_, rc_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace atd_recording
RCLCPP_COMPONENTS_REGISTER_NODE(atd_recording::RecordingManager)
