#include "atd_recording/native_session.hpp"
#include "atd_recording/core.hpp"
#include "atd_recording/file_checks.hpp"

#include <atomic>
#include <chrono>
#include <exception>
#include <mutex>
#include <thread>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/recorder.hpp"

namespace atd_recording
{
namespace
{
class Recorder : public rosbag2_transport::Recorder
{
public:
  using rosbag2_transport::Recorder::Recorder;
  std::string active_uri() {return get_storage_options().uri;}
};

}  // namespace

class NativeSession::Impl
{
public:
  Impl(const Options & options, std::string path, const std::string & instance,
    rclcpp::Context::SharedPtr context) : path_(std::move(path)), context_(context)
  {
    rosbag2_storage::StorageOptions storage;
    storage.uri = path_;
    storage.storage_id = "mcap";
    storage.max_cache_size = options.cache_bytes;
    rosbag2_transport::RecordOptions record;
    record.topics = options.topics;
    record.all_topics = false;
    record.start_paused = false;
    record.use_sim_time = false;
    record.disable_keyboard_controls = true;
    for (const auto & topic : options.best_effort) {
      record.topic_qos_profile_overrides.emplace(topic, rclcpp::QoS(10).best_effort().durability_volatile());
    }
    for (const auto & topic : options.transient_local) {
      record.topic_qos_profile_overrides.insert_or_assign(topic,
        rclcpp::QoS(100).reliable().transient_local());
    }
    writer_ = std::make_shared<rosbag2_cpp::Writer>();
    rclcpp::NodeOptions node_options;
    node_options.context(context).use_global_arguments(false).start_parameter_services(false);
    node_options.arguments({"--ros-args", "-r", "__ns:=/atd_recording_internal/s" + instance});
    // This overload does NOT record in the constructor. Public record() has no bool return.
    recorder_ = std::make_shared<Recorder>(writer_, storage, record, "native_recorder", node_options);
    recorder_->set_on_start_recording_callback([this]() {started_.store(true);});
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(recorder_);
  }
  ~Impl() {try {close();} catch (...) {}}
  void start()
  {
    spin_thread_ = std::thread([this]() {
        // Lyrical cancel() before spin() is not persistent. A separate stop flag
        // and bounded spin_once avoid a startup/open-failure -> join deadlock.
        try {
          while (!stop_spin_.load() && rclcpp::ok(context_)) {
            executor_->spin_once(std::chrono::milliseconds(100));
          }
        }
        catch (const std::exception & error) {std::lock_guard lock(error_mutex_); spin_error_ = error.what();}
        catch (...) {std::lock_guard lock(error_mutex_); spin_error_ = "recorder executor failed";}
        spin_exited_.store(true);
      });
    recorder_->record(path_);
    if (!started_.load()) {throw std::runtime_error("Recorder returned without start callback");}
    if (recorder_->active_uri() != path_) {throw std::runtime_error("Recorder changed requested bag path");}
  }
  void close()
  {
    if (closed_) {return;}
    closed_ = true;
    std::exception_ptr failure;
    try {recorder_->stop();} catch (...) {failure = std::current_exception();}
    // record() can throw after writer.open but before in_recording_ becomes true;
    // Recorder.stop() is then a no-op. Keep and close the underlying writer too.
    try {writer_->close();} catch (...) {if (!failure) {failure = std::current_exception();}}
    // If close threw after clearing the writer's open flag, let Recorder clear its state.
    try {recorder_->stop();} catch (...) {if (!failure) {failure = std::current_exception();}}
    stop_spin_.store(true);
    try {executor_->cancel();} catch (...) {if (!failure) {failure = std::current_exception();}}
    if (spin_thread_.joinable()) {spin_thread_.join();}
    try {executor_->remove_node(recorder_);} catch (...) {if (!failure) {failure = std::current_exception();}}
    if (failure) {std::rethrow_exception(failure);}
  }
  std::string health() const
  {
    std::lock_guard lock(error_mutex_);
    return !spin_error_.empty() ? spin_error_ : spin_exited_.load() ? "recorder executor stopped" : "";
  }
private:
  std::string path_;
  rclcpp::Context::SharedPtr context_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  std::shared_ptr<Recorder> recorder_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic_bool started_{false}, spin_exited_{false}, stop_spin_{false};
  mutable std::mutex error_mutex_;
  std::string spin_error_;
  bool closed_{};
};

NativeSession::NativeSession(const Options & options, const std::string & path,
  const std::string & instance, rclcpp::Context::SharedPtr context)
: impl_(std::make_unique<Impl>(options, path, instance, std::move(context))) {}
NativeSession::~NativeSession() = default;
void NativeSession::start() {impl_->start();}
void NativeSession::close() {impl_->close();}
std::string NativeSession::health() const {return impl_->health();}

ClosedBag check_closed_bag(const std::string & path, const Options & options)
{
  ClosedBag result;
  rosbag2_storage::MetadataIo reader;
  if (!reader.metadata_file_exists(path)) {result.reason = "metadata_missing"; return result;}
  rosbag2_storage::BagMetadata metadata;
  try {metadata = reader.read_metadata(path);} catch (...) {result.reason = "metadata_invalid"; return result;}
  result.message_count = metadata.message_count;
  result.have_count = true;
  const auto files = check_mcap_files(path, metadata.relative_file_paths);
  if (!files.reason.empty()) {result.reason = files.reason; return result;}
  result.size_bytes = files.size_bytes;
  result.have_size = true;
  Metadata summary{metadata.storage_identifier, metadata.message_count, {}};
  for (const auto & topic : metadata.topics_with_message_count) {
    summary.topics.push_back({topic.topic_metadata.name, topic.topic_metadata.type, topic.message_count});
  }
  result.reason = validate_metadata(summary, options.topics, options.required);
  result.success = result.reason.empty();
  if (result.success) {result.reason = "recording_completed";}
  return result;
}
}  // namespace atd_recording
