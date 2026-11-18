#include "atd_recording/core.hpp"
#include "atd_recording/file_checks.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

using atd_recording::Core;
using atd_recording::Metadata;
using atd_recording::Request;
using atd_recording::State;

namespace
{
size_t checks{};
void check(bool value, int line)
{
  ++checks;
  if (!value) {throw std::runtime_error("check failed on line " + std::to_string(line));}
}
#define CHECK(expression) check((expression), __LINE__)
constexpr int64_t now = 10000000000LL;

void lifecycle()
{
  Core core;
  CHECK(core.state() == State::idle);
  CHECK(!core.last_request());
  auto decision = core.request({"joy", 1, now}, "joy", now);
  CHECK(decision.result == "accepted" && decision.start && !decision.stop);
  CHECK(core.state() == State::starting);
  CHECK(core.request({"joy", 1, now + 1}, "joy", now + 1).reason == "already_starting");
  CHECK(core.started() && core.state() == State::recording);
  CHECK(core.request({"propo", 1, now + 2}, "propo", now + 2).reason == "already_recording");
  decision = core.request({"propo", 2, now + 3}, "propo", now + 3);
  CHECK(decision.stop && decision.result == "accepted");
  CHECK(core.state() == State::stopping && core.cancelled());
  CHECK(core.request({"joy", 1, now + 4}, "joy", now + 4).reason == "busy_stopping");
  CHECK(core.request({"propo", 2, now + 4}, "propo", now + 4).reason == "already_stopping");
  core.finish(false);
  CHECK(core.state() == State::idle && !core.cancelled());
  CHECK(core.request({"joy", 2, now + 5}, "joy", now + 5).reason == "already_idle");
  CHECK(core.request({"joy", 1, now + 6}, "joy", now + 6).start);
  core.finish(true);
  CHECK(core.state() == State::error);
  CHECK(core.request({"joy", 1, now + 7}, "joy", now + 7).start);
  CHECK(core.last_request()->decision.reason == "start_accepted");
}

void cancellation_and_priority()
{
  Core core;
  CHECK(core.request({"joy", 1, now}, "joy", now).start);
  CHECK(core.request({"joy", 2, now}, "joy", now).stop);
  CHECK(!core.started());  // STOP arrived while worker was opening Writer.
  CHECK(core.request({"joy", 1, now}, "joy", now).reason == "stop_has_priority");
  core.finish(false);
  CHECK(core.request({"propo", 1, now}, "propo", now).reason == "stop_has_priority");
  CHECK(core.request({"propo", 1, now + 1}, "propo", now + 1).start);
  CHECK(core.request({"propo", 2, now + 1}, "propo", now + 1).stop);
  CHECK(core.request({"propo", 2, now + 1}, "propo", now + 1).reason == "duplicate_request");
  CHECK(core.request({"joy", 1, now - 1}, "joy", now).reason == "out_of_order_request");
  core.finish(false);
  core.shutdown();
  CHECK(core.request({"joy", 1, now + 2}, "joy", now + 2).reason == "shutting_down");
  Core recording;
  recording.request({"joy", 1, now}, "joy", now);
  CHECK(recording.started()); recording.shutdown();
  CHECK(recording.cancelled() && recording.state() == State::stopping);
}

void request_validation()
{
  Core core;
  CHECK(core.request({"propo", 1, now}, "joy", now).reason == "invalid_source");
  CHECK(core.request({"joy", 1, now}, "invalid", now).reason == "invalid_source");
  CHECK(core.request({"joy", 3, now}, "joy", now).reason == "invalid_action");
  CHECK(core.request({"joy", 1, 0}, "joy", now).reason == "zero_stamp");
  CHECK(core.request({"joy", 1, -1}, "joy", now).reason == "zero_stamp");
  CHECK(core.request({"joy", 1, now, false}, "joy", now).reason == "invalid_stamp");
  CHECK(core.request({"joy", 1, now + 1}, "joy", now).reason == "future_request");
  CHECK(core.request({"joy", 1, now - 1000000001LL}, "joy", now).reason == "stale_request");
  CHECK(core.state() == State::idle);
  CHECK(core.request({"joy", 1, now - 1000000000LL}, "joy", now).start); // Inclusive freshness limit.
  CHECK(core.started());
  CHECK(core.request({"joy", 2, now + 1}, "joy", now).reason == "future_request");
  CHECK(core.state() == State::recording); // Invalid requests never stop/start or poison watermark.
  CHECK(core.request({"joy", 2, now}, "joy", now).stop);
  bool invalid_timeout = false;
  try {Core invalid(0);} catch (const std::invalid_argument &) {invalid_timeout = true;}
  CHECK(invalid_timeout);
}

void metadata_validation()
{
  const std::vector<std::string> topics{"/image", "/info", "/tf_static", "/optional"};
  const std::map<std::string, std::string> required{{"/image", "Image"}, {"/info", "Info"}, {"/tf_static", "TF"}};
  const Metadata good{"mcap", 12, {{"/image", "Image", 5}, {"/info", "Info", 5}, {"/tf_static", "TF", 1}, {"/optional", "Other", 1}}};
  const auto reason = [&](const Metadata & value) {return atd_recording::validate_metadata(value, topics, required);};
  CHECK(reason(good).empty());
  auto bad = good; bad.storage = "sqlite3"; CHECK(reason(bad) == "metadata_invalid");
  bad = good; bad.message_count = 0; CHECK(reason(bad) == "empty_bag");
  bad = good; --bad.message_count; CHECK(reason(bad) == "metadata_invalid");
  bad = good; bad.topics[0].type = "wrong"; CHECK(reason(bad) == "required_type_mismatch");
  bad = good; bad.topics[0].count = 0; bad.message_count -= 5; CHECK(reason(bad) == "required_topic_empty");
  bad = good; bad.topics.erase(bad.topics.begin()); bad.message_count -= 5; CHECK(reason(bad) == "required_topic_missing");
  bad = good; bad.topics[0].name = "/unexpected"; CHECK(reason(bad) == "metadata_invalid");
  bad = good; bad.topics.push_back(bad.topics.front()); bad.message_count += 5; CHECK(reason(bad) == "metadata_invalid");
  bad = good; bad.topics.back().type.clear(); CHECK(reason(bad) == "metadata_invalid");
  CHECK(atd_recording::validate_metadata({"mcap", 1, {{"/image", "Image", UINT64_MAX}, {"/info", "Info", 2}}}, topics, {}).size() > 0);
  CHECK(atd_recording::validate_metadata({"mcap", 0, {}}, topics, {}).size() > 0); // Inputs cannot succeed empty.
  CHECK(atd_recording::validate_metadata({"mcap", 1, {{"/image", "Image", 1}}}, topics, {}).empty());
}

void file_validation()
{
  namespace fs = std::filesystem;
  std::string pattern = (fs::temp_directory_path() / "atd-record-files-XXXXXX").string();
  char * created = ::mkdtemp(pattern.data());
  if (!created) {throw std::runtime_error("mkdtemp failed");}
  const fs::path root(created);
  struct Cleanup {fs::path root; ~Cleanup() {std::error_code error; fs::remove_all(root, error);}} cleanup{root};
  const std::string magic("\x89MCAP0\r\n", 8);
  const auto write = [&](const std::string & file, const std::string & bytes) {std::ofstream out(root / file, std::ios::binary); out << bytes;};
  write("one.mcap", magic + "test envelope payload" + magic);
  write("two.mcap", magic + "second" + magic);
  const auto files = [&](std::vector<std::string> names) {return atd_recording::check_mcap_files(root, names);};
  CHECK(files({"one.mcap"}).reason.empty());
  CHECK(files({"one.mcap", "two.mcap"}).size_bytes == fs::file_size(root / "one.mcap") + fs::file_size(root / "two.mcap"));
  CHECK(files({}).reason == "mcap_missing");
  CHECK(files({"none.mcap"}).reason == "mcap_missing");
  CHECK(files({"one.mcap", "one.mcap"}).reason == "metadata_invalid");
  CHECK(files({"../one.mcap"}).reason == "metadata_invalid");
  CHECK(files({"/one.mcap"}).reason == "metadata_invalid");
  CHECK(files({"one.db3"}).reason == "metadata_invalid");
  CHECK(files({"sub/one.mcap"}).reason == "metadata_invalid");
  fs::create_symlink(root / "one.mcap", root / "link.mcap");
  CHECK(files({"link.mcap"}).reason == "mcap_missing");
  fs::create_directory(root / "directory.mcap");
  CHECK(files({"directory.mcap"}).reason == "mcap_missing");
  write("short.mcap", magic); CHECK(files({"short.mcap"}).reason == "mcap_missing");
  write("broken.mcap", magic + "incomplete trailer"); CHECK(files({"broken.mcap"}).reason == "mcap_missing");
  write("broken.mcap", "wrong header" + magic); CHECK(files({"broken.mcap"}).reason == "mcap_missing");
}
}  // namespace

int main()
{
  try {
    lifecycle(); cancellation_and_priority(); request_validation(); metadata_validation(); file_validation();
    std::cout << "recording core/files: " << checks << " checks passed\n";
    return 0;
  } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
