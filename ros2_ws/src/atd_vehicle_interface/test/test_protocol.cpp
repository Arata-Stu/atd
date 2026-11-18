#include "atd_vehicle_interface/protocol.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace atd_vehicle_interface;

void check(bool condition, const char * message)
{
  if (!condition) {std::cerr << message << '\n'; std::exit(1);}
}

std::vector<std::string> fields()
{
  // firmware send_sensors() の並び。チャンネルと各状態値を異なる値にして転記ずれを検出する。
  std::vector<std::string> result{"JPX1", "T", "12", "600000", "4", "33", "2", "1", "7", "3"};
  for (unsigned channel = 0; channel < 16; ++channel) {result.push_back(std::to_string(172 + channel * 100));}
  const std::vector<std::string> tail{"44", "500000", "10000", "1", "1", "6000", "1", "1", "1", "128", "5", "6", "8", "9"};
  result.insert(result.end(), tail.begin(), tail.end());
  return result;
}

std::string frame(const std::vector<std::string> & values)
{
  std::string payload;
  for (const auto & value : values) {if (!payload.empty()) {payload += ',';} payload += value;}
  std::ostringstream result;
  result << payload << ',' << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << crc16(payload) << "\r\n";
  return result.str();
}

int main()
{
  check(!polling_gap_expired(10.0, 10.01), "regular 10ms polling gap allowed");
  const double rc_deadline = 10.0 + kRcFreshnessSeconds;
  check(!polling_gap_expired(10.0, rc_deadline), "exact 50ms polling gap allowed");
  check(polling_gap_expired(10.0, std::nextafter(rc_deadline, 11.0)), "polling gap beyond RC deadline expires");
  check(polling_gap_expired(10.0, 10.1), "100ms executor stall invalidates buffered history before 300ms transport timeout");
  check(polling_gap_expired(10.0, 9.99), "reversed host time invalidates polling epoch");
  check(crc16("123456789") == 0x29b1U, "CCITT-FALSE check vector");
  auto values = fields();
  check(values.size() == 40, "wire fixture field count");
  const auto packet = frame(values);
  auto parsed = parse_telemetry(packet);
  check(parsed.has_value(), "valid JPX1 frame");
  check(parsed->sequence == 12 && parsed->board_us == 600000 && parsed->sbus_frames == 33, "header counters");
  check(parsed->channels.front() == 172 && parsed->channels.back() == 1672, "all16 raw channels");
  check(parsed->sbus_flags == 3 && parsed->uart_errors == 7 && parsed->rpm_edges == 44, "flags and RPM indices");
  check(parsed->rpm_last_us32 == 500000 && parsed->rpm_period_us32 == 10000 && parsed->rpm_period_recent && parsed->rpm_seen, "RPM fields");
  check(parsed->vbec_mv == 6000 && parsed->power_ready && parsed->sbus_fresh && parsed->active_path == 1, "board state indices");
  check(parsed->fault_bits == 128 && parsed->tx0_dropped == 5 && parsed->tx1_dropped == 6 && parsed->rx0_rejected == 8 && parsed->rx1_rejected == 9, "diagnostic indices");
  auto damaged = packet;
  damaged[8] = damaged[8] == '0' ? '1' : '0';
  check(!parse_telemetry(damaged), "CRC rejects damaged payload");
  for (const auto & value : {"2048", "-1", "+1", " 1", "1x", ""}) {
    auto invalid = values;
    invalid[10] = value;
    check(!parse_telemetry(frame(invalid)), "bad raw value rejected even with valid CRC");
  }
  for (const size_t position : {29U, 30U, 32U, 33U}) {
    auto invalid = values;
    invalid[position] = "2";
    check(!parse_telemetry(frame(invalid)), "boolean values strict");
  }
  for (const auto & invalid_field : std::vector<std::pair<size_t, std::string>>{
    {2, "4294967296"}, {3, "18446744073709551616"}, {9, "16"}, {31, "65536"}, {34, "4"}})
  {
    auto invalid = values;
    invalid[invalid_field.first] = invalid_field.second;
    check(!parse_telemetry(frame(invalid)), "numeric overflow/range rejected");
  }
  auto missing = values;
  missing.pop_back();
  check(!parse_telemetry(frame(missing)), "missing field rejected");
  auto extra = values;
  extra.push_back("0");
  check(!parse_telemetry(frame(extra)), "extra field rejected");
  auto other = values;
  other[0] = "JPB1";
  check(!parse_telemetry(frame(other)), "other protocol rejected");

  Lines lines;
  unsigned count{};
  const auto callback = [&](std::string_view line) {check(parse_telemetry(line).has_value(), "fragments recover whole line"); ++count;};
  lines.feed(packet.data(), 13, callback);
  check(count == 0, "partial frame not emitted");
  lines.feed(packet.data() + 13, packet.size() - 13, callback);
  lines.feed(packet.data(), packet.size(), callback);
  check(count == 2, "CRLF not two frames");
  const std::string invalid_line = std::string(600, 'x') + "\r\n";
  lines.feed(invalid_line.data(), invalid_line.size(), callback);
  const std::string nul_line("JPX1\0,T,123\n", 12);
  lines.feed(nul_line.data(), nul_line.size(), callback);
  lines.feed(packet.data(), packet.size(), callback);
  check(count == 3 && lines.discarded() == 2, "overflow/NUL discarded through delimiter");

  Progress progress;
  auto sample = *parsed;
  check(progress.accept(sample, 10.0) == Progress::Result::accepted && progress.rc_valid(10.0, 0.3), "first valid frame");
  check(progress.accept(sample, 10.1) == Progress::Result::rejected, "duplicate not fresh");
  check(!progress.fresh(10.31, 0.3), "duplicate did not renew timeout");
  ++sample.sequence;
  sample.board_us += 20000;
  check(progress.accept(sample, 10.04) == Progress::Result::accepted, "new telemetry accepted with same SBUS counter");
  check(!progress.rc_valid(10.051, 0.3), "same SBUS frame cannot refresh RC deadline");
  ++sample.sequence;
  sample.board_us += 20000;
  ++sample.sbus_frames;
  sample.sbus_flags = 4;
  check(progress.accept(sample, 10.06) == Progress::Result::accepted && !progress.rc_valid(10.06, 0.3), "lost flag invalidates raw despite sbus_fresh");
  ++sample.sequence;
  sample.board_us += 20000;
  sample.sbus_flags = 0;
  check(progress.accept(sample, 10.07) == Progress::Result::accepted && !progress.rc_valid(10.07, 0.3), "clearing flag without RC progress cannot restore validity");
  ++sample.sequence;
  sample.board_us += 20000;
  ++sample.sbus_frames;
  check(progress.accept(sample, 10.08) == Progress::Result::accepted && progress.rc_valid(10.08, 0.3), "new good SBUS restores observation");
  sample.sequence -= 2;
  sample.board_us += 20000;
  check(progress.accept(sample, 10.09) == Progress::Result::rejected, "reverse seq rejected");
  sample.sequence = 0;
  sample.board_us = 100;
  sample.sbus_frames = 1;
  check(progress.accept(sample, 10.1) == Progress::Result::reset, "board clock reset starts new session");
  check(advancing(0, UINT32_MAX) && !advancing(UINT32_MAX, 0), "uint32 sequence wrap");
  std::cout << "pico protocol, range, fragmentation and freshness tests passed\n";
}
