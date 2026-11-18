#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

namespace atd_recording
{
struct FileCheck {std::string reason; uint64_t size_bytes{};};

// Check the MCAP envelope and all metadata references after Writer.close().
// This does not decode records or replace a full MCAP CRC/data-quality audit.
inline FileCheck check_mcap_files(const std::filesystem::path & directory,
  const std::vector<std::string> & relatives)
{
  namespace fs = std::filesystem;
  if (relatives.empty()) {return {"mcap_missing", 0};}
  std::set<std::string> files;
  uint64_t total{};
  static constexpr std::array<char, 8> magic{static_cast<char>(0x89), 'M', 'C', 'A', 'P', '0', '\r', '\n'};
  for (const auto & relative : relatives) {
    const fs::path file(relative);
    if (file.is_absolute() || file.has_parent_path() || file.extension() != ".mcap" ||
      !files.insert(relative).second) {return {"metadata_invalid", 0};}
    const auto full = directory / file;
    std::error_code error;
    if (fs::is_symlink(full, error) || error || !fs::is_regular_file(full, error) || error) {
      return {"mcap_missing", 0};
    }
    const auto size = fs::file_size(full, error);
    if (error || size < magic.size() * 2 || total > UINT64_MAX - size) {return {"mcap_missing", 0};}
    std::ifstream stream(full, std::ios::binary);
    std::array<char, 8> start{}, end{};
    stream.read(start.data(), start.size());
    stream.seekg(-static_cast<std::streamoff>(end.size()), std::ios::end);
    stream.read(end.data(), end.size());
    if (!stream.good() || start != magic || end != magic) {return {"mcap_missing", 0};}
    total += size;
  }
  return {"", total};
}
}  // namespace atd_recording
