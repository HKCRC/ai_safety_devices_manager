#include "spd_lidar/spd_lidar_core.hpp"

#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace spd_lidar {

namespace {

constexpr uint8_t kHeader1 = 0x55;
constexpr uint8_t kHeader2 = 0xAA;
constexpr uint8_t kCmdSingle = 0x88;
constexpr size_t kFrameSize = 8;

std::string formatHex(const std::vector<uint8_t>& data) {
  std::ostringstream oss;
  for (size_t i = 0; i < data.size(); ++i) {
    oss << " 0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        << static_cast<int>(data[i]);
  }
  return oss.str();
}

}  // namespace

uint8_t SpdLidarCore::checksumSend(const std::vector<uint8_t>& frame7) const {
  uint32_t sum = 0;
  for (size_t i = 2; i <= 6 && i < frame7.size(); ++i) {
    sum += frame7[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

uint8_t SpdLidarCore::checksumRecv(const uint8_t* frame8) const {
  uint32_t sum = 0;
  for (size_t i = 0; i < 7; ++i) {
    sum += frame8[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

bool SpdLidarCore::parseHexLine(const std::string& line, std::vector<uint8_t>* out) const {
  out->clear();
  std::istringstream iss(line);
  std::string token;
  while (iss >> token) {
    if (token == "0x" || token == "0X") {
      continue;
    }
    if (token.rfind("0x", 0) == 0 || token.rfind("0X", 0) == 0) {
      token = token.substr(2);
    }
    if (token.empty()) {
      continue;
    }
    char* end = nullptr;
    unsigned long value = std::strtoul(token.c_str(), &end, 16);
    if (end == token.c_str() || value > 0xFF) {
      return false;
    }
    out->push_back(static_cast<uint8_t>(value));
  }
  return !out->empty();
}

void SpdLidarCore::handleInputLine(const std::string& line) {
  if (line == "single") {
    std::vector<uint8_t> cmd = {kHeader1, kHeader2, kCmdSingle, 0xFF, 0xFF, 0xFF, 0xFF};
    cmd.push_back(checksumSend(cmd));
    on_send(cmd);
    emitLog("send:" + formatHex(cmd));
    return;
  }

  std::vector<uint8_t> cmd;
  if (parseHexLine(line, &cmd)) {
    if (cmd.size() == 7) {
      cmd.push_back(checksumSend(cmd));
    } else if (cmd.size() != 8) {
      emitLog("Need 7 or 8 bytes, got " + std::to_string(cmd.size()));
      return;
    }
    on_send(cmd);
    emitLog("send:" + formatHex(cmd));
    return;
  }

  emitLog("Invalid input. Use 'single' or hex bytes.");
}

void SpdLidarCore::handleRecvBytes(const uint8_t* data, size_t len) {
  recv_buf_.insert(recv_buf_.end(), data, data + len);
  emitFrameIfComplete();
}

void SpdLidarCore::reset() {
  recv_buf_.clear();
}

void SpdLidarCore::emitFrameIfComplete() {
  while (recv_buf_.size() >= kFrameSize) {
    size_t start = 0;
    while (start + 2 < recv_buf_.size()) {
      if (recv_buf_[start] == kHeader1 && recv_buf_[start + 1] == kHeader2 &&
          recv_buf_[start + 2] == kCmdSingle) {
        break;
      }
      ++start;
    }
    if (start > 0) {
      recv_buf_.erase(recv_buf_.begin(), recv_buf_.begin() + static_cast<long>(start));
    }
    if (recv_buf_.size() < kFrameSize) {
      break;
    }

    SpdLidarFrame parsed;
    parsed.raw.assign(recv_buf_.begin(), recv_buf_.begin() + static_cast<long>(kFrameSize));
    parsed.valid_header = (parsed.raw[0] == kHeader1 && parsed.raw[1] == kHeader2 &&
                           parsed.raw[2] == kCmdSingle);
    parsed.status = parsed.raw[3];
    parsed.data = static_cast<uint16_t>((parsed.raw[5] << 8) | parsed.raw[6]);
    parsed.checksum_ok = (checksumRecv(parsed.raw.data()) == parsed.raw[7]);

    on_frame(parsed);
    recv_buf_.erase(recv_buf_.begin(), recv_buf_.begin() + static_cast<long>(kFrameSize));
  }
}

void SpdLidarCore::emitLog(const std::string& text) {
  on_log(text);
}

}  // namespace spd_lidar
