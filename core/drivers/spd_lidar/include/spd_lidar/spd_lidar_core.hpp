#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <boost/signals2.hpp>

namespace spd_lidar {

struct SpdLidarFrame {
  bool valid_header = false;
  bool checksum_ok = false;
  uint8_t status = 0;
  uint16_t data = 0;
  std::vector<uint8_t> raw;
};

class SpdLidarCore {
 public:
  boost::signals2::signal<void(const std::vector<uint8_t>&)> on_send;
  boost::signals2::signal<void(const SpdLidarFrame&)> on_frame;
  boost::signals2::signal<void(const std::string&)> on_log;

  void handleInputLine(const std::string& line);
  void handleRecvBytes(const uint8_t* data, size_t len);
  void reset();

 private:
  uint8_t checksumSend(const std::vector<uint8_t>& frame7) const;
  uint8_t checksumRecv(const uint8_t* frame8) const;
  bool parseHexLine(const std::string& line, std::vector<uint8_t>* out) const;
  void emitFrameIfComplete();
  void emitLog(const std::string& text);

  std::vector<uint8_t> recv_buf_;
};

}  // namespace spd_lidar
