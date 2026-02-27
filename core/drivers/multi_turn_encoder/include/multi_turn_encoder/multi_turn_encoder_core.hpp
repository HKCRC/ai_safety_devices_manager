#pragma once

#include "multi_turn_encoder_rtu.h"

#include <memory>
#include <mutex>
#include <string>

namespace multi_turn_encoder {

class MultiTurnEncoderCore {
 public:
  enum class Transport {
    RTU,
    TCP,
  };

  struct LatestData {
    bool valid;
    bool connected;
    bool running;
    double timestamp;
    double duration;
    double turns_raw;
    double turns_filtered;
    double velocity;
  };

  MultiTurnEncoderCore(const std::string& device,
                       int baud,
                       char parity,
                       int data_bit,
                       int stop_bit,
                       int slave);
  MultiTurnEncoderCore(const std::string& ip, int port, int slave);
  ~MultiTurnEncoderCore();

  bool connect();
  void run();
  void stop();
  bool isConnected() const;
  bool isRunning() const;
  LatestData getLatest() const;

 private:
  std::unique_ptr<MultiTurnEncoderRTU> encoder_;
  mutable std::mutex mutex_;
  bool running_;
  Transport transport_;
};

}  // namespace multi_turn_encoder
