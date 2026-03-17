#include "multi_turn_encoder/multi_turn_encoder_core.hpp"

#include <utility>

namespace multi_turn_encoder {

MultiTurnEncoderCore::MultiTurnEncoderCore(const std::string& device,
                                           int baud,
                                           char parity,
                                           int data_bit,
                                           int stop_bit,
                                           int slave)
    : encoder_(new MultiTurnEncoderRTU(device.c_str(), baud, parity, data_bit, stop_bit, slave)),
      running_(false),
      linear_enable_(false),
      linear_k_(1.0),
      linear_b_(0.0),
      transport_(Transport::RTU) {}

MultiTurnEncoderCore::MultiTurnEncoderCore(const std::string& ip, int port, int slave)
    : encoder_(new MultiTurnEncoderRTU(ip.c_str(), port, slave)),
      running_(false),
      linear_enable_(false),
      linear_k_(1.0),
      linear_b_(0.0),
      transport_(Transport::TCP) {}

MultiTurnEncoderCore::~MultiTurnEncoderCore() {
  stop();
  if (encoder_) {
    encoder_->disconnect();
  }
}

bool MultiTurnEncoderCore::connect() {
  if (!encoder_) return false;
  return encoder_->connect();
}

void MultiTurnEncoderCore::run() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!encoder_ || running_) return;
  encoder_->run();
  running_ = true;
}

void MultiTurnEncoderCore::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!encoder_ || !running_) return;
  encoder_->stop();
  running_ = false;
}

bool MultiTurnEncoderCore::isConnected() const {
  if (!encoder_) return false;
  return encoder_->isConnected();
}

bool MultiTurnEncoderCore::isRunning() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

void MultiTurnEncoderCore::setLinearTransform(bool enable, double k, double b) {
  std::lock_guard<std::mutex> lock(mutex_);
  linear_enable_ = enable;
  linear_k_ = k;
  linear_b_ = b;
}

MultiTurnEncoderCore::LatestData MultiTurnEncoderCore::getLatest() const {
  LatestData out{};
  out.valid = false;
  out.connected = isConnected();
  out.running = isRunning();

  if (!encoder_) return out;

  const StampedDouble raw = encoder_->getData();
  const MultiTurnEncoderRTU::StampedEncoderData enc = encoder_->getEncoderData();
  out.timestamp = raw.timestamp;
  out.duration = raw.time_variance;
  out.turns_raw = raw.value;
  out.turns_filtered = enc.value;
  bool linear_enable = false;
  double linear_k = 1.0;
  double linear_b = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    linear_enable = linear_enable_;
    linear_k = linear_k_;
    linear_b = linear_b_;
  }
  out.turns_calibrated = linear_enable ? (linear_k * out.turns_raw + linear_b) : out.turns_raw;
  out.velocity = enc.velocity;

  // A sample is considered valid after timestamp is populated by read loop.
  out.valid = (raw.timestamp > 0.0);
  return out;
}

}  // namespace multi_turn_encoder
