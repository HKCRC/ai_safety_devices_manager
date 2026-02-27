#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

namespace ai_safety_controller {
namespace common {

// Serialize requests targeting the same gateway endpoint.
class GatewaySerialGuard {
 public:
  GatewaySerialGuard(const std::string& endpoint_key, std::uint32_t min_gap_ms = 120)
      : endpoint_key_(endpoint_key),
        min_gap_(std::chrono::milliseconds(min_gap_ms)),
        lock_(globalMutex()) {
    auto& last = lastSendByEndpoint();
    const auto it = last.find(endpoint_key_);
    if (it != last.end()) {
      const auto due = it->second + min_gap_;
      const auto now = std::chrono::steady_clock::now();
      if (due > now) std::this_thread::sleep_for(due - now);
    }
  }

  ~GatewaySerialGuard() {
    lastSendByEndpoint()[endpoint_key_] = std::chrono::steady_clock::now();
  }

  GatewaySerialGuard(const GatewaySerialGuard&) = delete;
  GatewaySerialGuard& operator=(const GatewaySerialGuard&) = delete;

 private:
  static std::mutex& globalMutex() {
    static std::mutex mtx;
    return mtx;
  }

  static std::unordered_map<std::string, std::chrono::steady_clock::time_point>& lastSendByEndpoint() {
    static std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_by_endpoint;
    return last_by_endpoint;
  }

  std::string endpoint_key_;
  std::chrono::steady_clock::duration min_gap_;
  std::unique_lock<std::mutex> lock_;
};

}  // namespace common
}  // namespace ai_safety_controller
