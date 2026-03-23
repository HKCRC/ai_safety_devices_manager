#pragma once

#include <cstdint>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

namespace io_relay {

class IoRelayCore {
 public:
  struct RetryPolicy {
    int max_retries = 2;
    int base_backoff_ms = 100;
    int max_backoff_ms = 500;
    int jitter_ms = 50;
    bool log_enabled = true;
  };

  IoRelayCore();
  IoRelayCore(const std::string& module_ip,
              uint16_t module_port,
              uint8_t module_slave_id);
  IoRelayCore(const std::string& module_ip,
              uint16_t module_port,
              uint8_t module_slave_id,
              const RetryPolicy& retry_policy);
  ~IoRelayCore();

  bool controlRelay(int relay_num, const std::string& status);
  bool readRelayStatus(int relay_num);  // relay_num <= 0 means read all
  bool getRelayState(int relay_num, bool* on);

 private:
  void waitForStartupStableWindow();
  std::vector<uint8_t> createModbusPacket(uint8_t function_code,
                                          uint16_t address,
                                          uint16_t value,
                                          uint16_t quantity,
                                          uint8_t unit_id,
                                          bool* ok);
  bool sendModbusPacket(const std::vector<uint8_t>& packet,
                        std::vector<uint8_t>* response,
                        const std::string& context,
                        double timeout_sec = 5.0);
  bool ensureConnectionLocked(double timeout_sec);
  void disconnectLocked();
  bool sendAndReceiveLocked(const std::vector<uint8_t>& packet,
                            std::vector<uint8_t>* response,
                            const std::string& context);
  bool parseReadCoilsResponse(const std::vector<uint8_t>& response,
                              int expected_count,
                              std::vector<bool>* states);
  bool readRelayStates(int relay_num, std::vector<bool>* states);
  bool readSingleRelayState(int relay_num, bool* on);
  bool parseRelayNum(int relay_num, uint16_t* coil_addr) const;

  const std::string module_ip_;
  const uint16_t module_port_;
  const uint8_t module_slave_id_;
  uint16_t transaction_id_;
  int socket_fd_;
  RetryPolicy retry_policy_;
  std::mutex socket_mutex_;
  std::chrono::steady_clock::time_point startup_stable_after_;
};

}  // namespace io_relay
