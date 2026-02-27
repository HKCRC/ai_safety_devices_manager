#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace io_relay {

class IoRelayCore {
 public:
  IoRelayCore();
  IoRelayCore(const std::string& module_ip, uint16_t module_port, uint8_t module_slave_id);
  ~IoRelayCore();

  void controlRelay(int relay_num, const std::string& status);
  void readRelayStatus(int relay_num);  // relay_num <= 0 means read all

 private:
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
  bool parseRelayNum(int relay_num, uint16_t* coil_addr) const;

  const std::string module_ip_;
  const uint16_t module_port_;
  const uint8_t module_slave_id_;
  uint16_t transaction_id_;
  int socket_fd_;
  std::mutex socket_mutex_;
};

}  // namespace io_relay
