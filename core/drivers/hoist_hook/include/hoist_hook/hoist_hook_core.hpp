#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace hoist_hook {

class HoistHookCore {
 public:
  HoistHookCore();
  HoistHookCore(const std::string& module_ip,
                uint16_t module_port,
                uint8_t hook_slave_id,
                uint8_t power_slave_id);
  ~HoistHookCore();

  void printRegisterGroups() const;
  void queryHookInfo(const std::string& info_type);
  void controlSpeaker(const std::string& mode);
  void controlWarningLight(const std::string& status);
  void genericRead(uint16_t address, uint16_t quantity, int function_code);
  void genericWrite(uint16_t address, uint16_t value, int function_code);

  static bool parseNumber(const std::string& text, int* out);
  static bool parseFunctionCode(const std::string& text,
                                const std::vector<int>& allowed,
                                int* out);

 private:
  struct RegisterGroup {
    uint16_t start;
    uint16_t end;
    std::string rw;
    std::string desc;
  };

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
  bool sendRead(uint8_t function_code,
                uint16_t address,
                uint16_t quantity,
                uint8_t unit_id,
                std::vector<uint8_t>* response,
                double timeout_sec = 5.0);
  bool parseRegisterResponse(const std::vector<uint8_t>& response,
                             uint8_t function_code,
                             uint16_t quantity,
                             std::vector<uint16_t>* values) const;
  bool confirmRiskyWrite(uint16_t addr) const;
  std::string describeRegister(uint16_t addr) const;

  void querySpeakerStatus();
  void queryLightStatus();
  void queryRfidInfo();
  void queryPowerInfo();
  void queryGpsInfo();

  const std::string module_ip_;
  const uint16_t module_port_;
  uint8_t hook_slave_id_;
  uint8_t power_slave_id_;
  uint16_t transaction_id_;
  int socket_fd_;
  std::mutex socket_mutex_;
  std::vector<RegisterGroup> register_groups_;
};

}  // namespace hoist_hook
