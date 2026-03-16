#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace battery {

class BatteryCore {
 public:
  struct RetryPolicy {
    int max_retries = 2;
    int base_backoff_ms = 100;
    int max_backoff_ms = 500;
    int jitter_ms = 50;
    bool log_enabled = true;
  };

  struct Summary {
    bool ok = false;
    float soc_percent = 0.0f;
    float voltage_v = 0.0f;
    float current_a = 0.0f;
    std::uint32_t remaining_discharge_min = 0;
    std::uint32_t remaining_charge_min = 0;
    bool has_charge_mos = false;
    std::uint16_t charge_mos = 0;
  };

  BatteryCore();
  BatteryCore(const std::string& module_ip,
              uint16_t module_port,
              uint8_t module_slave_id,
              uint8_t battery_slave_id);
  BatteryCore(const std::string& module_ip,
              uint16_t module_port,
              uint8_t module_slave_id,
              uint8_t battery_slave_id,
              const RetryPolicy& retry_policy);
  ~BatteryCore();

  void printRegisterGroups() const;
  void queryBatteryInfo(const std::string& info_type);
  void scanBatterySlaveIds(int start_id, int end_id);
  void setBatteryAddr(int new_addr);
  void genericRead(uint16_t address, uint16_t quantity, int function_code);
  void genericWrite(uint16_t address, uint16_t value, int function_code);

  static bool parseNumber(const std::string& text, int* out);
  static bool parseFunctionCode(const std::string& text,
                                const std::vector<int>& allowed,
                                int* out);

  bool isOnline(double timeout_sec = 1.0);
  bool readSummary(Summary* out, double timeout_sec = 5.0);
  void setChargeTimeDebugEnabled(bool enabled);

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
  bool sendBatteryRead(uint8_t function_code,
                       uint16_t address,
                       uint16_t quantity,
                       uint8_t unit_id,
                       std::vector<uint8_t>* response,
                       double timeout_sec = 5.0);
  bool parseRegisterResponse(const std::vector<uint8_t>& response,
                             uint8_t function_code,
                             uint16_t quantity,
                             std::vector<uint16_t>* values) const;
  bool ensureConnectionLocked(double timeout_sec);
  void disconnectLocked();
  bool sendAndReceiveLocked(const std::vector<uint8_t>& packet,
                            std::vector<uint8_t>* response,
                            const std::string& context);
  bool confirmRiskyWrite(uint16_t addr) const;
  std::string describeBatteryRegister(uint16_t addr) const;
  int16_t toSigned16(uint16_t value) const;

  const std::string module_ip_;
  const uint16_t module_port_;
  const uint8_t module_slave_id_;
  uint8_t battery_slave_id_;
  uint16_t transaction_id_;
  int socket_fd_;
  RetryPolicy retry_policy_;
  bool charge_time_debug_enabled_ = false;
  std::mutex socket_mutex_;
  std::vector<RegisterGroup> register_groups_;
};

}  // namespace battery
