#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace hoist_hook {

class HoistHookCore {
 public:
  enum class Transport { TCP, RTU };
  struct RetryPolicy {
    int max_retries = 2;
    int base_backoff_ms = 100;
    int max_backoff_ms = 500;
    int jitter_ms = 50;
  };

  HoistHookCore();
  /** Modbus TCP: connect to module_ip:module_port, unit_id = hook_slave_id / power_slave_id */
  HoistHookCore(const std::string& module_ip,
                uint16_t module_port,
                uint8_t hook_slave_id,
                uint8_t power_slave_id);
  HoistHookCore(const std::string& module_ip,
                uint16_t module_port,
                uint8_t hook_slave_id,
                uint8_t power_slave_id,
                const RetryPolicy& retry_policy);
  /** Modbus RTU over serial: device (e.g. /dev/ttyUSB0), 9600 8N1, hook/power slave ids */
  HoistHookCore(const std::string& device,
                int baud,
                char parity,
                int data_bit,
                int stop_bit,
                uint8_t hook_slave_id,
                uint8_t power_slave_id);
  HoistHookCore(const std::string& device,
                int baud,
                char parity,
                int data_bit,
                int stop_bit,
                uint8_t hook_slave_id,
                uint8_t power_slave_id,
                const RetryPolicy& retry_policy);
  ~HoistHookCore();

  void printRegisterGroups() const;
  void queryHookInfo(const std::string& info_type);
  void controlSpeaker(const std::string& mode, bool quiet = false);
  void controlWarningLight(const std::string& status);
  void genericRead(uint16_t address, uint16_t quantity, int function_code);
  /** skip_confirm=true 用于喇叭/灯/音量等交互控制；quiet=true 不打印写入成功，用于轮播时避免刷屏 */
  void genericWrite(uint16_t address, uint16_t value, int function_code, bool skip_confirm = false, bool quiet = false);

  static bool parseNumber(const std::string& text, int* out);
  static bool parseFunctionCode(const std::string& text,
                                const std::vector<int>& allowed,
                                int* out);

  struct PowerSummary {
    bool ok = false;
    float battery_percent = 0.0f;
    std::uint32_t remaining_discharge_min = 0;
    bool is_charging = false;
    std::uint32_t remaining_charge_min = 0;
    float voltage_v = 0.0f;
    float current_a = 0.0f;
  };

  bool readPowerSummary(PowerSummary* out, double timeout_sec = 2.0);

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
  static uint16_t crc16Modbus(const uint8_t* data, size_t len);

  void querySpeakerStatus();
  void queryLightStatus();
  void queryRfidInfo();
  void queryPowerInfo();
  void queryGpsInfo();
  void queryHeartbeat();
  void queryWorkMode();

  const Transport transport_;
  const std::string module_ip_;
  const uint16_t module_port_;
  const std::string device_;
  const int baud_;
  const char parity_;
  const int data_bit_;
  const int stop_bit_;
  uint8_t hook_slave_id_;
  uint8_t power_slave_id_;
  uint16_t transaction_id_;
  int socket_fd_;
  int serial_fd_;
  RetryPolicy retry_policy_;
  bool print_enabled_;
  std::mutex socket_mutex_;
  std::vector<RegisterGroup> register_groups_;
};

}  // namespace hoist_hook
