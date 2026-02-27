#pragma once

#include "ai_safety_controller/common/status.hpp"
#include "ai_safety_controller/sensor_factory/sensor_factory.hpp"

#include <memory>
#include <string>
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#ifdef ASC_ENABLE_BATTERY
#include "battery/battery_core.hpp"
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
#include "hoist_hook/hoist_hook_core.hpp"
#endif
#ifdef ASC_ENABLE_IO_RELAY
#include "io_relay/io_relay_core.hpp"
#endif
#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
#include "multi_turn_encoder/multi_turn_encoder_core.hpp"
#endif
#ifdef ASC_ENABLE_SOLAR
#include "solar/solar_core.hpp"
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
#include "spd_lidar/spd_lidar_core.hpp"
#endif

namespace ai_safety_controller {

class DriverAdapter {
 public:
  virtual ~DriverAdapter() = default;
  virtual const std::string& name() const = 0;
  virtual Status init() = 0;
  virtual Status start() = 0;
  virtual Status stop() = 0;
  virtual Status query(const std::vector<std::string>& args) = 0;
  virtual std::vector<std::string> availableCommands() const = 0;
};

class Interface {
 public:
  struct BatteryDefaults {
    bool enable = true;
    std::string module_ip = "192.168.1.12";
    int module_port = 502;
    int module_slave_id = 3;
    int battery_slave_id = 2;
    double query_hz = 0.0;
  };

  struct SolarDefaults {
    bool enable = true;
    std::string module_ip = "192.168.1.12";
    int module_port = 502;
    int module_slave_id = 3;
    int solar_slave_id = 4;
    double query_hz = 0.0;
  };

  struct IoRelayDefaults {
    bool enable = true;
    std::string module_ip = "192.168.1.12";
    int module_port = 502;
    int module_slave_id = 3;
    double query_hz = 0.0;
  };

  struct HoistHookDefaults {
    bool enable = true;
    std::string module_ip = "192.168.1.12";
    int module_port = 502;
    int hook_slave_id = 3;
    int power_slave_id = 4;
    double query_hz = 0.0;
  };

  struct EncoderDefaults {
    bool enable = true;
    std::string transport = "rtu";
    std::string device = "/dev/ttyUSB0";
    int baud = 9600;
    char parity = 'N';
    int data_bit = 8;
    int stop_bit = 1;
    int slave = 1;
    std::string ip = "192.168.1.100";
    int port = 502;
    double query_hz = 0.0;
  };

  struct SpdLidarInstanceDefaults {
    std::string id = "default";
    bool enable = true;
    std::string mode = "server";
    std::string local_ip = "192.168.0.201";
    int local_port = 8234;
    std::string device_ip = "192.168.0.7";
    int device_port = 8234;
    std::string role;
    int priority = 0;
  };

  Interface();
  ~Interface();

  Status loadConfig(const std::string& path);
  const std::string& loadedConfigPath() const;
  const BatteryDefaults& batteryDefaults() const;
  const SolarDefaults& solarDefaults() const;
  const IoRelayDefaults& ioRelayDefaults() const;
  const HoistHookDefaults& hoistHookDefaults() const;
  const EncoderDefaults& encoderDefaults() const;
  const std::vector<SpdLidarInstanceDefaults>& spdLidarInstances() const;
  double spdLidarQueryHz() const;

  Status init();
  Status start();
  Status stop();
  Status query(const std::string& sensor, const std::vector<std::string>& args);
  std::vector<std::string> enabledSensors() const;
  Status dispatchCommand(const std::string& sensor, const std::vector<std::string>& args);
  std::vector<std::string> availableCommands(const std::string& sensor) const;

#ifdef ASC_ENABLE_BATTERY
  battery::BatteryCore* battery();
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
  hoist_hook::HoistHookCore* hoistHook();
#endif
#ifdef ASC_ENABLE_IO_RELAY
  io_relay::IoRelayCore* ioRelay();
#endif
#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
  multi_turn_encoder::MultiTurnEncoderCore* multiTurnEncoder();
#endif
#ifdef ASC_ENABLE_SOLAR
  solar::SolarCore* solar();
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
  spd_lidar::SpdLidarCore* spdLidar();
#endif

 private:
  static bool parseInt(const std::string& text, int* out);
  static bool parseBool(const std::string& text, bool* out);
  static bool parseDouble(const std::string& text, double* out);

  Status loadDefaultConfigIfPresent();
  static std::string extractObjectBody(const std::string& json_text, const std::string& key);
  static std::string extractArrayBody(const std::string& json_text, const std::string& key);
  static std::vector<std::string> splitTopLevelObjects(const std::string& array_body);
  static bool extractStringValue(const std::string& object_body, const std::string& key, std::string* out);
  static bool extractIntValue(const std::string& object_body, const std::string& key, int* out);
  static bool extractBoolValue(const std::string& object_body, const std::string& key, bool* out);
  static bool extractDoubleValue(const std::string& object_body, const std::string& key, double* out);
  void applyBatteryDefaultsFromJson(const std::string& json_text);
  void applySolarDefaultsFromJson(const std::string& json_text);
  void applyIoRelayDefaultsFromJson(const std::string& json_text);
  void applyHoistHookDefaultsFromJson(const std::string& json_text);
  void applyEncoderDefaultsFromJson(const std::string& json_text);
  void applySpdLidarDefaultsFromJson(const std::string& json_text);
  void buildDriverAdapters();
  void startAutoQueryPolling();
  void stopAutoQueryPolling();
  Status queryWithCapturedOutput(const std::string& sensor,
                                 const std::vector<std::string>& args,
                                 std::string* captured_output);
  void startSnapshotPrinter();
  void stopSnapshotPrinter();
  void printSnapshotTick();

#ifdef ASC_ENABLE_BATTERY
  Status queryBattery(const std::vector<std::string>& args);
#endif
#ifdef ASC_ENABLE_SOLAR
  Status querySolar(const std::vector<std::string>& args);
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
  Status queryHoistHook(const std::vector<std::string>& args);
#endif
#ifdef ASC_ENABLE_IO_RELAY
  Status queryIoRelay(const std::vector<std::string>& args);
#endif
#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
  Status queryMultiTurnEncoder(const std::vector<std::string>& args);
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
  Status querySpdLidar(const std::vector<std::string>& args);
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
  spd_lidar::SpdLidarCore* findSpdLidarById(const std::string& id);
  const spd_lidar::SpdLidarCore* findSpdLidarById(const std::string& id) const;
#endif

  bool initialized_;
  bool started_;
  bool config_loaded_;
  double spd_lidar_query_hz_;
  std::string loaded_config_path_;
  BatteryDefaults battery_defaults_;
  SolarDefaults solar_defaults_;
  IoRelayDefaults io_relay_defaults_;
  HoistHookDefaults hoist_hook_defaults_;
  EncoderDefaults encoder_defaults_;
  std::vector<SpdLidarInstanceDefaults> spd_lidar_instances_;
  SensorFactory factory_;
  std::unordered_map<std::string, std::unique_ptr<DriverAdapter>> drivers_;
  std::atomic<bool> auto_query_running_;
  std::atomic<bool> snapshot_printer_running_;
  std::vector<std::thread> auto_query_threads_;
  std::thread snapshot_printer_thread_;
  std::unordered_map<std::string, std::string> latest_query_output_;
  std::unordered_map<std::string, Status> latest_query_status_;
  std::unordered_map<std::string, std::chrono::system_clock::time_point> latest_query_time_;
  mutable std::mutex snapshot_mutex_;
  mutable std::mutex output_mutex_;

#ifdef ASC_ENABLE_BATTERY
  std::unique_ptr<battery::BatteryCore> battery_;
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
  std::unique_ptr<hoist_hook::HoistHookCore> hoist_hook_;
#endif
#ifdef ASC_ENABLE_IO_RELAY
  std::unique_ptr<io_relay::IoRelayCore> io_relay_;
#endif
#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
  std::unique_ptr<multi_turn_encoder::MultiTurnEncoderCore> multi_turn_encoder_;
#endif
#ifdef ASC_ENABLE_SOLAR
  std::unique_ptr<solar::SolarCore> solar_;
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
  std::unordered_map<std::string, std::unique_ptr<spd_lidar::SpdLidarCore>> spd_lidar_instances_core_;
#endif
};

}  // namespace ai_safety_controller
