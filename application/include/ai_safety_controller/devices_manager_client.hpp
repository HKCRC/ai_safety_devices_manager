#pragma once

#include "ai_safety_common/shared_memory_types.hpp"
#include "ai_safety_controller/common/status.hpp"

#include <atomic>
#include <boost/signals2.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace ai_safety_controller {

class Interface;

/**
 * 门面类：供主工程（如 main.cpp）统一调用设备管理能力。
 * 内部持有 Interface，对外暴露 loadConfig / init / start / stop / getDeviceStatus，
 * 以及 SignalSendDeviceStatus（状态变化时尽快推送，并保留定时保活推送，与 SignalSendAlarm 用法一致）。
 * 以及 SignalSendCraneState（对外 CraneState 信号接口，供主工程按需 connect）。
 * 用法示例（主工程侧）：
 *   DevicesManagerClient client;
 *   client.SignalSendDeviceStatus.connect([](ai_safety_common::DeviceStatus d) { ... });
 *   if (!client.loadConfig(config_path).ok) { ... }
 *   if (!client.init().ok) { ... }
 *   client.start();  // 内部状态变化时尽快推送 DeviceStatus，并保留周期性推送
 */
class DevicesManagerClient {
 public:
  DevicesManagerClient();
  ~DevicesManagerClient();

  DevicesManagerClient(const DevicesManagerClient&) = delete;
  DevicesManagerClient& operator=(const DevicesManagerClient&) = delete;

  /** 加载配置文件（JSON），同 Interface::loadConfig */
  Status loadConfig(const std::string& path);
  /** 初始化设备与驱动，同 Interface::init */
  Status init();
  /** 启动轮询等，同 Interface::start；同时启动通知线程，在状态变化时尽快触发 SignalSendDeviceStatus */
  Status start();
  /** 停止轮询与定时推送，同 Interface::stop */
  Status stop();

  /**
   * 获取当前聚合的设备状态（小车/吊钩/太阳能等）。
   * 主工程可将此返回值直接作为 AISampler::SignalTowerInfo 的返回值。
   */
  ai_safety_common::DeviceStatus getDeviceStatus() const;

  /** 获取当前吊钩/小车距离状态。 */
  ai_safety_common::CraneState getCraneState() const;
  /** 获取各单点激光最近一次有效原始值（单位 mm，key 为实例 id）。 */
  std::unordered_map<std::string, std::uint16_t> getLatestLidarRawMm() const;

  /**
   * 获取当前报警状态（四个 bool 字段）。
   * 字段定义见 ai_safety_common::AlertMessage。
   */
  boost::signals2::signal<void(ai_safety_common::AlertMessage&)> SignalGetAlertMessage;

  /**
   * 获取电源控制指令（Battery Button）：
   * 0 = None, 1 = PowerOn, 2 = PowerOff。
   */
  boost::signals2::signal<void(std::uint8_t&)> SignalGetBatteryButtonSignals;

  /**
   * 状态推送信号：内部在 DeviceStatus 变化时尽快触发，同时保留周期性推送，主工程 connect 接收。
   * 用法与 AISampler::SignalSendAlarm 一致；回调可能在内部线程执行，若需更新 Qt UI 请投递到主线程。
   */
  boost::signals2::signal<void(ai_safety_common::DeviceStatus)> SignalSendDeviceStatus;

  /**
   * 对外 CraneState 信号接口（signal-slot 风格）。
   * 当前仅暴露接口，内部如何填充 CraneState 后续再接入。
   */
  boost::signals2::signal<void(const ai_safety_common::CraneState&)> SignalSendCraneState;

  /** 是否已成功 init（且未析构） */
  bool isInitialized() const;
  /** 是否已 start（且未 stop） */
  bool isStarted() const;

 private:
  enum class SpeakerMode {
    Off,       // 两路都关
    M7,
    M3,
    Off7MOnly, // 只关 7m，不写 3m 寄存器
    Off3MOnly, // 只关 3m，不写 7m 寄存器
  };
  enum class BothSpeakerStage {
    Playing3M,
    GapAfter3M,
    Playing7M,
    GapAfter7M,
  };
  using PowerCommand = ai_safety_common::JoystickControlData::PowerCommand;

  void notifyThreadFunc();
  void applySpeakerControlByAlert(const ai_safety_common::AlertMessage& alert);
  bool applySpeakerMode(SpeakerMode mode, bool quiet = false);
  void applyBatteryButtonControl(std::uint8_t raw_cmd, bool force_send = false);
  void restoreBatteryButtonPowerStateFromRelays(bool log_output = true);
  static const char* toSpeakerCtlArg(SpeakerMode mode);

  std::unique_ptr<Interface> impl_;
  bool initialized_ = false;
  bool started_ = false;
  std::optional<SpeakerMode> applied_speaker_mode_;
  bool both_round_robin_active_ = false;
  BothSpeakerStage both_stage_ = BothSpeakerStage::Playing3M;
  std::chrono::steady_clock::time_point both_stage_deadline_{};
  std::chrono::milliseconds both_play_window_{5000};
  std::chrono::milliseconds both_switch_gap_{200};
  std::chrono::steady_clock::time_point last_push_ts_{};
  std::chrono::steady_clock::time_point next_relay_state_sync_ts_{};
  std::vector<int> battery_button_relay_channels_{};
  ai_safety_common::DeviceStatus last_sent_device_status_{};
  bool has_last_sent_device_status_ = false;
  ai_safety_common::CraneState last_sent_crane_state_{};
  bool has_last_sent_crane_state_ = false;
  std::optional<PowerCommand> last_battery_button_cmd_;
  std::optional<PowerCommand> last_received_battery_button_cmd_;
  std::chrono::milliseconds status_push_keepalive_interval_{1000};
  std::chrono::milliseconds relay_state_sync_interval_{3000};
  std::atomic<bool> notify_stop_{false};
  std::thread notify_thread_;
};

}  // namespace ai_safety_controller
