#include "ai_safety_controller/devices_manager_client.hpp"
#include "ai_safety_controller/interface.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

namespace ai_safety_controller {

namespace {

bool equalsBatteryInfo(const ai_safety_common::DeviceStatus::BatteryInfo& lhs,
                       const ai_safety_common::DeviceStatus::BatteryInfo& rhs) {
  return lhs.percent == rhs.percent &&
         lhs.remainingMin == rhs.remainingMin &&
         lhs.isCharging == rhs.isCharging &&
         lhs.chargingTimeMin == rhs.chargingTimeMin &&
         lhs.voltageV == rhs.voltageV &&
         lhs.currentA == rhs.currentA;
}

bool equalsDeviceStatus(const ai_safety_common::DeviceStatus& lhs,
                        const ai_safety_common::DeviceStatus& rhs) {
  return lhs.solarCharge == rhs.solarCharge &&
         lhs.trolleyState == rhs.trolleyState &&
         equalsBatteryInfo(lhs.trolleyBattery, rhs.trolleyBattery) &&
         lhs.hookState == rhs.hookState &&
         equalsBatteryInfo(lhs.hookBattery, rhs.hookBattery);
}

bool equalsCraneState(const ai_safety_common::CraneState& lhs,
                      const ai_safety_common::CraneState& rhs) {
  return lhs.hookToTrolleyDistanceM == rhs.hookToTrolleyDistanceM &&
         lhs.groundToTrolleyDistanceM == rhs.groundToTrolleyDistanceM;
}

}  // namespace

DevicesManagerClient::DevicesManagerClient() : impl_(std::make_unique<Interface>()) {}

DevicesManagerClient::~DevicesManagerClient() {
  stop();
}

const char* DevicesManagerClient::toSpeakerCtlArg(SpeakerMode mode) {
  switch (mode) {
    case SpeakerMode::M7:
      return "7m";
    case SpeakerMode::M3:
      return "3m";
    case SpeakerMode::Off7MOnly:
      return "7m_off";
    case SpeakerMode::Off3MOnly:
      return "3m_off";
    case SpeakerMode::Off:
    default:
      return "off";
  }
}

bool DevicesManagerClient::applySpeakerMode(SpeakerMode mode, bool quiet) {
  if (!impl_) return false;
  if (applied_speaker_mode_.has_value() && applied_speaker_mode_.value() == mode) return true;
  std::vector<std::string> args{"speaker_ctl", toSpeakerCtlArg(mode)};
  if (quiet) args.push_back("quiet");
  const Status ctl = impl_->dispatchCommand("hoist_hook", args);
  if (ctl.ok) {
    applied_speaker_mode_ = (mode == SpeakerMode::Off7MOnly || mode == SpeakerMode::Off3MOnly)
                                ? SpeakerMode::Off
                                : mode;
    return true;
  }
  return false;
}

void DevicesManagerClient::applySpeakerControlByAlert(const ai_safety_common::AlertMessage& alert) {
  if (!impl_) return;

  const bool trig3 = alert.Enable3Alert && alert.Alert3M;
  const bool trig7 = alert.Enable7Alert && alert.Alert7M;
  const auto now = std::chrono::steady_clock::now();

  if (trig3 && trig7) {
    // 双路轮播：quiet=true 避免刷屏干扰命令行输入
    if (!both_round_robin_active_) {
      both_round_robin_active_ = true;
      both_stage_ = BothSpeakerStage::Playing3M;
      if (applySpeakerMode(SpeakerMode::M3, true)) {
        both_stage_deadline_ = now + both_play_window_;
      } else {
        both_stage_deadline_ = now + std::chrono::milliseconds(200);
      }
      return;
    }
    if (now < both_stage_deadline_) {
      return;
    }

    switch (both_stage_) {
      case BothSpeakerStage::Playing3M:
        if (applySpeakerMode(SpeakerMode::Off, true)) {
          both_stage_ = BothSpeakerStage::GapAfter3M;
          both_stage_deadline_ = now + both_switch_gap_;
        } else {
          both_stage_deadline_ = now + std::chrono::milliseconds(200);
        }
        break;
      case BothSpeakerStage::GapAfter3M:
        if (applySpeakerMode(SpeakerMode::M7, true)) {
          both_stage_ = BothSpeakerStage::Playing7M;
          both_stage_deadline_ = now + both_play_window_;
        } else {
          both_stage_deadline_ = now + std::chrono::milliseconds(200);
        }
        break;
      case BothSpeakerStage::Playing7M:
        if (applySpeakerMode(SpeakerMode::Off, true)) {
          both_stage_ = BothSpeakerStage::GapAfter7M;
          both_stage_deadline_ = now + both_switch_gap_;
        } else {
          both_stage_deadline_ = now + std::chrono::milliseconds(200);
        }
        break;
      case BothSpeakerStage::GapAfter7M:
        if (applySpeakerMode(SpeakerMode::M3, true)) {
          both_stage_ = BothSpeakerStage::Playing3M;
          both_stage_deadline_ = now + both_play_window_;
        } else {
          both_stage_deadline_ = now + std::chrono::milliseconds(200);
        }
        break;
    }
    return;
  }

  const bool was_both_round_robin = both_round_robin_active_;
  both_round_robin_active_ = false;
  if (trig3) {
    (void)applySpeakerMode(SpeakerMode::M3);
  } else if (trig7) {
    (void)applySpeakerMode(SpeakerMode::M7);
  } else {
    // 关喇叭：双路轮播刚结束则两路都关；否则只关当前播的那一路
    if (was_both_round_robin) {
      (void)applySpeakerMode(SpeakerMode::Off);
    } else if (applied_speaker_mode_.has_value()) {
      if (applied_speaker_mode_.value() == SpeakerMode::M3) {
        (void)applySpeakerMode(SpeakerMode::Off3MOnly);
      } else if (applied_speaker_mode_.value() == SpeakerMode::M7) {
        (void)applySpeakerMode(SpeakerMode::Off7MOnly);
      } else {
        (void)applySpeakerMode(SpeakerMode::Off);
      }
    } else {
      (void)applySpeakerMode(SpeakerMode::Off);
    }
  }
}

void DevicesManagerClient::applyBatteryButtonControl(std::uint8_t raw_cmd, bool force_send) {
  if (!impl_) return;
  if (raw_cmd > static_cast<std::uint8_t>(PowerCommand::PowerOff)) return;

  const PowerCommand cmd = static_cast<PowerCommand>(raw_cmd);
  if (cmd == PowerCommand::None) return;

  if (battery_button_relay_channels_.empty()) return;

  bool all_ok = true;
  for (size_t i = 0; i < battery_button_relay_channels_.size(); ++i) {
    const std::vector<std::string> args{
        (cmd == PowerCommand::PowerOn) ? "on" : "off",
        std::to_string(battery_button_relay_channels_[i])};
    const Status ctl = impl_->dispatchCommand("io_relay", args);
    if (!ctl.ok) all_ok = false;
  }
  if (all_ok) {
    impl_->setPowerCommand(cmd);
    last_battery_button_cmd_ = cmd;
  }
}

void DevicesManagerClient::restoreBatteryButtonPowerStateFromRelays(bool log_output) {
  if (!impl_ || battery_button_relay_channels_.empty()) return;
#ifdef ASC_ENABLE_IO_RELAY
  io_relay::IoRelayCore* relay = impl_->ioRelay();
  if (!relay) return;

  bool any_on = false;
  bool any_off = false;
  for (size_t i = 0; i < battery_button_relay_channels_.size(); ++i) {
    bool on = false;
    const int ch = battery_button_relay_channels_[i];
    if (!relay->getRelayState(ch, &on)) {
      if (log_output) {
        std::cout << "[startup] restore power state skipped: failed to read io_relay channel " << ch << "\n";
      }
      return;
    }
    any_on = any_on || on;
    any_off = any_off || !on;
  }

  PowerCommand restored_cmd = PowerCommand::None;
  if (any_on) {
    restored_cmd = PowerCommand::PowerOn;
    if (log_output) {
      if (any_off) {
        std::cout << "[startup] restore power state: at least one relay channel is on\n";
      } else {
        std::cout << "[startup] restore power state: relays are on\n";
      }
    }
  } else {
    restored_cmd = PowerCommand::PowerOff;
    if (log_output) {
      std::cout << "[startup] restore power state: relays are off\n";
    }
  }

  const PowerCommand previous_cmd = impl_->getPowerCommand();
  impl_->setPowerCommand(restored_cmd);
  if (restored_cmd == PowerCommand::PowerOn || restored_cmd == PowerCommand::PowerOff) {
    last_battery_button_cmd_ = restored_cmd;
  } else {
    last_battery_button_cmd_.reset();
  }
  if (!log_output && previous_cmd != restored_cmd) {
    std::cout << "[runtime] sync power state from relays: "
              << (restored_cmd == PowerCommand::PowerOn ? "on" : "off") << "\n";
  }
#else
  impl_->setPowerCommand(PowerCommand::None);
#endif
}

void DevicesManagerClient::notifyThreadFunc() {
  while (!notify_stop_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (notify_stop_) break;
    if (impl_) {
      if (!SignalGetAlertMessage.empty()) {
        ai_safety_common::AlertMessage alert{};
        SignalGetAlertMessage(alert);
        applySpeakerControlByAlert(alert);
      }
      if (!SignalGetBatteryButtonSignals.empty()) {
        std::uint8_t raw_cmd = 0;
        SignalGetBatteryButtonSignals(raw_cmd);
        if (raw_cmd <= static_cast<std::uint8_t>(PowerCommand::PowerOff)) {
          const PowerCommand cmd = static_cast<PowerCommand>(raw_cmd);
          if (cmd == PowerCommand::None) {
            last_received_battery_button_cmd_.reset();
          } else if (!last_received_battery_button_cmd_.has_value() ||
                     last_received_battery_button_cmd_.value() != cmd) {
            last_received_battery_button_cmd_ = cmd;
            applyBatteryButtonControl(raw_cmd);
          }
        }
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_relay_state_sync_ts_) {
        restoreBatteryButtonPowerStateFromRelays(false);
        next_relay_state_sync_ts_ = now + relay_state_sync_interval_;
      }
      const ai_safety_common::DeviceStatus device_status = getDeviceStatus();
      const ai_safety_common::CraneState crane_state = getCraneState();
      const bool device_status_changed =
          !has_last_sent_device_status_ ||
          !equalsDeviceStatus(last_sent_device_status_, device_status);
      const bool crane_state_changed =
          !has_last_sent_crane_state_ ||
          !equalsCraneState(last_sent_crane_state_, crane_state);
      const bool keepalive_due = (now - last_push_ts_) >= status_push_keepalive_interval_;

      if (device_status_changed || keepalive_due) {
        SignalSendDeviceStatus(device_status);
        last_sent_device_status_ = device_status;
        has_last_sent_device_status_ = true;
      }
      if (crane_state_changed || keepalive_due) {
        SignalSendCraneState(crane_state);
        last_sent_crane_state_ = crane_state;
        has_last_sent_crane_state_ = true;
      }
      if (device_status_changed || crane_state_changed || keepalive_due) {
        last_push_ts_ = now;
      }
    }
  }
}

Status DevicesManagerClient::loadConfig(const std::string& path) {
  return impl_ ? impl_->loadConfig(path) : Status{false, "no interface"};
}

Status DevicesManagerClient::init() {
  if (!impl_) return Status{false, "no interface"};
  Status s = impl_->init();
  if (s.ok) initialized_ = true;
  return s;
}

Status DevicesManagerClient::start() {
  if (!impl_) return Status{false, "no interface"};
  if (started_) return Status{true, "devices manager client already started"};
  Status s = impl_->start();
  if (!s.ok) return s;
  started_ = true;
  applied_speaker_mode_.reset();
  both_round_robin_active_ = false;
  both_stage_ = BothSpeakerStage::Playing3M;
  both_stage_deadline_ = std::chrono::steady_clock::now();
  // Clamp to sane bounds for reliable RTU/TCP device command pacing.
  const int play_window_ms = impl_->hoistHookDefaults().both_speaker_play_window_ms;
  const int switch_gap_ms = impl_->hoistHookDefaults().both_speaker_switch_gap_ms;
  both_play_window_ = std::chrono::milliseconds(std::clamp(play_window_ms, 500, 60000));
  both_switch_gap_ = std::chrono::milliseconds(std::clamp(switch_gap_ms, 100, 10000));
  battery_button_relay_channels_ = impl_->ioRelayDefaults().battery_button_relay_channels;
  battery_button_relay_channels_.erase(
      std::remove_if(battery_button_relay_channels_.begin(),
                     battery_button_relay_channels_.end(),
                     [](int ch) { return ch < 1 || ch > 16; }),
      battery_button_relay_channels_.end());
  std::sort(battery_button_relay_channels_.begin(), battery_button_relay_channels_.end());
  battery_button_relay_channels_.erase(
      std::unique(battery_button_relay_channels_.begin(), battery_button_relay_channels_.end()),
      battery_button_relay_channels_.end());
  has_last_sent_device_status_ = false;
  has_last_sent_crane_state_ = false;
  last_battery_button_cmd_.reset();
  last_received_battery_button_cmd_.reset();
  restoreBatteryButtonPowerStateFromRelays();
  last_push_ts_ = std::chrono::steady_clock::now() - status_push_keepalive_interval_;
  next_relay_state_sync_ts_ = std::chrono::steady_clock::now() + relay_state_sync_interval_;
  notify_stop_ = false;
  notify_thread_ = std::thread(&DevicesManagerClient::notifyThreadFunc, this);
  return s;
}

Status DevicesManagerClient::stop() {
  if (!impl_) return Status{false, "no interface"};
  notify_stop_ = true;
  if (notify_thread_.joinable())
    notify_thread_.join();
  Status s = impl_->stop();
  if (s.ok) started_ = false;
  return s;
}

ai_safety_common::DeviceStatus DevicesManagerClient::getDeviceStatus() const {
  if (!impl_) return ai_safety_common::DeviceStatus{};
  return impl_->getDeviceStatus();
}

ai_safety_common::CraneState DevicesManagerClient::getCraneState() const {
  if (!impl_) return ai_safety_common::CraneState{};
  return impl_->getCraneState();
}

std::unordered_map<std::string, std::uint16_t> DevicesManagerClient::getLatestLidarRawMm() const {
  if (!impl_) return {};
  return impl_->getLatestLidarRawMm();
}

bool DevicesManagerClient::isInitialized() const { return initialized_; }

bool DevicesManagerClient::isStarted() const { return started_; }

}  // namespace ai_safety_controller
