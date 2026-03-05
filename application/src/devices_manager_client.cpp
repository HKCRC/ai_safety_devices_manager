#include "ai_safety_controller/devices_manager_client.hpp"
#include "ai_safety_controller/interface.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

namespace ai_safety_controller {

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
    case SpeakerMode::Off:
    default:
      return "off";
  }
}

bool DevicesManagerClient::applySpeakerMode(SpeakerMode mode) {
  if (!impl_) return false;
  if (applied_speaker_mode_.has_value() && applied_speaker_mode_.value() == mode) return true;
  const std::vector<std::string> args{"speaker_ctl", toSpeakerCtlArg(mode)};
  const Status ctl = impl_->dispatchCommand("hoist_hook", args);
  if (ctl.ok) {
    applied_speaker_mode_ = mode;
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
    if (!both_round_robin_active_) {
      both_round_robin_active_ = true;
      both_stage_ = BothSpeakerStage::Playing3M;
      if (applySpeakerMode(SpeakerMode::M3)) {
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
        if (applySpeakerMode(SpeakerMode::Off)) {
          both_stage_ = BothSpeakerStage::GapAfter3M;
          both_stage_deadline_ = now + both_switch_gap_;
        } else {
          both_stage_deadline_ = now + std::chrono::milliseconds(200);
        }
        break;
      case BothSpeakerStage::GapAfter3M:
        if (applySpeakerMode(SpeakerMode::M7)) {
          both_stage_ = BothSpeakerStage::Playing7M;
          both_stage_deadline_ = now + both_play_window_;
        } else {
          both_stage_deadline_ = now + std::chrono::milliseconds(200);
        }
        break;
      case BothSpeakerStage::Playing7M:
        if (applySpeakerMode(SpeakerMode::Off)) {
          both_stage_ = BothSpeakerStage::GapAfter7M;
          both_stage_deadline_ = now + both_switch_gap_;
        } else {
          both_stage_deadline_ = now + std::chrono::milliseconds(200);
        }
        break;
      case BothSpeakerStage::GapAfter7M:
        if (applySpeakerMode(SpeakerMode::M3)) {
          both_stage_ = BothSpeakerStage::Playing3M;
          both_stage_deadline_ = now + both_play_window_;
        } else {
          both_stage_deadline_ = now + std::chrono::milliseconds(200);
        }
        break;
    }
    return;
  }

  both_round_robin_active_ = false;
  if (trig3) {
    (void)applySpeakerMode(SpeakerMode::M3);
  } else if (trig7) {
    (void)applySpeakerMode(SpeakerMode::M7);
  } else {
    (void)applySpeakerMode(SpeakerMode::Off);
  }
}

void DevicesManagerClient::applyBatteryButtonControl(std::uint8_t raw_cmd) {
  if (!impl_) return;
  if (raw_cmd > static_cast<std::uint8_t>(PowerCommand::PowerOff)) return;

  const PowerCommand cmd = static_cast<PowerCommand>(raw_cmd);
  if (cmd == PowerCommand::None) {
    last_battery_button_cmd_.reset();
    return;
  }
  if (last_battery_button_cmd_.has_value() && last_battery_button_cmd_.value() == cmd) return;

  const std::vector<std::string> args{
      (cmd == PowerCommand::PowerOn) ? "on" : "off",
      std::to_string(battery_button_relay_channel_)};
  const Status ctl = impl_->dispatchCommand("io_relay", args);
  if (ctl.ok) {
    last_battery_button_cmd_ = cmd;
  }
}

void DevicesManagerClient::notifyThreadFunc() {
  while (!notify_stop_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (notify_stop_) break;
    if (impl_) {
      if (!getAlertMessage.empty()) {
        auto alert_opt = getAlertMessage();
        if (alert_opt) {
          applySpeakerControlByAlert(*alert_opt);
        }
      }
      if (!getBatteryButtonSignals.empty()) {
        auto cmd_opt = getBatteryButtonSignals();
        if (cmd_opt) {
          applyBatteryButtonControl(*cmd_opt);
        }
      }
      const auto now = std::chrono::steady_clock::now();
      if (now - last_push_ts_ >= std::chrono::seconds(1)) {
        setDeviceStatus(getDeviceStatus());
        setCraneState(getCraneState());
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
  battery_button_relay_channel_ =
      std::clamp(impl_->ioRelayDefaults().battery_button_relay_channel, 1, 16);
  last_battery_button_cmd_.reset();
  last_push_ts_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
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

bool DevicesManagerClient::isInitialized() const { return initialized_; }

bool DevicesManagerClient::isStarted() const { return started_; }

}  // namespace ai_safety_controller
