/**
 * main_test: 仅与 ai_safety_devices_manager 交互的测试程序。
 * - Pull 槽：通过终端命令设置 getAlertMessage / getBatteryButtonSignals 的返回值。
 * - Push 槽：设备管理定时推送的数据会缓存，可通过终端命令读取并打印。
 *
 * 命令: help | alert <enable3|enable7|3m|7m> <on|off> | power <none|on|off> | status | crane | quit
 */

#include "ai_safety_controller/devices_manager_client.hpp"
#include "ai_safety_common/shared_memory_types.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#if defined(__linux__) || defined(__unix__)
#include <sys/select.h>
#include <unistd.h>
#endif

namespace {

std::atomic<bool> g_running{true};

void on_signal(int) {
  g_running.store(false);
}

std::string to_string(ai_safety_common::DeviceStatus::SolarChargeState s) {
  using S = ai_safety_common::DeviceStatus::SolarChargeState;
  if (s == S::NotCharging) return "NotCharging";
  if (s == S::Charging) return "Charging";
  if (s == S::Fault) return "Fault";
  return "Unknown";
}

std::string to_string(ai_safety_common::DeviceStatus::EquipmentState s) {
  using E = ai_safety_common::DeviceStatus::EquipmentState;
  if (s == E::Offline) return "Offline";
  if (s == E::Standby) return "Standby";
  if (s == E::Active) return "Active";
  return "Unknown";
}

void print_battery_info(const char* label, const ai_safety_common::DeviceStatus::BatteryInfo& b) {
  std::cout << "  " << label << ": percent=" << static_cast<int>(b.percent)
            << "% remainingMin=" << b.remainingMin
            << " isCharging=" << (b.isCharging ? 1 : 0)
            << " chargingTimeMin=" << b.chargingTimeMin
            << " voltageV=" << b.voltageV << " currentA=" << b.currentA << "\n";
}

struct TestState {
  std::mutex mtx;
  ai_safety_common::AlertMessage pull_alert;
  std::uint8_t pull_power = 0;  // 0=None 1=PowerOn 2=PowerOff
  ai_safety_common::DeviceStatus last_device_status;
  ai_safety_common::CraneState last_crane_state;
  bool has_device_status = false;
  bool has_crane_state = false;
};

void print_help() {
  std::cout << "  help                    - 本帮助\n"
            << "  alert [key on|off]...   - 设置 getAlertMessage，可多组 (key: enable3 enable7 3m 7m)\n"
            << "  power <none|on|off>     - 设置 getBatteryButtonSignals 返回值\n"
            << "  status                  - 从 push 槽读取并打印最近一次 DeviceStatus\n"
            << "  crane                   - 从 push 槽读取并打印最近一次 CraneState\n"
            << "  quit                    - 退出\n";
}

bool parse_on_off(const std::string& s, bool* out) {
  if (s == "on") { *out = true; return true; }
  if (s == "off") { *out = false; return true; }
  return false;
}

bool run_command(TestState& state, const std::string& line) {
  std::istringstream iss(line);
  std::string cmd;
  if (!(iss >> cmd)) return true;

  if (cmd == "help") {
    print_help();
    return true;
  }
  if (cmd == "quit") {
    g_running.store(false);
    return true;
  }
  if (cmd == "status") {
    std::lock_guard<std::mutex> lock(state.mtx);
    if (!state.has_device_status) {
      std::cout << "[push] 尚未收到 setDeviceStatus，请稍后再试\n";
      return true;
    }
    const auto& d = state.last_device_status;
    std::cout << "[push] DeviceStatus:\n";
    std::cout << "  solarCharge=" << to_string(d.solarCharge)
              << " trolleyState=" << to_string(d.trolleyState)
              << " hookState=" << to_string(d.hookState) << "\n";
    print_battery_info("trolleyBattery", d.trolleyBattery);
    print_battery_info("hookBattery", d.hookBattery);
    return true;
  }
  if (cmd == "crane") {
    std::lock_guard<std::mutex> lock(state.mtx);
    if (!state.has_crane_state) {
      std::cout << "[push] 尚未收到 setCraneState，请稍后再试\n";
      return true;
    }
    const auto& c = state.last_crane_state;
    std::cout << "[push] CraneState: hookToTrolley=" << c.hookToTrolleyDistanceM
              << "m groundToTrolley=" << c.groundToTrolleyDistanceM << "m\n";
    return true;
  }
  if (cmd == "alert") {
    std::string key, onoff;
    int applied = 0;
    std::lock_guard<std::mutex> lock(state.mtx);
    while (iss >> key >> onoff) {
      bool val;
      if (!parse_on_off(onoff, &val)) {
        std::cout << "expected on|off for key '" << key << "'\n";
        continue;
      }
      if (key == "enable3") state.pull_alert.Enable3Alert = val;
      else if (key == "enable7") state.pull_alert.Enable7Alert = val;
      else if (key == "3m") state.pull_alert.Alert3M = val;
      else if (key == "7m") state.pull_alert.Alert7M = val;
      else {
        std::cout << "unknown key '" << key << "' (use enable3 enable7 3m 7m)\n";
        continue;
      }
      ++applied;
    }
    if (applied == 0) {
      std::cout << "usage: alert [<enable3|enable7|3m|7m> <on|off>]+\n";
      return true;
    }
    std::cout << "[pull] getAlertMessage: Enable3Alert=" << state.pull_alert.Enable3Alert
              << " Enable7Alert=" << state.pull_alert.Enable7Alert
              << " Alert3M=" << state.pull_alert.Alert3M
              << " Alert7M=" << state.pull_alert.Alert7M << "\n";
    return true;
  }
  if (cmd == "power") {
    std::string arg;
    if (!(iss >> arg)) {
      std::cout << "usage: power <none|on|off>\n";
      return true;
    }
    std::lock_guard<std::mutex> lock(state.mtx);
    if (arg == "none") state.pull_power = 0;
    else if (arg == "on") state.pull_power = 1;
    else if (arg == "off") state.pull_power = 2;
    else {
      std::cout << "expected none|on|off\n";
      return true;
    }
    std::cout << "[pull] getBatteryButtonSignals: " << arg << " (" << static_cast<int>(state.pull_power) << ")\n";
    return true;
  }

  std::cout << "unknown command (help for list)\n";
  return true;
}

// 从 stdin 读一行，超时则返回 false（未读到完整行）
bool read_line_with_timeout(std::string& line, int timeout_ms) {
#if defined(__linux__) || defined(__unix__)
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  int r = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
  if (r <= 0) return false;
  if (!std::getline(std::cin, line)) return false;
  return true;
#else
  (void)timeout_ms;
  if (!std::getline(std::cin, line)) return false;
  return true;
#endif
}

}  // namespace

int main(int argc, char* argv[]) {
  // 默认使用本模块 config；从仓库根目录运行时可通过参数传入路径
  std::string config_path = "config/common_config.json";
  if (argc >= 2 && argv[1][0] != '\0') {
    config_path = argv[1];
  }

  std::cout << "main_test: devices_manager client (config=" << config_path << ")\n";
  print_help();

  TestState state;
  // 默认全部关闭，仅用户通过 alert <key> on 设置的项为 true
  state.pull_alert.Enable3Alert = false;
  state.pull_alert.Enable7Alert = false;
  state.pull_alert.Alert3M = false;
  state.pull_alert.Alert7M = false;

  ai_safety_controller::DevicesManagerClient client;

  // Pull 槽：从 state 读取，由终端命令更新
  client.getAlertMessage.connect([&state]() {
    std::lock_guard<std::mutex> lock(state.mtx);
    return state.pull_alert;
  });
  client.getBatteryButtonSignals.connect([&state]() {
    std::lock_guard<std::mutex> lock(state.mtx);
    return state.pull_power;
  });

  if (!client.loadConfig(config_path).ok) {
    std::cerr << "loadConfig failed\n";
    return 1;
  }
  if (!client.init().ok) {
    std::cerr << "init failed\n";
    return 1;
  }

  // Push 槽：只缓存最新数据，不打印；用户输入 status/crane 时再按需打印
  client.setDeviceStatus.connect([&state](const ai_safety_common::DeviceStatus& d) {
    std::lock_guard<std::mutex> lock(state.mtx);
    state.last_device_status = d;
    state.has_device_status = true;
  });
  client.setCraneState.connect([&state](const ai_safety_common::CraneState& c) {
    std::lock_guard<std::mutex> lock(state.mtx);
    state.last_crane_state = c;
    state.has_crane_state = true;
  });

  if (!client.start().ok) {
    std::cerr << "start failed\n";
    return 1;
  }

  std::signal(SIGINT, on_signal);
  std::cout << "Running. Type commands (help for list), or Ctrl+C to stop.\n";

  std::string line;
  while (g_running.load()) {
    if (read_line_with_timeout(line, 300)) {
      if (!line.empty()) {
        run_command(state, line);
      }
    }
  }

  client.stop();
  std::cout << "main_test: stopped.\n";
  return 0;
}
