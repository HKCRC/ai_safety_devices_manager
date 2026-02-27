#include "ai_safety_controller/interface.hpp"

#include <atomic>
#include <csignal>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cerrno>
#include <sys/select.h>
#include <unistd.h>

namespace {

std::atomic<bool> g_stop_requested(false);

void signalHandler(int) {
  g_stop_requested = true;
}

std::vector<std::string> tokenize(const std::string& line) {
  std::istringstream iss(line);
  std::vector<std::string> tokens;
  std::string token;
  while (iss >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

void printHelp() {
  std::cout << "Commands:\n"
            << "  help\n"
            << "  start\n"
            << "  stop\n"
            << "  loadcfg <path>\n"
            << "  showcfg\n"
            << "  sensors\n"
            << "  cmds <sensor>\n"
            << "  <sensor> <cmd> [args...]\n"
            << "  quit\n";
}

}  // namespace

int main() {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  ai_safety_controller::Interface sdk;
  const ai_safety_controller::Status status = sdk.init();
  if (!status.ok) {
    std::cerr << "SDK init failed: " << status.message << std::endl;
    return 1;
  }

  std::cout << "SDK init success: " << status.message << std::endl;
  std::cout << "Enabled sensors:" << std::endl;
  for (const auto& sensor : sdk.enabledSensors()) {
    std::cout << "  - " << sensor << std::endl;
  }
  printHelp();
  const auto start_status = sdk.start();
  std::cout << (start_status.ok ? "ok: " : "error: ")
            << "auto start: " << start_status.message << "\n";

  std::string line;
  while (true) {
    if (g_stop_requested) {
      const auto r = sdk.stop();
      std::cout << "\n"
                << (r.ok ? "ok: " : "error: ")
                << "signal stop: " << r.message << "\n";
      break;
    }

    std::cout << "\nasc> " << std::flush;
    while (true) {
      if (g_stop_requested) break;
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(STDIN_FILENO, &rfds);
      timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = 200000;  // 200ms polling for prompt responsiveness to Ctrl+C.
      const int ret = ::select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
      if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds)) break;
      if (ret < 0 && errno != EINTR) {
        const auto r = sdk.stop();
        std::cout << "\n"
                  << (r.ok ? "ok: " : "error: ")
                  << "stdin select stop: " << r.message << "\n";
        return 0;
      }
    }
    if (g_stop_requested) continue;
    if (!std::getline(std::cin, line)) {
      const auto r = sdk.stop();
      std::cout << "\n"
                << (r.ok ? "ok: " : "error: ")
                << "stdin close stop: " << r.message << "\n";
      break;
    }
    const auto tokens = tokenize(line);
    if (tokens.empty()) continue;

    if (tokens[0] == "quit" || tokens[0] == "exit") {
      const auto r = sdk.stop();
      std::cout << (r.ok ? "ok: " : "error: ") << r.message << "\n";
      break;
    }
    if (tokens[0] == "help") {
      printHelp();
      continue;
    }
    if (tokens[0] == "start") {
      const auto r = sdk.start();
      std::cout << (r.ok ? "ok: " : "error: ") << r.message << "\n";
      continue;
    }
    if (tokens[0] == "stop") {
      const auto r = sdk.stop();
      std::cout << (r.ok ? "ok: " : "error: ") << r.message << "\n";
      continue;
    }
    if (tokens[0] == "loadcfg") {
      if (tokens.size() < 2) {
        std::cout << "usage: loadcfg <path>\n";
        continue;
      }
      const auto r = sdk.loadConfig(tokens[1]);
      std::cout << (r.ok ? "ok: " : "error: ") << r.message << "\n";
      continue;
    }
    if (tokens[0] == "showcfg") {
      const auto& b = sdk.batteryDefaults();
      const auto& s = sdk.solarDefaults();
      const auto& io = sdk.ioRelayDefaults();
      const auto& hh = sdk.hoistHookDefaults();
      const auto& cfg = sdk.encoderDefaults();
      const auto& lidars = sdk.spdLidarInstances();
      std::cout << "loaded_config: "
                << (sdk.loadedConfigPath().empty() ? "(builtin/default)" : sdk.loadedConfigPath()) << "\n";
      std::cout << "battery module_ip=" << b.module_ip
                << " enable=" << (b.enable ? "true" : "false")
                << " module_port=" << b.module_port
                << " module_slave_id=" << b.module_slave_id
                << " battery_slave_id=" << b.battery_slave_id
                << " query_hz=" << b.query_hz << "\n";
      std::cout << "solar module_ip=" << s.module_ip
                << " enable=" << (s.enable ? "true" : "false")
                << " module_port=" << s.module_port
                << " module_slave_id=" << s.module_slave_id
                << " solar_slave_id=" << s.solar_slave_id
                << " query_hz=" << s.query_hz << "\n";
      std::cout << "io_relay module_ip=" << io.module_ip
                << " enable=" << (io.enable ? "true" : "false")
                << " module_port=" << io.module_port
                << " module_slave_id=" << io.module_slave_id
                << " query_hz=" << io.query_hz << "\n";
      std::cout << "hoist_hook module_ip=" << hh.module_ip
                << " enable=" << (hh.enable ? "true" : "false")
                << " module_port=" << hh.module_port
                << " hook_slave_id=" << hh.hook_slave_id
                << " power_slave_id=" << hh.power_slave_id
                << " query_hz=" << hh.query_hz << "\n";
      std::cout << "encoder transport=" << cfg.transport
                << " enable=" << (cfg.enable ? "true" : "false")
                << " device=" << cfg.device
                << " baud=" << cfg.baud
                << " parity=" << cfg.parity
                << " data_bit=" << cfg.data_bit
                << " stop_bit=" << cfg.stop_bit
                << " slave=" << cfg.slave
                << " ip=" << cfg.ip
                << " port=" << cfg.port
                << " query_hz=" << cfg.query_hz << "\n";
      std::cout << "spd_lidar query_hz=" << sdk.spdLidarQueryHz()
                << " instances=" << lidars.size() << "\n";
      for (size_t i = 0; i < lidars.size(); ++i) {
        std::cout << "  [" << i << "] id=" << lidars[i].id
                  << " enable=" << (lidars[i].enable ? "true" : "false")
                  << " mode=" << lidars[i].mode
                  << " local_ip=" << lidars[i].local_ip
                  << " local_port=" << lidars[i].local_port
                  << " device_ip=" << lidars[i].device_ip
                  << " device_port=" << lidars[i].device_port;
        if (!lidars[i].role.empty()) std::cout << " role=" << lidars[i].role;
        std::cout << " priority=" << lidars[i].priority << "\n";
      }
      continue;
    }
    if (tokens[0] == "sensors") {
      for (const auto& sensor : sdk.enabledSensors()) {
        std::cout << "  - " << sensor << "\n";
      }
      continue;
    }
    if (tokens[0] == "cmds") {
      if (tokens.size() < 2) {
        std::cout << "usage: cmds <sensor>\n";
        continue;
      }
      const auto cmds = sdk.availableCommands(tokens[1]);
      if (cmds.empty()) {
        std::cout << "unknown sensor\n";
        continue;
      }
      std::cout << tokens[1] << " commands:\n";
      for (const auto& cmd : cmds) {
        std::cout << "  - " << cmd << "\n";
      }
      continue;
    }

    const std::string sensor = tokens[0];
    std::vector<std::string> args(tokens.begin() + 1, tokens.end());
    const auto result = sdk.dispatchCommand(sensor, args);
    if (!result.ok) {
      std::cout << "error: " << result.message << "\n";
    }
  }

  return 0;
}
