#include "ai_safety_controller/interface.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <unordered_map>
#include <utility>
#include <vector>
#include <cerrno>
#include <cstring>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace ai_safety_controller {

namespace {

std::string joinArgs(const std::vector<std::string>& args, size_t start) {
  std::ostringstream oss;
  for (size_t i = start; i < args.size(); ++i) {
    if (i > start) oss << " ";
    oss << args[i];
  }
  return oss.str();
}

std::string formatEpochSeconds(double ts) {
  if (ts <= 0.0) return "n/a";
  const std::time_t sec = static_cast<std::time_t>(ts);
  const int ms = static_cast<int>((ts - static_cast<double>(sec)) * 1000.0);
  std::tm tm_buf{};
  localtime_r(&sec, &tm_buf);
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%F %T")
      << "." << std::setfill('0') << std::setw(3) << std::max(0, ms);
  return oss.str();
}

bool spdLidarExchangeTcp(const ai_safety_controller::Interface::SpdLidarInstanceDefaults& cfg,
                         const std::vector<uint8_t>& request,
                         std::vector<uint8_t>* response,
                         std::string* error) {
  if (!response) return false;
  response->clear();

  // Current simulator/integration uses client mode: connect to device endpoint.
  std::string ip = cfg.device_ip;
  int port = cfg.device_port;
  if (cfg.mode == "server") {
    // Fallback for server-mode configs: try local endpoint.
    ip = cfg.local_ip;
    port = cfg.local_port;
  }
  if (ip.empty() || port <= 0) {
    if (error) *error = "invalid spd_lidar endpoint";
    return false;
  }

  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    if (error) *error = std::string("socket failed: ") + std::strerror(errno);
    return false;
  }

  timeval tv{};
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    if (error) *error = "invalid ip: " + ip;
    ::close(fd);
    return false;
  }
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    if (error) *error = std::string("connect failed: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }

  const ssize_t sent = ::send(fd, request.data(), request.size(), 0);
  if (sent < 0 || static_cast<size_t>(sent) != request.size()) {
    if (error) *error = std::string("send failed: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }

  uint8_t buf[256];
  const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
  if (n <= 0) {
    if (error) *error = std::string("recv failed: ") + std::strerror(errno);
    ::close(fd);
    return false;
  }
  response->assign(buf, buf + n);
  ::close(fd);
  return true;
}

class FunctionDriverAdapter : public DriverAdapter {
 public:
  using StatusFn = std::function<Status()>;
  using QueryFn = std::function<Status(const std::vector<std::string>&)>;
  using CommandsFn = std::function<std::vector<std::string>()>;

  FunctionDriverAdapter(std::string driver_name,
                        StatusFn init_fn,
                        StatusFn start_fn,
                        StatusFn stop_fn,
                        QueryFn query_fn,
                        CommandsFn commands_fn)
      : name_(std::move(driver_name)),
        init_fn_(std::move(init_fn)),
        start_fn_(std::move(start_fn)),
        stop_fn_(std::move(stop_fn)),
        query_fn_(std::move(query_fn)),
        commands_fn_(std::move(commands_fn)) {}

  const std::string& name() const override { return name_; }
  Status init() override { return init_fn_ ? init_fn_() : Status{true, "ok"}; }
  Status start() override { return start_fn_ ? start_fn_() : Status{true, "ok"}; }
  Status stop() override { return stop_fn_ ? stop_fn_() : Status{true, "ok"}; }
  Status query(const std::vector<std::string>& args) override {
    return query_fn_ ? query_fn_(args) : Status{false, "query unsupported"};
  }
  std::vector<std::string> availableCommands() const override {
    return commands_fn_ ? commands_fn_() : std::vector<std::string>{};
  }

 private:
  std::string name_;
  StatusFn init_fn_;
  StatusFn start_fn_;
  StatusFn stop_fn_;
  QueryFn query_fn_;
  CommandsFn commands_fn_;
};

}  // namespace

Interface::Interface()
    : initialized_(false),
      started_(false),
      config_loaded_(false),
      spd_lidar_query_hz_(0.0),
      auto_query_running_(false),
      snapshot_printer_running_(false) {}

Interface::~Interface() {
  stopAutoQueryPolling();
  stopSnapshotPrinter();
  if (started_) {
    for (std::unordered_map<std::string, std::unique_ptr<DriverAdapter>>::iterator it = drivers_.begin();
         it != drivers_.end(); ++it) {
      it->second->stop();
    }
    started_ = false;
  }
}

bool Interface::parseInt(const std::string& text, int* out) {
  if (!out) return false;
  try {
    size_t idx = 0;
    *out = std::stoi(text, &idx, 0);
    return idx == text.size();
  } catch (...) {
    return false;
  }
}

bool Interface::parseBool(const std::string& text, bool* out) {
  if (!out) return false;
  if (text == "true" || text == "1" || text == "on") {
    *out = true;
    return true;
  }
  if (text == "false" || text == "0" || text == "off") {
    *out = false;
    return true;
  }
  return false;
}

bool Interface::parseDouble(const std::string& text, double* out) {
  if (!out) return false;
  try {
    size_t idx = 0;
    *out = std::stod(text, &idx);
    return idx == text.size();
  } catch (...) {
    return false;
  }
}

const std::string& Interface::loadedConfigPath() const {
  return loaded_config_path_;
}

const Interface::BatteryDefaults& Interface::batteryDefaults() const {
  return battery_defaults_;
}

const Interface::SolarDefaults& Interface::solarDefaults() const {
  return solar_defaults_;
}

const Interface::IoRelayDefaults& Interface::ioRelayDefaults() const {
  return io_relay_defaults_;
}

const Interface::HoistHookDefaults& Interface::hoistHookDefaults() const {
  return hoist_hook_defaults_;
}

const Interface::EncoderDefaults& Interface::encoderDefaults() const {
  return encoder_defaults_;
}

const std::vector<Interface::SpdLidarInstanceDefaults>& Interface::spdLidarInstances() const {
  return spd_lidar_instances_;
}

double Interface::spdLidarQueryHz() const {
  return spd_lidar_query_hz_;
}

std::string Interface::extractObjectBody(const std::string& json_text, const std::string& key) {
  const std::string marker = "\"" + key + "\"";
  const size_t key_pos = json_text.find(marker);
  if (key_pos == std::string::npos) return "";

  const size_t brace_pos = json_text.find('{', key_pos);
  if (brace_pos == std::string::npos) return "";

  int depth = 0;
  for (size_t i = brace_pos; i < json_text.size(); ++i) {
    if (json_text[i] == '{') ++depth;
    else if (json_text[i] == '}') --depth;
    if (depth == 0) {
      return json_text.substr(brace_pos + 1, i - brace_pos - 1);
    }
  }
  return "";
}

std::string Interface::extractArrayBody(const std::string& json_text, const std::string& key) {
  const std::string marker = "\"" + key + "\"";
  const size_t key_pos = json_text.find(marker);
  if (key_pos == std::string::npos) return "";

  const size_t bracket_pos = json_text.find('[', key_pos);
  if (bracket_pos == std::string::npos) return "";

  int depth = 0;
  for (size_t i = bracket_pos; i < json_text.size(); ++i) {
    if (json_text[i] == '[') ++depth;
    else if (json_text[i] == ']') --depth;
    if (depth == 0) {
      return json_text.substr(bracket_pos + 1, i - bracket_pos - 1);
    }
  }
  return "";
}

std::vector<std::string> Interface::splitTopLevelObjects(const std::string& array_body) {
  std::vector<std::string> out;
  int depth = 0;
  size_t start = std::string::npos;
  for (size_t i = 0; i < array_body.size(); ++i) {
    const char ch = array_body[i];
    if (ch == '{') {
      if (depth == 0) start = i;
      ++depth;
    } else if (ch == '}') {
      --depth;
      if (depth == 0 && start != std::string::npos) {
        out.push_back(array_body.substr(start + 1, i - start - 1));
        start = std::string::npos;
      }
    }
  }
  return out;
}

bool Interface::extractStringValue(const std::string& object_body, const std::string& key, std::string* out) {
  if (!out) return false;
  const std::regex re("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
  std::smatch m;
  if (!std::regex_search(object_body, m, re)) return false;
  *out = m[1].str();
  return true;
}

bool Interface::extractIntValue(const std::string& object_body, const std::string& key, int* out) {
  if (!out) return false;
  const std::regex re("\"" + key + "\"\\s*:\\s*(-?[0-9]+)");
  std::smatch m;
  if (!std::regex_search(object_body, m, re)) return false;
  return parseInt(m[1].str(), out);
}

bool Interface::extractBoolValue(const std::string& object_body, const std::string& key, bool* out) {
  if (!out) return false;
  const std::regex re("\"" + key + "\"\\s*:\\s*(true|false|1|0)");
  std::smatch m;
  if (!std::regex_search(object_body, m, re)) return false;
  return parseBool(m[1].str(), out);
}

bool Interface::extractDoubleValue(const std::string& object_body, const std::string& key, double* out) {
  if (!out) return false;
  const std::regex re("\"" + key + "\"\\s*:\\s*(-?(?:[0-9]+(?:\\.[0-9]+)?|\\.[0-9]+))");
  std::smatch m;
  if (!std::regex_search(object_body, m, re)) return false;
  return parseDouble(m[1].str(), out);
}

void Interface::applyBatteryDefaultsFromJson(const std::string& json_text) {
  const std::string runtime_body = extractObjectBody(json_text, "runtime");
  if (runtime_body.empty()) return;
  const std::string body = extractObjectBody(runtime_body, "battery");
  if (body.empty()) return;

  bool enable = true;
  if (extractBoolValue(body, "enable", &enable)) battery_defaults_.enable = enable;
  std::string module_ip;
  if (extractStringValue(body, "module_ip", &module_ip)) battery_defaults_.module_ip = module_ip;
  int module_port = 0;
  if (extractIntValue(body, "module_port", &module_port)) battery_defaults_.module_port = module_port;
  int module_slave_id = 0;
  if (extractIntValue(body, "module_slave_id", &module_slave_id)) battery_defaults_.module_slave_id = module_slave_id;
  int battery_slave_id = 0;
  if (extractIntValue(body, "battery_slave_id", &battery_slave_id)) battery_defaults_.battery_slave_id = battery_slave_id;
  double query_hz = 0.0;
  if (extractDoubleValue(body, "query_hz", &query_hz)) battery_defaults_.query_hz = query_hz;
}

void Interface::applySolarDefaultsFromJson(const std::string& json_text) {
  const std::string runtime_body = extractObjectBody(json_text, "runtime");
  if (runtime_body.empty()) return;
  const std::string body = extractObjectBody(runtime_body, "solar");
  if (body.empty()) return;

  bool enable = true;
  if (extractBoolValue(body, "enable", &enable)) solar_defaults_.enable = enable;
  std::string module_ip;
  if (extractStringValue(body, "module_ip", &module_ip)) solar_defaults_.module_ip = module_ip;
  int module_port = 0;
  if (extractIntValue(body, "module_port", &module_port)) solar_defaults_.module_port = module_port;
  int module_slave_id = 0;
  if (extractIntValue(body, "module_slave_id", &module_slave_id)) solar_defaults_.module_slave_id = module_slave_id;
  int solar_slave_id = 0;
  if (extractIntValue(body, "solar_slave_id", &solar_slave_id)) solar_defaults_.solar_slave_id = solar_slave_id;
  double query_hz = 0.0;
  if (extractDoubleValue(body, "query_hz", &query_hz)) solar_defaults_.query_hz = query_hz;
}

void Interface::applyIoRelayDefaultsFromJson(const std::string& json_text) {
  const std::string runtime_body = extractObjectBody(json_text, "runtime");
  if (runtime_body.empty()) return;
  const std::string body = extractObjectBody(runtime_body, "io_relay");
  if (body.empty()) return;

  bool enable = true;
  if (extractBoolValue(body, "enable", &enable)) io_relay_defaults_.enable = enable;
  std::string module_ip;
  if (extractStringValue(body, "module_ip", &module_ip)) io_relay_defaults_.module_ip = module_ip;
  int module_port = 0;
  if (extractIntValue(body, "module_port", &module_port)) io_relay_defaults_.module_port = module_port;
  int module_slave_id = 0;
  if (extractIntValue(body, "module_slave_id", &module_slave_id)) io_relay_defaults_.module_slave_id = module_slave_id;
  double query_hz = 0.0;
  if (extractDoubleValue(body, "query_hz", &query_hz)) io_relay_defaults_.query_hz = query_hz;
}

void Interface::applyHoistHookDefaultsFromJson(const std::string& json_text) {
  const std::string runtime_body = extractObjectBody(json_text, "runtime");
  if (runtime_body.empty()) return;
  const std::string body = extractObjectBody(runtime_body, "hoist_hook");
  if (body.empty()) return;

  bool enable = true;
  if (extractBoolValue(body, "enable", &enable)) hoist_hook_defaults_.enable = enable;
  std::string module_ip;
  if (extractStringValue(body, "module_ip", &module_ip)) hoist_hook_defaults_.module_ip = module_ip;
  int module_port = 0;
  if (extractIntValue(body, "module_port", &module_port)) hoist_hook_defaults_.module_port = module_port;
  int hook_slave_id = 0;
  if (extractIntValue(body, "hook_slave_id", &hook_slave_id)) hoist_hook_defaults_.hook_slave_id = hook_slave_id;
  int power_slave_id = 0;
  if (extractIntValue(body, "power_slave_id", &power_slave_id)) hoist_hook_defaults_.power_slave_id = power_slave_id;
  double query_hz = 0.0;
  if (extractDoubleValue(body, "query_hz", &query_hz)) hoist_hook_defaults_.query_hz = query_hz;
}

void Interface::applyEncoderDefaultsFromJson(const std::string& json_text) {
  const std::string runtime_body = extractObjectBody(json_text, "runtime");
  if (runtime_body.empty()) return;
  const std::string body = extractObjectBody(runtime_body, "multi_turn_encoder");
  if (body.empty()) return;

  bool enable = true;
  if (extractBoolValue(body, "enable", &enable)) encoder_defaults_.enable = enable;
  std::string transport;
  if (extractStringValue(body, "transport", &transport)) encoder_defaults_.transport = transport;
  std::string device;
  if (extractStringValue(body, "device", &device)) encoder_defaults_.device = device;
  int baud = 0;
  if (extractIntValue(body, "baud", &baud)) encoder_defaults_.baud = baud;
  std::string parity;
  if (extractStringValue(body, "parity", &parity) && !parity.empty()) encoder_defaults_.parity = parity[0];
  int data_bit = 0;
  if (extractIntValue(body, "data_bit", &data_bit)) encoder_defaults_.data_bit = data_bit;
  int stop_bit = 0;
  if (extractIntValue(body, "stop_bit", &stop_bit)) encoder_defaults_.stop_bit = stop_bit;
  int slave = 0;
  if (extractIntValue(body, "slave", &slave)) encoder_defaults_.slave = slave;
  std::string ip;
  if (extractStringValue(body, "ip", &ip)) encoder_defaults_.ip = ip;
  int port = 0;
  if (extractIntValue(body, "port", &port)) encoder_defaults_.port = port;
  double query_hz = 0.0;
  if (extractDoubleValue(body, "query_hz", &query_hz)) encoder_defaults_.query_hz = query_hz;
}

void Interface::applySpdLidarDefaultsFromJson(const std::string& json_text) {
  spd_lidar_query_hz_ = 0.0;
  const std::string runtime_body = extractObjectBody(json_text, "runtime");
  if (runtime_body.empty()) return;
  const std::string body = extractObjectBody(runtime_body, "spd_lidar");
  if (body.empty()) return;
  double query_hz = 0.0;
  if (extractDoubleValue(body, "query_hz", &query_hz)) spd_lidar_query_hz_ = query_hz;

  spd_lidar_instances_.clear();
  const std::string instances_body = extractArrayBody(body, "instances");
  if (!instances_body.empty()) {
    const std::vector<std::string> object_bodies = splitTopLevelObjects(instances_body);
    for (size_t i = 0; i < object_bodies.size(); ++i) {
      SpdLidarInstanceDefaults one;
      std::string id;
      if (extractStringValue(object_bodies[i], "id", &id) && !id.empty()) one.id = id;
      bool enable = true;
      if (extractBoolValue(object_bodies[i], "enable", &enable)) one.enable = enable;
      std::string mode;
      if (extractStringValue(object_bodies[i], "mode", &mode)) one.mode = mode;
      std::string local_ip;
      if (extractStringValue(object_bodies[i], "local_ip", &local_ip)) one.local_ip = local_ip;
      int local_port = 0;
      if (extractIntValue(object_bodies[i], "local_port", &local_port)) one.local_port = local_port;
      std::string device_ip;
      if (extractStringValue(object_bodies[i], "device_ip", &device_ip)) one.device_ip = device_ip;
      int device_port = 0;
      if (extractIntValue(object_bodies[i], "device_port", &device_port)) one.device_port = device_port;
      std::string role;
      if (extractStringValue(object_bodies[i], "role", &role)) one.role = role;
      int priority = 0;
      if (extractIntValue(object_bodies[i], "priority", &priority)) one.priority = priority;
      spd_lidar_instances_.push_back(one);
    }
    return;
  }

  // Backward compatibility for legacy single-object configuration.
  SpdLidarInstanceDefaults one;
  std::string mode;
  if (extractStringValue(body, "mode", &mode)) one.mode = mode;
  std::string local_ip;
  if (extractStringValue(body, "local_ip", &local_ip)) one.local_ip = local_ip;
  int local_port = 0;
  if (extractIntValue(body, "local_port", &local_port)) one.local_port = local_port;
  std::string device_ip;
  if (extractStringValue(body, "device_ip", &device_ip)) one.device_ip = device_ip;
  int device_port = 0;
  if (extractIntValue(body, "device_port", &device_port)) one.device_port = device_port;
  spd_lidar_instances_.push_back(one);
}

Status Interface::loadConfig(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return Status{false, "failed to open config file: " + path};
  }
  std::ostringstream ss;
  ss << ifs.rdbuf();
  const std::string json_text = ss.str();

  applyBatteryDefaultsFromJson(json_text);
  applySolarDefaultsFromJson(json_text);
  applyIoRelayDefaultsFromJson(json_text);
  applyHoistHookDefaultsFromJson(json_text);
  applyEncoderDefaultsFromJson(json_text);
  applySpdLidarDefaultsFromJson(json_text);

  config_loaded_ = true;
  loaded_config_path_ = path;
  return Status{true, "config loaded: " + path};
}

Status Interface::loadDefaultConfigIfPresent() {
  if (config_loaded_) return Status{true, "config already loaded"};

  std::vector<std::string> candidates;
  const char* env_cfg = std::getenv("ASC_CONFIG");
  if (env_cfg && env_cfg[0] != '\0') candidates.push_back(env_cfg);
  candidates.push_back("config/common_config.json");
  candidates.push_back("../config/common_config.json");
  candidates.push_back("../../config/common_config.json");

  for (const auto& path : candidates) {
    if (std::filesystem::exists(path)) {
      return loadConfig(path);
    }
  }
  return Status{true, "default config not found, using builtin defaults"};
}

void Interface::buildDriverAdapters() {
  drivers_.clear();

#ifdef ASC_ENABLE_BATTERY
  if (battery_) {
    drivers_["battery"] = std::make_unique<FunctionDriverAdapter>(
        "battery",
        []() { return Status{true, "ok"}; },
        []() { return Status{true, "battery is request-response driver"}; },
        []() { return Status{true, "battery is request-response driver"}; },
        [this](const std::vector<std::string>& args) { return queryBattery(args); },
        []() {
          return std::vector<std::string>{
              "map", "basic", "cell", "temp", "mos", "protect", "all", "scan", "addr", "get", "set"};
        });
  }
#endif
#ifdef ASC_ENABLE_SOLAR
  if (solar_) {
    drivers_["solar"] = std::make_unique<FunctionDriverAdapter>(
        "solar",
        []() { return Status{true, "ok"}; },
        []() { return Status{true, "solar is request-response driver"}; },
        []() { return Status{true, "solar is request-response driver"}; },
        [this](const std::vector<std::string>& args) { return querySolar(args); },
        []() {
          return std::vector<std::string>{"map", "basic", "status", "all", "scan", "get", "set"};
        });
  }
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
  if (hoist_hook_) {
    drivers_["hoist_hook"] = std::make_unique<FunctionDriverAdapter>(
        "hoist_hook",
        []() { return Status{true, "ok"}; },
        []() { return Status{true, "hoist_hook is request-response driver"}; },
        []() { return Status{true, "hoist_hook is request-response driver"}; },
        [this](const std::vector<std::string>& args) { return queryHoistHook(args); },
        []() {
          return std::vector<std::string>{"map", "speaker", "light", "rfid", "power", "gps", "all",
                                          "speaker_ctl", "light_ctl", "get", "set"};
        });
  }
#endif
#ifdef ASC_ENABLE_IO_RELAY
  if (io_relay_) {
    drivers_["io_relay"] = std::make_unique<FunctionDriverAdapter>(
        "io_relay",
        []() { return Status{true, "ok"}; },
        []() { return Status{true, "io_relay is request-response driver"}; },
        []() { return Status{true, "io_relay is request-response driver"}; },
        [this](const std::vector<std::string>& args) { return queryIoRelay(args); },
        []() { return std::vector<std::string>{"on", "off", "read"}; });
  }
#endif
#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
  if (multi_turn_encoder_) {
    drivers_["multi_turn_encoder"] = std::make_unique<FunctionDriverAdapter>(
        "multi_turn_encoder",
        []() { return Status{true, "ok"}; },
        [this]() {
          const bool ok = multi_turn_encoder_->connect();
          if (!ok) return Status{false, "encoder connect failed"};
          multi_turn_encoder_->run();
          return Status{true, "encoder started"};
        },
        [this]() {
          multi_turn_encoder_->stop();
          return Status{true, "encoder stopped"};
        },
        [this](const std::vector<std::string>& args) { return queryMultiTurnEncoder(args); },
        []() { return std::vector<std::string>{"connect", "run", "get", "status", "stop"}; });
  }
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
  if (!spd_lidar_instances_core_.empty()) {
    drivers_["spd_lidar"] = std::make_unique<FunctionDriverAdapter>(
        "spd_lidar",
        []() { return Status{true, "ok"}; },
        []() { return Status{true, "spd_lidar adapter started"}; },
        []() { return Status{true, "spd_lidar adapter stopped"}; },
        [this](const std::vector<std::string>& args) { return querySpdLidar(args); },
        []() { return std::vector<std::string>{"list", "status", "send"}; });
  }
#endif
}

void Interface::startAutoQueryPolling() {
  stopAutoQueryPolling();
  auto_query_running_ = true;
  struct PollTask {
    std::string sensor;
    std::string snapshot_key;
    std::vector<std::string> args;
    std::chrono::steady_clock::duration period;
    std::chrono::steady_clock::time_point next_due;
  };
  std::vector<PollTask> tasks;
  const auto now = std::chrono::steady_clock::now();

  const auto add_task = [&](const std::string& sensor,
                            const std::string& snapshot_key,
                            double hz,
                            const std::vector<std::string>& args) {
    if (hz <= 0.0) return;
    if (drivers_.find(sensor) == drivers_.end()) return;
    const double safe_hz = std::min(std::max(hz, 0.1), 50.0);
    PollTask t;
    t.sensor = sensor;
    t.snapshot_key = snapshot_key;
    t.args = args;
    t.period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / safe_hz));
    t.next_due = now;
    tasks.push_back(t);
  };

#ifdef ASC_ENABLE_BATTERY
  add_task("battery", "battery", battery_defaults_.query_hz, {"basic"});
#endif
#ifdef ASC_ENABLE_SOLAR
  add_task("solar", "solar", solar_defaults_.query_hz, {"status"});
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
  add_task("hoist_hook", "hoist_hook", hoist_hook_defaults_.query_hz, {"all"});
#endif
#ifdef ASC_ENABLE_IO_RELAY
  add_task("io_relay", "io_relay", io_relay_defaults_.query_hz, {"read"});
#endif
#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
  add_task("multi_turn_encoder", "multi_turn_encoder", encoder_defaults_.query_hz, {"get"});
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
  if (spd_lidar_query_hz_ > 0.0) {
    for (size_t i = 0; i < spd_lidar_instances_.size(); ++i) {
      if (!spd_lidar_instances_[i].enable) continue;
      add_task("spd_lidar",
               "spd_lidar:" + spd_lidar_instances_[i].id,
               spd_lidar_query_hz_,
               {"send", spd_lidar_instances_[i].id, "single"});
    }
  }
#endif

  if (!tasks.empty()) {
    auto_query_threads_.emplace_back([this, tasks]() mutable {
      while (auto_query_running_) {
        const auto tick = std::chrono::steady_clock::now();
        bool ran = false;
        for (size_t i = 0; i < tasks.size(); ++i) {
          if (tick < tasks[i].next_due) continue;
          std::string captured;
          const Status s = queryWithCapturedOutput(tasks[i].sensor, tasks[i].args, &captured);
          {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            latest_query_output_[tasks[i].snapshot_key] = captured;
            latest_query_status_[tasks[i].snapshot_key] = s;
            latest_query_time_[tasks[i].snapshot_key] = std::chrono::system_clock::now();
          }
          tasks[i].next_due = std::chrono::steady_clock::now() + tasks[i].period;
          ran = true;
          break;  // Strictly serialize all queries.
        }
        if (!ran) std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    });
  }

  startSnapshotPrinter();
}

void Interface::stopAutoQueryPolling() {
  auto_query_running_ = false;
  for (size_t i = 0; i < auto_query_threads_.size(); ++i) {
    if (auto_query_threads_[i].joinable()) auto_query_threads_[i].join();
  }
  auto_query_threads_.clear();
}

Status Interface::queryWithCapturedOutput(const std::string& sensor,
                                          const std::vector<std::string>& args,
                                          std::string* captured_output) {
  std::ostringstream oss;
  std::streambuf* old_cout = nullptr;
  std::streambuf* old_cerr = nullptr;
  Status s;
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    old_cout = std::cout.rdbuf(oss.rdbuf());
    old_cerr = std::cerr.rdbuf(oss.rdbuf());
    s = query(sensor, args);
    std::cout.rdbuf(old_cout);
    std::cerr.rdbuf(old_cerr);
  }
  if (captured_output) *captured_output = oss.str();
  return s;
}

void Interface::startSnapshotPrinter() {
  stopSnapshotPrinter();
  snapshot_printer_running_ = true;
  snapshot_printer_thread_ = std::thread([this]() {
    const std::chrono::milliseconds period(1000);
    while (snapshot_printer_running_) {
      printSnapshotTick();
      std::this_thread::sleep_for(period);
    }
  });
}

void Interface::stopSnapshotPrinter() {
  snapshot_printer_running_ = false;
  if (snapshot_printer_thread_.joinable()) snapshot_printer_thread_.join();
}

void Interface::printSnapshotTick() {
  std::unordered_map<std::string, std::string> outputs;
  std::unordered_map<std::string, Status> statuses;
  std::unordered_map<std::string, std::chrono::system_clock::time_point> times;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    outputs = latest_query_output_;
    statuses = latest_query_status_;
    times = latest_query_time_;
  }
  if (outputs.empty()) return;

  std::vector<std::string> sensors;
  sensors.reserve(outputs.size());
  for (std::unordered_map<std::string, std::string>::const_iterator it = outputs.begin(); it != outputs.end(); ++it) {
    sensors.push_back(it->first);
  }
  std::sort(sensors.begin(), sensors.end());

  std::lock_guard<std::mutex> lock(output_mutex_);
  for (size_t i = 0; i < sensors.size(); ++i) {
    const std::string& sensor = sensors[i];
    const Status& s = statuses[sensor];
    const std::time_t t = std::chrono::system_clock::to_time_t(times[sensor]);
    const std::string ts = std::string(std::ctime(&t));
    std::cout << "[snapshot] " << sensor
              << " ok=" << (s.ok ? "true" : "false")
              << " time=" << (ts.empty() ? "" : ts.substr(11, 8)) << "\n";
    const std::string& out = outputs[sensor];
    if (!out.empty()) {
      std::cout << out;
      if (out.back() != '\n') std::cout << "\n";
    } else if (!s.ok) {
      std::cout << "  " << s.message << "\n";
    } else {
      std::cout << "  (no output)\n";
    }
  }
}

Status Interface::start() {
  if (!initialized_) return Status{false, "sdk not initialized"};
  if (started_) return Status{true, "all drivers already started"};

  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    std::cout << "[startup-summary] drivers and auto-query plan\n";
#ifdef ASC_ENABLE_BATTERY
    std::cout << "  - battery: enabled=" << (battery_ ? "true" : "false")
              << ", query_hz=" << battery_defaults_.query_hz << "\n";
#endif
#ifdef ASC_ENABLE_SOLAR
    std::cout << "  - solar: enabled=" << (solar_ ? "true" : "false")
              << ", query_hz=" << solar_defaults_.query_hz << "\n";
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
    std::cout << "  - hoist_hook: enabled=" << (hoist_hook_ ? "true" : "false")
              << ", query_hz=" << hoist_hook_defaults_.query_hz << "\n";
#endif
#ifdef ASC_ENABLE_IO_RELAY
    std::cout << "  - io_relay: enabled=" << (io_relay_ ? "true" : "false")
              << ", query_hz=" << io_relay_defaults_.query_hz << "\n";
#endif
#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
    std::cout << "  - multi_turn_encoder: enabled=" << (multi_turn_encoder_ ? "true" : "false")
              << ", query_hz=" << encoder_defaults_.query_hz << "\n";
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
    size_t lidar_enabled_count = 0;
    for (size_t i = 0; i < spd_lidar_instances_.size(); ++i) {
      if (spd_lidar_instances_[i].enable) ++lidar_enabled_count;
    }
    std::cout << "  - spd_lidar: enabled_instances=" << lidar_enabled_count
              << ", query_hz=" << spd_lidar_query_hz_ << "\n";
#endif
  }

  for (std::unordered_map<std::string, std::unique_ptr<DriverAdapter>>::iterator it = drivers_.begin();
       it != drivers_.end(); ++it) {
    const Status s = it->second->start();
    if (!s.ok) return Status{false, "start failed on " + it->first + ": " + s.message};
  }
  startAutoQueryPolling();
  started_ = true;
  return Status{true, "all drivers started"};
}

Status Interface::stop() {
  if (!initialized_) return Status{false, "sdk not initialized"};
  if (!started_) return Status{true, "all drivers already stopped"};
  stopAutoQueryPolling();
  stopSnapshotPrinter();
  for (std::unordered_map<std::string, std::unique_ptr<DriverAdapter>>::iterator it = drivers_.begin();
       it != drivers_.end(); ++it) {
    const Status s = it->second->stop();
    if (!s.ok) return Status{false, "stop failed on " + it->first + ": " + s.message};
  }
  started_ = false;
  return Status{true, "all drivers stopped"};
}

Status Interface::query(const std::string& sensor, const std::vector<std::string>& args) {
  if (!initialized_) return Status{false, "sdk not initialized"};
  std::unordered_map<std::string, std::unique_ptr<DriverAdapter>>::iterator it = drivers_.find(sensor);
  if (it == drivers_.end()) return Status{false, "sensor not enabled or unknown sensor"};
  return it->second->query(args);
}


Status Interface::init() {
  if (initialized_) {
    return Status{true, "ai_safety_controller sdk already initialized"};
  }

  const Status cfg_status = loadDefaultConfigIfPresent();
  if (!cfg_status.ok) return cfg_status;

#ifdef ASC_ENABLE_BATTERY
  if (battery_defaults_.enable) {
    battery_ = std::make_unique<battery::BatteryCore>(
        battery_defaults_.module_ip,
        static_cast<uint16_t>(battery_defaults_.module_port),
        static_cast<uint8_t>(battery_defaults_.module_slave_id),
        static_cast<uint8_t>(battery_defaults_.battery_slave_id));
  }
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
  if (hoist_hook_defaults_.enable) {
    hoist_hook_ = std::make_unique<hoist_hook::HoistHookCore>(
        hoist_hook_defaults_.module_ip,
        static_cast<uint16_t>(hoist_hook_defaults_.module_port),
        static_cast<uint8_t>(hoist_hook_defaults_.hook_slave_id),
        static_cast<uint8_t>(hoist_hook_defaults_.power_slave_id));
  }
#endif
#ifdef ASC_ENABLE_IO_RELAY
  if (io_relay_defaults_.enable) {
    io_relay_ = std::make_unique<io_relay::IoRelayCore>(
        io_relay_defaults_.module_ip,
        static_cast<uint16_t>(io_relay_defaults_.module_port),
        static_cast<uint8_t>(io_relay_defaults_.module_slave_id));
  }
#endif
#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
  if (encoder_defaults_.enable) {
    if (encoder_defaults_.transport == "tcp") {
      multi_turn_encoder_ = std::make_unique<multi_turn_encoder::MultiTurnEncoderCore>(
          encoder_defaults_.ip, encoder_defaults_.port, encoder_defaults_.slave);
    } else {
      multi_turn_encoder_ = std::make_unique<multi_turn_encoder::MultiTurnEncoderCore>(
          encoder_defaults_.device,
          encoder_defaults_.baud,
          encoder_defaults_.parity,
          encoder_defaults_.data_bit,
          encoder_defaults_.stop_bit,
          encoder_defaults_.slave);
    }
  }
#endif
#ifdef ASC_ENABLE_SOLAR
  if (solar_defaults_.enable) {
    solar_ = std::make_unique<solar::SolarCore>(
        solar_defaults_.module_ip,
        static_cast<uint16_t>(solar_defaults_.module_port),
        static_cast<uint8_t>(solar_defaults_.module_slave_id),
        static_cast<uint8_t>(solar_defaults_.solar_slave_id));
  }
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
  if (spd_lidar_instances_.empty()) {
    spd_lidar_instances_.push_back(SpdLidarInstanceDefaults{});
  }
  spd_lidar_instances_core_.clear();
  for (size_t i = 0; i < spd_lidar_instances_.size(); ++i) {
    const SpdLidarInstanceDefaults& cfg = spd_lidar_instances_[i];
    if (!cfg.enable) continue;
    std::unique_ptr<spd_lidar::SpdLidarCore> lidar = std::make_unique<spd_lidar::SpdLidarCore>();
    const std::string id = cfg.id;
    lidar->on_log.connect([id](const std::string& text) {
      std::cout << "[spd_lidar:" << id << "] " << text << "\n";
    });
    lidar->on_frame.connect([id](const spd_lidar::SpdLidarFrame& frame) {
      const double distance_m = static_cast<double>(frame.data) / 1000.0;
      std::cout << "[spd_lidar:" << id << "] "
                << "distance=" << frame.data << "mm (" << std::fixed << std::setprecision(3)
                << distance_m << "m)"
                << " status=0x" << std::hex << std::uppercase << static_cast<int>(frame.status)
                << std::dec
                << " checksum_ok=" << (frame.checksum_ok ? "true" : "false")
                << "\n";
    });
    spd_lidar::SpdLidarCore* lidar_raw = lidar.get();
    lidar->on_send.connect([cfg, id, lidar_raw](const std::vector<uint8_t>& req) {
      std::vector<uint8_t> resp;
      std::string err;
      if (!spdLidarExchangeTcp(cfg, req, &resp, &err)) {
        std::cout << "[spd_lidar:" << id << "] net error: " << err << "\n";
        return;
      }
      if (!resp.empty()) {
        lidar_raw->handleRecvBytes(resp.data(), resp.size());
      } else {
        std::cout << "[spd_lidar:" << id << "] net error: empty response\n";
      }
    });
    spd_lidar_instances_core_[id] = std::move(lidar);
  }
#endif

  buildDriverAdapters();
  for (std::unordered_map<std::string, std::unique_ptr<DriverAdapter>>::iterator it = drivers_.begin();
       it != drivers_.end(); ++it) {
    const Status s = it->second->init();
    if (!s.ok) return Status{false, "init failed on " + it->first + ": " + s.message};
  }

  initialized_ = true;
  Status status;
  status.ok = true;
  status.message = "ai_safety_controller sdk initialized";
  if (!loaded_config_path_.empty()) {
    status.message += " with config: " + loaded_config_path_;
  }
  return status;
}

std::vector<std::string> Interface::enabledSensors() const {
  std::vector<std::string> out;
  for (std::unordered_map<std::string, std::unique_ptr<DriverAdapter>>::const_iterator it = drivers_.begin();
       it != drivers_.end(); ++it) {
    if (factory_.isSupported(it->first)) out.push_back(it->first);
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<std::string> Interface::availableCommands(const std::string& sensor) const {
  std::unordered_map<std::string, std::unique_ptr<DriverAdapter>>::const_iterator it = drivers_.find(sensor);
  if (it == drivers_.end()) return {};
  return it->second->availableCommands();
}

Status Interface::dispatchCommand(const std::string& sensor, const std::vector<std::string>& args) {
  std::lock_guard<std::mutex> lock(output_mutex_);
  return query(sensor, args);
}

#ifdef ASC_ENABLE_BATTERY
Status Interface::queryBattery(const std::vector<std::string>& args) {
  if (!battery_) return Status{false, "battery not enabled"};
  if (args.empty()) return Status{false, "missing command"};
  const std::string& cmd = args[0];
  if (cmd == "map") battery_->printRegisterGroups();
  else if (cmd == "basic" || cmd == "cell" || cmd == "temp" || cmd == "mos" || cmd == "protect" ||
           cmd == "all") battery_->queryBatteryInfo(cmd);
  else if (cmd == "scan") {
    int start = 1;
    int end = 16;
    if (args.size() >= 2 && !parseInt(args[1], &start)) return Status{false, "invalid scan start"};
    if (args.size() >= 3 && !parseInt(args[2], &end)) return Status{false, "invalid scan end"};
    battery_->scanBatterySlaveIds(start, end);
  } else if (cmd == "addr") {
    if (args.size() < 2) return Status{false, "usage: battery addr <new_addr>"};
    int new_addr = 0;
    if (!parseInt(args[1], &new_addr)) return Status{false, "invalid addr value"};
    battery_->setBatteryAddr(new_addr);
  } else if (cmd == "get") {
    if (args.size() < 2) return Status{false, "usage: battery get <addr> [qty] [fc]"};
    int addr = 0, qty = 1, fc = -1;
    if (!parseInt(args[1], &addr)) return Status{false, "invalid addr"};
    if (args.size() >= 3 && !parseInt(args[2], &qty)) return Status{false, "invalid qty"};
    if (args.size() >= 4 && !parseInt(args[3], &fc)) return Status{false, "invalid fc"};
    battery_->genericRead(static_cast<uint16_t>(addr), static_cast<uint16_t>(qty), fc);
  } else if (cmd == "set") {
    if (args.size() < 3) return Status{false, "usage: battery set <addr> <value> [fc]"};
    int addr = 0, val = 0, fc = -1;
    if (!parseInt(args[1], &addr) || !parseInt(args[2], &val)) return Status{false, "invalid addr/value"};
    if (args.size() >= 4 && !parseInt(args[3], &fc)) return Status{false, "invalid fc"};
    battery_->genericWrite(static_cast<uint16_t>(addr), static_cast<uint16_t>(val), fc);
  } else {
    return Status{false, "unknown battery command"};
  }
  return Status{true, "ok"};
}
#endif

#ifdef ASC_ENABLE_SOLAR
Status Interface::querySolar(const std::vector<std::string>& args) {
  if (!solar_) return Status{false, "solar not enabled"};
  if (args.empty()) return Status{false, "missing command"};
  const std::string& cmd = args[0];
  if (cmd == "map") solar_->printRegisterGroups();
  else if (cmd == "basic" || cmd == "status" || cmd == "all") solar_->querySolarInfo(cmd);
  else if (cmd == "scan") {
    int start = 1;
    int end = 16;
    if (args.size() >= 2 && !parseInt(args[1], &start)) return Status{false, "invalid scan start"};
    if (args.size() >= 3 && !parseInt(args[2], &end)) return Status{false, "invalid scan end"};
    solar_->scanSolarSlaveIds(start, end);
  } else if (cmd == "get") {
    if (args.size() < 2) return Status{false, "usage: solar get <addr> [qty] [fc]"};
    int addr = 0, qty = 1, fc = -1;
    if (!parseInt(args[1], &addr)) return Status{false, "invalid addr"};
    if (args.size() >= 3 && !parseInt(args[2], &qty)) return Status{false, "invalid qty"};
    if (args.size() >= 4 && !parseInt(args[3], &fc)) return Status{false, "invalid fc"};
    solar_->genericRead(static_cast<uint16_t>(addr), static_cast<uint16_t>(qty), fc);
  } else if (cmd == "set") {
    if (args.size() < 3) return Status{false, "usage: solar set <addr> <value> [fc]"};
    int addr = 0, val = 0, fc = -1;
    if (!parseInt(args[1], &addr) || !parseInt(args[2], &val)) return Status{false, "invalid addr/value"};
    if (args.size() >= 4 && !parseInt(args[3], &fc)) return Status{false, "invalid fc"};
    solar_->genericWrite(static_cast<uint16_t>(addr), static_cast<uint16_t>(val), fc);
  } else {
    return Status{false, "unknown solar command"};
  }
  return Status{true, "ok"};
}
#endif

#ifdef ASC_ENABLE_HOIST_HOOK
Status Interface::queryHoistHook(const std::vector<std::string>& args) {
  if (!hoist_hook_) return Status{false, "hoist_hook not enabled"};
  if (args.empty()) return Status{false, "missing command"};
  const std::string& cmd = args[0];
  if (cmd == "map") hoist_hook_->printRegisterGroups();
  else if (cmd == "speaker" || cmd == "light" || cmd == "rfid" || cmd == "power" || cmd == "gps" ||
           cmd == "all") hoist_hook_->queryHookInfo(cmd);
  else if (cmd == "speaker_ctl") {
    if (args.size() < 2) return Status{false, "usage: hoist_hook speaker_ctl <off|7m|3m|both>"};
    hoist_hook_->controlSpeaker(args[1]);
  } else if (cmd == "light_ctl") {
    if (args.size() < 2) return Status{false, "usage: hoist_hook light_ctl <on|off>"};
    hoist_hook_->controlWarningLight(args[1]);
  } else if (cmd == "get") {
    if (args.size() < 2) return Status{false, "usage: hoist_hook get <addr> [qty] [fc]"};
    int addr = 0, qty = 1, fc = -1;
    if (!parseInt(args[1], &addr)) return Status{false, "invalid addr"};
    if (args.size() >= 3 && !parseInt(args[2], &qty)) return Status{false, "invalid qty"};
    if (args.size() >= 4 && !parseInt(args[3], &fc)) return Status{false, "invalid fc"};
    hoist_hook_->genericRead(static_cast<uint16_t>(addr), static_cast<uint16_t>(qty), fc);
  } else if (cmd == "set") {
    if (args.size() < 3) return Status{false, "usage: hoist_hook set <addr> <value> [fc]"};
    int addr = 0, val = 0, fc = -1;
    if (!parseInt(args[1], &addr) || !parseInt(args[2], &val)) return Status{false, "invalid addr/value"};
    if (args.size() >= 4 && !parseInt(args[3], &fc)) return Status{false, "invalid fc"};
    hoist_hook_->genericWrite(static_cast<uint16_t>(addr), static_cast<uint16_t>(val), fc);
  } else {
    return Status{false, "unknown hoist_hook command"};
  }
  return Status{true, "ok"};
}
#endif

#ifdef ASC_ENABLE_IO_RELAY
Status Interface::queryIoRelay(const std::vector<std::string>& args) {
  if (!io_relay_) return Status{false, "io_relay not enabled"};
  if (args.empty()) return Status{false, "missing command"};
  const std::string& cmd = args[0];
  if (cmd == "on" || cmd == "off") {
    if (args.size() < 2) return Status{false, "usage: io_relay on|off <channel>"};
    int ch = 0;
    if (!parseInt(args[1], &ch)) return Status{false, "invalid channel"};
    io_relay_->controlRelay(ch, cmd);
  } else if (cmd == "read") {
    int ch = 0;
    if (args.size() >= 2 && !parseInt(args[1], &ch)) return Status{false, "invalid channel"};
    io_relay_->readRelayStatus(ch);
  } else {
    return Status{false, "unknown io_relay command"};
  }
  return Status{true, "ok"};
}
#endif

#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
Status Interface::queryMultiTurnEncoder(const std::vector<std::string>& args) {
  if (!multi_turn_encoder_) return Status{false, "multi_turn_encoder not enabled"};
  if (args.empty()) return Status{false, "missing command"};
  const std::string& cmd = args[0];
  if (cmd == "connect") {
    const bool ok = multi_turn_encoder_->connect();
    return Status{ok, ok ? "encoder connected" : "encoder connect failed"};
  }
  if (cmd == "run") {
    multi_turn_encoder_->run();
    return Status{true, "encoder run started"};
  }
  if (cmd == "stop") {
    multi_turn_encoder_->stop();
    return Status{true, "encoder stopped"};
  }
  if (cmd == "status") {
    const bool connected = multi_turn_encoder_->isConnected();
    const bool running = multi_turn_encoder_->isRunning();
    std::cout << "[multi_turn_encoder] connected=" << (connected ? "true" : "false")
              << " running=" << (running ? "true" : "false") << "\n";
    return Status{true, "ok"};
  }
  if (cmd == "get") {
    const auto data = multi_turn_encoder_->getLatest();
    std::ostringstream ts_epoch_ss;
    ts_epoch_ss << std::fixed << std::setprecision(3) << data.timestamp;
    std::cout << "[multi_turn_encoder] valid=" << (data.valid ? "true" : "false")
              << " ts_epoch=" << ts_epoch_ss.str()
              << " ts_local=\"" << formatEpochSeconds(data.timestamp) << "\""
              << " turns_raw=" << data.turns_raw
              << " turns_filtered=" << data.turns_filtered
              << " velocity=" << data.velocity << "\n";
    return Status{true, "ok"};
  }
  return Status{false, "unknown multi_turn_encoder command"};
}
#endif

#ifdef ASC_ENABLE_SPD_LIDAR
Status Interface::querySpdLidar(const std::vector<std::string>& args) {
  if (args.empty()) return Status{false, "missing command"};
  const std::string& cmd = args[0];
  if (cmd == "list" || cmd == "status") {
    std::cout << "[spd_lidar] configured instances:\n";
    for (size_t i = 0; i < spd_lidar_instances_.size(); ++i) {
      const SpdLidarInstanceDefaults& cfg = spd_lidar_instances_[i];
      const bool initialized = findSpdLidarById(cfg.id) != nullptr;
      std::cout << "  - id=" << cfg.id
                << " enable=" << (cfg.enable ? "true" : "false")
                << " mode=" << cfg.mode
                << " local=" << cfg.local_ip << ":" << cfg.local_port
                << " device=" << cfg.device_ip << ":" << cfg.device_port
                << " initialized=" << (initialized ? "true" : "false");
      if (!cfg.role.empty()) std::cout << " role=" << cfg.role;
      std::cout << " priority=" << cfg.priority << "\n";
    }
    return Status{true, "ok"};
  }
  if (cmd == "send") {
    if (args.size() < 3) return Status{false, "usage: spd_lidar send <id|all> <single|hex bytes>"};
    const std::string& target = args[1];
    const std::string payload = joinArgs(args, 2);
    if (target == "all") {
      int sent = 0;
      for (std::unordered_map<std::string, std::unique_ptr<spd_lidar::SpdLidarCore>>::iterator it =
               spd_lidar_instances_core_.begin();
           it != spd_lidar_instances_core_.end(); ++it) {
        if (!it->second) continue;
        it->second->handleInputLine(payload);
        ++sent;
      }
      if (sent == 0) return Status{false, "no enabled spd_lidar instance"};
      return Status{true, "ok"};
    }
    spd_lidar::SpdLidarCore* one = findSpdLidarById(target);
    if (!one) return Status{false, "unknown spd_lidar id: " + target};
    one->handleInputLine(payload);
    return Status{true, "ok"};
  }
  return Status{false, "usage: spd_lidar <list|status|send>"};
}
#endif

#ifdef ASC_ENABLE_BATTERY
battery::BatteryCore* Interface::battery() {
  return battery_.get();
}
#endif

#ifdef ASC_ENABLE_HOIST_HOOK
hoist_hook::HoistHookCore* Interface::hoistHook() {
  return hoist_hook_.get();
}
#endif

#ifdef ASC_ENABLE_IO_RELAY
io_relay::IoRelayCore* Interface::ioRelay() {
  return io_relay_.get();
}
#endif

#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
multi_turn_encoder::MultiTurnEncoderCore* Interface::multiTurnEncoder() {
  return multi_turn_encoder_.get();
}
#endif

#ifdef ASC_ENABLE_SOLAR
solar::SolarCore* Interface::solar() {
  return solar_.get();
}
#endif

#ifdef ASC_ENABLE_SPD_LIDAR
spd_lidar::SpdLidarCore* Interface::spdLidar() {
  if (spd_lidar_instances_core_.empty()) return nullptr;
  return spd_lidar_instances_core_.begin()->second.get();
}

spd_lidar::SpdLidarCore* Interface::findSpdLidarById(const std::string& id) {
  std::unordered_map<std::string, std::unique_ptr<spd_lidar::SpdLidarCore>>::iterator it =
      spd_lidar_instances_core_.find(id);
  if (it == spd_lidar_instances_core_.end()) return nullptr;
  return it->second.get();
}

const spd_lidar::SpdLidarCore* Interface::findSpdLidarById(const std::string& id) const {
  std::unordered_map<std::string, std::unique_ptr<spd_lidar::SpdLidarCore>>::const_iterator it =
      spd_lidar_instances_core_.find(id);
  if (it == spd_lidar_instances_core_.end()) return nullptr;
  return it->second.get();
}
#endif

}  // namespace ai_safety_controller
