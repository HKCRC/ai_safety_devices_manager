#include "ai_safety_controller/interface.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <algorithm>
#include <cmath>
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
const char* kSpdLidarVerticalAngleToVerticalKey = "vertical_angle_to_vertical_deg";

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

std::string formatHexBytes(const std::vector<uint8_t>& data) {
  std::ostringstream oss;
  for (size_t i = 0; i < data.size(); ++i) {
    if (i > 0) oss << " ";
    oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        << static_cast<int>(data[i]);
  }
  return oss.str();
}

bool spdLidarExchangeOnConnectedFd(int fd,
                                   const std::vector<uint8_t>& request,
                                   std::vector<uint8_t>* response,
                                   std::string* error) {
  if (!response) return false;
  response->clear();

  timeval tv{};
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  const ssize_t sent = ::send(fd, request.data(), request.size(), 0);
  if (sent < 0 || static_cast<size_t>(sent) != request.size()) {
    if (error) *error = std::string("send failed: ") + std::strerror(errno);
    return false;
  }

  uint8_t buf[256];
  const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
  if (n <= 0) {
    if (error) *error = std::string("recv failed: ") + std::strerror(errno);
    return false;
  }
  response->assign(buf, buf + n);
  return true;
}

std::string sockaddrIp(const sockaddr_in& addr) {
  char buf[INET_ADDRSTRLEN] = {0};
  if (::inet_ntop(AF_INET, &addr.sin_addr, buf, sizeof(buf)) == nullptr) {
    return std::string{};
  }
  return std::string(buf);
}

int createSpdLidarListenSocket(const std::string& bind_ip,
                               int bind_port,
                               std::string* actual_bind_ip,
                               std::string* error) {
  const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd < 0) {
    if (error) *error = std::string("server socket failed: ") + std::strerror(errno);
    return -1;
  }
  int reuse_addr = 1;
  ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr));

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_port = htons(static_cast<uint16_t>(bind_port));
  if (::inet_pton(AF_INET, bind_ip.c_str(), &local_addr.sin_addr) != 1) {
    if (error) *error = "invalid local ip: " + bind_ip;
    ::close(listen_fd);
    return -1;
  }
  if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) != 0) {
    if (errno == EADDRNOTAVAIL) {
      sockaddr_in any_addr{};
      any_addr.sin_family = AF_INET;
      any_addr.sin_port = htons(static_cast<uint16_t>(bind_port));
      any_addr.sin_addr.s_addr = htonl(INADDR_ANY);
      if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&any_addr), sizeof(any_addr)) != 0) {
        if (error) {
          *error = "bind failed on " + bind_ip + ":" + std::to_string(bind_port) +
                   ", fallback 0.0.0.0 failed: " + std::strerror(errno);
        }
        ::close(listen_fd);
        return -1;
      }
      if (actual_bind_ip) *actual_bind_ip = "0.0.0.0";
      std::cout << "[spd_lidar] warning: local_ip " << bind_ip
                << " not present on host, fallback bind 0.0.0.0:" << bind_port << "\n";
    } else {
      if (error) *error = std::string("bind failed: ") + std::strerror(errno);
      ::close(listen_fd);
      return -1;
    }
  } else if (actual_bind_ip) {
    *actual_bind_ip = bind_ip;
  }

  if (::listen(listen_fd, 8) != 0) {
    if (error) *error = std::string("listen failed: ") + std::strerror(errno);
    ::close(listen_fd);
    return -1;
  }
  return listen_fd;
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
      snapshot_printer_running_(false)
#ifdef ASC_ENABLE_SPD_LIDAR
      ,
      trolley_lidar_has_valid_frame_(false)
#endif
{}

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

void Interface::setDeviceStatus(const DeviceStatus& data) {
  std::lock_guard<std::mutex> lock(device_status_mutex_);
  latest_device_status_ = data;
}

DeviceStatus Interface::getDeviceStatus() const {
  std::lock_guard<std::mutex> lock(device_status_mutex_);
  return latest_device_status_;
}

void Interface::setPowerCommand(PowerCommand cmd) {
  latest_power_command_.store(static_cast<std::uint8_t>(cmd), std::memory_order_relaxed);
}

PowerCommand Interface::getPowerCommand() const {
  const std::uint8_t raw = latest_power_command_.load(std::memory_order_relaxed);
  if (raw > static_cast<std::uint8_t>(PowerCommand::PowerOff)) {
    return PowerCommand::None;
  }
  return static_cast<PowerCommand>(raw);
}

CraneState Interface::getCraneState() const {
  std::lock_guard<std::mutex> lock(crane_state_mutex_);
  return latest_crane_state_;
}

std::unordered_map<std::string, std::uint16_t> Interface::getLatestLidarRawMm() const {
  std::lock_guard<std::mutex> lock(crane_state_mutex_);
  return latest_lidar_raw_mm_;
}

void Interface::setCraneState(const CraneState& data) {
  std::lock_guard<std::mutex> lock(crane_state_mutex_);
  latest_crane_state_ = data;
}

void Interface::updateCraneStateFromEncoder(double turns_value) {
  // Normalized value placeholder: current integration maps encoder turns 1:1 to meters.
  const double normalized_m = std::max(0.0, turns_value);
  CraneState crane = getCraneState();
  crane.hookToTrolleyDistanceM = static_cast<float>(normalized_m);
  setCraneState(crane);
}

void Interface::updateCraneStateFromLidarMeasurement(const std::string& id,
                                                     std::uint16_t raw_mm,
                                                     double projected_distance_m) {
  std::lock_guard<std::mutex> lock(crane_state_mutex_);
  latest_lidar_raw_mm_[id] = raw_mm;
  latest_lidar_projected_distance_m_[id] = std::max(0.0, projected_distance_m);
  if (latest_lidar_projected_distance_m_.empty()) return;

  double sum = 0.0;
  for (const std::pair<const std::string, double>& kv : latest_lidar_projected_distance_m_) {
    sum += kv.second;
  }
  const double avg = sum / static_cast<double>(latest_lidar_projected_distance_m_.size());
  latest_crane_state_.groundToTrolleyDistanceM = static_cast<float>(avg);
}

AlertMessage Interface::getAlertMessage() const {
  std::lock_guard<std::mutex> lock(alert_message_mutex_);
  return latest_alert_message_;
}

std::uint8_t Interface::getBatteryButtonSignals() const {
  std::lock_guard<std::mutex> lock(battery_button_signals_mutex_);
  return latest_battery_button_signals_;
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

bool Interface::extractIntArrayValue(const std::string& object_body,
                                     const std::string& key,
                                     std::vector<int>* out) {
  if (!out) return false;
  const std::regex re("\"" + key + "\"\\s*:\\s*\\[([^\\]]*)\\]");
  std::smatch m;
  if (!std::regex_search(object_body, m, re)) return false;
  out->clear();
  const std::string items = m[1].str();
  const std::regex item_re("-?[0-9]+");
  for (std::sregex_iterator it(items.begin(), items.end(), item_re), end; it != end; ++it) {
    int value = 0;
    if (parseInt(it->str(), &value)) out->push_back(value);
  }
  return true;
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
  bool charge_time_debug = false;
  if (extractBoolValue(body, "charge_time_debug", &charge_time_debug)) {
    battery_defaults_.charge_time_debug = charge_time_debug;
  }
  double query_hz = 0.0;
  if (extractDoubleValue(body, "query_hz", &query_hz)) battery_defaults_.query_hz = query_hz;
  const std::string retry_body = extractObjectBody(body, "retry");
  if (!retry_body.empty()) {
    int max_retries = 0;
    if (extractIntValue(retry_body, "max_retries", &max_retries))
      battery_defaults_.retry_policy.max_retries = std::max(0, max_retries);
    int base_backoff_ms = 0;
    if (extractIntValue(retry_body, "base_backoff_ms", &base_backoff_ms))
      battery_defaults_.retry_policy.base_backoff_ms = std::max(0, base_backoff_ms);
    int max_backoff_ms = 0;
    if (extractIntValue(retry_body, "max_backoff_ms", &max_backoff_ms))
      battery_defaults_.retry_policy.max_backoff_ms = std::max(0, max_backoff_ms);
    int jitter_ms = 0;
    if (extractIntValue(retry_body, "jitter_ms", &jitter_ms))
      battery_defaults_.retry_policy.jitter_ms = std::max(0, jitter_ms);
    bool retry_log_enabled = true;
    if (extractBoolValue(retry_body, "log_enabled", &retry_log_enabled))
      battery_defaults_.retry_policy.log_enabled = retry_log_enabled;
  }
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
  double sample_timeout_sec = 0.0;
  if (extractDoubleValue(body, "sample_timeout_sec", &sample_timeout_sec)) {
    solar_defaults_.sample_timeout_sec = std::min(std::max(sample_timeout_sec, 0.1), 10.0);
  }
  int stale_timeout_ms = 0;
  if (extractIntValue(body, "stale_timeout_ms", &stale_timeout_ms)) {
    solar_defaults_.stale_timeout_ms = std::max(100, stale_timeout_ms);
  }
  double query_hz = 0.0;
  if (extractDoubleValue(body, "query_hz", &query_hz)) solar_defaults_.query_hz = query_hz;
  const std::string retry_body = extractObjectBody(body, "retry");
  if (!retry_body.empty()) {
    int max_retries = 0;
    if (extractIntValue(retry_body, "max_retries", &max_retries))
      solar_defaults_.retry_policy.max_retries = std::max(0, max_retries);
    int base_backoff_ms = 0;
    if (extractIntValue(retry_body, "base_backoff_ms", &base_backoff_ms))
      solar_defaults_.retry_policy.base_backoff_ms = std::max(0, base_backoff_ms);
    int max_backoff_ms = 0;
    if (extractIntValue(retry_body, "max_backoff_ms", &max_backoff_ms))
      solar_defaults_.retry_policy.max_backoff_ms = std::max(0, max_backoff_ms);
    int jitter_ms = 0;
    if (extractIntValue(retry_body, "jitter_ms", &jitter_ms))
      solar_defaults_.retry_policy.jitter_ms = std::max(0, jitter_ms);
    bool retry_log_enabled = true;
    if (extractBoolValue(retry_body, "log_enabled", &retry_log_enabled))
      solar_defaults_.retry_policy.log_enabled = retry_log_enabled;
  }
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
  std::vector<int> battery_button_relay_channels;
  if (extractIntArrayValue(body, "battery_button_relay_channels", &battery_button_relay_channels)) {
    std::vector<int> channels_filtered;
    channels_filtered.reserve(battery_button_relay_channels.size());
    for (size_t i = 0; i < battery_button_relay_channels.size(); ++i) {
      const int ch = battery_button_relay_channels[i];
      if (ch >= 1 && ch <= 16) channels_filtered.push_back(ch);
    }
    std::sort(channels_filtered.begin(), channels_filtered.end());
    channels_filtered.erase(std::unique(channels_filtered.begin(), channels_filtered.end()),
                           channels_filtered.end());
    io_relay_defaults_.battery_button_relay_channels = channels_filtered;
  }
  double query_hz = 0.0;
  if (extractDoubleValue(body, "query_hz", &query_hz)) io_relay_defaults_.query_hz = query_hz;
  const std::string retry_body = extractObjectBody(body, "retry");
  if (!retry_body.empty()) {
    int max_retries = 0;
    if (extractIntValue(retry_body, "max_retries", &max_retries))
      io_relay_defaults_.retry_policy.max_retries = std::max(0, max_retries);
    int base_backoff_ms = 0;
    if (extractIntValue(retry_body, "base_backoff_ms", &base_backoff_ms))
      io_relay_defaults_.retry_policy.base_backoff_ms = std::max(0, base_backoff_ms);
    int max_backoff_ms = 0;
    if (extractIntValue(retry_body, "max_backoff_ms", &max_backoff_ms))
      io_relay_defaults_.retry_policy.max_backoff_ms = std::max(0, max_backoff_ms);
    int jitter_ms = 0;
    if (extractIntValue(retry_body, "jitter_ms", &jitter_ms))
      io_relay_defaults_.retry_policy.jitter_ms = std::max(0, jitter_ms);
    bool retry_log_enabled = true;
    if (extractBoolValue(retry_body, "log_enabled", &retry_log_enabled))
      io_relay_defaults_.retry_policy.log_enabled = retry_log_enabled;
  }
}

void Interface::applyHoistHookDefaultsFromJson(const std::string& json_text) {
  const std::string runtime_body = extractObjectBody(json_text, "runtime");
  if (runtime_body.empty()) return;
  const std::string body = extractObjectBody(runtime_body, "hoist_hook");
  if (body.empty()) return;

  bool enable = true;
  if (extractBoolValue(body, "enable", &enable)) hoist_hook_defaults_.enable = enable;
  std::string transport;
  if (extractStringValue(body, "transport", &transport)) hoist_hook_defaults_.transport = transport;
  std::string module_ip;
  if (extractStringValue(body, "module_ip", &module_ip)) hoist_hook_defaults_.module_ip = module_ip;
  int module_port = 0;
  if (extractIntValue(body, "module_port", &module_port)) hoist_hook_defaults_.module_port = module_port;
  std::string device;
  if (extractStringValue(body, "device", &device)) hoist_hook_defaults_.device = device;
  int baud = 0;
  if (extractIntValue(body, "baud", &baud)) hoist_hook_defaults_.baud = baud;
  std::string parity_str;
  if (extractStringValue(body, "parity", &parity_str) && !parity_str.empty())
    hoist_hook_defaults_.parity = parity_str[0];
  int data_bit = 0;
  if (extractIntValue(body, "data_bit", &data_bit)) hoist_hook_defaults_.data_bit = data_bit;
  int stop_bit = 0;
  if (extractIntValue(body, "stop_bit", &stop_bit)) hoist_hook_defaults_.stop_bit = stop_bit;
  int hook_slave_id = 0;
  if (extractIntValue(body, "hook_slave_id", &hook_slave_id)) hoist_hook_defaults_.hook_slave_id = hook_slave_id;
  int power_slave_id = 0;
  if (extractIntValue(body, "power_slave_id", &power_slave_id)) hoist_hook_defaults_.power_slave_id = power_slave_id;
  bool heartbeat_enable = false;
  if (extractBoolValue(body, "heartbeat_enable", &heartbeat_enable)) {
    hoist_hook_defaults_.heartbeat_enable = heartbeat_enable;
  }
  int heartbeat_period_ms = 0;
  if (extractIntValue(body, "heartbeat_period_ms", &heartbeat_period_ms) && heartbeat_period_ms > 0) {
    hoist_hook_defaults_.heartbeat_period_ms = heartbeat_period_ms;
  }
  int heartbeat_start_value = 0;
  if (extractIntValue(body, "heartbeat_start_value", &heartbeat_start_value) &&
      heartbeat_start_value >= 0 && heartbeat_start_value <= 65535) {
    hoist_hook_defaults_.heartbeat_start_value = heartbeat_start_value;
  }
  bool heartbeat_log_enabled = false;
  if (extractBoolValue(body, "heartbeat_log_enabled", &heartbeat_log_enabled)) {
    hoist_hook_defaults_.heartbeat_log_enabled = heartbeat_log_enabled;
  }
  bool time_sync_enable = false;
  if (extractBoolValue(body, "time_sync_enable", &time_sync_enable)) {
    hoist_hook_defaults_.time_sync_enable = time_sync_enable;
  }
  int time_sync_period_ms = 0;
  if (extractIntValue(body, "time_sync_period_ms", &time_sync_period_ms) && time_sync_period_ms > 0) {
    hoist_hook_defaults_.time_sync_period_ms = time_sync_period_ms;
  }
  bool time_sync_log_enabled = false;
  if (extractBoolValue(body, "time_sync_log_enabled", &time_sync_log_enabled)) {
    hoist_hook_defaults_.time_sync_log_enabled = time_sync_log_enabled;
  }
  int speaker_volume = -1;
  if (extractIntValue(body, "speaker_volume", &speaker_volume)) {
    if (speaker_volume >= 0 && speaker_volume <= 30) {
      hoist_hook_defaults_.speaker_volume = speaker_volume;
    }
  }
  double query_hz = 0.0;
  if (extractDoubleValue(body, "query_hz", &query_hz)) hoist_hook_defaults_.query_hz = query_hz;
  int both_speaker_play_window_ms = 0;
  if (extractIntValue(body, "both_speaker_play_window_ms", &both_speaker_play_window_ms) &&
      both_speaker_play_window_ms > 0) {
    hoist_hook_defaults_.both_speaker_play_window_ms = both_speaker_play_window_ms;
  }
  int both_speaker_switch_gap_ms = 0;
  if (extractIntValue(body, "both_speaker_switch_gap_ms", &both_speaker_switch_gap_ms) &&
      both_speaker_switch_gap_ms > 0) {
    hoist_hook_defaults_.both_speaker_switch_gap_ms = both_speaker_switch_gap_ms;
  }
  // Backward compatibility for old single-interval config.
  int both_speaker_switch_interval_ms = 0;
  if (extractIntValue(body, "both_speaker_switch_interval_ms", &both_speaker_switch_interval_ms) &&
      both_speaker_switch_interval_ms > 0) {
    hoist_hook_defaults_.both_speaker_play_window_ms = both_speaker_switch_interval_ms;
  }
  const std::string retry_body = extractObjectBody(body, "retry");
  if (!retry_body.empty()) {
    int max_retries = 0;
    if (extractIntValue(retry_body, "max_retries", &max_retries))
      hoist_hook_defaults_.retry_policy.max_retries = std::max(0, max_retries);
    int base_backoff_ms = 0;
    if (extractIntValue(retry_body, "base_backoff_ms", &base_backoff_ms))
      hoist_hook_defaults_.retry_policy.base_backoff_ms = std::max(0, base_backoff_ms);
    int max_backoff_ms = 0;
    if (extractIntValue(retry_body, "max_backoff_ms", &max_backoff_ms))
      hoist_hook_defaults_.retry_policy.max_backoff_ms = std::max(0, max_backoff_ms);
    int jitter_ms = 0;
    if (extractIntValue(retry_body, "jitter_ms", &jitter_ms))
      hoist_hook_defaults_.retry_policy.jitter_ms = std::max(0, jitter_ms);
    bool retry_log_enabled = true;
    if (extractBoolValue(retry_body, "log_enabled", &retry_log_enabled))
      hoist_hook_defaults_.retry_policy.log_enabled = retry_log_enabled;
  }
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
  bool linear_enable = false;
  if (extractBoolValue(body, "linear_enable", &linear_enable)) encoder_defaults_.linear_enable = linear_enable;
  double linear_k = 0.0;
  if (extractDoubleValue(body, "linear_k", &linear_k)) encoder_defaults_.linear_k = linear_k;
  double linear_b = 0.0;
  if (extractDoubleValue(body, "linear_b", &linear_b)) encoder_defaults_.linear_b = linear_b;
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
      double vertical_angle_to_vertical_deg = 0.0;
      if (extractDoubleValue(object_bodies[i],
                             kSpdLidarVerticalAngleToVerticalKey,
                             &vertical_angle_to_vertical_deg)) {
        one.vertical_angle_to_vertical_deg = vertical_angle_to_vertical_deg;
      }
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
  double vertical_angle_to_vertical_deg = 0.0;
  if (extractDoubleValue(body, kSpdLidarVerticalAngleToVerticalKey, &vertical_angle_to_vertical_deg)) {
    one.vertical_angle_to_vertical_deg = vertical_angle_to_vertical_deg;
  }
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
        [this]() {
          hoist_hook_->startHeartbeat();
          hoist_hook_->startTimeSync();
          return Status{true, "hoist_hook started"};
        },
        [this]() {
          hoist_hook_->stopHeartbeat();
          hoist_hook_->stopTimeSync();
          return Status{true, "hoist_hook stopped"};
        },
        [this](const std::vector<std::string>& args) { return queryHoistHook(args); },
        []() {
          return std::vector<std::string>{"map",
                                          "speaker",
                                          "light",
                                          "rfid",
                                          "power",
                                          "gps",
                                          "all",
                                          "heartbeat",
                                          "mode",
                                          "speaker_ctl",
                                          "light_ctl",
                                          "get",
                                          "set"};
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
        [this]() { return startSpdLidarServers(); },
        [this]() { return stopSpdLidarServers(); },
        [this](const std::vector<std::string>& args) { return querySpdLidar(args); },
        []() { return std::vector<std::string>{"list", "status", "send"}; });
  }
#endif

  // Logical device-level adapter for querying aggregated DeviceStatus.
  drivers_["device"] = std::make_unique<FunctionDriverAdapter>(
      "device",
      []() { return Status{true, "ok"}; },
      []() { return Status{true, "device adapter started"}; },
      []() { return Status{true, "device adapter stopped"}; },
      [this](const std::vector<std::string>& args) -> Status {
        if (!args.empty() && args[0] != "status") {
          return Status{false, "unknown device command"};
        }

#ifdef ASC_ENABLE_SOLAR
        updateSolarChargeStateFromDriver();
#endif
        updateTrolleyStateFromDrivers();
        updateHookStateFromDriver();

        const DeviceStatus d = getDeviceStatus();

        std::cout << "DeviceStatus:\n";
        // solar
        std::cout << "  solarCharge=" << static_cast<int>(d.solarCharge) << "\n";
        // trolley
        std::cout << "  trolleyState=" << static_cast<int>(d.trolleyState) << "\n";
        std::cout << "  trolleyBattery.percent=" << static_cast<int>(d.trolleyBattery.percent) << "%\n";
        std::cout << "  trolleyBattery.remainingMin=" << d.trolleyBattery.remainingMin << " min\n";
        std::cout << "  trolleyBattery.isCharging=" << (d.trolleyBattery.isCharging ? "true" : "false")
                  << "\n";
        std::cout << "  trolleyBattery.chargingTimeMin=" << d.trolleyBattery.chargingTimeMin << " min\n";
        std::cout << "  trolleyBattery.voltageV=" << d.trolleyBattery.voltageV << " V\n";
        std::cout << "  trolleyBattery.currentA=" << d.trolleyBattery.currentA << " A\n";

        // hook
        std::cout << "  hookState=" << static_cast<int>(d.hookState) << "\n";
        std::cout << "  hookBattery.percent=" << static_cast<int>(d.hookBattery.percent) << "%\n";
        std::cout << "  hookBattery.remainingMin=" << d.hookBattery.remainingMin << " min\n";
        std::cout << "  hookBattery.isCharging=" << (d.hookBattery.isCharging ? "true" : "false")
                  << "\n";
        std::cout << "  hookBattery.chargingTimeMin=" << d.hookBattery.chargingTimeMin << " min\n";
        std::cout << "  hookBattery.voltageV=" << d.hookBattery.voltageV << " V\n";
        std::cout << "  hookBattery.currentA=" << d.hookBattery.currentA << " A\n";

        return Status{true, "ok"};
      },
      []() { return std::vector<std::string>{"status"}; });
}

void Interface::startAutoQueryPolling() {
  stopAutoQueryPolling();
  auto_query_running_ = true;
  struct PollTask {
    std::string sensor;
    std::chrono::steady_clock::duration period;
    std::chrono::steady_clock::time_point next_due;
  };
  std::vector<PollTask> tasks;
  const auto now = std::chrono::steady_clock::now();

  const auto add_task = [&](const std::string& sensor,
                            double hz) {
    if (hz <= 0.0) return;
    if (drivers_.find(sensor) == drivers_.end()) return;
    const double safe_hz = std::min(std::max(hz, 0.1), 50.0);
    PollTask t;
    t.sensor = sensor;
    t.period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / safe_hz));
    t.next_due = now;
    tasks.push_back(t);
  };

#ifdef ASC_ENABLE_BATTERY
  add_task("battery", battery_defaults_.query_hz);
#endif
#ifdef ASC_ENABLE_SOLAR
  add_task("solar", solar_defaults_.query_hz);
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
  add_task("hoist_hook", hoist_hook_defaults_.query_hz);
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
  add_task("spd_lidar", spd_lidar_query_hz_);
#endif

  if (!tasks.empty()) {
    auto_query_threads_.emplace_back([this, tasks]() mutable {
      while (auto_query_running_) {
        const auto tick = std::chrono::steady_clock::now();
        bool ran = false;
        for (size_t i = 0; i < tasks.size(); ++i) {
          if (tick < tasks[i].next_due) continue;
          // 静默轮询，仅更新 DeviceStatus，不在终端打印
          if (tasks[i].sensor == "battery") {
            updateTrolleyStateFromDrivers();
          } else if (tasks[i].sensor == "solar") {
            updateSolarChargeStateFromDriver();
          } else if (tasks[i].sensor == "hoist_hook") {
            updateHookStateFromDriver();
          } else if (tasks[i].sensor == "spd_lidar") {
            // 单点激光雷达：发送 single 查询触发测距，响应经 on_frame 更新 groundToTrolley
            query("spd_lidar", std::vector<std::string>{"send", "all", "single"});
          }
          tasks[i].next_due = std::chrono::steady_clock::now() + tasks[i].period;
          ran = true;
          break;  // Strictly serialize all queries.
        }
        if (!ran) std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    });
  }
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

void Interface::updateTrolleyStateFromDrivers() {
  DeviceStatus data = getDeviceStatus();

#ifdef ASC_ENABLE_BATTERY
  // 未启用电池功能或电池驱动未实例化：保持 Unknown（表示此功能不适用/未配置）
  if (!battery_defaults_.enable || !battery_) {
    data.trolleyState = DeviceStatus::EquipmentState::Unknown;
    setDeviceStatus(data);
    return;
  }

  const bool battery_ok = battery_->isOnline();
  if (!battery_ok) {
    data.trolleyState = DeviceStatus::EquipmentState::Offline;
    setDeviceStatus(data);
    return;
  }

  {
    battery::BatteryCore::Summary summary;
    if (battery_->readSummary(&summary)) {
      DeviceStatus::BatteryInfo info;
      float soc = summary.soc_percent;
      if (soc < 0.0f) soc = 0.0f;
      if (soc > 100.0f) soc = 100.0f;
      info.percent = static_cast<std::uint8_t>(soc + 0.5f);
      info.voltageV = summary.voltage_v;
      info.currentA = summary.current_a;
      info.remainingMin = summary.remaining_discharge_min;
      info.chargingTimeMin = summary.remaining_charge_min;

      bool is_charging = false;
      const float current_a = summary.current_a;
      if (summary.has_charge_mos) {
        if (summary.charge_mos != 0) {
          is_charging = (current_a > 0.05f) ||
                        (current_a > -0.05f && current_a < 0.05f);
        }
      } else {
        if (current_a > 0.05f) {
          is_charging = true;
        }
      }
      info.isCharging = is_charging;
      data.trolleyBattery = info;
    }
  }
#endif

  const PowerCommand power_cmd = getPowerCommand();
  if (power_cmd == PowerCommand::PowerOff || power_cmd == PowerCommand::None) {
    data.trolleyState = DeviceStatus::EquipmentState::Standby;
    setDeviceStatus(data);
    return;
  }

  bool encoder_ok = false;
#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
  if (multi_turn_encoder_) {
    const multi_turn_encoder::MultiTurnEncoderCore::LatestData latest =
        multi_turn_encoder_->getLatest();
    encoder_ok = latest.valid && latest.connected;
    if (encoder_ok) {
      updateCraneStateFromEncoder(latest.turns_calibrated);
    }
  }
#endif

  bool lidar_ok = false;
#ifdef ASC_ENABLE_SPD_LIDAR
  lidar_ok = trolley_lidar_has_valid_frame_.load(std::memory_order_relaxed);
#endif

  // 到这里说明小车电池在线（online），根据传感器数据区分 Active / Standby
  if (encoder_ok || lidar_ok) {
    data.trolleyState = DeviceStatus::EquipmentState::Active;
  } else {
    data.trolleyState = DeviceStatus::EquipmentState::Standby;
  }

  setDeviceStatus(data);
}

void Interface::updateHookStateFromDriver() {
  DeviceStatus data = getDeviceStatus();

#ifdef ASC_ENABLE_HOIST_HOOK
  if (!hoist_hook_ || !hoist_hook_defaults_.enable) {
    data.hookState = DeviceStatus::EquipmentState::Unknown;
    setDeviceStatus(data);
    return;
  }

  hoist_hook::HoistHookCore::PowerSummary summary;
  if (!hoist_hook_->readPowerSummary(&summary)) {
    data.hookState = DeviceStatus::EquipmentState::Offline;
    setDeviceStatus(data);
    return;
  }

  {
    DeviceStatus::BatteryInfo info;
    float soc = summary.battery_percent;
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 100.0f) soc = 100.0f;
    info.percent = static_cast<std::uint8_t>(soc + 0.5f);
    info.voltageV = summary.voltage_v;
    info.currentA = summary.current_a;
    info.remainingMin = summary.remaining_discharge_min;
    info.chargingTimeMin = summary.remaining_charge_min;
    info.isCharging = summary.is_charging;
    data.hookBattery = info;
  }

  hook_battery_last_update_ = std::chrono::system_clock::now();
  data.hookState = DeviceStatus::EquipmentState::Active;
  setDeviceStatus(data);
#else
  (void)data;
#endif
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
              << ", query_hz=" << encoder_defaults_.query_hz
              << ", linear_enable=" << (encoder_defaults_.linear_enable ? "true" : "false")
              << ", linear_k=" << encoder_defaults_.linear_k
              << ", linear_b=" << encoder_defaults_.linear_b << "\n";
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
        static_cast<uint8_t>(battery_defaults_.battery_slave_id),
        battery::BatteryCore::RetryPolicy{
            battery_defaults_.retry_policy.max_retries,
            battery_defaults_.retry_policy.base_backoff_ms,
            battery_defaults_.retry_policy.max_backoff_ms,
            battery_defaults_.retry_policy.jitter_ms,
            battery_defaults_.retry_policy.log_enabled});
    battery_->setChargeTimeDebugEnabled(battery_defaults_.charge_time_debug);
  }
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
  if (hoist_hook_defaults_.enable) {
    if (hoist_hook_defaults_.transport == "rtu") {
      hoist_hook_ = std::make_unique<hoist_hook::HoistHookCore>(
          hoist_hook_defaults_.device,
          hoist_hook_defaults_.baud,
          hoist_hook_defaults_.parity,
          hoist_hook_defaults_.data_bit,
          hoist_hook_defaults_.stop_bit,
          static_cast<uint8_t>(hoist_hook_defaults_.hook_slave_id),
          static_cast<uint8_t>(hoist_hook_defaults_.power_slave_id),
          hoist_hook::HoistHookCore::RetryPolicy{
              hoist_hook_defaults_.retry_policy.max_retries,
              hoist_hook_defaults_.retry_policy.base_backoff_ms,
              hoist_hook_defaults_.retry_policy.max_backoff_ms,
              hoist_hook_defaults_.retry_policy.jitter_ms,
              hoist_hook_defaults_.retry_policy.log_enabled});
    } else {
      hoist_hook_ = std::make_unique<hoist_hook::HoistHookCore>(
          hoist_hook_defaults_.module_ip,
          static_cast<uint16_t>(hoist_hook_defaults_.module_port),
          static_cast<uint8_t>(hoist_hook_defaults_.hook_slave_id),
          static_cast<uint8_t>(hoist_hook_defaults_.power_slave_id),
          hoist_hook::HoistHookCore::RetryPolicy{
              hoist_hook_defaults_.retry_policy.max_retries,
              hoist_hook_defaults_.retry_policy.base_backoff_ms,
              hoist_hook_defaults_.retry_policy.max_backoff_ms,
              hoist_hook_defaults_.retry_policy.jitter_ms,
              hoist_hook_defaults_.retry_policy.log_enabled});
    }
    if (hoist_hook_ && hoist_hook_defaults_.speaker_volume >= 0 &&
        hoist_hook_defaults_.speaker_volume <= 30) {
      // Apply startup speaker volume from config on module instantiation.
      hoist_hook_->genericWrite(static_cast<uint16_t>(0x0067),
                                static_cast<uint16_t>(hoist_hook_defaults_.speaker_volume),
                                0x06,
                                true);
    }
    if (hoist_hook_) {
      hoist_hook_->configureHeartbeat(
          hoist_hook_defaults_.heartbeat_enable,
          hoist_hook_defaults_.heartbeat_period_ms,
          static_cast<std::uint16_t>(hoist_hook_defaults_.heartbeat_start_value),
          hoist_hook_defaults_.heartbeat_log_enabled);
      hoist_hook_->configureTimeSync(
          hoist_hook_defaults_.time_sync_enable,
          hoist_hook_defaults_.time_sync_period_ms,
          hoist_hook_defaults_.time_sync_log_enabled);
    }
  }
#endif
#ifdef ASC_ENABLE_IO_RELAY
  if (io_relay_defaults_.enable) {
    io_relay_ = std::make_unique<io_relay::IoRelayCore>(
        io_relay_defaults_.module_ip,
        static_cast<uint16_t>(io_relay_defaults_.module_port),
        static_cast<uint8_t>(io_relay_defaults_.module_slave_id),
        io_relay::IoRelayCore::RetryPolicy{
            io_relay_defaults_.retry_policy.max_retries,
            io_relay_defaults_.retry_policy.base_backoff_ms,
            io_relay_defaults_.retry_policy.max_backoff_ms,
            io_relay_defaults_.retry_policy.jitter_ms,
            io_relay_defaults_.retry_policy.log_enabled});
    if (io_relay_defaults_.battery_button_relay_channels.empty()) {
      std::cout << "[io_relay] battery_button_relay_channels is empty, "
                   "battery button control is disabled\n";
    } else {
      std::cout << "[io_relay] battery_button_relay_channels:";
      for (size_t i = 0; i < io_relay_defaults_.battery_button_relay_channels.size(); ++i) {
        std::cout << (i == 0 ? " " : ", ")
                  << io_relay_defaults_.battery_button_relay_channels[i];
      }
      std::cout << "\n";
    }
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
    if (multi_turn_encoder_) {
      multi_turn_encoder_->setLinearTransform(
          encoder_defaults_.linear_enable, encoder_defaults_.linear_k, encoder_defaults_.linear_b);
    }
  }
#endif
#ifdef ASC_ENABLE_SOLAR
  if (solar_defaults_.enable) {
    solar_ = std::make_unique<solar::SolarCore>(
        solar_defaults_.module_ip,
        static_cast<uint16_t>(solar_defaults_.module_port),
        static_cast<uint8_t>(solar_defaults_.module_slave_id),
        static_cast<uint8_t>(solar_defaults_.solar_slave_id),
        solar::SolarCore::RetryPolicy{
            solar_defaults_.retry_policy.max_retries,
            solar_defaults_.retry_policy.base_backoff_ms,
            solar_defaults_.retry_policy.max_backoff_ms,
            solar_defaults_.retry_policy.jitter_ms,
            solar_defaults_.retry_policy.log_enabled});
    solar_->setChargeSampleTimeoutSec(solar_defaults_.sample_timeout_sec);
    solar_charge_last_ok_ms_.store(0, std::memory_order_relaxed);
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
    std::cout << "[spd_lidar:" << cfg.id << "] instantiate with mode=" << cfg.mode
              << " local=" << cfg.local_ip << ":" << cfg.local_port
              << " device=" << cfg.device_ip << ":" << cfg.device_port;
    if (!cfg.role.empty()) std::cout << " role=" << cfg.role;
    std::cout << " vertical_angle_to_vertical_deg=" << cfg.vertical_angle_to_vertical_deg << "\n";
    std::unique_ptr<spd_lidar::SpdLidarCore> lidar = std::make_unique<spd_lidar::SpdLidarCore>();
    const std::string id = cfg.id;
    lidar->on_log.connect([](const std::string&) {});  // 静默，避免轮询刷屏
    lidar->on_frame.connect([this, id, cfg](const spd_lidar::SpdLidarFrame& frame) {
      const double distance_m = static_cast<double>(frame.data) / 1000.0;
      const bool lidar_value_valid = (frame.data != 65535u);
      if (frame.valid_header && frame.checksum_ok && lidar_value_valid) {
        trolley_lidar_has_valid_frame_.store(true, std::memory_order_relaxed);
        constexpr double kPi = 3.14159265358979323846;
        const double angle_rad = cfg.vertical_angle_to_vertical_deg * kPi / 180.0;
        const double projected_m = distance_m * std::cos(angle_rad);
        updateCraneStateFromLidarMeasurement(id, frame.data, projected_m);
      }
    });
    spd_lidar::SpdLidarCore* lidar_raw = lidar.get();
    lidar->on_send.connect([this, cfg, id, lidar_raw](const std::vector<uint8_t>& req) {
      std::vector<uint8_t> resp;
      std::string err;
      if (!spdLidarExchange(cfg, req, &resp, &err)) {
        if (cfg.mode == "server" && (err == "accept timeout" || err == "client not connected")) {
          bool should_log = false;
          {
            std::lock_guard<std::mutex> lock(spd_lidar_log_mutex_);
            if (spd_lidar_wait_logged_.insert(id).second) {
              should_log = true;
            }
          }
          if (should_log) {
            std::cout << "[spd_lidar:" << id << "] waiting client connection at "
                      << cfg.local_ip << ":" << cfg.local_port << "\n";
          }
        } else {
          std::cout << "[spd_lidar:" << id << "] net error: " << err << "\n";
        }
        return;
      }
      {
        std::lock_guard<std::mutex> lock(spd_lidar_log_mutex_);
        spd_lidar_wait_logged_.erase(id);
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

  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    std::cout << "[init-summary] configured and instantiated drivers\n";
#ifdef ASC_ENABLE_BATTERY
    std::cout << "  - battery: configured=" << (battery_defaults_.enable ? "true" : "false")
              << ", instantiated=" << (battery_ ? "true" : "false") << "\n";
#endif
#ifdef ASC_ENABLE_SOLAR
    std::cout << "  - solar: configured=" << (solar_defaults_.enable ? "true" : "false")
              << ", instantiated=" << (solar_ ? "true" : "false") << "\n";
#endif
#ifdef ASC_ENABLE_HOIST_HOOK
    std::cout << "  - hoist_hook: configured=" << (hoist_hook_defaults_.enable ? "true" : "false")
              << ", instantiated=" << (hoist_hook_ ? "true" : "false") << "\n";
#endif
#ifdef ASC_ENABLE_IO_RELAY
    std::cout << "  - io_relay: configured=" << (io_relay_defaults_.enable ? "true" : "false")
              << ", instantiated=" << (io_relay_ ? "true" : "false") << "\n";
#endif
#ifdef ASC_ENABLE_MULTI_TURN_ENCODER
    std::cout << "  - multi_turn_encoder: configured=" << (encoder_defaults_.enable ? "true" : "false")
              << ", instantiated=" << (multi_turn_encoder_ ? "true" : "false") << "\n";
#endif
#ifdef ASC_ENABLE_SPD_LIDAR
    for (size_t i = 0; i < spd_lidar_instances_.size(); ++i) {
      const SpdLidarInstanceDefaults& cfg = spd_lidar_instances_[i];
      const bool instantiated = spd_lidar_instances_core_.find(cfg.id) != spd_lidar_instances_core_.end();
      std::cout << "  - spd_lidar:" << cfg.id
                << ": configured=" << (cfg.enable ? "true" : "false")
                << ", instantiated=" << (instantiated ? "true" : "false") << "\n";
    }
#endif
  }

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
void Interface::updateSolarChargeStateFromDriver() {
  const std::int64_t stale_timeout_ms = static_cast<std::int64_t>(
      std::max(100, solar_defaults_.stale_timeout_ms));
  DeviceStatus data = getDeviceStatus();
  if (!solar_) {
    if (solar_defaults_.enable) {
      data.solarCharge = DeviceStatus::SolarChargeState::Fault;
    }
    setDeviceStatus(data);
    return;
  }

  // Rule 1: if trolley battery is already offline, solar state is considered faulted.
  if (data.trolleyState == DeviceStatus::EquipmentState::Offline) {
    data.solarCharge = DeviceStatus::SolarChargeState::Fault;
    setDeviceStatus(data);
    return;
  }

  solar::SolarCore::ChargeStatusSample sample;
  if (!solar_->readChargeStatusSample(&sample) || !sample.ok) {
    const std::int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();
    const std::int64_t last_ok_ms = solar_charge_last_ok_ms_.load(std::memory_order_relaxed);
    const bool stale = (last_ok_ms <= 0) || ((now_ms - last_ok_ms) > stale_timeout_ms);
    if (stale) {
      data.solarCharge = DeviceStatus::SolarChargeState::Fault;
    }
    setDeviceStatus(data);
    return;
  }

  const std::int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();
  solar_charge_last_ok_ms_.store(now_ms, std::memory_order_relaxed);

  if (solar::SolarCore::hasChargeFault(sample.charge_status_word)) {
    data.solarCharge = DeviceStatus::SolarChargeState::Fault;
  } else if (sample.battery_current_a > 0.05) {
    data.solarCharge = DeviceStatus::SolarChargeState::Charging;
  } else {
    data.solarCharge = DeviceStatus::SolarChargeState::NotCharging;
  }
  setDeviceStatus(data);
}

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
  if (cmd == "basic" || cmd == "status" || cmd == "all") {
    updateSolarChargeStateFromDriver();
  }
  return Status{true, "ok"};
}
#endif

#ifdef ASC_ENABLE_HOIST_HOOK
Status Interface::queryHoistHook(const std::vector<std::string>& args) {
  if (!hoist_hook_) return Status{false, "hoist_hook not enabled"};
  if (args.empty()) {
    std::cout << "[hoist_hook] usage:\n"
              << "  hoist_hook map\n"
              << "  hoist_hook speaker|light|rfid|power|gps|all|heartbeat|mode\n"
              << "  hoist_hook speaker_ctl <off|7m|3m|both|7m_off|3m_off>\n"
              << "  hoist_hook light_ctl <on|off>\n"
              << "  hoist_hook volume <0-30>\n"
              << "  hoist_hook get <addr> [qty] [fc]\n"
              << "  hoist_hook set <addr> <value> [fc]\n";
    return Status{false, "missing command"};
  }
  const std::string& cmd = args[0];
  if (cmd == "map") hoist_hook_->printRegisterGroups();
  else if (cmd == "speaker" || cmd == "light" || cmd == "rfid" || cmd == "power" ||
           cmd == "gps" || cmd == "all" || cmd == "heartbeat" || cmd == "mode") {
    hoist_hook_->queryHookInfo(cmd);
  }
  else if (cmd == "speaker_ctl") {
    if (args.size() < 2) return Status{false, "usage: hoist_hook speaker_ctl <off|7m|3m|both|7m_off|3m_off> [quiet]"};
    const bool quiet = (args.size() >= 3 && args[2] == "quiet");
    hoist_hook_->controlSpeaker(args[1], quiet);
  } else if (cmd == "light_ctl") {
    if (args.size() < 2) return Status{false, "usage: hoist_hook light_ctl <on|off>"};
    hoist_hook_->controlWarningLight(args[1]);
  } else if (cmd == "volume") {
    if (args.size() < 2) return Status{false, "usage: hoist_hook volume <0-30>"};
    int vol = 0;
    if (!parseInt(args[1], &vol)) return Status{false, "invalid volume"};
    if (vol < 0 || vol > 30) return Status{false, "volume out of range (0-30)"};
    // 文档：DEC103(0x0067) 设置音量 0~30；交互控制免确认
    hoist_hook_->genericWrite(static_cast<uint16_t>(0x0067), static_cast<uint16_t>(vol), 0x06, true);
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
    std::cout << "[hoist_hook] unknown command: " << cmd << "\n"
              << "  usage: hoist_hook speaker_ctl <off|7m|3m|both|7m_off|3m_off>\n"
              << "         hoist_hook light_ctl <on|off>\n"
              << "         hoist_hook volume <0-30>\n"
              << "         hoist_hook speaker|light|rfid|power|gps|all\n"
              << "         hoist_hook get <addr> [qty] [fc]\n"
              << "         hoist_hook set <addr> <value> [fc]\n";
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
    if (!io_relay_->controlRelay(ch, cmd)) {
      return Status{false, "io_relay control failed"};
    }
  } else if (cmd == "read") {
    int ch = 0;
    if (args.size() >= 2 && !parseInt(args[1], &ch)) return Status{false, "invalid channel"};
    if (!io_relay_->readRelayStatus(ch)) {
      return Status{false, "io_relay read failed"};
    }
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
              << " turns_calibrated=" << data.turns_calibrated
              << " velocity=" << data.velocity << "\n";
    return Status{true, "ok"};
  }
  return Status{false, "unknown multi_turn_encoder command"};
}
#endif

#ifdef ASC_ENABLE_SPD_LIDAR
std::string Interface::spdLidarEndpointKey(const std::string& ip, int port) {
  return ip + ":" + std::to_string(port);
}

void Interface::closeSpdLidarServerConnectionLocked(const std::string& id) {
  std::unordered_map<std::string, SpdLidarServerConnectionState>::iterator it =
      spd_lidar_server_connections_.find(id);
  if (it == spd_lidar_server_connections_.end()) return;
  if (it->second.conn_fd >= 0) {
    ::close(it->second.conn_fd);
    it->second.conn_fd = -1;
  }
  it->second.peer_ip.clear();
  it->second.peer_port = 0;
}

Status Interface::startSpdLidarServers() {
  bool has_server_instance = false;
  for (size_t i = 0; i < spd_lidar_instances_.size(); ++i) {
    if (spd_lidar_instances_[i].enable && spd_lidar_instances_[i].mode == "server") {
      has_server_instance = true;
      break;
    }
  }
  if (!has_server_instance) return Status{true, "spd_lidar has no server listeners"};

  {
    std::lock_guard<std::mutex> lock(spd_lidar_server_mutex_);
    if (spd_lidar_server_running_) {
      return Status{true, "spd_lidar listeners already started"};
    }
    spd_lidar_server_endpoints_.clear();
    spd_lidar_server_connections_.clear();
    for (size_t i = 0; i < spd_lidar_instances_.size(); ++i) {
      const SpdLidarInstanceDefaults& cfg = spd_lidar_instances_[i];
      if (!cfg.enable || cfg.mode != "server") continue;
      const std::string key = spdLidarEndpointKey(cfg.local_ip, cfg.local_port);
      SpdLidarServerEndpointState& endpoint = spd_lidar_server_endpoints_[key];
      endpoint.bind_ip = cfg.local_ip;
      endpoint.bind_port = cfg.local_port;
      endpoint.instance_ids.push_back(cfg.id);
      spd_lidar_server_connections_[cfg.id] = SpdLidarServerConnectionState{};
    }
  }

  std::vector<std::string> endpoint_keys;
  {
    std::lock_guard<std::mutex> lock(spd_lidar_server_mutex_);
    spd_lidar_server_running_ = true;
    for (std::unordered_map<std::string, SpdLidarServerEndpointState>::iterator it =
             spd_lidar_server_endpoints_.begin();
         it != spd_lidar_server_endpoints_.end(); ++it) {
      endpoint_keys.push_back(it->first);
    }
  }

  for (size_t i = 0; i < endpoint_keys.size(); ++i) {
    const std::string& key = endpoint_keys[i];
    std::string error;
    std::string actual_bind_ip;
    std::string bind_ip;
    int bind_port = 0;
    {
      std::lock_guard<std::mutex> lock(spd_lidar_server_mutex_);
      std::unordered_map<std::string, SpdLidarServerEndpointState>::iterator it =
          spd_lidar_server_endpoints_.find(key);
      if (it == spd_lidar_server_endpoints_.end()) continue;
      bind_ip = it->second.bind_ip;
      bind_port = it->second.bind_port;
    }

    const int listen_fd = createSpdLidarListenSocket(bind_ip, bind_port, &actual_bind_ip, &error);
    if (listen_fd < 0) {
      stopSpdLidarServers();
      return Status{false, "spd_lidar listen failed on " + key + ": " + error};
    }

    {
      std::lock_guard<std::mutex> lock(spd_lidar_server_mutex_);
      std::unordered_map<std::string, SpdLidarServerEndpointState>::iterator it =
          spd_lidar_server_endpoints_.find(key);
      if (it == spd_lidar_server_endpoints_.end()) {
        ::close(listen_fd);
        continue;
      }
      it->second.listen_fd = listen_fd;
      if (!actual_bind_ip.empty()) it->second.bind_ip = actual_bind_ip;
      it->second.accept_thread = std::thread([this, key]() { spdLidarAcceptLoop(key); });
      std::cout << "[spd_lidar] listening at " << it->second.bind_ip << ":" << it->second.bind_port
                << " for " << it->second.instance_ids.size() << " instance(s)\n";
    }
  }

  return Status{true, "spd_lidar listeners started"};
}

Status Interface::stopSpdLidarServers() {
  std::vector<std::thread> threads;
  {
    std::lock_guard<std::mutex> lock(spd_lidar_server_mutex_);
    spd_lidar_server_running_ = false;
    for (std::unordered_map<std::string, SpdLidarServerEndpointState>::iterator it =
             spd_lidar_server_endpoints_.begin();
         it != spd_lidar_server_endpoints_.end(); ++it) {
      if (it->second.listen_fd >= 0) {
        ::close(it->second.listen_fd);
        it->second.listen_fd = -1;
      }
      if (it->second.accept_thread.joinable()) {
        threads.push_back(std::move(it->second.accept_thread));
      }
    }
    for (std::unordered_map<std::string, SpdLidarServerConnectionState>::iterator it =
             spd_lidar_server_connections_.begin();
         it != spd_lidar_server_connections_.end(); ++it) {
      if (it->second.conn_fd >= 0) {
        ::close(it->second.conn_fd);
        it->second.conn_fd = -1;
      }
    }
    spd_lidar_server_connections_.clear();
    spd_lidar_server_endpoints_.clear();
  }

  for (size_t i = 0; i < threads.size(); ++i) {
    if (threads[i].joinable()) threads[i].join();
  }
  return Status{true, "spd_lidar listeners stopped"};
}

std::string Interface::matchSpdLidarServerInstance(const std::string& endpoint_key,
                                                   const std::string& peer_ip,
                                                   int peer_port) const {
  std::string ip_match;
  std::string fallback_match;
  for (size_t i = 0; i < spd_lidar_instances_.size(); ++i) {
    const SpdLidarInstanceDefaults& cfg = spd_lidar_instances_[i];
    if (!cfg.enable || cfg.mode != "server") continue;
    if (spdLidarEndpointKey(cfg.local_ip, cfg.local_port) != endpoint_key) continue;
    if (cfg.device_ip == peer_ip && cfg.device_port == peer_port) return cfg.id;
    if (ip_match.empty() && cfg.device_ip == peer_ip) ip_match = cfg.id;
    if (fallback_match.empty()) fallback_match = cfg.id;
  }
  if (!ip_match.empty()) return ip_match;
  return fallback_match;
}

void Interface::spdLidarAcceptLoop(const std::string& endpoint_key) {
  while (spd_lidar_server_running_) {
    int listen_fd = -1;
    {
      std::lock_guard<std::mutex> lock(spd_lidar_server_mutex_);
      std::unordered_map<std::string, SpdLidarServerEndpointState>::iterator it =
          spd_lidar_server_endpoints_.find(endpoint_key);
      if (it == spd_lidar_server_endpoints_.end()) return;
      listen_fd = it->second.listen_fd;
    }
    if (listen_fd < 0) return;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(listen_fd, &read_fds);
    timeval accept_tv{};
    accept_tv.tv_sec = 1;
    accept_tv.tv_usec = 0;
    const int ready = ::select(listen_fd + 1, &read_fds, nullptr, nullptr, &accept_tv);
    if (!spd_lidar_server_running_) break;
    if (ready == 0) continue;
    if (ready < 0) {
      if (errno == EINTR || errno == EBADF) continue;
      std::cout << "[spd_lidar] accept loop error on " << endpoint_key
                << ": " << std::strerror(errno) << "\n";
      continue;
    }

    sockaddr_in peer_addr{};
    socklen_t peer_len = sizeof(peer_addr);
    const int conn_fd = ::accept(listen_fd, reinterpret_cast<sockaddr*>(&peer_addr), &peer_len);
    if (conn_fd < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
      if (spd_lidar_server_running_) {
        std::cout << "[spd_lidar] accept failed on " << endpoint_key
                  << ": " << std::strerror(errno) << "\n";
      }
      continue;
    }

    const std::string peer_ip = sockaddrIp(peer_addr);
    const int peer_port = static_cast<int>(ntohs(peer_addr.sin_port));
    const std::string instance_id = matchSpdLidarServerInstance(endpoint_key, peer_ip, peer_port);
    if (instance_id.empty()) {
      std::cout << "[spd_lidar] reject unknown client " << peer_ip << ":" << peer_port
                << " on " << endpoint_key << "\n";
      ::close(conn_fd);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(spd_lidar_server_mutex_);
      if (!spd_lidar_server_running_) {
        ::close(conn_fd);
        break;
      }
      closeSpdLidarServerConnectionLocked(instance_id);
      SpdLidarServerConnectionState& conn = spd_lidar_server_connections_[instance_id];
      conn.conn_fd = conn_fd;
      conn.peer_ip = peer_ip;
      conn.peer_port = peer_port;
    }
    {
      std::lock_guard<std::mutex> lock(spd_lidar_log_mutex_);
      spd_lidar_wait_logged_.erase(instance_id);
    }
    std::cout << "[spd_lidar:" << instance_id << "] client connected from "
              << peer_ip << ":" << peer_port << "\n";
  }
}

bool Interface::spdLidarExchange(const SpdLidarInstanceDefaults& cfg,
                                 const std::vector<uint8_t>& request,
                                 std::vector<uint8_t>* response,
                                 std::string* error) {
  if (cfg.mode != "server") {
    if (!response) return false;
    response->clear();
    if (cfg.device_ip.empty() || cfg.device_port <= 0) {
      if (error) *error = "invalid spd_lidar device endpoint";
      return false;
    }

    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      if (error) *error = std::string("socket failed: ") + std::strerror(errno);
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(cfg.device_port));
    if (::inet_pton(AF_INET, cfg.device_ip.c_str(), &addr.sin_addr) != 1) {
      if (error) *error = "invalid ip: " + cfg.device_ip;
      ::close(fd);
      return false;
    }
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      if (error) *error = std::string("connect failed: ") + std::strerror(errno);
      ::close(fd);
      return false;
    }

    const bool ok = spdLidarExchangeOnConnectedFd(fd, request, response, error);
    ::close(fd);
    return ok;
  }

  std::lock_guard<std::mutex> lock(spd_lidar_server_mutex_);
  std::unordered_map<std::string, SpdLidarServerConnectionState>::iterator it =
      spd_lidar_server_connections_.find(cfg.id);
  if (it == spd_lidar_server_connections_.end() || it->second.conn_fd < 0) {
    if (error) *error = "client not connected";
    return false;
  }

  const int fd = it->second.conn_fd;
  const bool ok = spdLidarExchangeOnConnectedFd(fd, request, response, error);
  if (!ok) {
    closeSpdLidarServerConnectionLocked(cfg.id);
  }
  return ok;
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
      bool connected = false;
      std::string peer_ip;
      int peer_port = 0;
      if (cfg.mode == "server") {
        std::lock_guard<std::mutex> lock(spd_lidar_server_mutex_);
        std::unordered_map<std::string, SpdLidarServerConnectionState>::const_iterator conn_it =
            spd_lidar_server_connections_.find(cfg.id);
        if (conn_it != spd_lidar_server_connections_.end() && conn_it->second.conn_fd >= 0) {
          connected = true;
          peer_ip = conn_it->second.peer_ip;
          peer_port = conn_it->second.peer_port;
        }
      }
      std::cout << "  - id=" << cfg.id
                << " enable=" << (cfg.enable ? "true" : "false")
                << " mode=" << cfg.mode
                << " local=" << cfg.local_ip << ":" << cfg.local_port
                << " device=" << cfg.device_ip << ":" << cfg.device_port
                << " initialized=" << (initialized ? "true" : "false");
      if (cfg.mode == "server") {
        std::cout << " connected=" << (connected ? "true" : "false");
        if (connected) std::cout << " peer=" << peer_ip << ":" << peer_port;
      }
      if (!cfg.role.empty()) std::cout << " role=" << cfg.role;
      std::cout << " vertical_angle_to_vertical_deg=" << cfg.vertical_angle_to_vertical_deg
                << "\n";
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
