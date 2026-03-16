#include "battery/battery_core.hpp"
#include "ai_safety_controller/common/gateway_serial.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <thread>

namespace battery {

namespace {

uint16_t readBe16(const uint8_t* p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

std::uint32_t estimateChargeRemainingMinutes(double q_rem_ah,
                                             double q_full_ah,
                                             double soc_percent,
                                             double current_a) {
  if (q_full_ah <= 0.0 || current_a <= 0.05) return 0u;
  if (soc_percent < 0.0) soc_percent = 0.0;
  if (soc_percent > 100.0) soc_percent = 100.0;

  // Preferred estimator: capacity gap / charging current.
  double t_cap_min = -1.0;
  const double q_gap_ah = q_full_ah - q_rem_ah;
  if (q_gap_ah > 0.0) {
    t_cap_min = (q_gap_ah / current_a) * 60.0;
  }

  // Fallback estimator: SOC gap / charging current.
  double t_soc_min = -1.0;
  const double q_soc_gap_ah = ((100.0 - soc_percent) / 100.0) * q_full_ah;
  if (q_soc_gap_ah > 0.0) {
    t_soc_min = (q_soc_gap_ah / current_a) * 60.0;
  }

  double chosen = t_cap_min >= 0.0 ? t_cap_min : t_soc_min;
  if (chosen < 0.0) return 0u;
  if (chosen > 24.0 * 60.0) chosen = 24.0 * 60.0;  // Clamp to one day.
  return static_cast<std::uint32_t>(chosen + 0.5);
}

int computeRetryDelayMs(const BatteryCore::RetryPolicy& policy, int retry_index) {
  if (retry_index <= 0) return 0;
  const int base = std::max(0, policy.base_backoff_ms);
  const int cap = std::max(base, policy.max_backoff_ms);
  int delay = base;
  for (int i = 1; i < retry_index && delay < cap; ++i) {
    delay = std::min(cap, delay * 2);
  }
  const int jitter = std::max(0, policy.jitter_ms);
  if (jitter > 0) {
    thread_local std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, jitter);
    delay += dist(rng);
  }
  return delay;
}

}  // namespace

BatteryCore::BatteryCore() : BatteryCore("192.168.1.12", 502, 3, 2) {}

BatteryCore::BatteryCore(const std::string& module_ip,
                         uint16_t module_port,
                         uint8_t module_slave_id,
                         uint8_t battery_slave_id)
    : BatteryCore(module_ip, module_port, module_slave_id, battery_slave_id, RetryPolicy()) {}

BatteryCore::BatteryCore(const std::string& module_ip,
                         uint16_t module_port,
                         uint8_t module_slave_id,
                         uint8_t battery_slave_id,
                         const RetryPolicy& retry_policy)
    : module_ip_(module_ip),
      module_port_(module_port),
      module_slave_id_(module_slave_id),
      battery_slave_id_(battery_slave_id),
      transaction_id_(0x31A6),
      socket_fd_(-1),
      retry_policy_(retry_policy),
      register_groups_({
          {0x0000, 0x000F, "读/写混合", "基础状态（SOC、电流电压、MOS、均衡位）"},
          {0x0010, 0x004F, "只读", "第1~64节电芯电压"},
          {0x0050, 0x0061, "只读", "第1~15路NTC温度 + 平均/最高/最低"},
          {0x0062, 0x0090, "读/写混合", "保护状态、串数、地址、波特率、保护阈值"},
          {0x0100, 0x0161, "读/写混合", "电流/电压/温度校准参数"},
          {0x0162, 0x0183, "只读", "蓝牙/GPS/绝缘/告警/SOH/大电流"},
          {0x0200, 0x0221, "读/写混合", "告警阈值与回环参数"},
          {0x0FA1, 0x0FB4, "读/写（高风险）", "调试/强制控制寄存器"},
          {0x5A60, 0x5A8E, "读/写（高风险）", "高级系统/网络/通信参数"},
      }) {}

BatteryCore::~BatteryCore() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  disconnectLocked();
}

bool BatteryCore::parseNumber(const std::string& text, int* out) {
  if (!out) return false;
  std::string t = text;
  try {
    size_t idx = 0;
    int base = 10;
    if (t.size() > 2 && t[0] == '0' && (t[1] == 'x' || t[1] == 'X')) {
      base = 16;
    }
    *out = std::stoi(t, &idx, base);
    return idx == t.size();
  } catch (...) {
    return false;
  }
}

bool BatteryCore::parseFunctionCode(const std::string& text,
                                    const std::vector<int>& allowed,
                                    int* out) {
  int parsed = 0;
  if (!parseNumber(text, &parsed)) return false;
  for (size_t i = 0; i < allowed.size(); ++i) {
    if (allowed[i] == parsed) {
      if (out) *out = parsed;
      return true;
    }
  }
  return false;
}

bool BatteryCore::isOnline(double timeout_sec) {
  std::vector<uint8_t> response;
  if (!sendBatteryRead(0x03, 0x0002, 1, battery_slave_id_, &response, timeout_sec)) {
    return false;
  }
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 1, &values)) {
    return false;
  }
  return !values.empty();
}

bool BatteryCore::readSummary(Summary* out, double timeout_sec) {
  if (!out) return false;
  *out = Summary{};

  if (battery_slave_id_ == module_slave_id_ || battery_slave_id_ < 2) {
    return false;
  }

  std::vector<uint8_t> response;
  if (!sendBatteryRead(0x03, 0x0000, 9, battery_slave_id_, &response, timeout_sec)) {
    return false;
  }
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 9, &values) || values.size() < 9) {
    return false;
  }

  std::vector<uint8_t> charge_mos_resp;
  bool has_charge_mos = false;
  uint16_t charge_mos = 0;
  if (sendBatteryRead(0x03, 0x000A, 1, battery_slave_id_, &charge_mos_resp, timeout_sec)) {
    std::vector<uint16_t> mos_values;
    if (parseRegisterResponse(charge_mos_resp, 0x03, 1, &mos_values) && !mos_values.empty()) {
      has_charge_mos = true;
      charge_mos = mos_values[0];
    }
  }

  Summary s;
  s.soc_percent = static_cast<float>(values[0] * 0.01);
  s.voltage_v = static_cast<float>(values[2] * 0.01);
  s.current_a = static_cast<float>(toSigned16(values[1]) * 0.01);
  s.remaining_discharge_min = (values[7] == 0xFFFFu) ? 0u : static_cast<std::uint32_t>(values[7]);
  const std::uint16_t charge_time_raw = values[8];
  const bool has_valid_charge_time_raw = (charge_time_raw != 0xFFFFu) && (charge_time_raw > 0u);
  const bool charge_path_available = !has_charge_mos || (charge_mos != 0u);
  const bool is_charging_now = charge_path_available && (s.current_a > 0.05f);
  const double q_rem_ah = static_cast<double>(values[3]) * 0.1;
  const double q_full_ah = static_cast<double>(values[4]) * 0.1;
  const std::uint32_t estimate_charge_min = is_charging_now
      ? estimateChargeRemainingMinutes(
            q_rem_ah, q_full_ah, static_cast<double>(s.soc_percent), static_cast<double>(s.current_a))
      : 0u;
  const char* charge_time_source = "none";
  if (has_valid_charge_time_raw) {
    // Prefer battery-reported charge remaining time when available.
    s.remaining_charge_min = static_cast<std::uint32_t>(charge_time_raw);
    charge_time_source = "raw";
  } else {
    if (is_charging_now) {
      s.remaining_charge_min = estimate_charge_min;
      charge_time_source = "estimate";
    } else {
      s.remaining_charge_min = 0u;
      charge_time_source = "not_charging";
    }
  }
  if (charge_time_debug_enabled_) {
    std::cout << "[battery] [charge_time_debug] raw=" << charge_time_raw
              << " valid_raw=" << (has_valid_charge_time_raw ? "true" : "false")
              << " estimate=" << estimate_charge_min
              << " selected=" << s.remaining_charge_min
              << " source=" << charge_time_source
              << " currentA=" << s.current_a
              << " soc=" << s.soc_percent
              << " q_rem_ah=" << q_rem_ah
              << " q_full_ah=" << q_full_ah
              << " charge_mos=" << (has_charge_mos ? std::to_string(charge_mos) : "n/a")
              << "\n";
  }
  s.has_charge_mos = has_charge_mos;
  s.charge_mos = charge_mos;
  s.ok = true;
  *out = s;
  return true;
}

void BatteryCore::setChargeTimeDebugEnabled(bool enabled) {
  charge_time_debug_enabled_ = enabled;
}

std::vector<uint8_t> BatteryCore::createModbusPacket(uint8_t function_code,
                                                     uint16_t address,
                                                     uint16_t value,
                                                     uint16_t quantity,
                                                     uint8_t unit_id,
                                                     bool* ok) {
  if (ok) *ok = false;
  if (function_code == 0x03 || function_code == 0x04) {
    transaction_id_ = static_cast<uint16_t>((transaction_id_ + 1) & 0xFFFF);
  }
  if (!(function_code == 0x03 || function_code == 0x04 || function_code == 0x06)) {
    std::cout << "[battery] ❌ 不支持的功能码\n";
    return {};
  }

  std::vector<uint8_t> pkt;
  pkt.reserve(12);
  const uint16_t protocol_id = 0x0000;
  const uint16_t length = 6;
  pkt.push_back(static_cast<uint8_t>((transaction_id_ >> 8) & 0xFF));
  pkt.push_back(static_cast<uint8_t>(transaction_id_ & 0xFF));
  pkt.push_back(static_cast<uint8_t>((protocol_id >> 8) & 0xFF));
  pkt.push_back(static_cast<uint8_t>(protocol_id & 0xFF));
  pkt.push_back(static_cast<uint8_t>((length >> 8) & 0xFF));
  pkt.push_back(static_cast<uint8_t>(length & 0xFF));
  pkt.push_back(unit_id);
  pkt.push_back(function_code);
  pkt.push_back(static_cast<uint8_t>((address >> 8) & 0xFF));
  pkt.push_back(static_cast<uint8_t>(address & 0xFF));
  const uint16_t data = (function_code == 0x06) ? value : quantity;
  pkt.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
  pkt.push_back(static_cast<uint8_t>(data & 0xFF));
  if (ok) *ok = true;
  return pkt;
}

bool BatteryCore::sendModbusPacket(const std::vector<uint8_t>& packet,
                                   std::vector<uint8_t>* response,
                                   const std::string& context,
                                   double timeout_sec) {
  if (!response) return false;
  response->clear();
  const std::string endpoint_key = module_ip_ + ":" + std::to_string(module_port_);
  ai_safety_controller::common::GatewaySerialGuard serial_guard(endpoint_key, 120);
  std::lock_guard<std::mutex> lock(socket_mutex_);
  const int max_retries = std::max(0, retry_policy_.max_retries);
  for (int attempt = 0; attempt <= max_retries; ++attempt) {
    if (attempt > 0) {
      const int delay_ms = computeRetryDelayMs(retry_policy_, attempt);
      if (delay_ms > 0) {
        if (retry_policy_.log_enabled) {
          std::cout << "[battery] ⚠️ 第" << attempt << "/" << max_retries
                    << "次重试，退避" << delay_ms << "ms: " << context << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
      }
    }
    if (!ensureConnectionLocked(timeout_sec)) {
      disconnectLocked();
      continue;
    }
    if (sendAndReceiveLocked(packet, response, context)) {
      disconnectLocked();
      return true;
    }
    disconnectLocked();
  }
  if (retry_policy_.log_enabled) {
    std::cout << "[battery] ❌ 重试耗尽，操作失败: " << context << "\n";
  }
  return false;
}

bool BatteryCore::ensureConnectionLocked(double timeout_sec) {
  if (socket_fd_ >= 0) return true;

  socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    std::cout << "[battery] ❌ socket 创建失败: " << std::strerror(errno) << "\n";
    return false;
  }

  timeval tv{};
  tv.tv_sec = static_cast<int>(timeout_sec);
  tv.tv_usec = static_cast<int>((timeout_sec - tv.tv_sec) * 1000000.0);
  ::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(module_port_);
  if (::inet_pton(AF_INET, module_ip_.c_str(), &addr.sin_addr) != 1) {
    std::cout << "[battery] ❌ 模块IP无效: " << module_ip_ << "\n";
    disconnectLocked();
    return false;
  }

  if (::connect(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::cout << "[battery] ❌ 连接失败: " << std::strerror(errno) << "\n";
    disconnectLocked();
    return false;
  }
  return true;
}

void BatteryCore::disconnectLocked() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool BatteryCore::sendAndReceiveLocked(const std::vector<uint8_t>& packet,
                                       std::vector<uint8_t>* response,
                                       const std::string& context) {
  if (::send(socket_fd_, packet.data(), packet.size(), 0) < 0) {
    std::cout << "[battery] ❌ 发送失败: " << std::strerror(errno) << "\n";
    return false;
  }
  uint8_t buf[1024];
  const ssize_t n = ::recv(socket_fd_, buf, sizeof(buf), 0);
  if (n <= 0) {
    std::cout << "[battery] ❌ 无响应: " << context << "\n";
    return false;
  }
  response->assign(buf, buf + n);
  return true;
}

bool BatteryCore::sendBatteryRead(uint8_t function_code,
                                  uint16_t address,
                                  uint16_t quantity,
                                  uint8_t unit_id,
                                  std::vector<uint8_t>* response,
                                  double timeout_sec) {
  bool ok = false;
  const std::vector<uint8_t> pkt =
      createModbusPacket(function_code, address, 0, quantity, unit_id, &ok);
  if (!ok) return false;
  std::ostringstream ctx;
  ctx << "电池读寄存器 fc=0x" << std::hex << std::uppercase << static_cast<int>(function_code)
      << ", uid=" << std::dec << static_cast<int>(unit_id) << ", addr=0x" << std::hex
      << std::uppercase << address << ", qty=" << std::dec << quantity;
  return sendModbusPacket(pkt, response, ctx.str(), timeout_sec);
}

bool BatteryCore::parseRegisterResponse(const std::vector<uint8_t>& response,
                                        uint8_t function_code,
                                        uint16_t quantity,
                                        std::vector<uint16_t>* values) const {
  if (!values) return false;
  values->clear();
  if (response.size() < 9) {
    std::cout << "[battery] ❌ 响应报文过短\n";
    return false;
  }
  const uint8_t recv_fc = response[7];
  if (recv_fc != function_code) {
    const uint8_t err = response.size() > 8 ? response[8] : 0;
    std::cout << "❌ 电池返回错误，错误码：0x" << std::hex << std::uppercase
              << static_cast<int>(err) << std::dec << "\n";
    return false;
  }
  const uint8_t data_len = response[8];
  const size_t expected_len = 9 + data_len;
  if (response.size() != expected_len) {
    std::cout << "[battery] ❌ 响应长度异常，预期" << expected_len << "字节，实际" << response.size()
              << "字节\n";
    return false;
  }
  if (data_len < quantity * 2) {
    std::cout << "[battery] ❌ 数据长度不足，无法解析" << quantity << "个寄存器\n";
    return false;
  }
  for (uint16_t i = 0; i < quantity; ++i) {
    const size_t base = 9 + i * 2;
    values->push_back(readBe16(&response[base]));
  }
  return true;
}

int16_t BatteryCore::toSigned16(uint16_t value) const {
  return static_cast<int16_t>(value);
}

std::string BatteryCore::describeBatteryRegister(uint16_t addr) const {
  if (addr >= 0x0010 && addr <= 0x004F) {
    std::ostringstream oss;
    oss << "第" << (addr - 0x0010 + 1) << "节电芯电压（mV）";
    return oss.str();
  }
  static const std::map<uint16_t, std::string> kMap = {
      {0x0000, "SOC（0.01%）"},      {0x0001, "总电流（0.01A）"},
      {0x0002, "总电压（0.01V）"},  {0x000A, "充电MOS状态"},
      {0x000B, "放电MOS状态"},      {0x0062, "保护状态位"},
      {0x0063, "实际电池串数"},     {0x0064, "RS485地址"},
      {0x0182, "SOH（0.1%）"},
  };
  std::map<uint16_t, std::string>::const_iterator it = kMap.find(addr);
  if (it != kMap.end()) return it->second;
  return "文档寄存器（未内置详细语义）";
}

void BatteryCore::printRegisterGroups() const {
  std::cout << "\n📚 电池文档寄存器分组（可读可写范围）\n";
  for (size_t i = 0; i < register_groups_.size(); ++i) {
    const RegisterGroup& g = register_groups_[i];
    std::cout << "  0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
              << g.start << "~0x" << std::setw(4) << g.end << std::dec << " | " << g.rw
              << " | " << g.desc << "\n";
  }
}

bool BatteryCore::confirmRiskyWrite(uint16_t addr) const {
  const bool risky = (addr >= 0x0FA1 && addr <= 0x0FB4) || (addr >= 0x5A60 && addr <= 0x5A8E);
  if (!risky) return true;
  std::cout << "[battery] ⚠️  检测到高风险写入地址，可能导致设备参数变化。请输入 YES 确认继续写入：";
  std::string input;
  std::getline(std::cin, input);
  return input == "YES";
}

void BatteryCore::genericRead(uint16_t address, uint16_t quantity, int function_code) {
  if (quantity < 1 || quantity > 125) {
    std::cout << "[battery] ❌ 数量超限，读寄存器数量需在1~125\n";
    return;
  }
  int fc = (function_code < 0) ? 0x03 : function_code;
  if (!(fc == 0x03 || fc == 0x04)) {
    std::cout << "❌ 电池读取仅支持功能码 0x03/0x04\n";
    return;
  }
  std::vector<uint8_t> response;
  if (!sendBatteryRead(static_cast<uint8_t>(fc), address, quantity, battery_slave_id_, &response)) {
    return;
  }
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, static_cast<uint8_t>(fc), quantity, &values)) {
    return;
  }
  std::cout << "✅ 电池寄存器读取结果（fc=0x" << std::hex << std::uppercase << fc << std::dec
            << "）\n";
  for (size_t i = 0; i < values.size(); ++i) {
    const uint16_t reg = static_cast<uint16_t>(address + i);
    std::cout << "  0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << reg
              << std::dec << " = " << values[i] << " (0x" << std::hex << std::uppercase
              << std::setw(4) << std::setfill('0') << values[i] << std::dec << ") | "
              << describeBatteryRegister(reg) << "\n";
  }
}

void BatteryCore::genericWrite(uint16_t address, uint16_t value, int function_code) {
  if (function_code >= 0 && function_code != 0x06) {
    std::cout << "❌ 当前仅支持电池单寄存器写（0x06）\n";
    return;
  }
  if (!confirmRiskyWrite(address)) {
    std::cout << "[battery] ℹ️ 已取消写入\n";
    return;
  }
  bool ok = false;
  const std::vector<uint8_t> packet =
      createModbusPacket(0x06, address, value, 0, battery_slave_id_, &ok);
  if (!ok) return;
  std::vector<uint8_t> response;
  if (!sendModbusPacket(packet, &response, "电池写寄存器")) return;
  if (response == packet) {
    std::cout << "✅ 电池写入成功：0x" << std::hex << std::uppercase << std::setw(4)
              << std::setfill('0') << address << std::dec << " <= " << value << "\n";
  } else {
    std::cout << "[battery] ⚠️ 写入响应异常\n";
  }
}

void BatteryCore::queryBatteryInfo(const std::string& info_type) {
  std::cout << "\n📡 正在查询电池" << info_type << "信息...\n";
  if (battery_slave_id_ == module_slave_id_ || battery_slave_id_ < 2) {
    std::cout << "❌ 电池站号配置无效\n";
    return;
  }

  if (info_type == "basic") {
    std::vector<uint8_t> response;
    if (!sendBatteryRead(0x03, 0x0000, 9, battery_slave_id_, &response)) return;
    std::vector<uint16_t> values;
    if (!parseRegisterResponse(response, 0x03, 9, &values)) return;
    std::vector<uint8_t> charge_mos_resp;
    bool has_charge_mos = false;
    uint16_t charge_mos = 0;
    if (sendBatteryRead(0x03, 0x000A, 1, battery_slave_id_, &charge_mos_resp)) {
      std::vector<uint16_t> mos_values;
      if (parseRegisterResponse(charge_mos_resp, 0x03, 1, &mos_values) && !mos_values.empty()) {
        has_charge_mos = true;
        charge_mos = mos_values[0];
      }
    }

    const double current_a = toSigned16(values[1]) * 0.01;
    std::string charge_state = "未知";
    if (has_charge_mos) {
      if (charge_mos == 0) {
        charge_state = "未充电";
      } else {
        charge_state = (current_a > 0.05) ? "充电中" : "允许充电(当前无明显充电电流)";
      }
    } else {
      if (current_a > 0.05) charge_state = "充电中";
      else if (current_a < -0.05) charge_state = "放电中";
      else charge_state = "静置";
    }

    const uint16_t discharge_time_raw = values[7];  // 0x0007
    const uint16_t charge_time_raw = values[8];     // 0x0008
    const auto format_minutes = [](uint16_t raw, const char* idle_text) -> std::string {
      if (raw == 0xFFFF) return idle_text;
      return std::to_string(raw) + "分钟";
    };
    std::cout << "✅ 电池关键信息：\n";
    std::cout << "  充电状态: " << charge_state;
    if (has_charge_mos) std::cout << " (MOS=" << charge_mos << ")";
    std::cout << "\n";
    std::cout << "  SOC: " << std::fixed << std::setprecision(2) << (values[0] * 0.01) << "%\n";
    std::cout << "  总电流: " << std::fixed << std::setprecision(2) << current_a << "A\n";
    std::cout << "  总电压: " << std::fixed << std::setprecision(2) << (values[2] * 0.01) << "V\n";
    std::cout << "  剩余放电时间: " << format_minutes(discharge_time_raw, "非放电状态")
              << " (raw=0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
              << discharge_time_raw << std::dec << ")\n";
    std::cout << "  剩余充电时间: " << format_minutes(charge_time_raw, "非充电状态")
              << " (raw=0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
              << charge_time_raw << std::dec << ")\n";
  } else if (info_type == "cell") {
    std::vector<uint8_t> response;
    if (!sendBatteryRead(0x03, 0x0010, 16, battery_slave_id_, &response)) return;
    std::vector<uint16_t> values;
    if (!parseRegisterResponse(response, 0x03, 16, &values)) return;
    uint16_t max_v = values[0];
    uint16_t min_v = values[0];
    for (size_t i = 1; i < values.size(); ++i) {
      if (values[i] > max_v) max_v = values[i];
      if (values[i] < min_v) min_v = values[i];
    }
    std::cout << "[battery] ✅ 16节电芯电压：\n";
    std::cout << "  最高: " << max_v << "mV, 最低: " << min_v << "mV, 压差: " << (max_v - min_v)
              << "mV\n";
    for (size_t i = 0; i < values.size(); ++i) {
      std::cout << "  第" << (i + 1) << "节: " << values[i] << "mV\n";
    }
  } else if (info_type == "temp") {
    std::vector<uint8_t> response;
    if (!sendBatteryRead(0x03, 0x0050, 2, battery_slave_id_, &response)) return;
    std::vector<uint16_t> values;
    if (!parseRegisterResponse(response, 0x03, 2, &values)) return;
    std::cout << "[battery] ✅ 温度信息：\n";
    std::cout << "  第1路NTC温度: " << std::fixed << std::setprecision(1) << toSigned16(values[0]) * 0.1
              << "℃\n";
    std::cout << "  第2路NTC温度: " << std::fixed << std::setprecision(1) << toSigned16(values[1]) * 0.1
              << "℃\n";
  } else if (info_type == "mos") {
    std::vector<uint8_t> c_resp;
    std::vector<uint8_t> d_resp;
    std::cout << "[battery] ✅ MOS管状态：\n";
    if (sendBatteryRead(0x03, 0x000A, 1, battery_slave_id_, &c_resp)) {
      std::vector<uint16_t> values;
      if (parseRegisterResponse(c_resp, 0x03, 1, &values)) {
        std::cout << "  充电MOS管状态: " << values[0] << "\n";
      }
    }
    if (sendBatteryRead(0x03, 0x000B, 1, battery_slave_id_, &d_resp)) {
      std::vector<uint16_t> values;
      if (parseRegisterResponse(d_resp, 0x03, 1, &values)) {
        std::cout << "  放电MOS管状态: " << values[0] << "\n";
      }
    }
  } else if (info_type == "protect") {
    std::vector<uint8_t> response;
    if (!sendBatteryRead(0x03, 0x0062, 1, battery_slave_id_, &response)) return;
    std::vector<uint16_t> values;
    if (!parseRegisterResponse(response, 0x03, 1, &values)) return;
    static const std::map<int, std::string> kBits = {
        {0, "单体过压保护"}, {1, "单体欠压保护"}, {2, "整组过压保护"}, {3, "整组欠压保护"},
        {4, "充电过温保护"}, {5, "充电低温保护"}, {6, "放电过温保护"}, {7, "放电低温保护"},
        {8, "充电过流保护"}, {9, "放电过流保护"}, {10, "短路保护"},
    };
    const uint16_t v = values[0];
    std::vector<std::string> active;
    for (std::map<int, std::string>::const_iterator it = kBits.begin(); it != kBits.end(); ++it) {
      if ((v >> it->first) & 0x1) active.push_back(it->second);
    }
    if (active.empty()) {
      std::cout << "[battery] ✅ 无保护状态，电池正常\n";
    } else {
      std::cout << "[battery] ⚠️ 存在保护/告警: ";
      for (size_t i = 0; i < active.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << active[i];
      }
      std::cout << "\n";
    }
  } else if (info_type == "all") {
    queryBatteryInfo("basic");
    queryBatteryInfo("cell");
    queryBatteryInfo("temp");
    queryBatteryInfo("mos");
    queryBatteryInfo("protect");
  } else {
    std::cout << "[battery] ❌ 未知 info_type: " << info_type << "\n";
  }
}

void BatteryCore::scanBatterySlaveIds(int start_id, int end_id) {
  if (start_id < 1 || end_id > 252 || start_id > end_id) {
    std::cout << "[battery] ❌ 参数错误，示例：scan 或 scan 1 16\n";
    return;
  }
  std::cout << "\n🔎 扫描电池站号: " << start_id << "~" << end_id << "\n";
  std::vector<int> found;
  for (int uid = start_id; uid <= end_id; ++uid) {
    if (uid == module_slave_id_) continue;
    std::vector<uint8_t> response;
    if (!sendBatteryRead(0x03, 0x0002, 1, static_cast<uint8_t>(uid), &response, 1.5)) continue;
    std::vector<uint16_t> values;
    if (!parseRegisterResponse(response, 0x03, 1, &values)) continue;
    std::cout << "✅ 站号" << uid << " 有响应，总电压=" << std::fixed << std::setprecision(2)
              << (values[0] * 0.01) << "V\n";
    found.push_back(uid);
  }
  if (found.empty()) {
    std::cout << "❌ 未发现可用电池从站\n";
  } else {
    std::cout << "🎯 可用电池站号: [";
    for (size_t i = 0; i < found.size(); ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << found[i];
    }
    std::cout << "]\n";
  }
}

void BatteryCore::setBatteryAddr(int new_addr) {
  if (new_addr < 1 || new_addr > 252) {
    std::cout << "[battery] ❌ 地址无效，需在1-252之间\n";
    return;
  }
  bool ok = false;
  const std::vector<uint8_t> packet = createModbusPacket(
      0x06, 0x0064, static_cast<uint16_t>(new_addr), 0, battery_slave_id_, &ok);
  if (!ok) return;
  std::vector<uint8_t> response;
  if (!sendModbusPacket(packet, &response, "电池地址修改")) return;
  if (response == packet) {
    battery_slave_id_ = static_cast<uint8_t>(new_addr);
    std::cout << "✅ 电池从站地址已修改为" << new_addr << "，重启电池生效\n";
  } else {
    std::cout << "[battery] ⚠️ 地址修改响应异常\n";
  }
}

}  // namespace battery
