#include "hoist_hook/hoist_hook_core.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

namespace hoist_hook {

namespace {

uint16_t readBe16(const uint8_t* p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

uint32_t mergeUid(uint16_t high_word, uint16_t low_word) {
  return (static_cast<uint32_t>(high_word) << 16) | low_word;
}

}  // namespace

HoistHookCore::HoistHookCore() : HoistHookCore("192.168.1.12", 502, 0x03, 0x04) {}

HoistHookCore::HoistHookCore(const std::string& module_ip,
                             uint16_t module_port,
                             uint8_t hook_slave_id,
                             uint8_t power_slave_id)
    : module_ip_(module_ip),
      module_port_(module_port),
      hook_slave_id_(hook_slave_id),
      power_slave_id_(power_slave_id),
      transaction_id_(0x31A6),
      socket_fd_(-1),
      register_groups_({
          {0x0000, 0x0063, "读/写混合", "指令寄存器（0~99）"},
          {0x0064, 0x00C7, "只读", "状态寄存器（100~199）"},
      }) {}

HoistHookCore::~HoistHookCore() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  disconnectLocked();
}

bool HoistHookCore::parseNumber(const std::string& text, int* out) {
  if (!out) return false;
  try {
    size_t idx = 0;
    int base = 10;
    if (text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
      base = 16;
    }
    *out = std::stoi(text, &idx, base);
    return idx == text.size();
  } catch (...) {
    return false;
  }
}

bool HoistHookCore::parseFunctionCode(const std::string& text,
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

std::vector<uint8_t> HoistHookCore::createModbusPacket(uint8_t function_code,
                                                       uint16_t address,
                                                       uint16_t value,
                                                       uint16_t quantity,
                                                       uint8_t unit_id,
                                                       bool* ok) {
  if (ok) *ok = false;
  if (!(function_code == 0x03 || function_code == 0x06)) {
    std::cout << "❌ 不支持的功能码，仅支持 0x03/0x06\n";
    return {};
  }

  transaction_id_ = static_cast<uint16_t>((transaction_id_ + 1) & 0xFFFF);
  const uint16_t protocol_id = 0x0000;
  const uint16_t length = 6;
  const uint16_t data = (function_code == 0x06) ? value : quantity;

  std::vector<uint8_t> pkt;
  pkt.reserve(12);
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
  pkt.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
  pkt.push_back(static_cast<uint8_t>(data & 0xFF));

  if (ok) *ok = true;
  return pkt;
}

bool HoistHookCore::sendModbusPacket(const std::vector<uint8_t>& packet,
                                     std::vector<uint8_t>* response,
                                     const std::string& context,
                                     double timeout_sec) {
  if (!response) return false;
  response->clear();
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (!ensureConnectionLocked(timeout_sec)) return false;
  if (sendAndReceiveLocked(packet, response, context)) {
    disconnectLocked();
    return true;
  }
  disconnectLocked();
  if (!ensureConnectionLocked(timeout_sec)) return false;
  const bool ok = sendAndReceiveLocked(packet, response, context);
  disconnectLocked();
  return ok;
}

bool HoistHookCore::ensureConnectionLocked(double timeout_sec) {
  if (socket_fd_ >= 0) return true;

  socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    std::cout << "❌ socket 创建失败: " << std::strerror(errno) << "\n";
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
    std::cout << "❌ 模块IP无效: " << module_ip_ << "\n";
    disconnectLocked();
    return false;
  }
  if (::connect(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::cout << "❌ 连接失败: " << std::strerror(errno) << "\n";
    disconnectLocked();
    return false;
  }
  return true;
}

void HoistHookCore::disconnectLocked() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool HoistHookCore::sendAndReceiveLocked(const std::vector<uint8_t>& packet,
                                         std::vector<uint8_t>* response,
                                         const std::string& context) {
  if (::send(socket_fd_, packet.data(), packet.size(), 0) < 0) {
    std::cout << "❌ 发送失败: " << std::strerror(errno) << "\n";
    return false;
  }
  uint8_t buf[1024];
  const ssize_t n = ::recv(socket_fd_, buf, sizeof(buf), 0);
  if (n <= 0) {
    std::cout << "❌ 无响应: " << context << "\n";
    return false;
  }
  response->assign(buf, buf + n);
  return true;
}

bool HoistHookCore::sendRead(uint8_t function_code,
                             uint16_t address,
                             uint16_t quantity,
                             uint8_t unit_id,
                             std::vector<uint8_t>* response,
                             double timeout_sec) {
  bool ok = false;
  const std::vector<uint8_t> packet =
      createModbusPacket(function_code, address, 0, quantity, unit_id, &ok);
  if (!ok) return false;
  std::ostringstream context;
  context << "吊钩读寄存器 fc=0x" << std::hex << std::uppercase << static_cast<int>(function_code)
          << ", uid=" << std::dec << static_cast<int>(unit_id) << ", addr=0x" << std::hex
          << std::uppercase << address << ", qty=" << std::dec << quantity;
  return sendModbusPacket(packet, response, context.str(), timeout_sec);
}

bool HoistHookCore::parseRegisterResponse(const std::vector<uint8_t>& response,
                                          uint8_t function_code,
                                          uint16_t quantity,
                                          std::vector<uint16_t>* values) const {
  if (!values) return false;
  values->clear();
  if (response.size() < 9) {
    std::cout << "❌ 响应报文过短\n";
    return false;
  }
  const uint8_t recv_fc = response[7];
  if (recv_fc != function_code) {
    const uint8_t err = response.size() > 8 ? response[8] : 0;
    std::cout << "❌ 设备返回错误，错误码：0x" << std::hex << std::uppercase
              << static_cast<int>(err) << std::dec << "\n";
    return false;
  }
  const uint8_t data_len = response[8];
  if (response.size() < static_cast<size_t>(9 + data_len)) {
    std::cout << "❌ 响应长度异常\n";
    return false;
  }
  if (data_len < quantity * 2) {
    std::cout << "❌ 数据长度不足\n";
    return false;
  }
  for (uint16_t i = 0; i < quantity; ++i) {
    const size_t base = 9 + i * 2;
    values->push_back(readBe16(&response[base]));
  }
  return true;
}

std::string HoistHookCore::describeRegister(uint16_t addr) const {
  static const std::map<uint16_t, std::string> kMap = {
      {0x0001, "警示灯控制位（1开/0关）"},
      {0x0002, "喇叭控制/状态位（bit0=7m, bit1=3m）"},
      {0x0003, "RFID有效组掩码（bit0~bit7）"},
      {0x0004, "RFID组1 UID高16位"},
      {0x0005, "RFID组1 UID低16位"},
      {0x0006, "RFID组1 RSSI/电量（高8位RSSI,低8位电量）"},
      {0x0064, "状态区起始（100）"},
  };
  std::map<uint16_t, std::string>::const_iterator it = kMap.find(addr);
  if (it != kMap.end()) return it->second;
  return "文档寄存器（语义待补充）";
}

bool HoistHookCore::confirmRiskyWrite(uint16_t addr) const {
  const bool risky = (addr >= 0x0000 && addr <= 0x0063);
  if (!risky) return true;
  std::cout << "⚠️  即将写入指令寄存器，可能触发设备动作。请输入 YES 确认继续写入：";
  std::string input;
  std::getline(std::cin, input);
  return input == "YES";
}

void HoistHookCore::printRegisterGroups() const {
  std::cout << "\n📚 吊钩寄存器分组\n";
  for (size_t i = 0; i < register_groups_.size(); ++i) {
    const RegisterGroup& g = register_groups_[i];
    std::cout << "  0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
              << g.start << "~0x" << std::setw(4) << g.end << std::dec << " | " << g.rw
              << " | " << g.desc << "\n";
  }
}

void HoistHookCore::genericRead(uint16_t address, uint16_t quantity, int function_code) {
  if (quantity < 1 || quantity > 125) {
    std::cout << "❌ 数量超限，读寄存器数量需在1~125\n";
    return;
  }
  const int fc = (function_code < 0) ? 0x03 : function_code;
  if (fc != 0x03) {
    std::cout << "❌ 当前仅支持 0x03 读取\n";
    return;
  }

  std::vector<uint8_t> response;
  if (!sendRead(static_cast<uint8_t>(fc), address, quantity, hook_slave_id_, &response)) return;

  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, static_cast<uint8_t>(fc), quantity, &values)) return;

  std::cout << "✅ 吊钩寄存器读取结果\n";
  for (size_t i = 0; i < values.size(); ++i) {
    const uint16_t reg = static_cast<uint16_t>(address + i);
    std::cout << "  0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << reg
              << std::dec << " = " << values[i] << " (0x" << std::hex << std::uppercase
              << std::setw(4) << std::setfill('0') << values[i] << std::dec << ") | "
              << describeRegister(reg) << "\n";
  }
}

void HoistHookCore::genericWrite(uint16_t address, uint16_t value, int function_code) {
  const int fc = (function_code < 0) ? 0x06 : function_code;
  if (fc != 0x06) {
    std::cout << "❌ 当前仅支持 0x06 写入\n";
    return;
  }
  if (!confirmRiskyWrite(address)) {
    std::cout << "ℹ️ 已取消写入\n";
    return;
  }

  bool ok = false;
  const std::vector<uint8_t> packet =
      createModbusPacket(static_cast<uint8_t>(fc), address, value, 0, hook_slave_id_, &ok);
  if (!ok) return;

  std::vector<uint8_t> response;
  if (!sendModbusPacket(packet, &response, "吊钩写寄存器")) return;
  if (response == packet) {
    std::cout << "✅ 写入成功：0x" << std::hex << std::uppercase << std::setw(4)
              << std::setfill('0') << address << std::dec << " <= " << value << "\n";
  } else {
    std::cout << "⚠️ 写入响应异常\n";
  }
}

void HoistHookCore::controlSpeaker(const std::string& mode) {
  uint16_t value = 0;
  if (mode == "off") {
    value = 0x0000;
  } else if (mode == "7m") {
    value = 0x0001;
  } else if (mode == "3m") {
    value = 0x0002;
  } else if (mode == "both") {
    value = 0x0003;
  } else {
    std::cout << "❌ speaker 模式仅支持 off/7m/3m/both\n";
    return;
  }

  std::cout << "🔊 设置喇叭模式: " << mode << "\n";
  genericWrite(0x0002, value, 0x06);
}

void HoistHookCore::controlWarningLight(const std::string& status) {
  if (!(status == "on" || status == "off")) {
    std::cout << "❌ light 状态仅支持 on/off\n";
    return;
  }
  const uint16_t value = (status == "on") ? 1 : 0;
  std::cout << "🚨 设置警示灯: " << status << "\n";
  genericWrite(0x0001, value, 0x06);
}

void HoistHookCore::querySpeakerStatus() {
  std::vector<uint8_t> response;
  if (!sendRead(0x03, 0x0002, 1, hook_slave_id_, &response)) return;
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 1, &values)) return;
  const uint16_t v = values[0];
  const bool m7 = (v & 0x01) != 0;
  const bool m3 = (v & 0x02) != 0;
  std::cout << "✅ 喇叭状态寄存器(0x0002)=0x" << std::hex << std::uppercase << v << std::dec << "\n";
  std::cout << "  7m语音: " << (m7 ? "开启" : "关闭") << "\n";
  std::cout << "  3m语音: " << (m3 ? "开启" : "关闭") << "\n";
  if (m3) {
    std::cout << "  当前优先级输出: 3m语音\n";
  } else if (m7) {
    std::cout << "  当前优先级输出: 7m语音\n";
  } else {
    std::cout << "  当前优先级输出: 停止播放\n";
  }
}

void HoistHookCore::queryLightStatus() {
  std::vector<uint8_t> response;
  if (!sendRead(0x03, 0x0001, 1, hook_slave_id_, &response)) return;
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 1, &values)) return;
  const uint16_t v = values[0];
  const bool on = (v & 0x0001) != 0;
  std::cout << "✅ 警示灯状态: " << (on ? "开启" : "关闭")
            << " (reg=0x0001, raw=0x" << std::hex << std::uppercase << v << std::dec << ")\n";
}

void HoistHookCore::queryRfidInfo() {
  std::vector<uint8_t> mask_resp;
  if (!sendRead(0x03, 0x0003, 1, hook_slave_id_, &mask_resp)) return;
  std::vector<uint16_t> mask_values;
  if (!parseRegisterResponse(mask_resp, 0x03, 1, &mask_values)) return;

  const uint16_t valid_mask = mask_values[0] & 0x00FF;
  std::vector<uint8_t> group_resp;
  if (!sendRead(0x03, 0x0004, 24, hook_slave_id_, &group_resp)) return;
  std::vector<uint16_t> groups;
  if (!parseRegisterResponse(group_resp, 0x03, 24, &groups)) return;

  std::cout << "✅ RFID有效组掩码: 0x" << std::hex << std::uppercase << valid_mask << std::dec << "\n";
  bool has_valid = false;
  for (int i = 0; i < 8; ++i) {
    const bool valid = ((valid_mask >> i) & 0x1) != 0;
    const size_t base = static_cast<size_t>(i) * 3;
    const uint16_t uid_high = groups[base];
    const uint16_t uid_low = groups[base + 1];
    const uint16_t rssi_batt = groups[base + 2];
    const uint8_t rssi_raw = static_cast<uint8_t>((rssi_batt >> 8) & 0xFF);
    const uint8_t battery_level = static_cast<uint8_t>(rssi_batt & 0xFF);

    std::cout << "  组" << (i + 1) << ": " << (valid ? "有效" : "无效");
    if (valid) {
      has_valid = true;
      const uint32_t uid = mergeUid(uid_high, uid_low);
      std::cout << ", UID=0x" << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
                << uid << std::dec << ", RSSI=-" << static_cast<int>(rssi_raw)
                << " dBm, 电量等级=" << static_cast<int>(battery_level);
    }
    std::cout << "\n";
  }
  if (!has_valid) {
    std::cout << "ℹ️ 当前没有有效RFID组\n";
  } else {
    int valid_count = 0;
    for (int i = 0; i < 8; ++i) {
      if (((valid_mask >> i) & 0x1) != 0) ++valid_count;
    }
    std::cout << "ℹ️ 有效RFID组数量: " << valid_count << "/8\n";
  }
}

void HoistHookCore::queryPowerInfo() {
  std::cout << "🔋 正在读取电源模块状态...\n";
  std::vector<uint8_t> response;
  if (!sendRead(0x03, 0x0064, 6, power_slave_id_, &response)) {
    std::cout << "⚠️ 电源模块读取失败，可使用 get 命令手动排查具体地址\n";
    return;
  }
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 6, &values)) {
    std::cout << "⚠️ 电源模块响应解析失败\n";
    return;
  }

  const double bus_voltage_v = values[0] * 0.01;   // common 0.01V scale assumption
  const double bus_current_a = values[1] * 0.01;   // common 0.01A scale assumption
  const double soc_percent = values[2] * 0.01;     // documented [0,10000] -> %
  const uint16_t status_word = values[3];

  std::cout << "✅ 电源模块状态（解析）\n";
  std::cout << "  母线电压(估算): " << std::fixed << std::setprecision(2) << bus_voltage_v << "V"
            << " (raw=" << values[0] << ")\n";
  std::cout << "  母线电流(估算): " << std::fixed << std::setprecision(2) << bus_current_a << "A"
            << " (raw=" << values[1] << ")\n";
  std::cout << "  电荷余量SOC: " << std::fixed << std::setprecision(2) << soc_percent << "%"
            << " (raw=" << values[2] << ")\n";
  std::cout << "  状态字: 0x" << std::hex << std::uppercase << status_word << std::dec << "\n";
  std::cout << "  温度/保留(raw): " << values[4] << ", " << values[5] << "\n";

  std::cout << "  原始寄存器(0x0064~0x0069):";
  for (size_t i = 0; i < values.size(); ++i) {
    const uint16_t reg = static_cast<uint16_t>(0x0064 + i);
    std::cout << " [" << "0x" << std::hex << std::uppercase << std::setw(4)
              << std::setfill('0') << reg << std::dec << "=" << values[i] << "]";
  }
  std::cout << "\n";
}

void HoistHookCore::queryGpsInfo() {
  std::cout << "🛰️ GPS 功能按需求暂不启用，当前仅保留接口占位。\n";
}

void HoistHookCore::queryHookInfo(const std::string& info_type) {
  if (info_type == "speaker") {
    querySpeakerStatus();
  } else if (info_type == "light") {
    queryLightStatus();
  } else if (info_type == "rfid") {
    queryRfidInfo();
  } else if (info_type == "power") {
    queryPowerInfo();
  } else if (info_type == "gps") {
    queryGpsInfo();
  } else if (info_type == "all") {
    querySpeakerStatus();
    queryLightStatus();
    queryRfidInfo();
    queryPowerInfo();
    queryGpsInfo();
  } else {
    std::cout << "❌ 未知 info_type: " << info_type << "\n";
  }
}

}  // namespace hoist_hook
