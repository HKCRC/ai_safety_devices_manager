#include "solar/solar_core.hpp"
#include "ai_safety_controller/common/gateway_serial.hpp"

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

namespace solar {

namespace {

uint16_t readBe16(const uint8_t* p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

}  // namespace

SolarCore::SolarCore() : SolarCore("192.168.1.12", 502, 3, 4) {}

SolarCore::SolarCore(const std::string& module_ip,
                     uint16_t module_port,
                     uint8_t module_slave_id,
                     uint8_t solar_slave_id)
    : module_ip_(module_ip),
      module_port_(module_port),
      module_slave_id_(module_slave_id),
      solar_slave_id_(solar_slave_id),
      transaction_id_(0x31A6),
      socket_fd_(-1),
      register_groups_({
          {0x2000, 0x200C, "只读", "开关量状态（超温、昼夜）"},
          {0x3000, 0x3010, "只读", "额定参数（阵列/电池/负载额定值）"},
          {0x3100, 0x311D, "只读", "实时参数（阵列/负载/温度/SOC等）"},
          {0x3200, 0x3202, "只读", "状态位（电池/充电/放电状态）"},
          {0x3302, 0x3313, "只读", "日电/月/年/总统计"},
          {0x331A, 0x331C, "只读", "电池电压/电流L/H"},
          {0x9000, 0x9070, "读/写混合", "蓄电池参数与管理参数"},
          {0x9013, 0x9015, "读/写混合", "实时时钟"},
          {0x9017, 0x9063, "读/写混合", "设备参数（温度阈值等）"},
          {0x901E, 0x9069, "读/写混合", "负载控制/光控/定时参数"},
          {0x0000, 0x000E, "线圈写", "开关量控制（05功能码）"},
      }) {}

SolarCore::~SolarCore() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  disconnectLocked();
}

bool SolarCore::parseNumber(const std::string& text, int* out) {
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

bool SolarCore::parseFunctionCode(const std::string& text,
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

std::vector<uint8_t> SolarCore::createModbusPacket(uint8_t function_code,
                                                   uint16_t address,
                                                   uint16_t value,
                                                   uint16_t quantity,
                                                   uint8_t unit_id,
                                                   bool* ok) {
  if (ok) *ok = false;
  if (function_code == 0x03 || function_code == 0x04) {
    transaction_id_ = static_cast<uint16_t>((transaction_id_ + 1) & 0xFFFF);
  }
  if (!(function_code == 0x03 || function_code == 0x04 || function_code == 0x05 ||
        function_code == 0x06)) {
    std::cout << "❌ 不支持的功能码\n";
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
  const uint16_t data = (function_code == 0x03 || function_code == 0x04) ? quantity : value;
  pkt.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
  pkt.push_back(static_cast<uint8_t>(data & 0xFF));
  if (ok) *ok = true;
  return pkt;
}

bool SolarCore::sendModbusPacket(const std::vector<uint8_t>& packet,
                                 std::vector<uint8_t>* response,
                                 const std::string& context,
                                 double timeout_sec) {
  if (!response) return false;
  response->clear();
  const std::string endpoint_key = module_ip_ + ":" + std::to_string(module_port_);
  ai_safety_controller::common::GatewaySerialGuard serial_guard(endpoint_key, 120);
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

bool SolarCore::ensureConnectionLocked(double timeout_sec) {
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

void SolarCore::disconnectLocked() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool SolarCore::sendAndReceiveLocked(const std::vector<uint8_t>& packet,
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

bool SolarCore::sendSolarRead(uint8_t function_code,
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
  ctx << "太阳能读寄存器 fc=0x" << std::hex << std::uppercase << static_cast<int>(function_code)
      << ", uid=" << std::dec << static_cast<int>(unit_id) << ", addr=0x" << std::hex
      << std::uppercase << address << ", qty=" << std::dec << quantity;
  return sendModbusPacket(pkt, response, ctx.str(), timeout_sec);
}

bool SolarCore::parseRegisterResponse(const std::vector<uint8_t>& response,
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
    std::cout << "❌ 太阳能返回错误，错误码：0x" << std::hex << std::uppercase
              << static_cast<int>(err) << std::dec << "\n";
    return false;
  }
  const uint8_t data_len = response[8];
  const size_t expected_len = 9 + data_len;
  if (response.size() != expected_len) {
    std::cout << "❌ 响应长度异常，预期" << expected_len << "字节，实际" << response.size()
              << "字节\n";
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

std::string SolarCore::describeSolarRegister(uint16_t addr) const {
  static const std::map<uint16_t, std::string> kMap = {
      {0x3100, "阵列电压（V/100）"},       {0x3101, "阵列电流（A/100）"},
      {0x3102, "发电功率L"},               {0x3103, "发电功率H"},
      {0x310C, "负载电压（V/100）"},       {0x310D, "负载电流（A/100）"},
      {0x310E, "负载功率L"},               {0x310F, "负载功率H"},
      {0x311A, "蓄电池剩余电量（%）"},     {0x3200, "蓄电池状态位"},
      {0x3201, "充电设备状态位"},          {0x3202, "放电设备状态位"},
      {0x331A, "蓄电池电压（V/100）"},     {0x331B, "蓄电池电流L"},
      {0x331C, "蓄电池电流H"},
  };
  std::map<uint16_t, std::string>::const_iterator it = kMap.find(addr);
  if (it != kMap.end()) return it->second;
  return "文档寄存器（未内置详细语义）";
}

int32_t SolarCore::parseSigned32FromLH(uint16_t low_word, uint16_t high_word) const {
  uint32_t raw = (static_cast<uint32_t>(high_word) << 16) | low_word;
  if (raw & 0x80000000u) {
    return static_cast<int32_t>(raw - 0x100000000ull);
  }
  return static_cast<int32_t>(raw);
}

void SolarCore::printRegisterGroups() const {
  std::cout << "\n📚 太阳能文档寄存器分组（可读可写范围）\n";
  for (size_t i = 0; i < register_groups_.size(); ++i) {
    const RegisterGroup& g = register_groups_[i];
    std::cout << "  0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
              << g.start << "~0x" << std::setw(4) << g.end << std::dec << " | " << g.rw
              << " | " << g.desc << "\n";
  }
}

bool SolarCore::confirmRiskyWrite(uint16_t addr) const {
  const bool risky = (addr == 0x000D || addr == 0x000E) || (addr >= 0x9000 && addr <= 0x9070);
  if (!risky) return true;
  std::cout << "⚠️  检测到高风险写入地址，可能导致设备参数变化。请输入 YES 确认继续写入：";
  std::string input;
  std::getline(std::cin, input);
  return input == "YES";
}

void SolarCore::genericRead(uint16_t address, uint16_t quantity, int function_code) {
  if (quantity < 1 || quantity > 125) {
    std::cout << "❌ 数量超限，读寄存器数量需在1~125\n";
    return;
  }
  int fc = (function_code < 0) ? 0x04 : function_code;
  if (!(fc == 0x03 || fc == 0x04)) {
    std::cout << "❌ 太阳能读取仅支持功能码 0x03/0x04\n";
    return;
  }
  std::vector<uint8_t> response;
  if (!sendSolarRead(static_cast<uint8_t>(fc), address, quantity, solar_slave_id_, &response)) {
    return;
  }
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, static_cast<uint8_t>(fc), quantity, &values)) return;
  std::cout << "✅ 太阳能寄存器读取结果（fc=0x" << std::hex << std::uppercase << fc << std::dec
            << "）\n";
  for (size_t i = 0; i < values.size(); ++i) {
    const uint16_t reg = static_cast<uint16_t>(address + i);
    std::cout << "  0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << reg
              << std::dec << " = " << values[i] << " (0x" << std::hex << std::uppercase
              << std::setw(4) << std::setfill('0') << values[i] << std::dec << ") | "
              << describeSolarRegister(reg) << "\n";
  }
}

void SolarCore::genericWrite(uint16_t address, uint16_t value, int function_code) {
  const int fc = (function_code < 0) ? 0x06 : function_code;
  if (!(fc == 0x05 || fc == 0x06)) {
    std::cout << "❌ 太阳能写入当前支持 0x05/0x06\n";
    return;
  }
  if (!confirmRiskyWrite(address)) {
    std::cout << "ℹ️ 已取消写入\n";
    return;
  }
  bool ok = false;
  const std::vector<uint8_t> packet = createModbusPacket(
      static_cast<uint8_t>(fc), address, value, 0, solar_slave_id_, &ok);
  if (!ok) return;
  std::vector<uint8_t> response;
  if (!sendModbusPacket(packet, &response, "太阳能写寄存器")) return;
  if (response == packet) {
    std::cout << "✅ 太阳能写入成功：0x" << std::hex << std::uppercase << std::setw(4)
              << std::setfill('0') << address << std::dec << " <= " << value << "\n";
  } else {
    std::cout << "⚠️ 写入响应异常\n";
  }
}

void SolarCore::querySolarInfo(const std::string& info_type) {
  std::cout << "\n📡 正在查询太阳能" << info_type << "信息...\n";
  if (solar_slave_id_ == module_slave_id_) {
    std::cout << "❌ 太阳能站号配置无效：与模块站号冲突\n";
    return;
  }

  if (info_type == "basic") {
    std::vector<uint8_t> pv_resp;
    std::vector<uint8_t> load_resp;
    std::vector<uint8_t> soc_resp;
    std::vector<uint8_t> batt_resp;
    sendSolarRead(0x04, 0x3100, 4, solar_slave_id_, &pv_resp);
    sendSolarRead(0x04, 0x310C, 4, solar_slave_id_, &load_resp);
    sendSolarRead(0x04, 0x311A, 1, solar_slave_id_, &soc_resp);
    sendSolarRead(0x04, 0x331A, 3, solar_slave_id_, &batt_resp);
    bool has_data = false;
    std::cout << "✅ 太阳能实时信息：\n";

    std::vector<uint16_t> values;
    if (parseRegisterResponse(pv_resp, 0x04, 4, &values)) {
      has_data = true;
      const double pv_voltage = values[0] / 100.0;
      const double pv_current = values[1] / 100.0;
      const double pv_power =
          ((static_cast<uint32_t>(values[3]) << 16) | values[2]) / 100.0;
      std::cout << "  光伏阵列电压: " << std::fixed << std::setprecision(2) << pv_voltage << "V\n";
      std::cout << "  光伏阵列电流: " << std::fixed << std::setprecision(2) << pv_current << "A\n";
      std::cout << "  光伏发电功率: " << std::fixed << std::setprecision(2) << pv_power << "W\n";
    }
    if (parseRegisterResponse(load_resp, 0x04, 4, &values)) {
      has_data = true;
      const double load_voltage = values[0] / 100.0;
      const double load_current = values[1] / 100.0;
      const double load_power =
          ((static_cast<uint32_t>(values[3]) << 16) | values[2]) / 100.0;
      std::cout << "  负载电压: " << std::fixed << std::setprecision(2) << load_voltage << "V\n";
      std::cout << "  负载电流: " << std::fixed << std::setprecision(2) << load_current << "A\n";
      std::cout << "  负载功率: " << std::fixed << std::setprecision(2) << load_power << "W\n";
    }
    if (parseRegisterResponse(soc_resp, 0x04, 1, &values)) {
      has_data = true;
      std::cout << "  蓄电池剩余电量: " << values[0] << "%\n";
    }
    if (parseRegisterResponse(batt_resp, 0x04, 3, &values)) {
      has_data = true;
      const double battery_voltage = values[0] / 100.0;
      const double battery_current = parseSigned32FromLH(values[1], values[2]) / 100.0;
      std::cout << "  蓄电池电压: " << std::fixed << std::setprecision(2) << battery_voltage << "V\n";
      std::cout << "  蓄电池电流: " << std::fixed << std::setprecision(2) << battery_current
                << "A（充电为正，放电为负）\n";
    }
    if (!has_data) {
      std::cout << "❌ 太阳能基础信息读取失败\n";
    }
  } else if (info_type == "status") {
    std::vector<uint8_t> pv_resp;
    std::vector<uint8_t> load_resp;
    if (!sendSolarRead(0x04, 0x3100, 4, solar_slave_id_, &pv_resp)) {
      std::cout << "❌ 太阳能状态信息读取失败：光伏阵列实时量读取失败\n";
      return;
    }
    if (!sendSolarRead(0x04, 0x310C, 4, solar_slave_id_, &load_resp)) {
      std::cout << "❌ 太阳能状态信息读取失败：负载实时量读取失败\n";
      return;
    }

    std::vector<uint16_t> pv_values;
    if (!parseRegisterResponse(pv_resp, 0x04, 4, &pv_values)) {
      std::cout << "❌ 太阳能状态信息解析失败：光伏阵列实时量\n";
      return;
    }
    std::vector<uint16_t> load_values;
    if (!parseRegisterResponse(load_resp, 0x04, 4, &load_values)) {
      std::cout << "❌ 太阳能状态信息解析失败：负载实时量\n";
      return;
    }

    const double pv_voltage = pv_values[0] / 100.0;
    const double pv_current = pv_values[1] / 100.0;
    const double pv_power =
        ((static_cast<uint32_t>(pv_values[3]) << 16) | pv_values[2]) / 100.0;
    const double load_voltage = load_values[0] / 100.0;
    const double load_current = load_values[1] / 100.0;
    const double load_power =
        ((static_cast<uint32_t>(load_values[3]) << 16) | load_values[2]) / 100.0;

    std::cout << "✅ 太阳能关键信息：\n";
    std::cout << "  光伏阵列电压: " << std::fixed << std::setprecision(2) << pv_voltage << "V\n";
    std::cout << "  光伏阵列电流: " << std::fixed << std::setprecision(2) << pv_current << "A\n";
    std::cout << "  光伏发电功率: " << std::fixed << std::setprecision(2) << pv_power << "W\n";
    std::cout << "  负载电压: " << std::fixed << std::setprecision(2) << load_voltage << "V\n";
    std::cout << "  负载电流: " << std::fixed << std::setprecision(2) << load_current << "A\n";
    std::cout << "  负载功率: " << std::fixed << std::setprecision(2) << load_power << "W\n";
  } else if (info_type == "all") {
    querySolarInfo("basic");
    querySolarInfo("status");
  } else {
    std::cout << "❌ 未知 info_type: " << info_type << "\n";
  }
}

void SolarCore::scanSolarSlaveIds(int start_id, int end_id) {
  if (start_id < 1 || end_id > 252 || start_id > end_id) {
    std::cout << "❌ 参数错误，示例：scan 或 scan 1 16\n";
    return;
  }
  std::cout << "\n🔎 扫描太阳能站号: " << start_id << "~" << end_id << "\n";
  std::vector<int> found;
  for (int uid = start_id; uid <= end_id; ++uid) {
    if (uid == module_slave_id_) continue;
    std::vector<uint8_t> response;
    if (!sendSolarRead(0x04, 0x3100, 1, static_cast<uint8_t>(uid), &response, 1.5)) continue;
    std::vector<uint16_t> values;
    if (!parseRegisterResponse(response, 0x04, 1, &values)) continue;
    std::cout << "✅ 站号" << uid << " 有响应，阵列电压=" << std::fixed << std::setprecision(2)
              << (values[0] / 100.0) << "V\n";
    found.push_back(uid);
  }
  if (found.empty()) {
    std::cout << "❌ 未发现可用太阳能从站\n";
  } else {
    std::cout << "🎯 可用太阳能站号: [";
    for (size_t i = 0; i < found.size(); ++i) {
      if (i > 0) std::cout << ", ";
      std::cout << found[i];
    }
    std::cout << "]\n";
  }
}

}  // namespace solar
