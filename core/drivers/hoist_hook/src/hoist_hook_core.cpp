#include "hoist_hook/hoist_hook_core.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
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

uint16_t HoistHookCore::crc16Modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int j = 0; j < 8; ++j) {
    if (crc & 1)
      crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
    else
      crc = static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

HoistHookCore::HoistHookCore() : HoistHookCore("192.168.1.12", 502, 0x03, 0x04) {}

HoistHookCore::HoistHookCore(const std::string& module_ip,
                             uint16_t module_port,
                             uint8_t hook_slave_id,
                             uint8_t power_slave_id)
    : transport_(Transport::TCP),
      module_ip_(module_ip),
      module_port_(module_port),
      device_(),
      baud_(9600),
      parity_('N'),
      data_bit_(8),
      stop_bit_(1),
      hook_slave_id_(hook_slave_id),
      power_slave_id_(power_slave_id),
      transaction_id_(0x31A6),
      socket_fd_(-1),
      serial_fd_(-1),
      print_enabled_(true),
      register_groups_({
          {0x0000, 0x0063, "读/写混合", "指令寄存器（0~99）"},
          {0x0064, 0x00C7, "只读", "状态寄存器（100~199）"},
      }) {}

HoistHookCore::HoistHookCore(const std::string& device,
                             int baud,
                             char parity,
                             int data_bit,
                             int stop_bit,
                             uint8_t hook_slave_id,
                             uint8_t power_slave_id)
    : transport_(Transport::RTU),
      module_ip_(),
      module_port_(0),
      device_(device),
      baud_(baud),
      parity_(parity),
      data_bit_(data_bit),
      stop_bit_(stop_bit),
      hook_slave_id_(hook_slave_id),
      power_slave_id_(power_slave_id),
      transaction_id_(0x31A6),
      socket_fd_(-1),
      serial_fd_(-1),
      print_enabled_(true),
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

  const uint16_t data = (function_code == 0x06) ? value : quantity;

  if (transport_ == Transport::RTU) {
    std::vector<uint8_t> pkt;
    pkt.reserve(8);
    pkt.push_back(unit_id);
    pkt.push_back(function_code);
    pkt.push_back(static_cast<uint8_t>((address >> 8) & 0xFF));
    pkt.push_back(static_cast<uint8_t>(address & 0xFF));
    pkt.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
    pkt.push_back(static_cast<uint8_t>(data & 0xFF));
    const uint16_t crc = crc16Modbus(pkt.data(), pkt.size());
    pkt.push_back(static_cast<uint8_t>(crc & 0xFF));
    pkt.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    if (ok) *ok = true;
    return pkt;
  }

  transaction_id_ = static_cast<uint16_t>((transaction_id_ + 1) & 0xFFFF);
  const uint16_t protocol_id = 0x0000;
  const uint16_t length = 6;

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
  (void)timeout_sec;
  if (transport_ == Transport::RTU) {
    if (serial_fd_ >= 0) return true;
    serial_fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      std::cout << "❌ 串口打开失败: " << device_ << " " << std::strerror(errno) << "\n";
      return false;
    }
    speed_t speed = B9600;
    if (baud_ == 19200) speed = B19200;
    else if (baud_ == 38400) speed = B38400;
    else if (baud_ == 57600) speed = B57600;
    else if (baud_ == 115200) speed = B115200;
    else if (baud_ != 9600) {
      std::cout << "❌ 不支持的波特率: " << baud_ << "\n";
      ::close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    struct termios tio;
    if (::tcgetattr(serial_fd_, &tio) != 0) {
      std::cout << "❌ tcgetattr 失败: " << std::strerror(errno) << "\n";
      ::close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    ::cfsetispeed(&tio, speed);
    ::cfsetospeed(&tio, speed);
    tio.c_cflag &= ~(PARENB | PARODD | CMSPAR);
    tio.c_cflag |= (CLOCAL | CREAD);
    if (parity_ == 'E' || parity_ == 'e') tio.c_cflag |= PARENB;
    else if (parity_ == 'O' || parity_ == 'o') tio.c_cflag |= (PARENB | PARODD);
    tio.c_cflag &= ~CSIZE;
    if (data_bit_ == 7) tio.c_cflag |= CS7;
    else tio.c_cflag |= CS8;
    if (stop_bit_ == 2) tio.c_cflag |= CSTOPB;
    else tio.c_cflag &= ~CSTOPB;
    tio.c_lflag = 0;
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 10;
    if (::tcsetattr(serial_fd_, TCSANOW, &tio) != 0) {
      std::cout << "❌ tcsetattr 失败: " << std::strerror(errno) << "\n";
      ::close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    return true;
  }

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
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool HoistHookCore::sendAndReceiveLocked(const std::vector<uint8_t>& packet,
                                         std::vector<uint8_t>* response,
                                         const std::string& context) {
  if (transport_ == Transport::RTU) {
    if (::write(serial_fd_, packet.data(), packet.size()) != static_cast<ssize_t>(packet.size())) {
      std::cout << "❌ 串口发送失败: " << std::strerror(errno) << "\n";
      return false;
    }
    response->clear();
    uint8_t buf[256];
    const int total_timeout_ms = 500;
    const int chunk_ms = 50;
    int elapsed = 0;
    while (elapsed < total_timeout_ms) {
      const ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
      if (n > 0) {
        response->insert(response->end(), buf, buf + n);
        if (response->size() >= 5) {
          const uint8_t byte_count = (*response)[2];
          const size_t expected = static_cast<size_t>(3) + byte_count + 2;
          if (response->size() >= expected) break;
        }
      }
      usleep(static_cast<useconds_t>(chunk_ms) * 1000);
      elapsed += chunk_ms;
    }
    if (response->empty()) {
      std::cout << "❌ 无响应: " << context << "\n";
      return false;
    }
    return true;
  }
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

  if (transport_ == Transport::RTU) {
    if (response.size() < 5) {
      std::cout << "❌ RTU 响应过短\n";
      return false;
    }
    const uint8_t recv_fc = response[1];
    if (recv_fc != function_code) {
      const uint8_t err = (recv_fc & 0x80) && response.size() > 2 ? response[2] : 0;
      std::cout << "❌ 设备返回错误，错误码：0x" << std::hex << std::uppercase
                << static_cast<int>(err) << std::dec << "\n";
      return false;
    }
    const uint8_t data_len = response[2];
    const size_t payload_end = static_cast<size_t>(3) + data_len;
    if (response.size() < payload_end + 2) {
      std::cout << "❌ RTU 响应长度异常\n";
      return false;
    }
    const uint16_t crc_recv = static_cast<uint16_t>(response[payload_end])
                             | (static_cast<uint16_t>(response[payload_end + 1]) << 8);
    const uint16_t crc_calc = crc16Modbus(response.data(), payload_end);
    if (crc_calc != crc_recv) {
      std::cout << "❌ RTU CRC 校验失败\n";
      return false;
    }
    if (data_len < quantity * 2) {
      std::cout << "❌ 数据长度不足\n";
      return false;
    }
    for (uint16_t i = 0; i < quantity; ++i) {
      const size_t base = 3 + i * 2;
      values->push_back(readBe16(&response[base]));
    }
    return true;
  }

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
      {0x0000, "爆闪灯/警示灯控制位（1开/0关）"},
      {0x0001, "7m喇叭控制位（写1触发7m语音）"},
      {0x0002, "3m喇叭控制位（写1触发3m语音）"},
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
  if (!print_enabled_) return;
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

  if (print_enabled_) {
    std::cout << "✅ 吊钩寄存器读取结果\n";
    for (size_t i = 0; i < values.size(); ++i) {
      const uint16_t reg = static_cast<uint16_t>(address + i);
      std::cout << "  0x" << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << reg
                << std::dec << " = " << values[i] << " (0x" << std::hex << std::uppercase
                << std::setw(4) << std::setfill('0') << values[i] << std::dec << ") | "
                << describeRegister(reg) << "\n";
    }
  }
}

void HoistHookCore::genericWrite(uint16_t address, uint16_t value, int function_code, bool skip_confirm) {
  const int fc = (function_code < 0) ? 0x06 : function_code;
  if (fc != 0x06) {
    std::cout << "❌ 当前仅支持 0x06 写入\n";
    return;
  }
  if (!skip_confirm && !confirmRiskyWrite(address)) {
    std::cout << "ℹ️ 已取消写入\n";
    return;
  }

  bool ok = false;
  const std::vector<uint8_t> packet =
      createModbusPacket(static_cast<uint8_t>(fc), address, value, 0, hook_slave_id_, &ok);
  if (!ok) return;

  std::vector<uint8_t> response;
  if (!sendModbusPacket(packet, &response, "吊钩写寄存器")) return;
  if (print_enabled_) {
    if (response == packet) {
      std::cout << "✅ 写入成功：0x" << std::hex << std::uppercase << std::setw(4)
                << std::setfill('0') << address << std::dec << " <= " << value << "\n";
    } else {
      std::cout << "⚠️ 写入响应异常\n";
    }
  }
}

void HoistHookCore::controlSpeaker(const std::string& mode) {
  // 文档：DEC1=7m, DEC2=3m，为独立寄存器，写1触发对应语音
  uint16_t v7 = 0;
  uint16_t v3 = 0;
  if (mode == "off") {
    v7 = 0;
    v3 = 0;
  } else if (mode == "7m") {
    v7 = 1;
    v3 = 0;
  } else if (mode == "3m") {
    v7 = 0;
    v3 = 1;
  } else if (mode == "both") {
    v7 = 1;
    v3 = 1;
  } else {
    std::cout << "❌ speaker 模式仅支持 off/7m/3m/both\n";
    return;
  }

  // 交互控制类命令在 print_status=false 时也需要有回显
  std::cout << "🔊 设置喇叭模式: " << mode << "\n";
  // 使用 skip_confirm 避免 off 时写两格弹出两次 YES，只生效第一次导致喇叭停不下来
  if (v7 != 0 || mode == "off") {
    genericWrite(0x0001, v7, 0x06, true);
  }
  if (v3 != 0 || mode == "off") {
    genericWrite(0x0002, v3, 0x06, true);
  }
}

void HoistHookCore::controlWarningLight(const std::string& status) {
  if (!(status == "on" || status == "off")) {
    std::cout << "❌ light 状态仅支持 on/off\n";
    return;
  }
  const uint16_t value = (status == "on") ? 1 : 0;
  // 交互控制类命令在 print_status=false 时也需要有回显
  std::cout << "🚨 设置警示灯: " << status << "\n";
  // 文档：爆闪灯控制寄存器在指令区地址 0，对应 DEC 0；交互控制免确认
  genericWrite(0x0000, value, 0x06, true);
}

void HoistHookCore::querySpeakerStatus() {
  std::vector<uint8_t> response;
  if (!sendRead(0x03, 0x0001, 2, hook_slave_id_, &response)) return;
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 2, &values)) return;
  if (!print_enabled_) return;
  const uint16_t v7 = values[0];
  const uint16_t v3 = values[1];
  const bool m7 = (v7 & 0x0001) != 0;
  const bool m3 = (v3 & 0x0001) != 0;
  std::cout << "✅ 喇叭状态寄存器(0x0001)=0x" << std::hex << std::uppercase << v7 << std::dec
            << " (7m)\n";
  std::cout << "✅ 喇叭状态寄存器(0x0002)=0x" << std::hex << std::uppercase << v3 << std::dec
            << " (3m)\n";
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
  if (!sendRead(0x03, 0x0000, 1, hook_slave_id_, &response)) return;
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 1, &values)) return;
  if (!print_enabled_) return;
  const uint16_t v = values[0];
  const bool on = (v & 0x0001) != 0;
  std::cout << "✅ 警示灯状态: " << (on ? "开启" : "关闭")
            << " (reg=0x0000, raw=0x" << std::hex << std::uppercase << v << std::dec << ")\n";
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

  if (!print_enabled_) return;
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
  if (print_enabled_) std::cout << "🔋 正在读取吊钩状态（灯/喇叭/电池/心跳/工作模式）...\n";
  std::vector<uint8_t> response;
  // 文档：状态寄存器 100~106 在吊钩从站(hook_slave_id)上，地址 0x0064 起共 7 个
  if (!sendRead(0x03, 0x0064, 7, hook_slave_id_, &response)) {
    std::cout << "⚠️ 吊钩状态读取失败，可使用 get 命令手动排查（吊钩从站 " << static_cast<int>(hook_slave_id_)
              << " 地址 0x64 数量 7）\n";
    return;
  }
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 7, &values)) {
    std::cout << "⚠️ 吊钩状态响应解析失败\n";
    return;
  }

  if (!print_enabled_) return;
  const uint16_t light_status = values[0];   // 100: 记录爆闪灯状态 开1/关0
  const uint16_t horn_status = values[1];    // 101: 记录喇叭状态 开1/关0
  const uint16_t battery_raw = values[2];    // 102: 电池电量 0~10000 -> 0~100%
  const uint16_t volume = values[3];        // 103: 设置音量 0~30
  const uint16_t heartbeat = values[4];     // 104: 心跳 0~65535
  const uint16_t remain_min = values[5];    // 105: 剩余放电时间(分钟)
  const uint16_t work_mode = values[6];      // 106: 工作模式 0待机 1工作 2低电量

  const double battery_percent = (battery_raw <= 10000) ? (battery_raw / 100.0) : 0.0;

  std::cout << "✅ 吊钩状态（寄存器 100~106）\n";
  std::cout << "  爆闪灯状态: " << (light_status ? "开" : "关") << " (reg100=" << light_status << ")\n";
  std::cout << "  喇叭状态: " << (horn_status ? "开" : "关") << " (reg101=" << horn_status << ")\n";
  std::cout << "  电池电量: " << std::fixed << std::setprecision(2) << battery_percent << "% (0~10000 raw=" << battery_raw << ")\n";
  std::cout << "  音量: " << volume << " (0~30)\n";
  std::cout << "  心跳: " << heartbeat << "\n";
  std::cout << "  剩余放电时间: " << remain_min << " 分钟\n";
  const char* mode_str = (work_mode == 0) ? "待机" : (work_mode == 1) ? "工作" : (work_mode == 2) ? "低电量(≤20%)" : "未知";
  std::cout << "  工作模式: " << mode_str << " (reg106=" << work_mode << ")\n";
}

void HoistHookCore::queryHeartbeat() {
  std::vector<uint8_t> response;
  // 心跳寄存器：DEC 104 -> 地址 0x0068
  if (!sendRead(0x03, 0x0068, 1, hook_slave_id_, &response)) return;
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 1, &values) || values.empty()) return;
  std::cout << "✅ 吊钩心跳: " << values[0] << " (reg104)\n";
}

void HoistHookCore::queryWorkMode() {
  std::vector<uint8_t> response;
  // 工作模式寄存器：DEC 106 -> 地址 0x006A
  if (!sendRead(0x03, 0x006A, 1, hook_slave_id_, &response)) return;
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 1, &values) || values.empty()) return;
  const uint16_t work_mode = values[0];
  const char* mode_str = (work_mode == 0) ? "待机"
                          : (work_mode == 1) ? "工作"
                          : (work_mode == 2) ? "低电量(≤20%)"
                                             : "未知";
  std::cout << "✅ 吊钩工作模式: " << mode_str << " (reg106=" << work_mode << ")\n";
}

bool HoistHookCore::readPowerSummary(PowerSummary* out, double timeout_sec) {
  if (!out) return false;
  *out = PowerSummary{};

  std::vector<uint8_t> response;
  // 状态寄存器 100~110 在吊钩从站(hook_slave_id)上，地址 0x0064 起共 11 个
  if (!sendRead(0x03, 0x0064, 11, hook_slave_id_, &response, timeout_sec)) {
    return false;
  }
  std::vector<uint16_t> values;
  if (!parseRegisterResponse(response, 0x03, 11, &values) || values.size() < 11) {
    return false;
  }

  const uint16_t battery_raw = values[2];   // 102: 电池电量 0~10000 -> 0~100%
  const uint16_t remain_min = values[5];    // 105: 剩余放电时间(分钟)
  const uint16_t charging_flag = values[7]; // 107: 是否在充电 0/1
  const uint16_t charge_min = values[8];    // 108: 充电剩余时间(分钟)
  const uint16_t voltage_raw = values[9];   // 109: 电压(0.01V)
  const uint16_t current_raw = values[10];  // 110: 电流(0.01A，有符号)

  PowerSummary s;
  if (battery_raw <= 10000) {
    s.battery_percent = static_cast<float>(battery_raw / 100.0);
  } else {
    s.battery_percent = 0.0f;
  }
  s.remaining_discharge_min = static_cast<std::uint32_t>(remain_min);
  // 107: 是否在充电
  s.is_charging = (charging_flag != 0);
  // 108: 充电剩余时间(分钟)，仅在正在充电时才使用
  s.remaining_charge_min = s.is_charging ? static_cast<std::uint32_t>(charge_min) : 0u;
  // 109: 电压(0.01V)
  s.voltage_v = static_cast<float>(voltage_raw) * 0.01f;
  // 110: 电流(0.01A)，有正负
  s.current_a = static_cast<float>(static_cast<int16_t>(current_raw)) * 0.01f;
  s.ok = true;
  *out = s;
  return true;
}

void HoistHookCore::queryGpsInfo() {
  if (print_enabled_) std::cout << "🛰️ GPS 功能按需求暂不启用，当前仅保留接口占位。\n";
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
  } else if (info_type == "heartbeat") {
    queryHeartbeat();
  } else if (info_type == "mode") {
    queryWorkMode();
  } else if (info_type == "gps") {
    queryGpsInfo();
  } else if (info_type == "all") {
    querySpeakerStatus();
    queryLightStatus();
    queryRfidInfo();
    queryPowerInfo();
    queryGpsInfo();
  } else if (print_enabled_) {
    std::cout << "❌ 未知 info_type: " << info_type << "\n";
  }
}

}  // namespace hoist_hook
