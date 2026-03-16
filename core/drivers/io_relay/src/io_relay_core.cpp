#include "io_relay/io_relay_core.hpp"
#include "ai_safety_controller/common/gateway_serial.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <random>
#include <thread>

namespace io_relay {

namespace {

int computeRetryDelayMs(const IoRelayCore::RetryPolicy& policy, int retry_index) {
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

IoRelayCore::IoRelayCore() : IoRelayCore("192.168.1.12", 502, 3, RetryPolicy()) {}

IoRelayCore::IoRelayCore(const std::string& module_ip,
                         uint16_t module_port,
                         uint8_t module_slave_id)
    : IoRelayCore(module_ip, module_port, module_slave_id, RetryPolicy()) {}

IoRelayCore::IoRelayCore(const std::string& module_ip,
                         uint16_t module_port,
                         uint8_t module_slave_id,
                         const RetryPolicy& retry_policy)
    : module_ip_(module_ip),
      module_port_(module_port),
      module_slave_id_(module_slave_id),
      transaction_id_(0x31A6),
      socket_fd_(-1),
      retry_policy_(retry_policy) {}

IoRelayCore::~IoRelayCore() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  disconnectLocked();
}

bool IoRelayCore::parseRelayNum(int relay_num, uint16_t* coil_addr) const {
  if (!coil_addr) return false;
  if (relay_num < 1 || relay_num > 16) return false;
  *coil_addr = static_cast<uint16_t>(relay_num - 1);  // 1->0x0000 ... 16->0x000F
  return true;
}

std::vector<uint8_t> IoRelayCore::createModbusPacket(uint8_t function_code,
                                                     uint16_t address,
                                                     uint16_t value,
                                                     uint16_t quantity,
                                                     uint8_t unit_id,
                                                     bool* ok) {
  if (ok) *ok = false;
  if (!(function_code == 0x01 || function_code == 0x05)) {
    std::cout << "[io_relay] ❌ 不支持的功能码\n";
    return {};
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

  const uint16_t data = (function_code == 0x05) ? value : quantity;
  pkt.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
  pkt.push_back(static_cast<uint8_t>(data & 0xFF));
  if (ok) *ok = true;
  return pkt;
}

bool IoRelayCore::sendModbusPacket(const std::vector<uint8_t>& packet,
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
          std::cout << "[io_relay] ⚠️ 第" << attempt << "/" << max_retries
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
    std::cout << "[io_relay] ❌ 重试耗尽，操作失败: " << context << "\n";
  }
  return false;
}

bool IoRelayCore::ensureConnectionLocked(double timeout_sec) {
  if (socket_fd_ >= 0) return true;

  socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    std::cout << "[io_relay] ❌ socket 创建失败: " << std::strerror(errno) << "\n";
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
    std::cout << "[io_relay] ❌ 模块IP无效: " << module_ip_ << "\n";
    disconnectLocked();
    return false;
  }
  if (::connect(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::cout << "[io_relay] ❌ 连接失败: " << std::strerror(errno) << "\n";
    disconnectLocked();
    return false;
  }
  return true;
}

void IoRelayCore::disconnectLocked() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool IoRelayCore::sendAndReceiveLocked(const std::vector<uint8_t>& packet,
                                       std::vector<uint8_t>* response,
                                       const std::string& context) {
  if (::send(socket_fd_, packet.data(), packet.size(), 0) < 0) {
    std::cout << "[io_relay] ❌ 发送失败: " << std::strerror(errno) << "\n";
    return false;
  }
  uint8_t buf[256];
  const ssize_t n = ::recv(socket_fd_, buf, sizeof(buf), 0);
  if (n <= 0) {
    std::cout << "[io_relay] ❌ 无响应: " << context << "\n";
    return false;
  }
  response->assign(buf, buf + n);
  return true;
}

void IoRelayCore::controlRelay(int relay_num, const std::string& status) {
  uint16_t coil_addr = 0;
  if (!parseRelayNum(relay_num, &coil_addr)) {
    std::cout << "[io_relay] ❌ 路数错误，仅支持1-16路\n";
    return;
  }
  if (!(status == "on" || status == "off")) {
    std::cout << "[io_relay] ❌ status 仅支持 on/off\n";
    return;
  }

  const uint16_t value = (status == "on") ? 0xFF00 : 0x0000;
  bool ok = false;
  const std::vector<uint8_t> packet =
      createModbusPacket(0x05, coil_addr, value, 0, module_slave_id_, &ok);
  if (!ok) return;

  std::vector<uint8_t> response;
  if (!sendModbusPacket(packet, &response, "继电器控制")) return;

  if (response == packet) {
    std::cout << "[io_relay] ✅ 第" << relay_num << "路继电器已" << (status == "on" ? "吸合" : "断开") << "\n";
  } else {
    std::cout << "[io_relay] ⚠️ 模块应答异常，响应长度=" << response.size() << "\n";
  }
}

void IoRelayCore::readRelayStatus(int relay_num) {
  bool ok = false;
  std::vector<uint8_t> packet;

  if (relay_num > 0) {
    uint16_t addr = 0;
    if (!parseRelayNum(relay_num, &addr)) {
      std::cout << "[io_relay] ❌ 路数错误，仅支持1-16路\n";
      return;
    }
    packet = createModbusPacket(0x01, addr, 0, 1, module_slave_id_, &ok);
  } else {
    packet = createModbusPacket(0x01, 0x0000, 0, 16, module_slave_id_, &ok);
  }
  if (!ok) return;

  std::vector<uint8_t> response;
  if (!sendModbusPacket(packet, &response, "继电器状态读取")) return;

  if (response.size() < 10) {
    std::cout << "[io_relay] ❌ 继电器状态响应长度异常\n";
    return;
  }
  if (response[7] != 0x01) {
    std::cout << "[io_relay] ❌ 继电器读取功能码异常: 0x" << std::hex << static_cast<int>(response[7])
              << std::dec << "\n";
    return;
  }

  const uint8_t byte_count = response[8];
  if (response.size() < static_cast<size_t>(9 + byte_count)) {
    std::cout << "[io_relay] ❌ 继电器状态数据长度异常\n";
    return;
  }

  if (relay_num > 0) {
    const bool on = (response[9] & 0x01) != 0;
    std::cout << "[io_relay] 📌 第" << relay_num << "路继电器状态：" << (on ? "吸合" : "断开") << "\n";
    return;
  }

  std::cout << "\n[io_relay] 📌 所有继电器状态：\n";
  for (int i = 1; i <= 16; ++i) {
    const int byte_idx = (i - 1) / 8;
    const int bit_idx = (i - 1) % 8;
    const bool on = ((response[9 + byte_idx] >> bit_idx) & 0x1) != 0;
    std::cout << "  第" << i << "路：" << (on ? "吸合" : "断开") << "\n";
  }
}

}  // namespace io_relay
