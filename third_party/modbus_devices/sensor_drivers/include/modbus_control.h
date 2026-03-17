#ifndef MODBUS_CONTROL_H
#define MODBUS_CONTROL_H

#include "modbus.h"
#include <stdint.h>
#include <functional>
#include <mutex>
#include <string>
#include <unistd.h>

class ModbusControl {
public:
    // Function pointer types for connection methods
    using ConnectFunc = std::function<bool(modbus_t*)>;
    using DisconnectFunc = std::function<void(modbus_t*)>;

    // Constructors and destructor
    ModbusControl(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave);
    ModbusControl(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave, uint32_t to_sec, uint32_t to_usec);
    ModbusControl(const char* ip, int port, int slave);
    ModbusControl(const char* ip, int port, int slave, uint32_t to_sec, uint32_t to_usec);
    ModbusControl(const std::string& ip_address, int port, int slave_address = 1);
    ~ModbusControl();

    // Connection management
    bool connect();
    void disconnect();
    bool isConnected() const;

    // Modbus operations
    bool readCoils(int addr, int nb, uint8_t* dest);
    bool writeCoil(int addr, uint8_t value);
    bool writeMultipleCoils(int addr, int nb, const uint8_t* values);
    bool readDiscreteInputs(int addr, int nb, uint8_t* values);
    bool readHoldingRegisters(int addr, int nb, uint16_t* dest);
    bool readInputRegisters(int addr, int nb, uint16_t* dest);
    bool readInputRegisters(int addr, int nb, uint16_t* dest, double& time_buffer, double& duration_buffer);
    bool writeRegister(int addr, uint16_t value);
    bool writeRegisters(int addr, int nb, const uint16_t* values);
    bool readHoldingRegisters(int addr, int nb, uint16_t* dest, double& time_buffer, double& duration_buffer);
    bool writeHoldingRegisters(int addr, int nb, const uint16_t* data);
    void setDisableCerr(bool disable) { disable_cerr = disable; }

private:
    // Helper methods
    void setupRTUFunctions();
    void setupTCPFunctions();
    bool ensureConnection();
    void handleError(const char* context);
    void handleError(const std::string& message);

    // Member variables
    modbus_t* ctx = nullptr;
    bool connected = false;
    std::mutex mutex_;
    ConnectFunc connectFunc;
    DisconnectFunc disconnectFunc;
    std::string ip_address_;
    int port_;
    int slave_address_;
    std::string rtu_device_;
    bool disable_cerr = false;
};

#endif // MODBUS_CONTROL_H