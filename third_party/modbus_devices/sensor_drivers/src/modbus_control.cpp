#include <iostream>
#include <string>
#include <memory>
#include <functional>
#include <mutex>
#include <ctime>
#include <cstring>
#include "modbus_control.h"
#include "modbus-private.h"


ModbusControl::ModbusControl(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave) {
    rtu_device_ = device ? device : "";
    ctx = modbus_new_rtu(device, baud, parity, data_bit, stop_bit);
    // ctx->debug = true;
    modbus_set_slave(ctx, slave);
    setupRTUFunctions();
    std::cout << "Modbus RTU instance:" << device << ":" << baud << ":" << parity << ":" << data_bit << ":" << stop_bit << ":" << slave << std::endl;
}

ModbusControl::ModbusControl(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave, uint32_t to_sec, uint32_t to_usec) {
    rtu_device_ = device ? device : "";
    ctx = modbus_new_rtu(device, baud, parity, data_bit, stop_bit);
    modbus_set_slave(ctx, slave);
    setupRTUFunctions();
    modbus_set_response_timeout(ctx,to_sec,to_usec);
    std::cout << "Modbus RTU instance:" << device << ":" << baud << ":" << parity << ":" << data_bit << ":" << stop_bit << ":" << slave << std::endl;
}

// Constructor for TCP
ModbusControl::ModbusControl(const char* ip, int port, int slave) {
    ctx = modbus_new_tcp(ip, port);
    modbus_set_slave(ctx, slave);
    setupTCPFunctions();
    std::cout << "Modbus TCP instance:" << ip << ":" << port << ":" << slave << std::endl;
}

// Constructor for TCP with time out
ModbusControl::ModbusControl(const char* ip, int port, int slave, uint32_t to_sec, uint32_t to_usec) {
    ctx = modbus_new_tcp(ip, port);
    modbus_set_slave(ctx, slave);
    setupTCPFunctions();
    modbus_set_response_timeout(ctx,to_sec,to_usec);
    std::cout << "Modbus TCP instance:" << ip << ":" << port << ":" << slave << std::endl;
    std::cout << "timeout time =" << to_sec << "s" << std::endl;
}

ModbusControl::~ModbusControl() {
    disconnect();
    if (ctx) {
        modbus_free(ctx);
    }
}

bool ModbusControl::connect() {
    if (!ctx) return false;
    return connectFunc(ctx);
}

void ModbusControl::disconnect() {
    if (ctx) {
        disconnectFunc(ctx);
        sleep(1); // Allow time for proper disconnection
    }
}

bool ModbusControl::isConnected() const {
    return connected;
}

// Read discrete inputs
bool ModbusControl::readDiscreteInputs(int addr, int nb, uint8_t* dest) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;
    
    if (modbus_read_input_bits(ctx, addr, nb, dest) == -1) {
        handleError("Read holding registers failed");
        return false;
    }
    return true;
}

// Read holding registers
bool ModbusControl::readHoldingRegisters(int addr, int nb, uint16_t* dest) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;
    
    if (modbus_read_registers(ctx, addr, nb, dest) == -1) {
        handleError("Read holding registers failed");
        return false;
    }
    return true;
}

// Read holding registers with timing measurements
bool ModbusControl::readHoldingRegisters(int addr, int nb, uint16_t* dest, double& time_buffer, double& duration_buffer) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;
    
    // get time using chrono
    auto start = std::chrono::high_resolution_clock::now();

    bool result = modbus_read_registers(ctx, addr, nb, dest) != -1;
    
    auto end = std::chrono::high_resolution_clock::now();
    
    // Calculate timestamps in seconds
    double start_time = std::chrono::duration_cast<std::chrono::seconds>(start.time_since_epoch()).count();
    double end_time = std::chrono::duration_cast<std::chrono::seconds>(end.time_since_epoch()).count();
    
    time_buffer = (start_time + end_time) / 2.0;  // Average timestamp
    duration_buffer = end_time - start_time;       // Duration
    
    if (!result) {
        handleError("Read holding registers failed");
        return false;
    }
    return true;
}

bool ModbusControl::writeHoldingRegisters(int addr, int nb, const uint16_t* data) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;

    // Clear errno before operation
    errno = 0;
    
    // Use modbus_write_registers for multiple register writes
    int result = modbus_write_registers(ctx, addr, nb, data);
    
    // Get error code immediately after operation
    int error_code = errno;
    
    if (result == -1) {
        std::cerr << "Write holding registers failed - Address: " << addr
                  << ", Count: " << nb
                  << ", Error: " << error_code << " (" << strerror(error_code) << ")"
                  << ", Modbus error: " << modbus_strerror(error_code)
                  << std::endl;
                  
        handleError("Write holding registers failed");
        return false;
    }
    
    // Optional debug output
    // std::cout << "Successfully wrote " << nb << " holding registers starting at address " << addr << std::endl;
    return true;
}

bool ModbusControl::readCoils(int addr, int nb, uint8_t* dest) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;
    if (modbus_read_bits(ctx, addr, nb, dest) == -1) {
        handleError("Read coils failed");
        return false;
    }
    return true;
}

bool ModbusControl::writeCoil(int addr, uint8_t value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;
    if (modbus_write_bit(ctx, addr, value) == -1) {
        handleError("Write coil failed");
        return false;
    }
    return true;
}

bool ModbusControl::writeMultipleCoils(int addr, int nb, const uint8_t* values) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;
    if (modbus_write_bits(ctx, addr, nb, values) == -1) {
        handleError("Write multiple coils failed");
        return false;
    }
    return true;
}

// Read input registers
bool ModbusControl::readInputRegisters(int addr, int nb, uint16_t* dest) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;
    if (modbus_read_input_registers(ctx, addr, nb, dest) == -1) {
        handleError("Read input registers failed");
        return false;
    }
    return true;
}

// Read input registers with timing measurements
bool ModbusControl::readInputRegisters(int addr, int nb, uint16_t* dest, double& time_buffer, double& duration_buffer) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;
    
    struct timespec start, end;
    clock_gettime(CLOCK_REALTIME, &start);
    
    bool result = modbus_read_input_registers(ctx, addr, nb, dest) != -1;
    
    clock_gettime(CLOCK_REALTIME, &end);
    
    // Calculate timestamps in seconds
    double start_time = start.tv_sec + (start.tv_nsec / 1e9);
    double end_time = end.tv_sec + (end.tv_nsec / 1e9);
    
    time_buffer = (start_time + end_time) / 2.0;  // Average timestamp
    duration_buffer = end_time - start_time;       // Duration
    
    if (!result) {
        handleError("Read input registers failed");
        return false;
    }
    return true;
}

// Write single register
bool ModbusControl::writeRegister(int addr, uint16_t value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;

    // Add more detailed debug logging
    // std::cout << "Writing to register - Address: " << addr 
    //           << ", Value: " << value 
    //           << " (0x" << std::hex << value << std::dec << ")" 
    //           << ", Connection status: " << (isConnected() ? "Connected" : "Disconnected")
    //           << std::endl;

    // Clear errno before the operation
    errno = 0;
    
    int result = modbus_write_register(ctx, addr, value);
    
    // Get errno immediately after the operation
    int error_code = errno;
    
    if (result == -1) {
        std::cout << "Write failed with errno: " << error_code 
                  << " - " << strerror(error_code)
                  << "\nModbus error: " << modbus_strerror(error_code) << std::endl;
                  
        // Print the context state
        std::cout << "Modbus context - "
                  << "Slave ID: " << static_cast<int>(ctx->slave)
                  << ", Connected: " << isConnected()
                  << std::endl;
                  
        handleError("Write register failed");
        return false;
    }
    
    // std::cout << "Write successful, bytes written: " << result << std::endl;
    return true;
}

// Write multiple registers
bool ModbusControl::writeRegisters(int addr, int nb, const uint16_t* values) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureConnection()) return false;

    if (modbus_write_registers(ctx, addr, nb, values) == -1) {
        handleError("Write registers failed");
        return false;
    }
    return true;
}

void ModbusControl::setupRTUFunctions() {
    // RTU specific connect/disconnect functions
    connectFunc = [this](modbus_t* ctx) {
        if (modbus_connect(ctx) == -1) {
            handleError("RTU connection failed");
            return false;
        }
        connected = true;
        return true;
    };

    disconnectFunc = [this](modbus_t* ctx) {
        modbus_close(ctx);
        connected = false;
    };
}

void ModbusControl::setupTCPFunctions() {
    // TCP specific connect/disconnect functions
    connectFunc = [this](modbus_t* ctx) {
        if (modbus_connect(ctx) == -1) {
            handleError("TCP connection failed");
            return false;
        }
        connected = true;
        return true;
    };

    disconnectFunc = [this](modbus_t* ctx) {
        modbus_close(ctx);
        connected = false;
    };
}

bool ModbusControl::ensureConnection() {
    if (!ctx) return false;
    if (!connected) {
        disconnect();
        std::cout << "ModbusControl::ensureConnection() reconnecting" << std::endl;
        return connect();
    }
    return true;
}

void ModbusControl::handleError(const char* context) {
    connected = false;
    int error_code = errno;
    if (disable_cerr) return;
    std::cerr << "Error: " << context;
    if (!rtu_device_.empty()) std::cerr << " (device: " << rtu_device_ << ")";
    std::cerr << " - System errno: " << error_code
              << " (" << strerror(error_code) << ")"
              << " - Modbus error: " << modbus_strerror(error_code)
              << std::endl;
}

