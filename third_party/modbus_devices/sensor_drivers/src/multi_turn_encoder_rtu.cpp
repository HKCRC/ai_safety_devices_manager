#include "multi_turn_encoder_rtu.h"
#include <memory>  // Add this for std::make_unique
#include <iostream>
#include <iomanip>

MultiTurnEncoderRTU::MultiTurnEncoderRTU(const char* ip, int port, int slave)
    : modbus_(std::make_unique<ModbusControl>(ip, port, slave, 10,10)) {
}

MultiTurnEncoderRTU::MultiTurnEncoderRTU(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave)
    : modbus_(std::make_unique<ModbusControl>(device, baud, parity, data_bit, stop_bit, slave)) {
}

bool MultiTurnEncoderRTU::connect() {
    return modbus_->connect();
}

bool MultiTurnEncoderRTU::disconnect() {
    modbus_->disconnect();
    return true;
}

bool MultiTurnEncoderRTU::isConnected() const {
    return modbus_->isConnected();
}

bool MultiTurnEncoderRTU::readEncoderPosition(int32_t& position) {
    uint16_t registers[2];
    if (!modbus_->readHoldingRegisters(0x00, 2, registers)) {
        return false;
    }
    position = (static_cast<int32_t>(registers[0]) << 16) | registers[1];
    return true;
}

bool MultiTurnEncoderRTU::readEncoderNumberOfTurns(double& totalTurns, double& time_buffer, double& duration_buffer) {
    uint16_t registers[2];
    
    // Read registers with timing measurements
    if (!modbus_->readHoldingRegisters(0x02, 2, registers, time_buffer, duration_buffer)) {
        return false;
    }
    
    // Calculate total turns: whole turns + fractional turns
    totalTurns = static_cast<double>(registers[0]) + (static_cast<double>(registers[1]) / 8192.0);
    return true;
}

bool MultiTurnEncoderRTU::writeEncoderPosition(int32_t position) {
    uint16_t registers[2];
    registers[0] = static_cast<uint16_t>((position >> 16) & 0xFFFF);
    registers[1] = static_cast<uint16_t>(position & 0xFFFF);
    return modbus_->writeRegisters(0x4A, 2, registers);
}

bool MultiTurnEncoderRTU::readEncoderSettings(EncoderSettings& settings) {
    uint16_t registers[4];
    if (!modbus_->readHoldingRegisters(0x44, 4, registers)) {
        return false;
    }
    
    settings.deviceAddress = registers[0];
    settings.baudRate = static_cast<BaudRate>(registers[1]);
    settings.countingDirection = static_cast<CountingDirection>(registers[2]);
    settings.parityCheck = static_cast<ParityCheck>(registers[3]);
    return true;
}

bool MultiTurnEncoderRTU::write485DeviceAddress(uint16_t address) {
    return modbus_->writeRegister(0x44, address);
}

bool MultiTurnEncoderRTU::writeBaudRate(BaudRate baudRate) {
    return modbus_->writeRegister(0x45, static_cast<uint16_t>(baudRate));
}

bool MultiTurnEncoderRTU::writeCountingDirection(CountingDirection direction) {
    return modbus_->writeRegister(0x46, static_cast<uint16_t>(direction));
}

bool MultiTurnEncoderRTU::writeParityCheck(ParityCheck parityCheck) {
    return modbus_->writeRegister(0x47, static_cast<uint16_t>(parityCheck));
}

bool MultiTurnEncoderRTU::readRawSettings(uint16_t registers[4]) {
    return modbus_->readHoldingRegisters(0x44, 4, registers);
}

MultiTurnEncoderRTU::EncoderSettingsString MultiTurnEncoderRTU::getEncoderSettings() {
    EncoderSettingsString settings;
    uint16_t registers[4];
    
    if (!readRawSettings(registers)) {
        return {"Error", "Error", "Error", "Error"};
    }
    
    // Device Address
    settings.deviceAddress = std::to_string(registers[0]);
    
    // Baud Rate
    switch (registers[1]) {
        case 1: settings.baudRate = "4800 bps"; break;
        case 2: settings.baudRate = "9600 bps"; break;
        case 3: settings.baudRate = "19200 bps"; break;
        case 4: settings.baudRate = "38400 bps"; break;
        case 5: settings.baudRate = "57600 bps"; break;
        case 6: settings.baudRate = "76800 bps"; break;
        case 7: settings.baudRate = "115200 bps"; break;
        default: settings.baudRate = "Unknown"; break;
    }
    
    // Counting Direction
    settings.countingDirection = (registers[2] == 0) ? 
        "Clockwise data addition" : 
        "Counterclockwise data addition";
    
    // Parity Check
    switch (registers[3]) {
        case 1: settings.parityCheck = "No check"; break;
        case 2: settings.parityCheck = "Odd check"; break;
        case 3: settings.parityCheck = "Even check"; break;
        default: settings.parityCheck = "Unknown"; break;
    }
    
    return settings;
}


double MultiTurnEncoderRTU::computeVelocity() {
    if (data_filtered_history_.size() < history_size_) {
        return 0.0;
    } else {
        double dt = data_filtered_history_.back().timestamp - data_filtered_history_.front().timestamp;
        if (dt == 0.0) {
            return 0.0;
        }
        return (data_filtered_history_.back().value - data_filtered_history_.front().value) / dt;
    }
}

void MultiTurnEncoderRTU::updateEncoderData(const double& turns, const double& timestamp, const double& duration) {
    double turns_filtered;
    if (!data_valid_) {
        turns_filtered = turns;
        data_valid_ = true;
    } else {
        turns_filtered = (1 - alpha_) * data_.value + alpha_ * turns;
    }
    StampedDouble data_filtered = {timestamp, duration, turns_filtered};
    data_filtered_history_.push_back(data_filtered);
    if (data_filtered_history_.size() > history_size_) {
        data_filtered_history_.pop_front();
    }
    double velocity = computeVelocity();
    encoder_data_ = {timestamp, duration, turns_filtered, velocity};
}

void MultiTurnEncoderRTU::run_once() {
    double turns, timestamp, duration;
    if (readEncoderNumberOfTurns(turns, timestamp, duration)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        updateEncoderData(turns, timestamp, duration);
        data_ = {
            timestamp,
            duration,
            turns
        };
        
    }
}

void MultiTurnEncoderRTU::run() {
    if (running_thread_) {
        return;
    }
    
    running_thread_ = true;
    run_thread_ = std::thread([this]() {
        while (running_thread_) {
            run_once();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void MultiTurnEncoderRTU::stop() {
    running_thread_ = false;
    if (run_thread_.joinable()) {
        run_thread_.join();
    }
}

MultiTurnEncoderRTU::~MultiTurnEncoderRTU() {
    stop();
}