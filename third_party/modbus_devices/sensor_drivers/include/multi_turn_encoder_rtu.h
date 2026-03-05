#pragma once
#include "modbus_control.h"
#include "standard_msg.h"
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <iostream>
#include <deque>

class MultiTurnEncoderRTU {
public:
    // Enum for baud rate settings
    enum class BaudRate {
        BAUD_4800 = 1,
        BAUD_9600 = 2,
        BAUD_19200 = 3,
        BAUD_38400 = 4,
        BAUD_57600 = 5,
        BAUD_76800 = 6,
        BAUD_115200 = 7
    };

    // Enum for counting direction
    enum class CountingDirection {
        CLOCKWISE = 0,
        COUNTERCLOCKWISE = 1
    };

    // Enum for parity check
    enum class ParityCheck {
        NO_CHECK = 1,
        ODD_CHECK = 2,
        EVEN_CHECK = 3
    };

    struct EncoderSettings {
        uint16_t deviceAddress;
        BaudRate baudRate;
        CountingDirection countingDirection;
        ParityCheck parityCheck;
    };

    struct EncoderSettingsString {
        std::string deviceAddress;
        std::string baudRate;
        std::string countingDirection;
        std::string parityCheck;
    };

    struct StampedEncoderData {
        double timestamp;
        double time_variance;
        double value;
        double velocity;
        StampedEncoderData(double val = 0.0) : timestamp(0.0), time_variance(0.0), value(val), velocity(0.0) {}

        StampedEncoderData(double ts, double var, double val, double vel) 
            : timestamp(ts), time_variance(var), value(val), velocity(vel) {}
    };

    MultiTurnEncoderRTU(const char* ip, int port, int slave = 1);
    MultiTurnEncoderRTU(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave);
    ~MultiTurnEncoderRTU();

    bool connect();
    bool disconnect();
    bool isConnected() const;

    // Position reading/writing functions
    bool readEncoderPosition(int32_t& position);
    bool readEncoderNumberOfTurns(double& totalTurns, double& time_buffer, double& duration_buffer);
    bool writeEncoderPosition(int32_t position);

    // Settings reading/writing functions
    bool readEncoderSettings(EncoderSettings& settings);
    bool write485DeviceAddress(uint16_t address);
    bool writeBaudRate(BaudRate baudRate);
    bool writeCountingDirection(CountingDirection direction);
    bool writeParityCheck(ParityCheck parityCheck);
    bool readRawSettings(uint16_t registers[4]);
    EncoderSettingsString getEncoderSettings();

    double computeVelocity();
    void updateEncoderData(const double& turns, const double& timestamp, const double& duration);
    void run_once();
    void run();
    void stop();
    
    StampedDouble getData() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return data_;
    }

    double getDataTimestamp() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return data_.timestamp;
    }

    StampedEncoderData getEncoderData() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return encoder_data_;
    }

private:
    std::unique_ptr<ModbusControl> modbus_;
    std::thread run_thread_;
    bool running_thread_{false};
    mutable std::mutex data_mutex_;
    StampedDouble data_;
    StampedEncoderData encoder_data_;
    std::deque<StampedDouble> data_filtered_history_;
    int history_size_ = 100;
    double alpha_ = 0.95;
    bool data_valid_;
}; 