#pragma once

#include <cstdint>

namespace ai_safety_controller {

struct DeviceStatus {
  enum class SolarChargeState : std::uint8_t {
    Unknown = 0,
    NotCharging,
    Charging,
    Fault
  };

  enum class EquipmentState : std::uint8_t {
    Unknown = 0,
    Offline,
    Standby,
    Active
  };

  struct BatteryInfo {
    std::uint8_t percent = 0;
    std::uint32_t remainingMin = 0;
    bool isCharging = false;
    std::uint32_t chargingTimeMin = 0;
    float voltageV = 0.0f;
    float currentA = 0.0f;
  };

  SolarChargeState solarCharge = SolarChargeState::Unknown;

  EquipmentState trolleyState = EquipmentState::Unknown;
  BatteryInfo trolleyBattery;

  EquipmentState hookState = EquipmentState::Unknown;
  BatteryInfo hookBattery;
};

}  // namespace ai_safety_controller
