#include "ai_safety_controller/sensor_factory/sensor_factory.hpp"

#include <algorithm>

namespace ai_safety_controller {

std::vector<std::string> SensorFactory::availableSensors() const {
  return {
      "battery",
      "hoist_hook",
      "io_relay",
      "multi_turn_encoder",
      "solar",
      "spd_lidar",
  };
}

bool SensorFactory::isSupported(const std::string& name) const {
  const auto sensors = availableSensors();
  return std::find(sensors.begin(), sensors.end(), name) != sensors.end();
}

}  // namespace ai_safety_controller
