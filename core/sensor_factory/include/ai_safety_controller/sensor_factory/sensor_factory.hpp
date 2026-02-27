#pragma once

#include <string>
#include <vector>

namespace ai_safety_controller {

class SensorFactory {
 public:
  std::vector<std::string> availableSensors() const;
  bool isSupported(const std::string& name) const;
};

}  // namespace ai_safety_controller
