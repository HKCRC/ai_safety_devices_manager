#pragma once

#include <string>

namespace ai_safety_controller {

struct Status {
  bool ok = true;
  std::string message;
};

}  // namespace ai_safety_controller
