cmake_minimum_required(VERSION 3.10)
project(ai_safety_controller_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ENABLE_BATTERY "Enable battery driver" ON)
option(ENABLE_HOIST_HOOK "Enable hoist hook driver" ON)
option(ENABLE_IO_RELAY "Enable io relay driver" ON)
option(ENABLE_MULTI_TURN_ENCODER "Enable multi turn encoder driver" ON)
option(ENABLE_SOLAR "Enable solar driver" ON)
option(ENABLE_SPD_LIDAR "Enable SPD lidar driver" ON)

# 使用 ai_safety_common 的 DeviceStatus，与主工程统一；若父级已 add 则复用
if(NOT TARGET ai_safety_common)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../ai_safety_common/CMakeLists.txt")
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ai_safety_common ${CMAKE_BINARY_DIR}/ai_safety_common)
  else()
    message(FATAL_ERROR "ai_safety_common not found. Build from repo root or set ai_safety_common path.")
  endif()
endif()

add_subdirectory(core)
add_subdirectory(application)
add_subdirectory(demo)

install(
  DIRECTORY config/
  DESTINATION config
)
