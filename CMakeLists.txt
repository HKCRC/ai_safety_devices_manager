cmake_minimum_required(VERSION 3.10)
project(ai_safety_controller_sdk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build options default to config/common_config.json -> build.ENABLE_* when present.
# You can still override any option with -DENABLE_*=ON/OFF at configure time.
set(ASC_COMMON_CONFIG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config/common_config.json")
set(ASC_COMMON_CONFIG_TEXT "")
if(EXISTS "${ASC_COMMON_CONFIG_PATH}")
  file(READ "${ASC_COMMON_CONFIG_PATH}" ASC_COMMON_CONFIG_TEXT)
endif()

function(asc_config_build_flag_default key out_var fallback_value)
  set(_value "${fallback_value}")
  if(NOT "${ASC_COMMON_CONFIG_TEXT}" STREQUAL "")
    string(REGEX MATCH "\"${key}\"[ \t\r\n]*:[ \t\r\n]*(true|false)"
           _match "${ASC_COMMON_CONFIG_TEXT}")
    if(CMAKE_MATCH_1 STREQUAL "true")
      set(_value ON)
    elseif(CMAKE_MATCH_1 STREQUAL "false")
      set(_value OFF)
    endif()
  endif()
  set(${out_var} "${_value}" PARENT_SCOPE)
endfunction()

asc_config_build_flag_default("ENABLE_BATTERY" ASC_ENABLE_BATTERY_DEFAULT ON)
asc_config_build_flag_default("ENABLE_HOIST_HOOK" ASC_ENABLE_HOIST_HOOK_DEFAULT ON)
asc_config_build_flag_default("ENABLE_IO_RELAY" ASC_ENABLE_IO_RELAY_DEFAULT ON)
asc_config_build_flag_default("ENABLE_MULTI_TURN_ENCODER" ASC_ENABLE_MULTI_TURN_ENCODER_DEFAULT ON)
asc_config_build_flag_default("ENABLE_SOLAR" ASC_ENABLE_SOLAR_DEFAULT ON)
asc_config_build_flag_default("ENABLE_SPD_LIDAR" ASC_ENABLE_SPD_LIDAR_DEFAULT ON)

option(ENABLE_BATTERY "Enable battery driver" ${ASC_ENABLE_BATTERY_DEFAULT})
option(ENABLE_HOIST_HOOK "Enable hoist hook driver" ${ASC_ENABLE_HOIST_HOOK_DEFAULT})
option(ENABLE_IO_RELAY "Enable io relay driver" ${ASC_ENABLE_IO_RELAY_DEFAULT})
option(ENABLE_MULTI_TURN_ENCODER "Enable multi turn encoder driver" ${ASC_ENABLE_MULTI_TURN_ENCODER_DEFAULT})
option(ENABLE_SOLAR "Enable solar driver" ${ASC_ENABLE_SOLAR_DEFAULT})
option(ENABLE_SPD_LIDAR "Enable SPD lidar driver" ${ASC_ENABLE_SPD_LIDAR_DEFAULT})

# Keep CMake cache aligned with config/common_config.json on every configure.
set(ENABLE_BATTERY "${ASC_ENABLE_BATTERY_DEFAULT}" CACHE BOOL "Enable battery driver" FORCE)
set(ENABLE_HOIST_HOOK "${ASC_ENABLE_HOIST_HOOK_DEFAULT}" CACHE BOOL "Enable hoist hook driver" FORCE)
set(ENABLE_IO_RELAY "${ASC_ENABLE_IO_RELAY_DEFAULT}" CACHE BOOL "Enable io relay driver" FORCE)
set(ENABLE_MULTI_TURN_ENCODER "${ASC_ENABLE_MULTI_TURN_ENCODER_DEFAULT}" CACHE BOOL "Enable multi turn encoder driver" FORCE)
set(ENABLE_SOLAR "${ASC_ENABLE_SOLAR_DEFAULT}" CACHE BOOL "Enable solar driver" FORCE)
set(ENABLE_SPD_LIDAR "${ASC_ENABLE_SPD_LIDAR_DEFAULT}" CACHE BOOL "Enable SPD lidar driver" FORCE)

message(STATUS
  "ai_safety_devices_manager build flags: "
  "ENABLE_BATTERY=${ENABLE_BATTERY}, "
  "ENABLE_HOIST_HOOK=${ENABLE_HOIST_HOOK}, "
  "ENABLE_IO_RELAY=${ENABLE_IO_RELAY}, "
  "ENABLE_MULTI_TURN_ENCODER=${ENABLE_MULTI_TURN_ENCODER}, "
  "ENABLE_SOLAR=${ENABLE_SOLAR}, "
  "ENABLE_SPD_LIDAR=${ENABLE_SPD_LIDAR}")

# 依赖 ai_safety_common（DeviceStatus 等）：若父工程已 add_subdirectory 则复用，否则从 ../ai_safety_common 拉入
# 构建请在本目录内进行：mkdir build && cd build && cmake .. && make
if(NOT TARGET ai_safety_common)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../ai_safety_common/CMakeLists.txt")
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ai_safety_common ${CMAKE_BINARY_DIR}/ai_safety_common)
  else()
    message(FATAL_ERROR "ai_safety_common not found. Put ai_safety_common next to this project (../ai_safety_common) or build from repo root.")
  endif()
endif()

find_package(Threads REQUIRED)
find_package(Boost REQUIRED)

add_subdirectory(core)
add_subdirectory(application)
add_subdirectory(demo)

# main_test：仅依赖 ai_safety_common + 本模块，在本目录 build 下构建
add_executable(main_test main_test.cpp)
target_link_libraries(main_test PRIVATE
  Threads::Threads
  Boost::boost
  ai_safety_common
  ai_safety_controller_application
)
target_include_directories(main_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

install(
  DIRECTORY config/
  DESTINATION config
)
