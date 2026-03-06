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
