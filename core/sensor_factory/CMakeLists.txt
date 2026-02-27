add_library(asc_sensor_factory STATIC
  src/sensor_factory.cpp
)

target_include_directories(asc_sensor_factory PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}/include"
)
