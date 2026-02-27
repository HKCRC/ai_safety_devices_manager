add_subdirectory(common)
add_subdirectory(sensor_factory)
set(ASC_COMMON_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/common/include")

set(ASC_BATTERY_TARGET "")
if(ENABLE_BATTERY)
  add_library(asc_battery STATIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/battery/src/battery_core.cpp"
  )
  target_include_directories(asc_battery PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/battery/include"
    "${ASC_COMMON_INCLUDE_DIR}"
  )
  set(ASC_BATTERY_TARGET asc_battery)
endif()

set(ASC_HOIST_HOOK_TARGET "")
if(ENABLE_HOIST_HOOK)
  add_library(asc_hoist_hook STATIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/hoist_hook/src/hoist_hook_core.cpp"
  )
  target_include_directories(asc_hoist_hook PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/hoist_hook/include"
    "${ASC_COMMON_INCLUDE_DIR}"
  )
  set(ASC_HOIST_HOOK_TARGET asc_hoist_hook)
endif()

set(ASC_IO_RELAY_TARGET "")
if(ENABLE_IO_RELAY)
  add_library(asc_io_relay STATIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/io_relay/src/io_relay_core.cpp"
  )
  target_include_directories(asc_io_relay PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/io_relay/include"
    "${ASC_COMMON_INCLUDE_DIR}"
  )
  set(ASC_IO_RELAY_TARGET asc_io_relay)
endif()

set(ASC_SOLAR_TARGET "")
if(ENABLE_SOLAR)
  add_library(asc_solar STATIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/solar/src/solar_core.cpp"
  )
  target_include_directories(asc_solar PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/solar/include"
    "${ASC_COMMON_INCLUDE_DIR}"
  )
  set(ASC_SOLAR_TARGET asc_solar)
endif()

set(ASC_SPD_LIDAR_TARGET "")
if(ENABLE_SPD_LIDAR)
  add_library(asc_spd_lidar STATIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/spd_lidar/src/spd_lidar_core.cpp"
  )
  target_include_directories(asc_spd_lidar PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/spd_lidar/include"
    "${ASC_COMMON_INCLUDE_DIR}"
  )
  set(ASC_SPD_LIDAR_TARGET asc_spd_lidar)
endif()

set(ASC_MULTI_TURN_ENCODER_TARGET "")
if(ENABLE_MULTI_TURN_ENCODER)
  set(MODBUS_DEVICES_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../modbus_devices")
  set(SENSOR_DRIVERS_INCLUDE_DIR "${MODBUS_DEVICES_DIR}/sensor_drivers/include")
  set(LIBMODBUS_DIR "${MODBUS_DEVICES_DIR}/third_party_libraries/libmodbus")

  add_library(asc_multi_turn_encoder STATIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/multi_turn_encoder/src/multi_turn_encoder_core.cpp"
    "${MODBUS_DEVICES_DIR}/sensor_drivers/src/multi_turn_encoder_rtu.cpp"
    "${MODBUS_DEVICES_DIR}/sensor_drivers/src/modbus_control.cpp"
    "${LIBMODBUS_DIR}/modbus.c"
    "${LIBMODBUS_DIR}/modbus-data.c"
    "${LIBMODBUS_DIR}/modbus-rtu.c"
    "${LIBMODBUS_DIR}/modbus-tcp.c"
  )
  target_include_directories(asc_multi_turn_encoder PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/drivers/multi_turn_encoder/include"
    "${SENSOR_DRIVERS_INCLUDE_DIR}"
    "${LIBMODBUS_DIR}"
    "${ASC_COMMON_INCLUDE_DIR}"
  )
  target_link_libraries(asc_multi_turn_encoder PUBLIC pthread)
  set(ASC_MULTI_TURN_ENCODER_TARGET asc_multi_turn_encoder)
endif()
