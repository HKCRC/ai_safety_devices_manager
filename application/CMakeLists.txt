add_library(ai_safety_controller_application STATIC
  src/interface.cpp
)

target_include_directories(ai_safety_controller_application PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}/include"
  "${CMAKE_CURRENT_LIST_DIR}/../core/common/include"
  "${CMAKE_CURRENT_LIST_DIR}/../core/sensor_factory/include"
  "${CMAKE_CURRENT_LIST_DIR}/../core/drivers/battery/include"
  "${CMAKE_CURRENT_LIST_DIR}/../core/drivers/hoist_hook/include"
  "${CMAKE_CURRENT_LIST_DIR}/../core/drivers/io_relay/include"
  "${CMAKE_CURRENT_LIST_DIR}/../core/drivers/multi_turn_encoder/include"
  "${CMAKE_CURRENT_LIST_DIR}/../core/drivers/solar/include"
  "${CMAKE_CURRENT_LIST_DIR}/../core/drivers/spd_lidar/include"
  "${CMAKE_CURRENT_LIST_DIR}/../../modbus_devices/sensor_drivers/include"
  "${CMAKE_CURRENT_LIST_DIR}/../../modbus_devices/third_party_libraries/libmodbus"
)

target_link_libraries(ai_safety_controller_application PUBLIC
  asc_common
  asc_sensor_factory
)

if(TARGET asc_battery)
  target_link_libraries(ai_safety_controller_application PUBLIC asc_battery)
  target_compile_definitions(ai_safety_controller_application PUBLIC ASC_ENABLE_BATTERY)
endif()
if(TARGET asc_hoist_hook)
  target_link_libraries(ai_safety_controller_application PUBLIC asc_hoist_hook)
  target_compile_definitions(ai_safety_controller_application PUBLIC ASC_ENABLE_HOIST_HOOK)
endif()
if(TARGET asc_io_relay)
  target_link_libraries(ai_safety_controller_application PUBLIC asc_io_relay)
  target_compile_definitions(ai_safety_controller_application PUBLIC ASC_ENABLE_IO_RELAY)
endif()
if(TARGET asc_multi_turn_encoder)
  target_link_libraries(ai_safety_controller_application PUBLIC asc_multi_turn_encoder)
  target_compile_definitions(ai_safety_controller_application PUBLIC ASC_ENABLE_MULTI_TURN_ENCODER)
endif()
if(TARGET asc_solar)
  target_link_libraries(ai_safety_controller_application PUBLIC asc_solar)
  target_compile_definitions(ai_safety_controller_application PUBLIC ASC_ENABLE_SOLAR)
endif()
if(TARGET asc_spd_lidar)
  target_link_libraries(ai_safety_controller_application PUBLIC asc_spd_lidar)
  target_compile_definitions(ai_safety_controller_application PUBLIC ASC_ENABLE_SPD_LIDAR)
endif()
