set(motor_controller_sources "main.cpp")
set(motor_controller_requires driver esp_event esp_http_server esp_netif esp_timer esp_wifi nvs_flash)
if(DEFINED ENV{CLEANY_MICROROS_ENABLED} AND "$ENV{CLEANY_MICROROS_ENABLED}" STREQUAL "1")
  list(APPEND motor_controller_requires micro_ros_espidf_component)
endif()

idf_component_register(
  SRCS ${motor_controller_sources}
  INCLUDE_DIRS "."
  REQUIRES ${motor_controller_requires}
)
if(DEFINED ENV{CLEANY_MICROROS_ENABLED} AND "$ENV{CLEANY_MICROROS_ENABLED}" STREQUAL "1")
  target_compile_definitions(${COMPONENT_LIB} PRIVATE CLEANY_MICROROS_ENABLED=1)
endif()
