idf_component_register(
  SRCS "main.cpp"
  INCLUDE_DIRS "."
  REQUIRES driver esp_event esp_http_server esp_netif esp_timer esp_wifi nvs_flash
)
