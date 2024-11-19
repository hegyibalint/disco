idf_component_register(
    SRCS "main.c"
    PRIV_REQUIRES driver nvs_flash esp_wifi esp_websocket_client
    INCLUDE_DIRS ""
)