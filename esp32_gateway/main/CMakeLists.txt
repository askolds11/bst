idf_component_register(
    SRCS
    "main.cpp"
    "sensor/aht10.cpp"
    "i2c/i2c.cpp"
    "nrf/nrf.cpp"
    "sd/sd.c"
    "sntp/sntp.c"
    INCLUDE_DIRS
    "."
    REQUIRES
    vfs
    esp_event
    nvs_flash
    protocol_examples_common
    esp_netif
    esp_timer
    nrf24l01
    fatfs
    sd_card
    esp_http_server
)