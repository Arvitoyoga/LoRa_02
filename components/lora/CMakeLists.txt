idf_component_register(
    SRCS "src/lora.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi esp_driver_gpio
    PRIV_REQUIRES esp_timer heap log freertos
)
