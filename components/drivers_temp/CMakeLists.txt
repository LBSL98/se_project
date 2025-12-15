idf_component_register(
    SRCS "temp_driver.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer dht
)
