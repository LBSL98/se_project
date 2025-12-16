idf_component_register(
    SRCS "ultrasonic_driver.c"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_timer esp_driver_gpio
)
