idf_component_register(
    SRCS "comm_blynk.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt comm_wifi drivers_temp drivers_sensors control drivers_motors
)
