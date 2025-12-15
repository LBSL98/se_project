idf_component_register(
    SRCS "safety_supervisor.c"
    INCLUDE_DIRS "."
    REQUIRES freertos drivers_motors
)
