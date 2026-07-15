idf_component_register(
    SRCS
        "micro-ros-esp32.cpp"
        "esp32_serial_transport.c"
    INCLUDE_DIRS "."
    REQUIRES
        micro_ros_espidf_component
        esp_driver_uart
)
