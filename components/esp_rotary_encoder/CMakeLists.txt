idf_component_register(
    SRCS
        "esp_rotary_encoder.cpp"
        
    INCLUDE_DIRS
        "include"

    REQUIRES
        "driver"
        "esp_timer"
)