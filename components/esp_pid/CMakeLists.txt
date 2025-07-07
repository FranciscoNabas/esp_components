idf_component_register(
    SRCS
        "esp_pid.cpp"
    
    INCLUDE_DIRS
        "include"

    REQUIRES
        "esp_timer"
)