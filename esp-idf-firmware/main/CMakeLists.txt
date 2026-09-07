idf_component_register(SRCS "main.c" "mpu6500.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_driver_gpio esp_driver_ledc esp_timer u8g2)
