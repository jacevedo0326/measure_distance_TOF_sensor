idf_component_register(SRCS "main_mpu.c" "mpu6500.c" "flash.c" "vl53l0x.c" "bluetooth.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver freertos ESP32-VL53L0X nvs_flash bt)