idf_component_register(SRCS "flash.c" "vl53l0x.c" "main.c" "bluetooth.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver freertos ESP32-VL53L0X nvs_flash bt)