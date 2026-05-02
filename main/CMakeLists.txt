idf_component_register(SRCS "main.c" "strategies.c"
                    INCLUDE_DIRS "." "include"
                    REQUIRES vl530x MMA845X motor_control esp_wifi nvs_flash esp_timer soc) 
