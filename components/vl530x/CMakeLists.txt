idf_component_register(SRCS "core/src/vl53l0x_api_calibration.c"
                            "core/src/vl53l0x_api_core.c"
                            "core/src/vl53l0x_api_ranging.c"
                            "core/src/vl53l0x_api_strings.c"
                            "core/src/vl53l0x_api.c"
                            "platform/src/vl53l0x_platform.c"
                            "platform/src/vl53l0x_esp32.c"
                    INCLUDE_DIRS "core/inc"
                                 "platform/inc"
                    REQUIRES driver esp_driver_gpio esp_driver_spi 
)
