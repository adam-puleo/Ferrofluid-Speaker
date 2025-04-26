idf_component_register(SRCS "Ferrofluid_Speaker.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c
                                esp_driver_gpio
                                esp_timer
                                arduino-audio-tools
                                ESP32-A2DP
                                STUSB4500
                                TAS5805M
                    )
