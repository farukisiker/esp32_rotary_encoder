idf_component_register(SRCS "pulse_counter.c" "app_main.c"
                            "app_uart.c"
                            "sick_od_mini.c"
                       PRIV_REQUIRES esp_driver_pcnt esp_driver_gpio esp_driver_uart 
                       INCLUDE_DIRS ".") 
