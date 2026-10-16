if(CONFIG_MY_BME280)
    zephyr_library()
    zephyr_library_sources(driver/my_sensor.c)
endif()
