cmake_minimum_required(VERSION 3.20.0)

get_filename_component(MY_SENSOR_MODULE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/.."
    ABSOLUTE
)

list(APPEND EXTRA_ZEPHYR_MODULES "${MY_SENSOR_MODULE_DIR}")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(my_sensor_sample)

target_sources(app PRIVATE src/main.c)
