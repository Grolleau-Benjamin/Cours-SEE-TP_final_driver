cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(dummy_driver)

target_sources(app PRIVATE driver/my_sensor.c)