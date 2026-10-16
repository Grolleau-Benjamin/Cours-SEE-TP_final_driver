#ifndef MY_SENSOR_H_
#define MY_SENSOR_H_

#include <zephyr/drivers/i2c.h>

#include <stdint.h>
#include <zephyr/kernel.h>

// register
#define MYSENSOR_REG_PRESS_MSB 0xF7
#define MYSENSOR_REG_ID 0xD0
#define MYSENSOR_REG_COMP_START 0x88
#define MYSENSOR_REG_HUM_COMP_PART1 0xA1
#define MYSENSOR_REG_HUM_COMP_PART2 0xE1
#define MYSENSOR_REG_STATUS 0xF3
#define MYSENSOR_REG_RESET 0xE0
#define MYSENSOR_REG_CONFIG 0xF5
#define MYSENSOR_REG_CTRL_MEAS 0xF4
#define MYSENSOR_REG_CTRL_HUM 0xF2

// Regiser values
#define MYSENSOR_CHIP_ID 0x60
#define MYSENSOR_STATUS_MEASURING 0x08
#define MYSENSOR_STATUS_IM_UPDATE 0x01
#define MYSENSOR_CMD_SOFT_RESET 0xB6
#define MYSENSOR_MODE_NORMAL 0x03

#define MYSENSOR_HUMIDITY_OVER 5

#define MYSENSOR_STANDBY (5 << 5)
#define MYSENSOR_FILTER (2 << 2)

#define MYSENSOR_PRESS_OVER (5 << 2)
#define MYSENSOR_TEMP_OVER (2 << 5)

#define MYSENSOR_CONFIG_VAL (MYSENSOR_STANDBY | MYSENSOR_FILTER | 0x00)
#define MYSENSOR_CTRL_MEAS_VAL (MYSENSOR_PRESS_OVER | MYSENSOR_TEMP_OVER | MYSENSOR_MODE_NORMAL)

#define MYSENSOR_Q31_SHIFT 0 // TODO: definir un shift
#define MYSENSOR_FRAME_SIZE 20

// TODO: define a raw struct `struct mysensor_raw` that will carry the raw values read from the sensor (temperature, pressure, humidity)

struct my_bme280_config
{
	//TODO
};

/* Etat propre a chaque instance (mutable) */
struct my_bme280_data
{
	// TODO all parameters to compensate the raw values (dig_t[1:3], dig_p[1:9], dig_h[1:6])
};

#endif
