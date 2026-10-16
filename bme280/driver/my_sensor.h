#ifndef MY_SENSOR_H_
#define MY_SENSOR_H_

#include <zephyr/drivers/i2c.h>

#include <stdint.h>
#include <zephyr/kernel.h>

// register
#define MYSENSOR_REG_PRESS_MSB 0xF7
#define MYSENSOR_REG_ID 0xD0
#define MYSENSOR_REG_COMP_T_START 0x88
#define MYSENSOR_REG_COMP_P_START 0x8E
#define MYSENSOR_REG_COMP_H1_START 0xA1
#define MYSENSOR_REG_COMP_H2_START 0xE1
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
struct mysensor_raw
{
	uint16_t temperature;
	uint16_t pressure;
	uint16_t humidity;
};

struct my_bme280_config
{
	struct i2c_dt_spec i2c;
};

/* Etat propre a chaque instance (mutable) */
struct my_bme280_data {
    uint16_t dig_t1;
    int16_t dig_t2, dig_t3;

    uint16_t dig_p1;
    int16_t dig_p2, dig_p3, dig_p4, dig_p5, dig_p6, dig_p7, dig_p8, dig_p9;

    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4, dig_h5;
    int8_t dig_h6;
};

#endif
