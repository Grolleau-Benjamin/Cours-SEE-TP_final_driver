#define DT_DRV_COMPAT tp_bme280

// TODO : toutes les fonctions ne sont pas notées. il manque les fonctions de
// - recuperation des parametres de compensation
// - compensation des valeurs brutes
// - autres si vous avez besoin

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/work.h>
#include <zephyr/sys/byteorder.h>
#include "my_sensor.h"

LOG_MODULE_REGISTER(MYSENSOR, CONFIG_SENSOR_LOG_LEVEL);	

#define MYSENSOR_START_UP_TIME_MS 2

static inline int mysensor_bus_check(const struct device *dev)
{
	const struct mysensor_config *cfg = dev->config;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		return -ENODEV;
	}

	return 0;
}

static inline int mysensor_reg_read(const struct device *dev, uint8_t start, uint8_t *buf, int size)
{
	const struct mysensor_config *cfg = dev->config;
	return i2c_burst_read_dt(&cfg->i2c, start, buf, size);
}

static inline int mysensor_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct mysensor_config *cfg = dev->config;
	return i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
}

static int mysensor_wait_until_ready(const struct device *dev, k_timeout_t timeout)
{
	k_timepoint_t deadline = sys_timepoint_calc(timeout);
	uint8_t status;
	int ret;

	while (true) {
		ret = mysensor_reg_read(dev, MYSENSOR_REG_STATUS, &status, 1);
		if (ret < 0) {
			return ret;
		}

		if ((status & (MYSENSOR_STATUS_MEASURING | MYSENSOR_STATUS_IM_UPDATE)) == 0) {
			return 0;
		}

		if (sys_timepoint_expired(deadline)) {
			return -ETIMEDOUT;
		}

		k_msleep(1);
	}
}

static int mysensor_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
	return -ENOTSUP;
}

static int mysensor_read_raw(const struct device *dev, struct mysensor_raw *raw)
{
	uint8_t buf[8];
	int ret;

	ret = mysensor_wait_until_ready(dev, K_MSEC(100));
	if (ret < 0) {
		return ret;
	}

	ret = mysensor_reg_read(dev, MYSENSOR_REG_PRESS_MSB, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	raw->pressure = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
	raw->temperature = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);
	raw->humidity = ((int32_t)buf[6] << 8) | buf[7];

	return 0;
}

static bool mysensor_channel_supported(struct sensor_chan_spec channel)
{
	if (channel.chan_idx != 0)
	{
		return false;
	}

	switch (channel.chan_type)
	{
	case SENSOR_CHAN_AMBIENT_TEMP:
	case SENSOR_CHAN_PRESS:
	case SENSOR_CHAN_HUMIDITY:
		return true;
	default:
		return false;
	}
}

// Fonction décrite dans la doc du BME
static int32_t mysensor_compensate_temperature(const struct mysensor_data *data, int32_t adc_T, int32_t *t_fine)
{
	int64_t var1, var2, delta;

	var1 = (((adc_T >> 3) - (int64_t)data->dig_T1 * 2)
		* data->dig_T2) >> 11;

	delta = (adc_T >> 4) - (int64_t)data->dig_T1;
	var2 = (((delta * delta) >> 12) * data->dig_T3) >> 14;

	*t_fine = (int32_t)(var1 + var2);

	return (int32_t)(((int64_t)*t_fine * 5 + 128) >> 8);
}

// Fonction décrite dans la doc du BME
static uint32_t mysensor_compensate_pressure(const struct mysensor_data *data, int32_t adc_P, int32_t t_fine)
{
	int64_t var1, var2, p;

	var1 = (int64_t)t_fine - 128000;

	var2 = var1 * var1 * data->dig_P6;
	var2 += var1 * data->dig_P5 * (INT64_C(1) << 17);
	var2 += (int64_t)data->dig_P4 * (INT64_C(1) << 35);

	var1 = ((var1 * var1 * data->dig_P3) >> 8)
	       + var1 * data->dig_P2 * (INT64_C(1) << 12);

	var1 = (((INT64_C(1) << 47) + var1) * data->dig_P1) >> 33;

	if (var1 == 0) {
		return 0;
	}

	p = 1048576 - adc_P;
	p = ((p * (INT64_C(1) << 31) - var2) * 3125) / var1;

	var1 = ((int64_t)data->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t)data->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (int64_t)data->dig_P7 * 16;

	return (uint32_t)p;
}

// Fonction décrite dans la doc du BME
static uint32_t mysensor_compensate_humidity(const struct mysensor_data *data, int32_t adc_H, int32_t t_fine)
{
	int64_t value, var1, var2;

	value = (int64_t)t_fine - 76800;

	var1 = (((int64_t)adc_H * 16384
		 - (int64_t)data->dig_H4 * 1048576
		 - data->dig_H5 * value) + 16384) >> 15;

	var2 = (((value * data->dig_H6) >> 10)
		* (((value * data->dig_H3) >> 11) + 32768)) >> 10;

	var2 = ((var2 + 2097152) * data->dig_H2 + 8192) >> 14;

	value = var1 * var2;

	value -= ((((value >> 15) * (value >> 15)) >> 7)
		  * data->dig_H1) >> 4;

	if (value < 0) {
		value = 0;
	}

	if (value > 419430400) {
		value = 419430400;
	}

	return (uint32_t)(value >> 12);
}

static void mysensor_read_work(struct rtio_iodev_sqe *request)
{
	const struct sensor_read_config *read_cfg = request->sqe.iodev->data;

	struct mysensor_raw raw;
	uint8_t *buffer;
	uint32_t buffer_size;

	int ret = rtio_sqe_rx_buf(request, MYSENSOR_FRAME_SIZE, MYSENSOR_FRAME_SIZE, &buffer, &buffer_size);
	if (ret < 0)
	{
		rtio_iodev_sqe_err(request, ret);
		return;
	}

	ret = mysensor_read_raw(read_cfg->sensor, &raw);
	if (ret < 0)
	{
		rtio_iodev_sqe_err(request, ret);
		return;
	}

	const struct mysensor_data *data = read_cfg->sensor->data;
	int32_t t_fine;

	int32_t temperature = mysensor_compensate_temperature(data, raw.temperature, &t_fine);
	uint32_t pressure = mysensor_compensate_pressure(data, raw.pressure, t_fine);
	uint32_t humidity = mysensor_compensate_humidity(data, raw.humidity, t_fine);

	int32_t temperature_mc = temperature * 10;
	int32_t pressure_mkpa = (int32_t)(pressure / 256);
	int32_t humidity_mpercent = (int32_t)(((uint64_t)humidity * 1000) / 1024);

	sys_put_le64(k_ticks_to_ns_floor64(k_uptime_ticks()), buffer);
	sys_put_le32((uint32_t)temperature_mc, buffer + 8);
	sys_put_le32((uint32_t)pressure_mkpa, buffer + 12);
	sys_put_le32((uint32_t)humidity_mpercent, buffer + 16);

	rtio_iodev_sqe_ok(request, 0);
}

static void mysensor_submit(const struct device *dev, struct rtio_iodev_sqe *request)
{
	ARG_UNUSED(dev);

	const struct sensor_read_config *read_cfg =
		request->sqe.iodev->data;

	if (request->sqe.op != RTIO_OP_RX || read_cfg->is_streaming || read_cfg->count == 0)
	{
		rtio_iodev_sqe_err(request, -ENOTSUP);
		return;
	}

	for (size_t i = 0; i < read_cfg->count; i++)
	{
		if (!mysensor_channel_supported(read_cfg->channels[i]))
		{
			rtio_iodev_sqe_err(request, -ENOTSUP);
			return;
		}
	}

	struct rtio_work_req *work = rtio_work_req_alloc();

	if (work == NULL)
	{
		rtio_iodev_sqe_err(request, -ENOMEM);
		return;
	}

	rtio_work_req_submit(work, request, mysensor_read_work);
}

static int mysensor_get_frame_count(const uint8_t *buffer, struct sensor_chan_spec channel, uint16_t *count)
{
	ARG_UNUSED(buffer);

	if (!mysensor_channel_supported(channel))
	{
		return -ENOTSUP;
	}

	*count = 1;
	return 0;
}

static int mysensor_get_size_info(struct sensor_chan_spec channel, size_t *base_size, size_t *frame_size)
{
	if (!mysensor_channel_supported(channel))
	{
		return -ENOTSUP;
	}

	*base_size = sizeof(struct sensor_q31_data);
	*frame_size = sizeof(struct sensor_q31_sample_data);

	return 0;
}

/* Fourni : conversion d'une valeur en milli-unités vers Q31. */
static q31_t mysensor_to_q31(int32_t milli_value)
{
	return (q31_t)(((int64_t)milli_value * (INT64_C(1) << (31 - MYSENSOR_Q31_SHIFT))) / 1000);
}

static int mysensor_decode(const uint8_t *buffer, struct sensor_chan_spec channel, uint32_t *fit, uint16_t max_count, void *output)
{
	if (!mysensor_channel_supported(channel))
	{
		return -EINVAL;
	}

	if (*fit != 0 || max_count == 0)
	{
		return 0;
	}

	struct sensor_q31_data *decoded = output;
	int32_t milli_value;

	switch (channel.chan_type)
	{
	case SENSOR_CHAN_AMBIENT_TEMP:
		milli_value = (int32_t)sys_get_le32(buffer + 8);
		return -ENOSYS;

	case SENSOR_CHAN_PRESS:
		milli_value = (int32_t)sys_get_le32(buffer + 12);
		return -ENOSYS;

	case SENSOR_CHAN_HUMIDITY:
		milli_value = (int32_t)sys_get_le32(buffer + 16);
		return -ENOSYS;

	default:
		return -ENOTSUP;
	}

	*decoded = (struct sensor_q31_data){0};

	decoded->header.base_timestamp_ns = sys_get_le64(buffer);
	decoded->header.reading_count = 1;
	decoded->shift = MYSENSOR_Q31_SHIFT;
	decoded->readings[0].value = mysensor_to_q31(milli_value);

	*fit = 1;
	return 1;
}

static const struct sensor_decoder_api mysensor_decoder_api = {
	.get_frame_count = mysensor_get_frame_count,
	.get_size_info = mysensor_get_size_info,
	.decode = mysensor_decode,
};

static int mysensor_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);

	*decoder = &mysensor_decoder_api;
	return 0;
}

static int mysensor_read_calibration(const struct device *dev)
{
	struct mysensor_data *data = dev->data;
	uint8_t calib[24];
	uint8_t hum[7];
	uint8_t h1;
	int ret;

	ret = mysensor_reg_read(dev, MYSENSOR_REG_COMP_START, calib, sizeof(calib));
	if (ret < 0) {
		return ret;
	}

	ret = mysensor_reg_read(dev, MYSENSOR_REG_HUM_COMP_PART1, &h1, 1);
	if (ret < 0) {
		return ret;
	}

	ret = mysensor_reg_read(dev, MYSENSOR_REG_HUM_COMP_PART2, hum, sizeof(hum));
	if (ret < 0) {
		return ret;
	}

	data->dig_T1 = sys_get_le16(&calib[0]);
	data->dig_T2 = (int16_t)sys_get_le16(&calib[2]);
	data->dig_T3 = (int16_t)sys_get_le16(&calib[4]);

	data->dig_P1 = sys_get_le16(&calib[6]);
	data->dig_P2 = (int16_t)sys_get_le16(&calib[8]);
	data->dig_P3 = (int16_t)sys_get_le16(&calib[10]);
	data->dig_P4 = (int16_t)sys_get_le16(&calib[12]);
	data->dig_P5 = (int16_t)sys_get_le16(&calib[14]);
	data->dig_P6 = (int16_t)sys_get_le16(&calib[16]);
	data->dig_P7 = (int16_t)sys_get_le16(&calib[18]);
	data->dig_P8 = (int16_t)sys_get_le16(&calib[20]);
	data->dig_P9 = (int16_t)sys_get_le16(&calib[22]);

	data->dig_H1 = h1;
	data->dig_H2 = (int16_t)sys_get_le16(&hum[0]);
	data->dig_H3 = hum[2];

	data->dig_H4 = (hum[3] << 4) | (hum[4] & 0x0F);
	data->dig_H5 = (hum[5] << 4) | (hum[4] >> 4);

	if (data->dig_H4 & 0x0800) {
		data->dig_H4 -= 4096;
	}

	if (data->dig_H5 & 0x0800) {
		data->dig_H5 -= 4096;
	}

	data->dig_H6 = (int8_t)hum[6];

	return 0;
}

static int mysensor_init(const struct device *dev)
{
	uint8_t chip_id;
	int ret;

	ret = mysensor_bus_check(dev);
	if (ret < 0) {
		return ret;
	}

	k_msleep(MYSENSOR_START_UP_TIME_MS);

	ret = mysensor_reg_read(dev, MYSENSOR_REG_ID, &chip_id, 1);
	if (ret < 0) {
		return ret;
	}

	if (chip_id != MYSENSOR_CHIP_ID) {
		return -ENODEV;
	}

	ret = mysensor_reg_write(dev, MYSENSOR_REG_RESET, MYSENSOR_CMD_SOFT_RESET);
	if (ret < 0) {
		return ret;
	}

	k_msleep(MYSENSOR_START_UP_TIME_MS);

	ret = mysensor_wait_until_ready(dev, K_MSEC(100));
	if (ret < 0) {
		return ret;
	}

	ret = mysensor_read_calibration(dev);
	if (ret < 0) {
		return ret;
	}

	ret = mysensor_reg_write(dev, MYSENSOR_REG_CTRL_HUM, MYSENSOR_HUMIDITY_OVER);
	if (ret < 0) {
		return ret;
	}

	ret = mysensor_reg_write(dev, MYSENSOR_REG_CONFIG, MYSENSOR_CONFIG_VAL);
	if (ret < 0) {
		return ret;
	}

	ret = mysensor_reg_write(dev, MYSENSOR_REG_CTRL_MEAS, MYSENSOR_CTRL_MEAS_VAL);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static DEVICE_API(sensor, mysensor_driver_api) = {
	.attr_set = mysensor_attr_set,
	.submit = mysensor_submit,
	.get_decoder = mysensor_get_decoder,
};

#define MYSENSOR_INIT(n)                                        \
	static const struct mysensor_config mysensor_config_##n = { \
		.i2c = I2C_DT_SPEC_INST_GET(n),                         \
	};                                                          \
	static struct mysensor_data mysensor_data_##n;              \
	SENSOR_DEVICE_DT_INST_DEFINE(n, mysensor_init, NULL,        \
								 &mysensor_data_##n,            \
								 &mysensor_config_##n,          \
								 POST_KERNEL,                   \
								 CONFIG_SENSOR_INIT_PRIORITY,   \
								 &mysensor_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MYSENSOR_INIT)