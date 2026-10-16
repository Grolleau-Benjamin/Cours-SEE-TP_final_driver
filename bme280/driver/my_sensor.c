#define DT_DRV_COMPAT /* TODO */

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
	const struct my_bme280_config* config = dev->config;
	if (i2c_is_ready_dt(&config->i2c))
		return 0;

	return -ENOSYS;
}

static inline int mysensor_reg_read(const struct device *dev, uint8_t start, uint8_t *buf, int size)
{
	const struct my_bme280_config* config = dev->config;
	if (mysensor_bus_check(dev) == 0)
		if (i2c_burst_read_dt(&config->i2c, start, buf, size) == 0)
			return 0;

	printk("Error: sensor reading for register %02X failed.", start);
	return -ENOSYS;
}

static inline int mysensor_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct my_bme280_config* config = dev->config;
	if (mysensor_bus_check(dev) == 0)
		if (i2c_reg_write_byte_dt(&config->i2c, reg, val) == 0)
			return 0;

	return -ENOSYS;
}

static int mysensor_wait_until_ready(const struct device *dev, k_timeout_t timeout)
{
	uint64_t elapsed_ms = 0;
	uint8_t buf_im_update;
	uint8_t buf_reg_status;

	do
	{
		if (elapsed_ms > k_ticks_to_ms_floor64(timeout.ticks))
		{
			printk("Sensor wait until ready timeout.");
			return -ENOSYS;
		}

		if (mysensor_reg_read(dev, MYSENSOR_STATUS_IM_UPDATE, &buf_im_update, 1) != 0)
			return -ENOSYS;

		if (mysensor_reg_read(dev, MYSENSOR_REG_STATUS, &buf_reg_status, 1) != 0)
			return -ENOSYS;

		k_msleep(10); // 100 Hz refresh rate.
		elapsed_ms += 10;
	}
	while ((buf_im_update != 0) && (buf_reg_status != 0));

	return 0;
}

static int mysensor_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
	// Pas utilisé dans notre cas
	return -ENOTSUP;
}

/**
 * Cette fontion est appelée à l'init et permet de recuperer les 
 * parametres de compensation du capteur. Ces parametres sont stockés
 * dans la structure my_bme280_data.
 */
static int mysensor_read_calibration(const struct device *dev)
{
	uint8_t dig_t_buf[3*2];
	uint8_t dig_p_buf[9*2];
	uint8_t dig_h1_buf[1*2];
	uint8_t dig_h2_buf[5*2];
	
	if (mysensor_reg_read(dev, MYSENSOR_REG_COMP_T_START, dig_t_buf, 3*2) != 0)
		return -ENOSYS;

	if (mysensor_reg_read(dev, MYSENSOR_REG_COMP_P_START, dig_p_buf, 9*2) != 0)
		return -ENOSYS;

	if (mysensor_reg_read(dev, MYSENSOR_REG_COMP_H1_START, dig_h1_buf, 1*2) != 0)
		return -ENOSYS;

	if (mysensor_reg_read(dev, MYSENSOR_REG_COMP_H2_START, dig_h2_buf, 5*2) != 0)
		return -ENOSYS;

	struct my_bme280_data* data = dev->data;
	
	data->dig_t1 = sys_get_le16(&dig_t_buf[0]);
	data->dig_t2 = sys_get_le16(&dig_t_buf[2]);
	data->dig_t3 = sys_get_le16(&dig_t_buf[4]);

	data->dig_p1 = sys_get_le16(&dig_p_buf[0]);
	data->dig_p2 = sys_get_le16(&dig_p_buf[2]);
	data->dig_p3 = sys_get_le16(&dig_p_buf[4]);
	data->dig_p4 = sys_get_le16(&dig_p_buf[6]);
	data->dig_p5 = sys_get_le16(&dig_p_buf[8]);
	data->dig_p6 = sys_get_le16(&dig_p_buf[10]);
	data->dig_p7 = sys_get_le16(&dig_p_buf[12]);
	data->dig_p8 = sys_get_le16(&dig_p_buf[14]);
	data->dig_p9 = sys_get_le16(&dig_p_buf[16]);

	data->dig_h1 = sys_get_le16(&dig_h1_buf[0]);
	data->dig_h2 = sys_get_le16(&dig_h2_buf[0]);
	data->dig_h3 = sys_get_le16(&dig_h2_buf[2]);
	data->dig_h4 = sys_get_le16(&dig_h2_buf[4]);
	data->dig_h5 = sys_get_le16(&dig_h2_buf[6]);
	data->dig_h6 = sys_get_le16(&dig_h2_buf[8]);

	return 0;
}

static int mysensor_read_raw(const struct device *dev, struct mysensor_raw *raw)
{
	if (mysensor_wait_until_ready(dev, K_SECONDS(1)) != 0)
		return -ENOSYS;

	uint8_t read_raw_buf[8*2];
	if (mysensor_reg_read(dev, MYSENSOR_REG_PRESS_MSB, read_raw_buf, 8*2) != 0)
		return -ENOSYS;

	raw->pressure = sys_get_le16(&read_raw_buf[0]);
	raw->temperature = sys_get_le16(&read_raw_buf[3]);
	raw->humidity = sys_get_le16(&read_raw_buf[6]);

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

static void mysensor_read_work(struct rtio_iodev_sqe *request)
{
	const struct sensor_read_config *read_cfg = request->sqe.iodev->data;
	const struct device *dev = read_cfg->sensor;
	const struct my_bme280_data *d = dev->data;
	struct mysensor_raw raw;
	uint8_t *buffer;
	uint32_t buffer_size;

	int ret = rtio_sqe_rx_buf(request, MYSENSOR_FRAME_SIZE, MYSENSOR_FRAME_SIZE,
				  &buffer, &buffer_size);
	if (ret < 0) {
		rtio_iodev_sqe_err(request, ret);
		return;
	}

	ret = mysensor_read_raw(dev, &raw);
	if (ret < 0) {
		rtio_iodev_sqe_err(request, ret);
		return;
	}

	int32_t t1 = (raw.temperature >> 3) - ((int32_t)d->dig_t1 << 1);
	int32_t t2 = (raw.temperature >> 4) - d->dig_t1;
	int32_t t_fine = ((t1 * d->dig_t2) >> 11) + ((((t2 * t2) >> 12) * d->dig_t3) >> 14);
	int32_t temp = ((t_fine * 5 + 128) >> 8) * 10;

	int64_t v1 = (int64_t)t_fine - 128000;
	int64_t v2 = v1 * v1 * d->dig_p6;
	v2 += (v1 * d->dig_p5) << 17;
	v2 += (int64_t)d->dig_p4 << 35;
	v1 = ((v1 * v1 * d->dig_p3) >> 8) + ((v1 * d->dig_p2) << 12);
	v1 = (((INT64_C(1) << 47) + v1) * d->dig_p1) >> 33;

	int32_t press = 0;
	if (v1 != 0) {
		int64_t p = 1048576 - raw.pressure;
		p = (((p << 31) - v2) * 3125) / v1;
		v1 = (d->dig_p9 * (p >> 13) * (p >> 13)) >> 25;
		v2 = (d->dig_p8 * p) >> 19;
		p = ((p + v1 + v2) >> 8) + ((int64_t)d->dig_p7 << 4);
		press = (int32_t)((p * 1000) >> 8);
	}

	int32_t v = t_fine - 76800;

	v = (((((raw.humidity << 14) - ((int32_t)d->dig_h4 << 20) - (d->dig_h5 * v)) + 16384) >> 15) * (((((((v * d->dig_h6) >> 10) *(((v * d->dig_h3) >> 11) + 32768)) >> 10) + 2097152) *
	       d->dig_h2 + 8192) >> 14));
	v -= (((((v >> 15) * (v >> 15)) >> 7) * d->dig_h1) >> 4);
	v = CLAMP(v, 0, 419430400);
	int32_t hum = (v >> 12) * 1000 / 1024;

	sys_put_le64(k_ticks_to_ns_floor64(k_uptime_ticks()), buffer);
	sys_put_le32((uint32_t)temp, buffer + 8);
	sys_put_le32((uint32_t)press, buffer + 12);
	sys_put_le32((uint32_t)hum, buffer + 16);

	rtio_iodev_sqe_ok(request, 0);
}

static void mysensor_submit(const struct device *dev, struct rtio_iodev_sqe *request)
{
	ARG_UNUSED(dev);

	const struct sensor_read_config *read_cfg = request->sqe.iodev->data;

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
		milli_value = (int32_t)sys_get_le32(buffer+8);
		return -ENOSYS;

	case SENSOR_CHAN_PRESS:
		milli_value = (int32_t)sys_get_le32(buffer+12);
		return -ENOSYS;

	case SENSOR_CHAN_HUMIDITY:
		milli_value = (int32_t)sys_get_le32(buffer+16);
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
	.decode = mysensor_decode
};

static int mysensor_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);

	*decoder = &mysensor_decoder_api;
	return 0;
}

static int mysensor_init(const struct device *dev)
{
	if (mysensor_bus_check(dev) != 0)
		return -ENODEV;

	uint8_t id;
	if (mysensor_reg_read(dev, MYSENSOR_REG_ID, &id, 1) != 0)
		return -ENODEV;

	if (id != 0x60)
	{
		printk("Error: wrong ID");
		return -ENODEV;
	}

	if (mysensor_reg_write(dev, MYSENSOR_REG_RESET, 0xB6) != 0)
		return -ENODEV;

	k_msleep(MYSENSOR_START_UP_TIME_MS);

	uint8_t status;
	for (int i = 0; i < 10; i++)
	{
		if (mysensor_reg_read(dev, MYSENSOR_REG_STATUS, &status, 1) != 0)
			return -ENODEV;

		if ((status & BIT(0)) == 0)
			break;

		k_msleep(MYSENSOR_START_UP_TIME_MS);
		if (i == 9)
			return -ENODEV;
	}

	if (mysensor_read_calibration(dev) != 0)
		return -ENODEV;

	if (mysensor_reg_write(dev, MYSENSOR_REG_CTRL_HUM, 0x01) != 0)
		return -ENODEV;

	if (mysensor_reg_write(dev, MYSENSOR_REG_CONFIG, 0x00) != 0)
		return -ENODEV;
	
	if (mysensor_reg_write(dev, MYSENSOR_REG_CTRL_MEAS, (1 << 5) | (1 << 2) | 0x03) != 0)
		return -ENODEV;

	return 0;
}

static DEVICE_API(sensor, mysensor_driver_api) = {
	.attr_set = mysensor_attr_set,
	.submit = mysensor_submit,
	.get_decoder = mysensor_get_decoder
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