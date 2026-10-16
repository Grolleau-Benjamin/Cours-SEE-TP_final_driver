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
	/* TODO : vérifier que le bus I2C est prêt. */
	if (i2c_is_ready_dt(&dev->config->i2c))
	{
		return 0;
	}
	else
	{
		return -ENODEV;
	}
}

static inline int mysensor_reg_read(const struct device *dev,
									uint8_t start, uint8_t *buf, int size)
{
	/* TODO : lire le registre */
	return i2c_burst_read_dt(&dev->config->i2c, start, buf, size);
}

static inline int mysensor_reg_write(const struct device *dev,
									 uint8_t reg, uint8_t val)
{
	/* TODO : écrire val dans le registre reg. */
	return i2c_reg_write_byte_dt(&dev->config->i2c, reg, val);
}

static int mysensor_wait_until_ready(const struct device *dev,
									 k_timeout_t timeout)
{
	/*
	 * TODO : attendre avec timeout que les bits MEASURING
	 * et IM_UPDATE de MYSENSOR_REG_STATUS soient à zéro.
	 */
	int status_value = i2c_reg_read_byte_dt(&dev->config->i2c, MYSENSOR_REG_STATUS, &status);
	int ret;
	while(((status_value & (MYSENSOR_STATUS_MEASURING | MYSENSOR_STATUS_IM_UPDATE)) != 0) && (timeout.ticks > 0)){
		ret = i2c_reg_read_byte_dt(&dev->config->i2c, MYSENSOR_REG_STATUS, &status);
		k_msleep(3);
	}
	if(ret)return 0;
	return -ETIMEDOUT;
}

static int mysensor_attr_set(const struct device *dev,
							 enum sensor_channel chan,
							 enum sensor_attribute attr,
							 const struct sensor_value *val)
{
	return -ENOTSUP;
}

static int mysensor_read_raw(const struct device *dev,
							 struct mysensor_raw *raw)
{
	/*
	 * TODO : attendre que le capteur soit prêt, lire les 8 octets
	 * depuis MYSENSOR_REG_PRESS_MSB et extraire les trois valeurs.
	 */
	if(mysensor_wait_until_ready(dev, K_MSEC(MYSENSOR_START_UP_TIME_MS)) != 0){
		return -ETIMEDOUT;
	}
	uint8_t buffer[8];
	int ret = mysensor_reg_read(dev, MYSENSOR_REG_PRESS_MSB, buffer, 8);
	if(ret != 0){
		return ret;
	}
	raw->pressure = buffer[0] << 8 | buffer[1];
	raw->temperature = buffer[3] << 8 | buffer[4];
	raw->humidity = buffer[6] << 8 | buffer[7];
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
	const struct sensor_read_config *read_cfg =
		request->sqe.iodev->data;

	struct mysensor_raw raw;
	uint8_t *buffer;
	uint32_t buffer_size;

	int ret = rtio_sqe_rx_buf(request,
							  MYSENSOR_FRAME_SIZE,
							  MYSENSOR_FRAME_SIZE,
							  &buffer,
							  &buffer_size);
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

	sys_put_le64(k_ticks_to_ns_floor64(k_uptime_ticks()), buffer);
	sys_put_le32(raw.temperature, buffer + 8);
	sys_put_le32(raw.pressure, buffer + 12);
	sys_put_le32(raw.humidity, buffer + 16);

	rtio_iodev_sqe_ok(request, 0);
}

static void mysensor_submit(const struct device *dev,
							struct rtio_iodev_sqe *request)
{
	ARG_UNUSED(dev);

	const struct sensor_read_config *read_cfg =
		request->sqe.iodev->data;

	if (request->sqe.op != RTIO_OP_RX ||
		read_cfg->is_streaming ||
		read_cfg->count == 0)
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

static int mysensor_get_frame_count(const uint8_t *buffer,
									struct sensor_chan_spec channel,
									uint16_t *count)
{
	ARG_UNUSED(buffer);

	if (!mysensor_channel_supported(channel))
	{
		return -ENOTSUP;
	}

	*count = 1;
	return 0;
}

static int mysensor_get_size_info(struct sensor_chan_spec channel,
								  size_t *base_size,
								  size_t *frame_size)
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
	return (q31_t)(((int64_t)milli_value *
					(INT64_C(1) << (31 - MYSENSOR_Q31_SHIFT))) /
				   1000);
}

static int mysensor_decode(const uint8_t *buffer,
						   struct sensor_chan_spec channel,
						   uint32_t *fit,
						   uint16_t max_count,
						   void *output)
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
		/*
		 * TODO : lire la température brute.
		 */
		milli_value = raw->temperature;
		return -ENOSYS;

	case SENSOR_CHAN_PRESS:
		/*
		 * TODO : lire la pression brute.
		 */
		milli_value = raw->pressure;
		return -ENOSYS;

	case SENSOR_CHAN_HUMIDITY:
		/*
		 * TODO : lire l'humidité brute.
		 */
		milli_value = raw->humidity;
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
	// TODO
	.decode = mysensor_decode,
	.get_size_info = mysensor_get_size_info,
	.get_frame_count = mysensor_get_frame_count,
};

static int mysensor_get_decoder(
	const struct device *dev,
	const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);

	*decoder = &mysensor_decoder_api;
	return 0;
}

static int mysensor_init(const struct device *dev)
{
	/*
	 * TODO : vérifier le bus et l'identifiant, effectuer le reset,
	 * puis configurer CTRL_HUM, CONFIG et CTRL_MEAS.
	 * Respecter les délais et propager les erreurs.
	 */
	uint8_t id;
	int ret;

	ret = mysensor_bus_check(dev);
	if(ret != 0){
		return ret;
	}

	ret = mysensor_reg_read(dev, MYSENSOR_REG_ID, &id, 1);
	if(ret != 0){
		return ret;
	}

	ret = mysensor_reg_write(dev, MYSENSOR_REG_RESET, MYSENSOR_CMD_SOFT_RESET);
	if(ret != 0){
		return ret;
	}
	k_msleep(MYSENSOR_START_UP_TIME_MS);

	ret = mysensor_wait_until_ready(dev, K_MSEC(10));
	if (ret != 0)
	{
		return ret;
	}

	ret = mysensor_reg_write(dev, MYSENSOR_REG_CTRL_HUM, MYSENSOR_HUMIDITY_OVER);
	if(ret != 0){
		return ret;
	}

	ret = mysensor_reg_write(dev, MYSENSOR_REG_CONFIG, MYSENSOR_CONFIG_VAL);
	if(ret != 0){
		return ret;
	}

	ret = mysensor_reg_write(dev, MYSENSOR_REG_CTRL_MEAS, MYSENSOR_CTRL_MEAS_VAL);
	if(ret != 0){
		return ret;
	}

	return 0;
}

static DEVICE_API(sensor, mysensor_driver_api) = {
	// TODO
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