#define DT_DRV_COMPAT bosch_my_bme280/* TODO */

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

static int32_t mysensor_compensate_temp(const struct mysensor_data *data, int32_t adc,
     int32_t *t_fine)
{
 int64_t delta = (adc >> 4) - data->dig_t1;
 int64_t var1 = (((adc >> 3) - (int64_t)data->dig_t1 * 2) * data->dig_t2) >> 11;
 int64_t var2 = (((delta * delta) >> 12) * data->dig_t3) >> 14;

 *t_fine = (int32_t)(var1 + var2);
 /* La formule donne des centiemes de degre ; le buffer contient des milliemes. */
 return (int32_t)((((int64_t)*t_fine * 5 + 128) >> 8) * 10);
}

static inline int mysensor_bus_check(const struct device *dev)
{
	const struct mysensor_config *cfg = dev->config;
	return i2c_is_ready_dt(&cfg->i2c) ? 0 : -ENODEV;
}

static inline int mysensor_reg_read(const struct device *dev,
				    uint8_t start, uint8_t *buf, int size)
{
	const struct mysensor_config *cfg = dev->config;
	return i2c_burst_read_dt(&cfg->i2c, start, buf, size);
}

static inline int mysensor_reg_write(const struct device *dev,
				     uint8_t reg, uint8_t val)
{
	const struct mysensor_config *cfg = dev->config;
	return i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
}

static int mysensor_wait_until_ready(const struct device *dev,
									 k_timeout_t timeout)
{
	/*
	 * TODO : attendre avec timeout que les bits MEASURING
	 * et IM_UPDATE de MYSENSOR_REG_STATUS soient à zéro.
	 */
	const struct mysensor_config *cfg = dev->config;
	uint8_t status;
	int ret;
	int status_value = i2c_reg_read_byte_dt(&cfg->i2c, MYSENSOR_REG_STATUS, &status);
	while(((status_value & (MYSENSOR_STATUS_MEASURING | MYSENSOR_STATUS_IM_UPDATE)) != 0) && (timeout.ticks > 0)){
		ret = i2c_reg_read_byte_dt(&cfg->i2c, MYSENSOR_REG_STATUS, &status);
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
	raw->pressure    = ((int32_t)buffer[0] << 12) | ((int32_t)buffer[1] << 4) | (buffer[2] >> 4);
    raw->temperature = ((int32_t)buffer[3] << 12) | ((int32_t)buffer[4] << 4) | (buffer[5] >> 4);
    raw->humidity    = ((int32_t)buffer[6] << 8)  | buffer[7];
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
static int mysensor_read_cal(const struct device *dev)
{
    struct mysensor_data *data = dev->data;
    uint8_t b1[24], h1, b2[7];
    int ret;

    ret = mysensor_reg_read(dev, 0x88, b1, sizeof(b1));   /* T1..T3, P1..P9 */
    if (ret != 0) {
        return ret;
    }
    ret = mysensor_reg_read(dev, 0xA1, &h1, 1);           /* H1 */
    if (ret != 0) {
        return ret;
    }
    ret = mysensor_reg_read(dev, 0xE1, b2, sizeof(b2));   /* H2..H6 */
    if (ret != 0) {
        return ret;
    }

    // Température
    data->dig_t1 = sys_get_le16(&b1[0]);
    data->dig_t2 = (int16_t)sys_get_le16(&b1[2]);
    data->dig_t3 = (int16_t)sys_get_le16(&b1[4]);

    // Pression
    data->dig_p1 = sys_get_le16(&b1[6]);
    data->dig_p2 = (int16_t)sys_get_le16(&b1[8]);
    data->dig_p3 = (int16_t)sys_get_le16(&b1[10]);
    data->dig_p4 = (int16_t)sys_get_le16(&b1[12]);
    data->dig_p5 = (int16_t)sys_get_le16(&b1[14]);
    data->dig_p6 = (int16_t)sys_get_le16(&b1[16]);
    data->dig_p7 = (int16_t)sys_get_le16(&b1[18]);
    data->dig_p8 = (int16_t)sys_get_le16(&b1[20]);
    data->dig_p9 = (int16_t)sys_get_le16(&b1[22]);

    // Humidité
    data->dig_h1 = h1;
    data->dig_h2 = (int16_t)sys_get_le16(&b2[0]);
    data->dig_h3 = b2[2];
    data->dig_h4 = (int16_t)(((int16_t)(int8_t)b2[3] << 4) | (b2[4] & 0x0F));
    data->dig_h5 = (int16_t)(((int16_t)(int8_t)b2[5] << 4) | (b2[4] >> 4));
    data->dig_h6 = (int8_t)b2[6];

    return 0;
}

static void mysensor_read_work(struct rtio_iodev_sqe *request)
{
	const struct sensor_read_config *read_cfg =
		request->sqe.iodev->data;
	const struct mysensor_data *data = read_cfg->sensor->data;

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
	int32_t t_fine;
	int32_t temperature = mysensor_compensate_temp(data, raw.temperature, &t_fine);

	sys_put_le64(k_ticks_to_ns_floor64(k_uptime_ticks()), buffer);
	sys_put_le32((uint32_t) temperature, buffer + 8);
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
		milli_value = (int32_t)sys_get_le32(buffer + 8);
		break;

	case SENSOR_CHAN_PRESS:
		/*
		 * TODO : lire la pression brute.
		 */
		milli_value = (int32_t)sys_get_le32(buffer + 12);
		break;

	case SENSOR_CHAN_HUMIDITY:
		/*
		 * TODO : lire l'humidité brute.
		 */
		milli_value = (int32_t)sys_get_le32(buffer + 16);
		break;

	default:
		break;
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

	ret = mysensor_read_cal(dev);
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