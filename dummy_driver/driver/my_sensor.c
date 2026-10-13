#define DT_DRV_COMPAT zephyr_tp_sensor

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/byteorder.h>

#include "my_sensor.h"

#define MC_TO_Q31(_mc, _sh)	((_mc * (1<<(31 - _sh))) / 1000)

/**
 * support only the temp channel
 * https://docs.zephyrproject.org/latest/doxygen/html/group__sensor__interface.html#gaaa1b502bc029b10d7b23b0a25ef4e934
 */
static bool my_sensor_channel_supported(struct sensor_chan_spec channel)
{
	if (channel.chan_type == SENSOR_CHAN_GAUGE_TEMP || channel.chan_type == SENSOR_CHAN_DIE_TEMP || channel.chan_type == SENSOR_CHAN_AMBIENT_TEMP )
		return true;

	return false;
}

static int my_sensor_init(const struct device *dev)
{
	const struct my_sensor_config *cfg = dev->config;
	struct my_sensor_data *data = dev->data;

	if (cfg->initial_mc < 0 || cfg->maximum_mc < cfg->initial_mc || cfg->maximum_mc > 100000)
	{
		return -EINVAL;
	}

	data->next_mc = cfg->initial_mc;

	return 0;
}

static void my_sensor_submit(const struct device *dev, struct rtio_iodev_sqe *request)
{
	const struct sensor_read_config *read_cfg = request->sqe.iodev->data;

	const struct my_sensor_config *cfg = dev->config;
	struct my_sensor_data *data = dev->data;

	uint8_t *buffer;
	uint32_t buffer_size;

	if (request->sqe.op != RTIO_OP_RX || read_cfg->is_streaming || read_cfg->count != 1 ||
		!my_sensor_channel_supported(read_cfg->channels[0]))
	{
		rtio_iodev_sqe_err(request, -ENOTSUP);
		return;
	}

	int ret = rtio_sqe_rx_buf(request, MY_SENSOR_FRAME_SIZE, MY_SENSOR_FRAME_SIZE, &buffer,
							  &buffer_size);
	if (ret < 0)
	{
		rtio_iodev_sqe_err(request, ret);
		return;
	}

	/**
	 * TODO:
	 * 1. get the temperature
	 * 2. write the next temp but check the maximum value
	 */

	uint32_t temperature_mc = data->next_mc;
	data->next_mc = (data->next_mc > cfg->maximum_mc) ? cfg->initial_mc : data->next_mc + CONFIG_MY_SENSOR_STEP_MC;

	sys_put_le64(k_ticks_to_ns_floor64(k_uptime_ticks()), buffer);
	sys_put_le32((uint32_t)temperature_mc, buffer + 8);
	rtio_iodev_sqe_ok(request, 0);
}

/* Chaque buffer = une mesure */
static int my_sensor_get_frame_count(const uint8_t *buffer, struct sensor_chan_spec channel,
									 uint16_t *count)
{
	ARG_UNUSED(buffer);
	if (!my_sensor_channel_supported(channel))
	{
		return -ENOTSUP;
	}
	*count = 1;
	return 0;
}

/* Taille necessaire pour les donnees decodees. */
static int my_sensor_get_size_info(struct sensor_chan_spec channel, size_t *base_size,
								   size_t *frame_size)
{
	if (!my_sensor_channel_supported(channel))
	{
		return -ENOTSUP;
	}

	*base_size = sizeof(struct sensor_q31_data);
	*frame_size = sizeof(struct sensor_q31_sample_data);

	return 0;
}

static int my_sensor_decode(const uint8_t *buffer, struct sensor_chan_spec channel, uint32_t *fit,
							uint16_t max_count, void *output)
{
	if (!my_sensor_channel_supported(channel))
	{
		return -EINVAL;
	}

	/*
	 * fit represente la progression dans le buffer.
	 * Une seule mesure est disponible, cf my_sensor_get_frame_count
	 */
	if (*fit != 0 || max_count == 0)
	{
		return 0;
	}

	// TODO: decode the value to a sensor_q31_data struct

	struct sensor_q31_data * q31_data = output;

	q31_data->header.base_timestamp_ns = sys_get_le64(buffer);
	q31_data->header.reading_count = 1;
	q31_data->shift = MY_SENSOR_Q31_SHIFT;

	/*
	 * Conversion millicelsius -> Q31 :
	 *
	 * q31 = temperature_mc * 2^(31 - shift) / 1000
	 *
	 * Calcul intermediaire sur 64 bits.
	 */
	// TODO: complete the conversion and assign to decoded->readings[0].temperature
	int64_t temp_u64 = 0;
	temp_u64 = sys_get_le32(buffer+8);
	q31_data->readings[0].temperature = MC_TO_Q31(temp_u64, q31_data->shift);
	*fit = 1;

	/* Nombre de mesures decodees. */
	return 1;
}

static const struct sensor_decoder_api my_sensor_decoder = {
	.get_frame_count = &my_sensor_get_frame_count,
	.get_size_info = &my_sensor_get_size_info,
	.decode = &my_sensor_decode
};

static int my_sensor_get_decoder(const struct device *dev,
								 const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);

	*decoder = &my_sensor_decoder;
	return 0;
}

static DEVICE_API(sensor, my_sensor_api) = {
	.submit = my_sensor_submit,
	.get_decoder = my_sensor_get_decoder,
};

#define MY_SENSOR_DEFINE(inst)                                                       \
	static struct my_sensor_data my_sensor_data_##inst;                              \
                                                                                     \
	static const struct my_sensor_config my_sensor_config_##inst = {                 \
		.initial_mc = DT_INST_PROP(inst, initial_millicelsius), 					 \
		.maximum_mc = DT_INST_PROP(inst, maximum_millicelsius),						 \
	};                                                                               \
                                                                                     \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, my_sensor_init, NULL, &my_sensor_data_##inst, \
								 &my_sensor_config_##inst, POST_KERNEL,              \
								 CONFIG_SENSOR_INIT_PRIORITY, &my_sensor_api);

DT_INST_FOREACH_STATUS_OKAY(MY_SENSOR_DEFINE)
