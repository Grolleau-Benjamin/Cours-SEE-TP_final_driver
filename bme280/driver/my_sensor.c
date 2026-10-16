#define DT_DRV_COMPAT bosch_my_bme280

#include <errno.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/work.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/clock.h>

#include "my_sensor.h"

LOG_MODULE_REGISTER(MYSENSOR, CONFIG_SENSOR_LOG_LEVEL);

#define MYSENSOR_START_UP_TIME_MS 2

static inline int mysensor_bus_check(const struct device *dev)
{

	const struct mysensor_config *cfg = dev->config;
	if(!i2c_is_ready_dt(&cfg->i2c)){
		return 0;
	}
	return -ENODEV;

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
    for (;;) {
        uint8_t status;
        int ret = mysensor_reg_read(dev, MYSENSOR_REG_STATUS, &status, 1);
        if (ret < 0) {
            return ret;
        }
        if (!(status & (MYSENSOR_STATUS_MEASURING | MYSENSOR_STATUS_IM_UPDATE))) {
            return 0;
        }
        if (sys_timepoint_expired(deadline)) {
            return -ETIMEDOUT;
        }
        k_sleep(K_MSEC(5));
    }
}

static int mysensor_attr_set(const struct device *dev,
							 enum sensor_channel chan,
							 enum sensor_attribute attr,
							 const struct sensor_value *val)
{
	ARG_UNUSED(dev); //Pour enlever les warnings et autres lors de la compilation
	ARG_UNUSED(chan);
	ARG_UNUSED(attr);
	ARG_UNUSED(val);
	return -ENOTSUP;
}

/**
 * Cette fontion est appelée à l'init et permet de recuperer les 
 * parametres de compensation du capteur. Ces parametres sont stockés
 * dans la structure mysensor_data.
 */
static int16_t mysensor_signed12(uint16_t value)
{
    return (int16_t)((value & 0x800) ? (int32_t)value - 4096 : value);
}

static int mysensor_read_calibration(const struct device *dev)
{
    struct mysensor_data *d = dev->data;
    uint8_t tp[24], h[7], h1;
    int ret = mysensor_reg_read(dev, MYSENSOR_REG_COMP_START, tp, sizeof(tp));
    if (ret < 0) {
        return ret;
    }
    ret = mysensor_reg_read(dev, MYSENSOR_REG_HUM_COMP_PART1, &h1, 1);
    if (ret < 0) {
        return ret;
    }
    ret = mysensor_reg_read(dev, MYSENSOR_REG_HUM_COMP_PART2, h, sizeof(h));
    if (ret < 0) {
        return ret;
    }

    d->dig_t1 = sys_get_le16(tp);
    d->dig_t2 = (int16_t)sys_get_le16(tp + 2);
    d->dig_t3 = (int16_t)sys_get_le16(tp + 4);
    d->dig_p1 = sys_get_le16(tp + 6);
    d->dig_p2 = (int16_t)sys_get_le16(tp + 8);
    d->dig_p3 = (int16_t)sys_get_le16(tp + 10);
    d->dig_p4 = (int16_t)sys_get_le16(tp + 12);
    d->dig_p5 = (int16_t)sys_get_le16(tp + 14);
    d->dig_p6 = (int16_t)sys_get_le16(tp + 16);
    d->dig_p7 = (int16_t)sys_get_le16(tp + 18);
    d->dig_p8 = (int16_t)sys_get_le16(tp + 20);
    d->dig_p9 = (int16_t)sys_get_le16(tp + 22);
    d->dig_h1 = h1;
    d->dig_h2 = (int16_t)sys_get_le16(h);
    d->dig_h3 = h[2];
    d->dig_h4 = mysensor_signed12(((uint16_t)h[3] << 4) | (h[4] & 0x0f));
    d->dig_h5 = mysensor_signed12(((uint16_t)h[5] << 4) | (h[4] >> 4));
    d->dig_h6 = (int8_t)h[6];

    return d->dig_p1 == 0 ? -EINVAL : 0;

}

static int mysensor_read_raw(const struct device *dev,
							 struct mysensor_raw *raw)
{
    int ret = mysensor_wait_until_ready(dev, K_MSEC(200));
    if (ret < 0) {
        return ret;
    }
    uint8_t bytes[8];
    ret = mysensor_reg_read(dev, MYSENSOR_REG_PRESS_MSB, bytes, sizeof(bytes));
    if (ret < 0) {
        return ret;
    }
    raw->pressure = ((uint32_t)bytes[0] << 12) |
                    ((uint32_t)bytes[1] << 4) | (bytes[2] >> 4);
    raw->temperature = ((uint32_t)bytes[3] << 12) |
                       ((uint32_t)bytes[4] << 4) | (bytes[5] >> 4);
    raw->humidity = ((uint32_t)bytes[6] << 8) | bytes[7];

    if (raw->pressure == 0x80000 || raw->temperature == 0x80000 ||
        raw->humidity == 0x8000) {
        return -ENODATA;
    }
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

static int mysensor_compensate(const struct mysensor_data *d,
                              const struct mysensor_raw *raw,
                              struct mysensor_measurement *m)
{
    int64_t delta = (int64_t)(raw->temperature >> 4) - d->dig_t1;
    int64_t a = (((int64_t)(raw->temperature >> 3) -
                  (int64_t)d->dig_t1 * 2) * d->dig_t2) >> 11;
    int64_t b = (((delta * delta) >> 12) * d->dig_t3) >> 14;
    int64_t fine = a + b;
    m->temperature_mc = (int32_t)(((fine * 5 + 128) >> 8) * 10);

    a = fine - 128000;
    b = a * a * d->dig_p6 + a * d->dig_p5 * INT64_C(131072)
        + d->dig_p4 * INT64_C(34359738368);
    a = ((a * a * d->dig_p3) >> 8) + a * d->dig_p2 * 4096;
    a = ((INT64_C(140737488355328) + a) * d->dig_p1) >> 33;
    if (a == 0) {
        return -ERANGE;
    }
    int64_t p = (((1048576 - (int64_t)raw->pressure) *
                   INT64_C(2147483648) - b) * 3125) / a;
    a = (d->dig_p9 * (p >> 13) * (p >> 13)) >> 25;
    b = (d->dig_p8 * p) >> 19;
    p = ((p + a + b) >> 8) + (int64_t)d->dig_p7 * 16;
    if (p < 0 || p / 256 > INT32_MAX) {
        return -ERANGE;
    }
    m->pressure_mkpa = (int32_t)(p / 256);

    int64_t v = fine - 76800;
    a = (((int64_t)raw->humidity * 16384 -
          (int64_t)d->dig_h4 * 1048576 - d->dig_h5 * v + 16384) >> 15);
    b = ((((((v * d->dig_h6) >> 10) *
            (((v * d->dig_h3) >> 11) + 32768)) >> 10) + 2097152) *
          d->dig_h2 + 8192) >> 14;
    v = a * b;
    v -= (((((v >> 15) * (v >> 15)) >> 7) * d->dig_h1) >> 4);
    if (v < 0) {
        v = 0;
    } else if (v > 419430400) {
        v = 419430400;
    }
    m->humidity_mpercent = (int32_t)(((v >> 12) * 1000) / 1024);
    return 0;
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

    struct mysensor_measurement measurement;
    const struct mysensor_data *data = read_cfg->sensor->data;
    ret = mysensor_compensate(data, &raw, &measurement);
    if (ret < 0) {
        rtio_iodev_sqe_err(request, ret);
        return;
    }
	sys_put_le64(k_ticks_to_ns_floor64(k_uptime_ticks()), buffer);
	/* Valeurs compensees en milli-unites, attendues par le decodeur. */
	sys_put_le32((uint32_t)measurement.temperature_mc, buffer + 8);
	sys_put_le32((uint32_t)measurement.pressure_mkpa, buffer + 12);
	sys_put_le32((uint32_t)measurement.humidity_mpercent, buffer + 16);

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
		milli_value = (int32_t)sys_get_le32(buffer + 8);
		break;

	case SENSOR_CHAN_PRESS:
		milli_value = (int32_t)sys_get_le32(buffer + 12);
		break;

	case SENSOR_CHAN_HUMIDITY:
		milli_value = (int32_t)sys_get_le32(buffer + 16);
		break;

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
	int ret = mysensor_bus_check(dev);
	if (ret < 0) {
		return ret;
	}
	k_sleep(K_MSEC(MYSENSOR_START_UP_TIME_MS));

	uint8_t id;
	ret = mysensor_reg_read(dev, MYSENSOR_REG_ID, &id, 1);
	if (ret < 0) {
		return ret;
	}
	if (id != MYSENSOR_CHIP_ID) {
		return -ENODEV;
	}
	ret = mysensor_reg_write(dev, MYSENSOR_REG_RESET, MYSENSOR_CMD_SOFT_RESET);
	if (ret < 0) {
		return ret;
	}
	k_sleep(K_MSEC(MYSENSOR_START_UP_TIME_MS));
	ret = mysensor_wait_until_ready(dev, K_MSEC(200));
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
	/* CTRL_MEAS valide CTRL_HUM et lance les mesures en mode normal. */
	ret = mysensor_reg_write(dev, MYSENSOR_REG_CTRL_MEAS, MYSENSOR_CTRL_MEAS_VAL);
	if (ret < 0) {
		return ret;
	}
	k_sleep(K_MSEC(100));
	return mysensor_wait_until_ready(dev, K_MSEC(200));
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
