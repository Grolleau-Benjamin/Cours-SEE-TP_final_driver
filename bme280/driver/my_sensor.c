#define DT_DRV_COMPAT bosh_my_bme280

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
    const struct my_bme280_config *config = dev->config;
    return i2c_is_ready_dt(&config->i2c);
}


static inline int mysensor_reg_read(const struct device *dev,
                                    uint8_t start, uint8_t *buf, int size)
{
    const struct my_bme280_config *config = dev->config;

    return i2c_burst_read_dt(&config->i2c, start, buf, size);
}

static inline int mysensor_reg_write(const struct device *dev,
                                     uint8_t reg, uint8_t val)
{
    const struct my_bme280_config *cfg = dev->config;

    int ret = i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
    return ret;
}

static int mysensor_wait_until_ready(const struct device *dev,
                                     k_timeout_t timeout)
{
		/*
	 * TODO : attendre avec timeout que les bits MEASURING
	 * et IM_UPDATE de MYSENSOR_REG_STATUS soient à zéro.
	 * 
	 * 			#define MYSENSOR_STATUS_MEASURING 0x08
				#define MYSENSOR_STATUS_IM_UPDATE 0x01
	 */

    uint8_t status;
    int ret;
    int64_t start = k_uptime_ticks();

    ret = mysensor_reg_read(dev, MYSENSOR_REG_STATUS, &status, 1);
    if (ret < 0) {
        return ret;
    }

    while ((status & MYSENSOR_STATUS_MEASURING) ||
           (status & MYSENSOR_STATUS_IM_UPDATE)) {

        if (k_uptime_ticks() - start >= timeout.ticks) {
            return -ETIMEDOUT;
        }

        k_sleep(K_MSEC(1));

        ret = mysensor_reg_read(dev, MYSENSOR_REG_STATUS, &status, 1);
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}



static int mysensor_attr_set(const struct device *dev,
							 enum sensor_channel chan,
							 enum sensor_attribute attr,
							 const struct sensor_value *val)
{
		// Pas utiliser dans notre cas

	return -ENOTSUP;
}

static int mysensor_get_calib(const struct device *dev)
{
    struct my_bme280_data *d = dev->data;
    uint8_t b[24], h1, e[7];
    int ret;

    ret = mysensor_reg_read(dev, MYSENSOR_REG_COMP_START, b, 24);
    if (ret < 0) return ret;
    ret = mysensor_reg_read(dev, MYSENSOR_REG_HUM_COMP_PART1, &h1, 1);
    if (ret < 0) return ret;
    ret = mysensor_reg_read(dev, MYSENSOR_REG_HUM_COMP_PART2, e, 7);
    if (ret < 0) return ret;

    d->dig_t1 = sys_get_le16(&b[0]);
    d->dig_t2 = (int16_t)sys_get_le16(&b[2]);
    d->dig_t3 = (int16_t)sys_get_le16(&b[4]);
    d->dig_p1 = sys_get_le16(&b[6]);
    d->dig_p2 = (int16_t)sys_get_le16(&b[8]);
    d->dig_p3 = (int16_t)sys_get_le16(&b[10]);
    d->dig_p4 = (int16_t)sys_get_le16(&b[12]);
    d->dig_p5 = (int16_t)sys_get_le16(&b[14]);
    d->dig_p6 = (int16_t)sys_get_le16(&b[16]);
    d->dig_p7 = (int16_t)sys_get_le16(&b[18]);
    d->dig_p8 = (int16_t)sys_get_le16(&b[20]);
    d->dig_p9 = (int16_t)sys_get_le16(&b[22]);

    d->dig_h1 = h1;
    d->dig_h2 = (int16_t)sys_get_le16(&e[0]);
    d->dig_h3 = e[2];
    d->dig_h4 = (int16_t)((int8_t)e[3] * 16) | (e[4] & 0x0F);
    d->dig_h5 = (int16_t)((int8_t)e[5] * 16) | (e[4] >> 4);
    d->dig_h6 = (int8_t)e[6];

    return 0;
}

static int32_t mysensor_comp_temp(const struct my_bme280_data *d,
                                  int32_t raw, int32_t *t_fine)
{
    int32_t v1 = (((raw >> 3) - ((int32_t)d->dig_t1 << 1)) * d->dig_t2) >> 11;
    int32_t dt = (raw >> 4) - (int32_t)d->dig_t1;
    int32_t v2 = (((dt * dt) >> 12) * d->dig_t3) >> 14;

    *t_fine = v1 + v2;
    return (((*t_fine * 5) + 128) >> 8) * 10;
}

static int32_t mysensor_comp_press(const struct my_bme280_data *d,
                                   int32_t raw, int32_t t_fine)
{
    int32_t v1 = (t_fine >> 1) - 64000;
    int32_t v2 = (((v1 >> 2) * (v1 >> 2)) >> 11) * d->dig_p6;
    uint32_t p;

    v2 = v2 + ((v1 * d->dig_p5) << 1);
    v2 = (v2 >> 2) + ((int32_t)d->dig_p4 << 16);
    v1 = (((d->dig_p3 * (((v1 >> 2) * (v1 >> 2)) >> 13)) >> 3) +
          (((int32_t)d->dig_p2 * v1) >> 1)) >> 18;
    v1 = ((32768 + v1) * d->dig_p1) >> 15;

    if (v1 == 0) {
        return 0;
    }

    p = (((uint32_t)(1048576 - raw)) - (v2 >> 12)) * 3125;

    if (p < 0x80000000) {
        p = (p << 1) / (uint32_t)v1;
    } else {
        p = (p / (uint32_t)v1) * 2;
    }

    v1 = ((int32_t)d->dig_p9 *
          ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    v2 = ((int32_t)(p >> 2) * d->dig_p8) >> 13;
    p = (uint32_t)((int32_t)p + ((v1 + v2 + d->dig_p7) >> 4));

    return (int32_t)p;
}

static int32_t mysensor_comp_hum(const struct my_bme280_data *d,
                                 int32_t raw, int32_t t_fine)
{
    int32_t v = t_fine - 76800;

    v = (((((raw << 14) - ((int32_t)d->dig_h4 << 20) -
            (d->dig_h5 * v)) + 16384) >> 15) *
         (((((((v * d->dig_h6) >> 10) *
              (((v * d->dig_h3) >> 11) + 32768)) >> 10) +
             2097152) * d->dig_h2 + 8192) >> 14));

    v -= (((((v >> 15) * (v >> 15)) >> 7) * d->dig_h1) >> 4);
    v = CLAMP(v, 0, 419430400);
    v >>= 12;

    return (int32_t)(((int64_t)v * 1000) / 1024);
}

static int mysensor_read_raw(const struct device *dev,
                             struct mysensor_raw *raw)                           
{
	/*
	 * TODO : attendre que le capteur soit prêt, lire les 8 octets
	 * depuis MYSENSOR_REG_PRESS_MSB et extraire les trois valeurs.
	 */
    uint8_t buf[8];
    int test;

    test = mysensor_wait_until_ready(dev, K_MSEC(100));
    if (test < 0)
    {
        return test;
    }
    
    test = mysensor_reg_read(dev, MYSENSOR_REG_PRESS_MSB, buf, 8);
    if (test < 0)
    {
        return test;
    }

    raw->pressure    = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    raw->temperature = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);
    raw->humidity    = ((int32_t)buf[6] << 8)  | buf[7];

    return 0;

}

static int mysensor_measure(const struct device *dev)
{
    struct my_bme280_data *d = dev->data;
    struct mysensor_raw raw;
    int32_t t_fine;
    int ret = mysensor_read_raw(dev, &raw);

    if (ret < 0) return ret;

    d->temp_milli = mysensor_comp_temp(d, raw.temperature, &t_fine);
    d->press_milli = mysensor_comp_press(d, raw.pressure, t_fine);
    d->hum_milli = mysensor_comp_hum(d, raw.humidity, t_fine);

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

    ret = mysensor_measure(read_cfg->sensor);
    if (ret < 0)
    {
        rtio_iodev_sqe_err(request, ret);
        return;
    }

    const struct my_bme280_data *d = read_cfg->sensor->data;

    sys_put_le64(k_ticks_to_ns_floor64(k_uptime_ticks()), buffer);
    sys_put_le32(d->temp_milli, buffer + 8);
    sys_put_le32(d->press_milli, buffer + 12);
    sys_put_le32(d->hum_milli, buffer + 16);

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
		milli_value = sys_get_le32(buffer + 8);
		break;

	case SENSOR_CHAN_PRESS:
		milli_value = sys_get_le32(buffer + 12);
		break;

	case SENSOR_CHAN_HUMIDITY:
		milli_value = sys_get_le32(buffer + 16);
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

static int mysensor_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    return mysensor_measure(dev);
}

static int mysensor_channel_get(const struct device *dev, enum sensor_channel chan,
                                struct sensor_value *val)
{
    const struct my_bme280_data *d = dev->data;
    int32_t m;

    switch (chan) {
    case SENSOR_CHAN_AMBIENT_TEMP: m = d->temp_milli;  break;
    case SENSOR_CHAN_PRESS:        m = d->press_milli; break;  
    case SENSOR_CHAN_HUMIDITY:     m = d->hum_milli;   break;
    default: return -ENOTSUP;
    }
    val->val1 = m / 1000;
    val->val2 = (m % 1000) * 1000;
    return 0;
}

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
    uint8_t id;
    int ret;

    if (!mysensor_bus_check(dev)) {
        return -ENODEV;
    }

    ret = mysensor_reg_read(dev, MYSENSOR_REG_ID, &id, 1);
    if (ret < 0) {
        return ret;
    }
    if (id != MYSENSOR_CHIP_ID) {
        LOG_ERR("Mauvais ID: 0x%02x", id);
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

    ret = mysensor_get_calib(dev);
    if (ret < 0) {
        return ret;
    }

    /* CTRL_HUM avant CTRL_MEAS (pris en compte seulement après l'écriture de CTRL_MEAS) */
    ret = mysensor_reg_write(dev, MYSENSOR_REG_CTRL_HUM, MYSENSOR_HUMIDITY_OVER);
    if (ret < 0) {
        return ret;
    }

    ret = mysensor_reg_write(dev, MYSENSOR_REG_CONFIG, MYSENSOR_CONFIG_VAL);
    if (ret < 0) {
        return ret;
    }
    /* En dernier : passe en mode normal */
    return mysensor_reg_write(dev, MYSENSOR_REG_CTRL_MEAS, MYSENSOR_CTRL_MEAS_VAL);
}

static DEVICE_API(sensor, mysensor_driver_api) = {
	// TODO
    .attr_set = mysensor_attr_set,
    .sample_fetch = mysensor_sample_fetch,
    .channel_get = mysensor_channel_get,
    .get_decoder = mysensor_get_decoder,
    .submit = mysensor_submit,
};

#define MYSENSOR_INIT(n)                                        \
	static const struct my_bme280_config mysensor_config##n = { \
		.i2c = I2C_DT_SPEC_INST_GET(n),                         \
	};                                                          \
	static struct my_bme280_data mysensor_data##n;              \
	SENSOR_DEVICE_DT_INST_DEFINE(n, mysensor_init, NULL,        \
								 &mysensor_data##n,            \
								 &mysensor_config##n,          \
								 POST_KERNEL,                   \
								 CONFIG_SENSOR_INIT_PRIORITY,   \
								 &mysensor_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MYSENSOR_INIT)