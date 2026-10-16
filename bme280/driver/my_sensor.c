#define DT_DRV_COMPAT /* TODO */

// TODO : toutes les fonctions ne sont pas notées. il manque les fonctions de
// - recuperation des parametres de compensation
// - compensation des valeurs brutes
// - autres si vous avez besoin

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/work.h>
#include <zephyr/sys/byteorder.h>

#include "mysensor.h"

LOG_MODULE_REGISTER(MYSENSOR, CONFIG_SENSOR_LOG_LEVEL);

#define MYSENSOR_START_UP_TIME_MS 2

struct my_bme280_data d;

static inline int mysensor_bus_check(const struct device *dev)
{
	struct my_bme280_config *cfg = dev->config;
	if (!cfg->i2c.i2c_is_ready_dt()) {
		return -ENOSYS;
	}
	return 0;
}

static inline int mysensor_reg_read(const struct device *dev,
									uint8_t start, uint8_t *buf, int size)
{
	struct my_bme280_config *cfg = dev->config;
	if (cfg->i2c.i2c_read_dt(&cfg->i2c, start, buf, size)) {
		return -ENOSYS;
	}
	return 0;
}

static inline int mysensor_reg_write(const struct device *dev,
									 uint8_t reg, uint8_t val)
{
	struct my_bme280_config *cfg = dev->config;
	if (cfg->i2c.i2c_write_dt(&cfg->i2c, reg, &val, 1)) {
		return -ENOSYS;
	}
	return 0;
}

static int mysensor_wait_until_ready(const struct device *dev,
									 k_timeout_t timeout)
{
	struct my_bme280_config *cfg = dev->config;
	uint8_t l_status;
	while (timeout.ticks > 0) {
		mysensor_reg_read(dev, MYSENSOR_REG_STATUS, &l_status, 1);
		if ((l_status & (MYSENSOR_STATUS_MEASURING | MYSENSOR_STATUS_IM_UPDATE)) == 0) {
			return 0;
		}
		k_sleep(K_MSEC(5));
		timeout.ticks--;
	}
	return -ENOSYS;
}

static int mysensor_attr_set(const struct device *dev,
							 enum sensor_channel chan,
							 enum sensor_attribute attr,
							 const struct sensor_value *val)
{
	// Pas utiliser dans notre cas
	return -ENOTSUP;
}

int32_t T_fine;

int32_t mysensor_compensate_temp(int32_t adc_T)
{
	int32_t var1, var2, T;


	var1 = ((((adc_T >> 3) - ((int32_t)d.dig_t1 <<1))) * ((int32_t)d.dig_t2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)d.dig_t1)) * ((adc_T >> 4) - ((int32_t)d.dig_t1))) >> 12) * ((int32_t)d.dig_t3)) >> 14;

	T_fine = var1 + var2;
	T = (T_fine * 5 + 128) >> 8;
	return T;
}

uint32_t mysensor_compensate_press(int32_t adc_P)
{
	int64_t var1, var2, P;
	var1 = ( (int64_t)T_fine) - 128000;
	var2 = var1 * var1 * (int64_t)d.dig_p6;
	var2 = var2 + ((var1*(int64_t)d.dig_p5) << 17) ;
	var2 = var2 + (((int64_t)d.dig_p4) << 35);
	var1 = ((var1 * var1 * (int64_t)d.dig_p3)>>8) + ((var1 * (int64_t)d.dig_p2) << 12) ;
	var1 = (((( (int64_t)1) << 47) +var1) ) * ( (int64_t) d.dig_p1) >>33;

	if (var1 == 0)
		return 0; // Return early to avoid division by 0

	P = 1048576-adc_P;
	P = (((P << 31)-var2) *3125) / var1;
	var1 = (((int64_t)d.dig_p9) * (P>>13) * (P>>13) ) >> 25;
	var2 = (((int64_t)d.dig_p8) * P) >> 19;
	P = ((P + var1 + var2) >> 8) + (((int64_t) d.dig_p7) << 4);
	return (uint32_t) P;
}

uint32_t mysensor_compensate_humidity(int32_t adc_H)
{
	int32_t v_x1_u32r;

	v_x1_u32r = (T_fine - ((int32_t) 76800) ) ;

	v_x1_u32r = (((((adc_H << 14) - (((int32_t)d.dig_h4) << 20) - (((int32_t)d.dig_h5) * v_x1_u32r)) 
				+ ((int32_t) 16384) ) >> 15) * (((((((v_x1_u32r *
				((int32_t) d.dig_h6)) >> 10) * (((v_x1_u32r * ((int32_t)d.dig_h3) ) >> 11) +
				((int32_t) 32768) ) ) >> 10) + ( (int32_t) 2097152)) * ((int32_t) d.dig_h2) +
				8192) >> 14));

	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15) ) >> 7) *
	((int32_t) d.dig_h1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t) (v_x1_u32r>>12);
}

/**
 * Cette fontion est appelée à l'init et permet de recuperer les 
 * parametres de compensation du capteur. Ces parametres sont stockés
 * dans la structure my_bme280_data.
 */
static int mysensor_read_calibration(const struct device *dev)
{
	uint8_t param_comp_temp_raw[6];
	uint8_t param_comp_press_raw[18];
	uint8_t param_comp_hum_raw[8];

	mysensor_reg_read(dev, MYSENSOR_REG_COMP_START, param_comp_temp_raw, 6);
	d.dig_t1 = sys_get_le16(param_comp_temp_raw[0]);
	d.dig_t2 = sys_get_le16(param_comp_temp_raw[2]);
	d.dig_t3 = sys_get_le16(param_comp_temp_raw[4]);

	mysensor_reg_read(dev, MYSENSOR_REG_COMP_START+6, param_comp_press_raw, 18);
	d.dig_p1 = sys_get_le16(param_comp_press_raw[0]);
	d.dig_p2 = sys_get_le16(param_comp_press_raw[2]);
	d.dig_p3 = sys_get_le16(param_comp_press_raw[4]);
	d.dig_p4 = sys_get_le16(param_comp_press_raw[6]);
	d.dig_p5 = sys_get_le16(param_comp_press_raw[8]);
	d.dig_p6 = sys_get_le16(param_comp_press_raw[10]);
	d.dig_p7 = sys_get_le16(param_comp_press_raw[12]);
	d.dig_p8 = sys_get_le16(param_comp_press_raw[14]);
	d.dig_p9 = sys_get_le16(param_comp_press_raw[16]);


	mysensor_reg_read(dev, MYSENSOR_REG_HUM_COMP_PART1, param_comp_hum_raw, 1);
	mysensor_reg_read(dev, MYSENSOR_REG_HUM_COMP_PART2, param_comp_hum_raw+1, 7);

	d.dig_h1 = (uint16_t)param_comp_hum_raw[0];
	d.dig_h2 = sys_get_le16(param_comp_hum_raw[1]);
	d.dig_h3 = (uint16_t)param_comp_hum_raw[3];

	uint8_t h4_raw_temp[2];
	uint8_t h5_raw_temp[2];

	uint8_t h4_raw_temp[0] = param_comp_hum_raw[4];
	uint8_t h4_raw_temp[1] = param_comp_hum_raw[5] & 0xF0; 

	uint8_t h5_raw_temp[0] = param_comp_hum_raw[4] & 0x0F;
	uint8_t h5_raw_temp[1] = param_comp_hum_raw[5]; 		

	d.dig_h4 = sys_get_le16(h4_raw_temp);
	d.dig_h5 = sys_get_le16(h5_raw_temp);
	d.dig_h6 = (uint16_t)param_comp_hum_raw[7];

	return -ENOSYS;
}

static int mysensor_read_raw(const struct device *dev,
							 struct mysensor_raw *raw)
{
	uint8_t l_buffer[8];

	int ret = mysensor_wait_until_ready(dev, K_MSEC(100));
	if (ret < 0) {
		return ret;
	}

	mysensor_reg_read(dev, MYSENSOR_REG_PRESS_MSB, l_buffer, sizeof(struct mysensor_raw));

	raw->temperature = sys_get_le(l_buffer + 3, 3);
	raw->pressure = sys_get_le(l_buffer + 0, 3);
	raw->humidity = sys_get_le(l_buffer + 6, 2);

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

	// TODO : remplir le buffer avec les valeurs brutes compensees et l'horodatage

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
		milli_value = sys_get_le32(buffer + 8);
		return -ENOSYS;

	case SENSOR_CHAN_PRESS:
		milli_value = sys_get_le32(buffer + 12);
		return -ENOSYS;

	case SENSOR_CHAN_HUMIDITY:
		milli_value = sys_get_le32(buffer + 16);
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
	.get_frame_count = &mysensor_get_frame_count,
	.get_size_info = &mysensor_get_size_info,
	.decode = &mysensor_decode
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

	uint8_t buff;
	mysensor_reg_read(dev, MYSENSOR_REG_ID, &buff, 1);
	if (buff != MYSENSOR_CHIP_ID)
		return -ENOSYS;

	if (!mysensor_reg_write(dev, MYSENSOR_REG_RESET, MYSENSOR_CMD_SOFT_RESET));
		return -ENOSYS;

	if (!mysensor_reg_write(dev, MYSENSOR_REG_CONFIG, MYSENSOR_CONFIG_VAL))
		return -ENOSYS;

	if (!mysensor_reg_write(dev, MYSENSOR_REG_CTRL_MEAS, MYSENSOR_CTRL_MEAS_VAL))
		return -ENOSYS;

	if (!mysensor_reg_write(dev, MYSENSOR_REG_CTRL_HUM, MYSENSOR_HUMIDITY_OVER))
		return -ENOSYS;

	return 0;

	
}

static DEVICE_API(sensor, mysensor_driver_api) = {
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