#define DT_DRV_COMPAT bosch_my_bme280

#include <errno.h>
#include <stdint.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/work.h>
#include <zephyr/sys/byteorder.h>

#include "my_sensor.h"

LOG_MODULE_REGISTER(MYSENSOR, CONFIG_SENSOR_LOG_LEVEL);

#define MYSENSOR_START_UP_TIME_MS 2


// Fonctions I2C      
                                                         
static inline int mysensor_bus_check(const struct device *dev)
{
    const struct mysensor_config *cfg = dev->config;

    if (!i2c_is_ready_dt(&cfg->i2c))
    {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    return 0;
}


static inline int mysensor_reg_read(
    const struct device *dev,
    uint8_t start,
    uint8_t *buf,
    int size)
{
    const struct mysensor_config *cfg = dev->config;

    return i2c_burst_read_dt(
        &cfg->i2c,
        start,
        buf,
        size);
}


static inline int mysensor_reg_write(
    const struct device *dev,
    uint8_t reg,
    uint8_t val)
{
    const struct mysensor_config *cfg = dev->config;

    return i2c_reg_write_byte_dt(
        &cfg->i2c,
        reg,
        val);
}


// Attente du BME280                                                           */

static int mysensor_wait_until_ready(
    const struct device *dev,
    k_timeout_t timeout)
{
    uint8_t status;
    int ret;

    k_timepoint_t end = sys_timepoint_calc(timeout);

    while (true)
    {
        ret = mysensor_reg_read(
            dev,
            MYSENSOR_REG_STATUS,
            &status,
            1);

        if (ret < 0)
        {
            return ret;
        }

        /*
         * Le capteur est disponible lorsque :
         * - MEASURING = 0
         * - IM_UPDATE = 0
         */
        if ((status &
             (MYSENSOR_STATUS_MEASURING |
              MYSENSOR_STATUS_IM_UPDATE)) == 0)
        {
            return 0;
        }

        if (sys_timepoint_expired(end))
        {
            return -ETIMEDOUT;
        }

        k_msleep(1);
    }
}


// Attributs

static int mysensor_attr_set(
    const struct device *dev,
    enum sensor_channel chan,
    enum sensor_attribute attr,
    const struct sensor_value *val)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(chan);
    ARG_UNUSED(attr);
    ARG_UNUSED(val);

    return -ENOTSUP;
}


// Lecture des coefficients de calibration                                    

static int mysensor_read_calibration(const struct device *dev)
{
    struct mysensor_data *data = dev->data;

    /*
     * Premier bloc :
     * 0x88 -> 0xA1
     *
     * Contient :
     * dig_T1 ... dig_T3
     * dig_P1 ... dig_P9
     * dig_H1
     */
    uint8_t calib1[26];

    /*
     * Deuxieme bloc :
     * 0xE1 -> 0xE7
     *
     * Contient :
     * dig_H2 ... dig_H6
     */
    uint8_t calib2[7];

    int ret;

    /*
     * Lecture du premier bloc.
     */
    ret = mysensor_reg_read(
        dev,
        MYSENSOR_REG_COMP_START,
        calib1,
        sizeof(calib1));

    if (ret < 0)
    {
        LOG_ERR(
            "Unable to read BME280 calibration block 1");

        return ret;
    }

    /*
     * Coefficients de temperature.
     */
    data->dig_t1 =
        sys_get_le16(&calib1[0]);

    data->dig_t2 =
        (int16_t)sys_get_le16(&calib1[2]);

    data->dig_t3 =
        (int16_t)sys_get_le16(&calib1[4]);

    /*
     * Coefficients de pression.
     */
    data->dig_p1 =
        sys_get_le16(&calib1[6]);

    data->dig_p2 =
        (int16_t)sys_get_le16(&calib1[8]);

    data->dig_p3 =
        (int16_t)sys_get_le16(&calib1[10]);

    data->dig_p4 =
        (int16_t)sys_get_le16(&calib1[12]);

    data->dig_p5 =
        (int16_t)sys_get_le16(&calib1[14]);

    data->dig_p6 =
        (int16_t)sys_get_le16(&calib1[16]);

    data->dig_p7 =
        (int16_t)sys_get_le16(&calib1[18]);

    data->dig_p8 =
        (int16_t)sys_get_le16(&calib1[20]);

    data->dig_p9 =
        (int16_t)sys_get_le16(&calib1[22]);

    /*
     * dig_H1 se trouve a l'adresse 0xA1.
     *
     * 0xA1 - 0x88 = 25.
     */
    data->dig_h1 = calib1[25];

    /*
     * Lecture du deuxieme bloc de calibration.
     */
    ret = mysensor_reg_read(
        dev,
        MYSENSOR_REG_HUM_COMP_PART2,
        calib2,
        sizeof(calib2));

    if (ret < 0)
    {
        LOG_ERR(
            "Unable to read BME280 calibration block 2");

        return ret;
    }

    /*
     * Coefficients d'humidite.
     */
    data->dig_h2 =
        (int16_t)sys_get_le16(&calib2[0]);

    data->dig_h3 =
        calib2[2];

    /*
     * dig_H4 est signe sur 12 bits.
     *
     * E4 -> bits 11..4
     * E5[3:0] -> bits 3..0
     */
    data->dig_h4 =
        ((int16_t)calib2[3] << 4) |
        (calib2[4] & 0x0F);

    if ((data->dig_h4 & 0x0800) != 0)
    {
        data->dig_h4 |= (int16_t)0xF000;
    }

    /*
     * dig_H5 est signe sur 12 bits.
     *
     * E6 -> bits 11..4
     * E5[7:4] -> bits 3..0
     */
    data->dig_h5 =
        ((int16_t)calib2[5] << 4) |
        (calib2[4] >> 4);

    if ((data->dig_h5 & 0x0800) != 0)
    {
        data->dig_h5 |= (int16_t)0xF000;
    }

    data->dig_h6 =
        (int8_t)calib2[6];

    LOG_INF("BME280 calibration loaded");

    return 0;
}

 // Compensation de la temperature 

static int32_t mysensor_compensate_temperature(struct mysensor_data *data, int32_t adc_t)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;

    var1 =
        ((((adc_t >> 3) -
           ((int32_t)data->dig_t1 << 1))) *
         ((int32_t)data->dig_t2)) >> 11;

    var2 =
        (((((adc_t >> 4) -
            ((int32_t)data->dig_t1)) *
           ((adc_t >> 4) -
            ((int32_t)data->dig_t1))) >> 12) *
         ((int32_t)data->dig_t3)) >> 14;

    /*
     * t_fine est utilise ensuite par les
     * compensations de pression et d'humidite.
     */
    data->t_fine = var1 + var2;

    /*
     * Resultat Bosch en 0.01 degC.
     */
    temperature =
        (data->t_fine * 5 + 128) >> 8;

    /*
     * Conversion en milli-degC.
     *
     * Exemple :
     * 25.34 degC -> 25340
     */
    return temperature * 10;
}


// Compensation de la pression                                                */


static int32_t mysensor_compensate_pressure(
    struct mysensor_data *data,
    int32_t adc_p)
{
    int64_t var1;
    int64_t var2;
    int64_t p;

    var1 =
        ((int64_t)data->t_fine) - 128000;

    var2 =
        var1 * var1 *
        (int64_t)data->dig_p6;

    var2 =
        var2 +
        ((var1 *
          (int64_t)data->dig_p5) << 17);

    var2 =
        var2 +
        (((int64_t)data->dig_p4) << 35);

    var1 =
        ((var1 * var1 *
          (int64_t)data->dig_p3) >> 8) +
        ((var1 *
          (int64_t)data->dig_p2) << 12);

    var1 =
        ((((INT64_C(1) << 47) + var1) *
          (int64_t)data->dig_p1) >> 33);

    /*
     * Protection contre une division par zero.
     */
    if (var1 == 0)
    {
        return 0;
    }

    p = 1048576 - adc_p;

    p =
        (((p << 31) - var2) * 3125) /
        var1;

    var1 =
        ((int64_t)data->dig_p9 *
         (p >> 13) *
         (p >> 13)) >> 25;

    var2 =
        ((int64_t)data->dig_p8 *
         p) >> 19;

    p =
        ((p + var1 + var2) >> 8) +
        ((int64_t)data->dig_p7 << 4);

    /*
     * La formule fournit une pression
     * au format Pa Q24.8.
     *
     * Apres >> 8 : pression en Pa.
     *
     * Zephyr utilise les kPa pour SENSOR_CHAN_PRESS.
     * 101325 Pa = 101325 milli-kPa.
     */
    return (int32_t)(p >> 8);
}


// Compensation de l'humidite

static int32_t mysensor_compensate_humidity(
    struct mysensor_data *data,
    int32_t adc_h)
{
    int32_t v;
    int64_t humidity;

    v = data->t_fine - 76800;

    v =
        (((((adc_h << 14) -
            ((int32_t)data->dig_h4 << 20) -
            ((int32_t)data->dig_h5 * v)) +
           16384) >> 15) *
         (((((((v *
               (int32_t)data->dig_h6) >> 10) *
             (((v *
                (int32_t)data->dig_h3) >> 11) +
              32768)) >> 10) +
           2097152) *
          (int32_t)data->dig_h2 +
          8192) >> 14));

    v =
        v -
        (((((v >> 15) *
            (v >> 15)) >> 7) *
          (int32_t)data->dig_h1) >> 4);

    /*
     * Limites definies par la formule Bosch.
     */
    if (v < 0)
    {
        v = 0;
    }
    else if (v > 419430400)
    {
        v = 419430400;
    }

    /*
     * Humidite en %RH au format Q22.10.
     */
    humidity =
        (int64_t)(v >> 12);

    /*
     * Conversion vers milli-%RH.
     *
     * Exemple :
     * 52.5 % -> 52500
     */
    humidity =
        (humidity * 1000) / 1024;

    return (int32_t)humidity;
}


// Lecture des donnees brutes + compensation

static int mysensor_read_raw(
    const struct device *dev,
    struct mysensor_raw *raw)
{
    struct mysensor_data *data = dev->data;

    uint8_t buffer[8];

    int32_t adc_p;
    int32_t adc_t;
    int32_t adc_h;

    int ret;

    /*
     * Attendre que le capteur ait termine
     * sa mesure.
     */
    ret = mysensor_wait_until_ready(
        dev,
        K_MSEC(100));

    if (ret < 0)
    {
        return ret;
    }

    /*
     * Lecture de 8 octets :
     *
     * F7 F8 F9 : pression
     * FA FB FC : temperature
     * FD FE    : humidite
     */
    ret = mysensor_reg_read(
        dev,
        MYSENSOR_REG_PRESS_MSB,
        buffer,
        sizeof(buffer));

    if (ret < 0)
    {
        return ret;
    }

    /*
     * Pression brute sur 20 bits.
     */
    adc_p =
        ((int32_t)buffer[0] << 12) |
        ((int32_t)buffer[1] << 4) |
        ((int32_t)buffer[2] >> 4);

    /*
     * Temperature brute sur 20 bits.
     */
    adc_t =
        ((int32_t)buffer[3] << 12) |
        ((int32_t)buffer[4] << 4) |
        ((int32_t)buffer[5] >> 4);

    /*
     * Humidite brute sur 16 bits.
     */
    adc_h =
        ((int32_t)buffer[6] << 8) |
        (int32_t)buffer[7];

    /*
     * IMPORTANT :
     *
     * La temperature doit etre compensee
     * en premier afin de calculer t_fine.
     */
    raw->temperature =
        mysensor_compensate_temperature(
            data,
            adc_t);

    raw->pressure =
        mysensor_compensate_pressure(
            data,
            adc_p);

    raw->humidity =
        mysensor_compensate_humidity(
            data,
            adc_h);

    return 0;
}

//  Canaux supportes    

static bool mysensor_channel_supported(
    struct sensor_chan_spec channel)
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


// Travail RTIO

static void mysensor_read_work(
    struct rtio_iodev_sqe *request)
{
    const struct sensor_read_config *read_cfg =
        request->sqe.iodev->data;

    struct mysensor_raw raw;

    uint8_t *buffer;
    uint32_t buffer_size;

    int ret;

    ret = rtio_sqe_rx_buf(
        request,
        MYSENSOR_FRAME_SIZE,
        MYSENSOR_FRAME_SIZE,
        &buffer,
        &buffer_size);

    if (ret < 0)
    {
        rtio_iodev_sqe_err(
            request,
            ret);

        return;
    }

    ret = mysensor_read_raw(
        read_cfg->sensor,
        &raw);

    if (ret < 0)
    {
        rtio_iodev_sqe_err(
            request,
            ret);

        return;
    }

    /*
     * Format de la trame :
     *
     * 0..7   : timestamp en ns
     * 8..11  : temperature en milli-degC
     * 12..15 : pression en Pa = milli-kPa
     * 16..19 : humidite en milli-%RH
     */
    sys_put_le64(
        k_ticks_to_ns_floor64(
            k_uptime_ticks()),
        buffer);

    sys_put_le32(
        (uint32_t)raw.temperature,
        buffer + 8);

    sys_put_le32(
        (uint32_t)raw.pressure,
        buffer + 12);

    sys_put_le32(
        (uint32_t)raw.humidity,
        buffer + 16);

    rtio_iodev_sqe_ok(
        request,
        0);
}


// Soumission d'une requete Sensor API

static void mysensor_submit(
    const struct device *dev,
    struct rtio_iodev_sqe *request)
{
    ARG_UNUSED(dev);

    const struct sensor_read_config *read_cfg =
        request->sqe.iodev->data;

    if (request->sqe.op != RTIO_OP_RX ||
        read_cfg->is_streaming ||
        read_cfg->count == 0)
    {
        rtio_iodev_sqe_err(
            request,
            -ENOTSUP);

        return;
    }

    for (size_t i = 0;
         i < read_cfg->count;
         i++)
    {
        if (!mysensor_channel_supported(
                read_cfg->channels[i]))
        {
            rtio_iodev_sqe_err(
                request,
                -ENOTSUP);

            return;
        }
    }

    struct rtio_work_req *work =
        rtio_work_req_alloc();

    if (work == NULL)
    {
        rtio_iodev_sqe_err(
            request,
            -ENOMEM);

        return;
    }

    rtio_work_req_submit(
        work,
        request,
        mysensor_read_work);
}


// Decoder : nombre de frames                                                 

static int mysensor_get_frame_count(
    const uint8_t *buffer,
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


// Decoder : taille                                                           

static int mysensor_get_size_info(
    struct sensor_chan_spec channel,
    size_t *base_size,
    size_t *frame_size)
{
    if (!mysensor_channel_supported(channel))
    {
        return -ENOTSUP;
    }

    *base_size =
        sizeof(struct sensor_q31_data);

    *frame_size =
        sizeof(struct sensor_q31_sample_data);

    return 0;
}


// Conversion milli-unite -> Q31                                              

static q31_t mysensor_to_q31(
    int32_t milli_value)
{
    return (q31_t)(
        ((int64_t)milli_value *
         (INT64_C(1) <<
          (31 - MYSENSOR_Q31_SHIFT))) /
        1000);
}


// Decoder                                                                   

static int mysensor_decode(
    const uint8_t *buffer,
    struct sensor_chan_spec channel,
    uint32_t *fit,
    uint16_t max_count,
    void *output)
{
    if (!mysensor_channel_supported(channel))
    {
        return -EINVAL;
    }

    if (*fit != 0 ||
        max_count == 0)
    {
        return 0;
    }

    struct sensor_q31_data *decoded =
        output;

    int32_t milli_value;

    switch (channel.chan_type)
    {
    case SENSOR_CHAN_AMBIENT_TEMP:

        milli_value =
            (int32_t)sys_get_le32(
                buffer + 8);

        break;

    case SENSOR_CHAN_PRESS:

        milli_value =
            (int32_t)sys_get_le32(
                buffer + 12);

        break;

    case SENSOR_CHAN_HUMIDITY:

        milli_value =
            (int32_t)sys_get_le32(
                buffer + 16);

        break;

    default:
        return -ENOTSUP;
    }

    *decoded =
        (struct sensor_q31_data){0};

    decoded->header.base_timestamp_ns =
        sys_get_le64(buffer);

    decoded->header.reading_count = 1;

    decoded->shift =
        MYSENSOR_Q31_SHIFT;

    decoded->readings[0].value =
        mysensor_to_q31(
            milli_value);

    *fit = 1;

    return 1;
}


// API Decoder        

static const struct sensor_decoder_api
mysensor_decoder_api =
{
    .get_frame_count =
        mysensor_get_frame_count,

    .get_size_info =
        mysensor_get_size_info,

    .decode =
        mysensor_decode,
};


static int mysensor_get_decoder(
    const struct device *dev,
    const struct sensor_decoder_api **decoder)
{
    ARG_UNUSED(dev);

    *decoder =
        &mysensor_decoder_api;

    return 0;
}


// Initialisation du BME280                                                  

static int mysensor_init(
    const struct device *dev)
{
    uint8_t id;
    int ret;

    /*
     * 1. Verification du bus I2C.
     */
    ret =
        mysensor_bus_check(dev);

    if (ret < 0)
    {
        LOG_ERR(
            "BME280 I2C bus unavailable");

        return ret;
    }

    /*
     * Temps de demarrage du BME280.
     */
    k_msleep(
        MYSENSOR_START_UP_TIME_MS);

    /*
     * 2. Lecture et verification du CHIP_ID.
     */
    ret =
        mysensor_reg_read(
            dev,
            MYSENSOR_REG_ID,
            &id,
            1);

    if (ret < 0)
    {
        LOG_ERR(
            "Unable to read BME280 ID (%d)",
            ret);

        return ret;
    }

    if (id != MYSENSOR_CHIP_ID)
    {
        LOG_ERR(
            "Invalid BME280 ID: 0x%02x",
            id);

        return -ENODEV;
    }

    LOG_INF(
        "BME280 detected, ID=0x%02x",
        id);

    /*
     * 3. Reset logiciel.
     */
    ret =
        mysensor_reg_write(
            dev,
            MYSENSOR_REG_RESET,
            MYSENSOR_CMD_SOFT_RESET);

    if (ret < 0)
    {
        LOG_ERR(
            "Unable to reset BME280 (%d)",
            ret);

        return ret;
    }

    /*
     * Le BME280 a besoin d'un petit delai
     * apres le reset.
     */
    k_msleep(
        MYSENSOR_START_UP_TIME_MS);

    /*
     * 4. Attente de la fin de la copie NVM.
     */
    ret =
        mysensor_wait_until_ready(
            dev,
            K_MSEC(100));

    if (ret < 0)
    {
        LOG_ERR(
            "BME280 not ready after reset (%d)",
            ret);

        return ret;
    }

    /*
     * 5. Recuperation des coefficients
     * de calibration.
     */
    ret =
        mysensor_read_calibration(dev);

    if (ret < 0)
    {
        LOG_ERR(
            "Unable to read BME280 calibration (%d)",
            ret);

        return ret;
    }

    /*
     * 6. Configuration de l'humidite.
     *
     * CTRL_HUM doit etre configure avant
     * CTRL_MEAS.
     */
    ret =
        mysensor_reg_write(
            dev,
            MYSENSOR_REG_CTRL_HUM,
            MYSENSOR_HUMIDITY_OVER);

    if (ret < 0)
    {
        LOG_ERR(
            "Unable to configure CTRL_HUM (%d)",
            ret);

        return ret;
    }

    /*
     * 7. Configuration du filtre
     * et du temps de standby.
     */
    ret =
        mysensor_reg_write(
            dev,
            MYSENSOR_REG_CONFIG,
            MYSENSOR_CONFIG_VAL);

    if (ret < 0)
    {
        LOG_ERR(
            "Unable to configure CONFIG (%d)",
            ret);

        return ret;
    }

    /*
     * 8. Configuration :
     * - temperature
     * - pression
     * - mode normal
     *
     * Cette ecriture valide egalement
     * CTRL_HUM.
     */
    ret =
        mysensor_reg_write(
            dev,
            MYSENSOR_REG_CTRL_MEAS,
            MYSENSOR_CTRL_MEAS_VAL);

    if (ret < 0)
    {
        LOG_ERR(
            "Unable to configure CTRL_MEAS (%d)",
            ret);

        return ret;
    }

    LOG_INF(
        "BME280 initialized");

    return 0;
}


// Sensor Driver API                                                          

static DEVICE_API(
    sensor,
    mysensor_driver_api) =
{
    .attr_set =
        mysensor_attr_set,

    .submit =
        mysensor_submit,

    .get_decoder =
        mysensor_get_decoder,
};


// Creation des instances Devicetree
#define MYSENSOR_INIT(n)                                            \
    static const struct mysensor_config mysensor_config_##n = {     \
        .i2c = I2C_DT_SPEC_INST_GET(n),                             \
    };                                                              \
                                                                    \
    static struct mysensor_data mysensor_data_##n;                  \
                                                                    \
    SENSOR_DEVICE_DT_INST_DEFINE(                                   \
        n,                                                          \
        mysensor_init,                                              \
        NULL,                                                       \
        &mysensor_data_##n,                                         \
        &mysensor_config_##n,                                       \
        POST_KERNEL,                                                \
        CONFIG_SENSOR_INIT_PRIORITY,                                \
        &mysensor_driver_api);


DT_INST_FOREACH_STATUS_OKAY(MYSENSOR_INIT)