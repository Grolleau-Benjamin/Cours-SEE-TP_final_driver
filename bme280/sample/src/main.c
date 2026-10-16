// TODO : Lire les valeurs toutes les 20s et les afficher
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SENSOR_NODE DT_NODELABEL(my_sensor)

/*
 * Une seule lecture demande les trois canaux.
 */
SENSOR_DT_READ_IODEV(
    sensor_iodev,
    SENSOR_NODE,
    {SENSOR_CHAN_AMBIENT_TEMP, 0},
    {SENSOR_CHAN_PRESS, 0},
    {SENSOR_CHAN_HUMIDITY, 0}
);

RTIO_DEFINE(sensor_rtio, 1, 1);

int main(void)
{
    const struct device *dev =
        DEVICE_DT_GET(SENSOR_NODE);

    const struct sensor_decoder_api *decoder;

    uint8_t buffer[64];

    int ret;

    if (!device_is_ready(dev))
    {
        printk("BME280 non disponible\n");
        return 0;
    }

    ret =
        sensor_get_decoder(
            dev,
            &decoder);

    if (ret < 0)
    {
        printk(
            "Erreur get_decoder : %d\n",
            ret);

        return 0;
    }

    while (1)
    {
        /*
         * Une acquisition du BME280.
         */
        ret =
            sensor_read(
                &sensor_iodev,
                &sensor_rtio,
                buffer,
                sizeof(buffer));

        if (ret < 0)
        {
            printk(
                "Erreur read : %d\n",
                ret);

            k_msleep(20000);
            continue;
        }

        /*
         * Temperature
         */
        struct sensor_q31_data temperature;

        struct sensor_decode_context temp_ctx =
            SENSOR_DECODE_CONTEXT_INIT(
                decoder,
                buffer,
                SENSOR_CHAN_AMBIENT_TEMP,
                0);

        ret =
            sensor_decode(
                &temp_ctx,
                &temperature,
                1);

        if (ret != 1)
        {
            printk(
                "Erreur decode temperature : %d\n",
                ret);

            k_msleep(20000);
            continue;
        }

        /*
         * Pression
         */
        struct sensor_q31_data pressure;

        struct sensor_decode_context press_ctx =
            SENSOR_DECODE_CONTEXT_INIT(
                decoder,
                buffer,
                SENSOR_CHAN_PRESS,
                0);

        ret =
            sensor_decode(
                &press_ctx,
                &pressure,
                1);

        if (ret != 1)
        {
            printk(
                "Erreur decode pression : %d\n",
                ret);

            k_msleep(20000);
            continue;
        }

        /*
         * Humidite
         */
        struct sensor_q31_data humidity;

        struct sensor_decode_context hum_ctx =
            SENSOR_DECODE_CONTEXT_INIT(
                decoder,
                buffer,
                SENSOR_CHAN_HUMIDITY,
                0);

        ret =
            sensor_decode(
                &hum_ctx,
                &humidity,
                1);

        if (ret != 1)
        {
            printk(
                "Erreur decode humidite : %d\n",
                ret);

            k_msleep(20000);
            continue;
        }

        /*
         * Affichage.
         */
        printk(
            "Temperature : %"
            PRIsensor_q31_data
            " deg C\n",
            PRIsensor_q31_data_arg(
                temperature,
                0));

        printk(
            "Pression    : %"
            PRIsensor_q31_data
            " kPa\n",
            PRIsensor_q31_data_arg(
                pressure,
                0));

        printk(
            "Humidite    : %"
            PRIsensor_q31_data
            " %%\n",
            PRIsensor_q31_data_arg(
                humidity,
                0));

        printk("\n");

        /*
         * Consigne du prof :
         * lecture toutes les 20 secondes.
         */
        k_msleep(20000);
    }

    return 0;
}