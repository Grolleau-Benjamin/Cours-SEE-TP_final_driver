#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define SENSOR_NODE DT_NODELABEL(my_sensor)

SENSOR_DT_READ_IODEV(bme280_iodev, SENSOR_NODE,
                    {SENSOR_CHAN_AMBIENT_TEMP, 0},
                    {SENSOR_CHAN_PRESS, 0},
                    {SENSOR_CHAN_HUMIDITY, 0});
RTIO_DEFINE(bme280_rtio, 1, 1);

int main(void)
{
    const struct device *dev = DEVICE_DT_GET(SENSOR_NODE);
    const struct sensor_decoder_api *decoder;
    const enum sensor_channel channels[] = {
        SENSOR_CHAN_AMBIENT_TEMP, SENSOR_CHAN_PRESS, SENSOR_CHAN_HUMIDITY,
    };
    const char *const names[] = {"Temperature", "Pression", "Humidite"};
    const char *const units[] = {"deg C", "kPa", "%RH"};
    uint8_t buffer[20];

    if (!device_is_ready(dev)) {
        printk("Capteur BME280 non disponible\n");
        return -ENODEV;
    }
    int ret = sensor_get_decoder(dev, &decoder);
    if (ret < 0) {
        printk("Erreur get_decoder : %d\n", ret);
        return ret;
    }

    while (true) {
        ret = sensor_read(&bme280_iodev, &bme280_rtio, buffer, sizeof(buffer));
        if (ret < 0) {
            printk("Erreur read : %d\n", ret);
        } else {
            for (size_t i = 0; i < ARRAY_SIZE(channels); i++) {
                struct sensor_q31_data value;
                struct sensor_decode_context ctx =
                    SENSOR_DECODE_CONTEXT_INIT(decoder, buffer, channels[i], 0);

                ret = sensor_decode(&ctx, &value, 1);
                if (ret != 1) {
                    printk("Erreur decode %s : %d\n", names[i], ret);
                    continue;
                }
                printk("%s : %" PRIsensor_q31_data " %s\n", names[i],
                       PRIsensor_q31_data_arg(value, 0), units[i]);
            }
        }
        k_sleep(K_SECONDS(20));
    }
}
