
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

int main(void)
{
    const struct device *sensor = DEVICE_DT_GET_ANY(bosh_my_bme280);

    if (!device_is_ready(sensor)) {
        printk("Erreur : capteur BME280 non disponible\n");
        return 0;
    }

    while (1) {
        struct sensor_value temp;
        struct sensor_value press;
        struct sensor_value hum;

        int ret = sensor_sample_fetch(sensor);

        if (ret == 0) {
            sensor_channel_get(sensor, SENSOR_CHAN_AMBIENT_TEMP, &temp);
            sensor_channel_get(sensor, SENSOR_CHAN_PRESS, &press);
            sensor_channel_get(sensor, SENSOR_CHAN_HUMIDITY, &hum);

            /* Temps écoulé depuis le démarrage */
            int64_t time_ms = k_uptime_get();
            int seconds = (time_ms / 1000) % 60;
            int minutes = (time_ms / 60000) % 60;
            int hours = (time_ms / 3600000);

            printk("\n");
            printk("====================================\n");
            printk("  MESURE BME280 - %02d:%02d:%02d\n",
                   hours, minutes, seconds);
            printk("====================================\n\n");

            printk("Temperature : %d.%06d C\n", temp.val1, temp.val2);
            printk("Pression    : %d.%06d Pa\n", press.val1, press.val2);
            printk("Humidite    : %d.%06d %%\n", hum.val1, hum.val2);

            printk("\n------------------------------------\n\n");
        } else {
            printk("Erreur lors de la lecture du BME280\n\n");
        }

        k_sleep(K_SECONDS(20));
    }

    return 0;
}