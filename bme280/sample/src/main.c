// TODO : Lire les valeurs toutes les 20s et les afficher
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SENSOR_NODE DT_NODELABEL(my_sensor)
SENSOR_DT_READ_IODEV(temp_iodev, SENSOR_NODE, {SENSOR_CHAN_AMBIENT_TEMP, 0});
SENSOR_DT_READ_IODEV(press_iodev, SENSOR_NODE, {SENSOR_CHAN_PRESS, 0});
SENSOR_DT_READ_IODEV(hum_iodev, SENSOR_NODE, {SENSOR_CHAN_HUMIDITY, 0});
RTIO_DEFINE(temp_rtio, 1, 1);
RTIO_DEFINE(press_rtio, 1, 1);
RTIO_DEFINE(hum_rtio, 1, 1);

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(SENSOR_NODE);
	const struct sensor_decoder_api *decoder;

	uint8_t buffer_temp[32];
    uint8_t buffer_pres[32];
    uint8_t buffer_hum[32];

	if (!device_is_ready(dev)) {
		printk("Capteur non disponible\n");
		return 0;
	}
	int ret = sensor_get_decoder(dev, &decoder);
	if (ret < 0) {
		printk("Erreur get_decoder : %d\n", ret);
		return 0;
	}

    ret = sensor_read(&temp_iodev, &temp_rtio, buffer_temp, sizeof(buffer_temp));
    if (ret < 0) {
        printk("Erreur read : %d\n", ret);
        return 0;
    }

    ret = sensor_read(&press_iodev, &press_rtio, buffer_press, sizeof(buffer_press));
    if (ret < 0) {
        printk("Erreur read : %d\n", ret);
        return 0;
    }

    ret = sensor_read(&hum_iodev, &hum_rtio, buffer_hum, sizeof(buffer_hum));
    if (ret < 0) {
        printk("Erreur read : %d\n", ret);
        return 0;
    }

		struct sensor_q31_data temperature;
        struct sensor_q31_data pressure;
        struct sensor_q31_data humidity; 
		struct sensor_decode_context temp_decode_ctx =
			SENSOR_DECODE_CONTEXT_INIT(decoder, buffer_temp, SENSOR_CHAN_AMBIENT_TEMP, 0);
        struct sensor_decode_context press_decode_ctx =
			SENSOR_DECODE_CONTEXT_INIT(decoder, buffer_pres, SENSOR_CHAN_PRESS, 0);
        struct sensor_decode_context hum_decode_ctx =
			SENSOR_DECODE_CONTEXT_INIT(decoder, buffer_hum, SENSOR_CHAN_HUMIDITY, 0);

		while (1)
        {
            uint8_t *buff;
            sensor_decode(&temp_decode_ctx, &temperature);
            sensor_decode(&press_decode_ctx, &pressure);
            sensor_decode(&hum_decode_ctx, &humidity);

            printk("T : %" PRIsensor_q31_data " °C | P : %" PRIsensor_q31_data " Pa | H : %" PRIsensor_q31_data " percent \n",
		       PRIsensor_q31_data_arg(temperature, 0), PRIsensor_q31_data_arg(pressure, 0), PRIsensor_q31_data_arg(humidity, 0));

            k_msleep(20000);
        }
    }

	return 0;
}