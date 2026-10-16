#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SENSOR_NODE DT_NODELABEL(my_sensor)

SENSOR_DT_READ_IODEV(temp_iodev, SENSOR_NODE, {SENSOR_CHAN_AMBIENT_TEMP, 0});
RTIO_DEFINE(temp_rtio, 1, 1);

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(SENSOR_NODE);
	const struct sensor_decoder_api *decoder;

	uint8_t buffer[32];

	if (!device_is_ready(dev)) {
		printk("Capteur non disponible\n");
		return 0;
	}
	int ret = sensor_get_decoder(dev, &decoder);
	if (ret < 0) {
		printk("Erreur get_decoder : %d\n", ret);
		return 0;
	}

	for (int i = 0; i < 5; i++) {
		ret = sensor_read(&temp_iodev, &temp_rtio, buffer, sizeof(buffer));
		if (ret < 0) {
			printk("Erreur read : %d\n", ret);
			return 0;
		}

		struct sensor_q31_data temperature;
		struct sensor_decode_context decode_ctx =
			SENSOR_DECODE_CONTEXT_INIT(decoder, buffer, SENSOR_CHAN_AMBIENT_TEMP, 0);

		ret = sensor_decode(&decode_ctx, &temperature, 1);
		if (ret != 1) {
			printk("Erreur decode : %d\n", ret);
			return 0;
		}

		printk("Temperature : %" PRIsensor_q31_data " deg C\n",
		       PRIsensor_q31_data_arg(temperature, 0));

		k_msleep(10000);
	}

	return 0;
}
