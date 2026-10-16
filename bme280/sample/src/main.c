#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(SENSOR_NODE);
	const struct sensor_decoder_api *decoder;
	uint8_t buffer[20];

	if (!device_is_ready(dev)) {
		printk("Capteur pas là\n");
		return 0;
	}

	while (true) {
		if (sensor_read(&temp_iodev, &temp_rtio,buffer, sizeof(buffer)) < 0) {
			return 0;
		}
	}
}