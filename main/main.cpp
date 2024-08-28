// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

#include "BME280.h"

static const char *TAG = "Test";

spi_host_device_t init_spi()
{
	spi_bus_config_t bus_cfg = {
		.mosi_io_num = GPIO_NUM_23,
		.miso_io_num = GPIO_NUM_19,
		.sclk_io_num = GPIO_NUM_18,
		.quadwp_io_num = GPIO_NUM_NC,
		.quadhd_io_num = GPIO_NUM_NC,
		.data4_io_num = GPIO_NUM_NC,
		.data5_io_num = GPIO_NUM_NC,
		.data6_io_num = GPIO_NUM_NC,
		.data7_io_num = GPIO_NUM_NC,
		.max_transfer_sz = 0,
		.flags = SPICOMMON_BUSFLAG_MASTER,
		.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
	};

	spi_bus_initialize(SPI3_HOST, &bus_cfg, 0);

	return SPI3_HOST;
}

i2c_master_bus_handle_t init_i2c()
{
	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = -1,
		.sda_io_num = GPIO_NUM_21,
		.scl_io_num = GPIO_NUM_22,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.intr_priority = 0,
		.trans_queue_depth = 0,
		.flags = {
			.enable_internal_pullup = true,
		},
	};

	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

	return bus_handle;
}


//

extern "C" void app_main(void)
{
	esp_log_level_set("*", ESP_LOG_VERBOSE);

	spi_host_device_t spi_host = init_spi();
	i2c_master_bus_handle_t i2c_host = init_i2c();

	BME280_I2C bme(i2c_host);

	vTaskDelay(pdMS_TO_TICKS(1000));

	bme.init();
	bme.settings_filt(BME280_FILTER_2);
	bme.settings_press(BME280_SAMPLES_1);
	bme.settings_temp(BME280_SAMPLES_1);
	bme.settings_hum(BME280_SAMPLES_1);
	bme.settings_sby(BME280_TSBY_0_5MS);

	bme.apply_settings();

	vTaskDelay(pdMS_TO_TICKS(1000));

	// bme.debug();

	vTaskDelay(pdMS_TO_TICKS(1000));

	printf("\nTemperature calculation (Data displayed are compensated values)\n");
	int8_t idx = 0;

	while (idx < 5)
	{
		BME280::Meas meas;
		bme.measure(meas);

		printf("Temperature[%d]: %lf *C\n", idx, meas.temperature);
		printf("Pressure[%d]:    %lf Pa\n", idx, meas.pressure);
		printf("Humidity[%d]:    %lf %%RH\n", idx, meas.humidity);

		// printf("Pres.norm.[%d]:  %lf Pa\n", idx, BME280::get_sea_level_pressure(meas, 220));
		// printf("Altitude[%d]:    %lf m\n", idx, BME280::get_sea_level_altitude(meas));

		idx++;
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	vTaskDelay(pdMS_TO_TICKS(1000));

	bme.deinit();
}
