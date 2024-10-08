// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_check.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

// #include "QMC5883L.h"
#include "BME280.h"

static const char *TAG = "Test";

spi_host_device_t init_spi()
{
	spi_bus_config_t bus_cfg = {
		.mosi_io_num = GPIO_NUM_23,
		.miso_io_num = GPIO_NUM_19, // GPIO_NUM_19, GPIO_NUM_NC
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
		.intr_flags = 0,
	};

	spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_DISABLED); // SPI_DMA_DISABLED // SPI_DMA_CH_AUTO

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

void i2c_probe(i2c_master_bus_handle_t bus_handle, uint16_t limit = 128)
{
	for (uint16_t address = 0; address < limit; ++address)
	{
		esp_err_t ret = i2c_master_probe(bus_handle, address, -1);
		switch (ret)
		{
		case ESP_OK:
			ESP_LOGD("I2C_Probe", "Addr: %04" PRIu16 " found device", address);
			break;
		case ESP_ERR_NOT_FOUND:
			ESP_LOGD("I2C_Probe", "Addr: %04" PRIu16 " NACK", address);
			break;
		case ESP_ERR_TIMEOUT:
			ESP_LOGD("I2C_Probe", "Addr: %04" PRIu16 " timeout", address);
			break;
		default:
			ESP_LOGD("I2C_Probe", "Addr: %04" PRIu16 " shit", address);
			break;
		}
	}
}

//

extern "C" void app_main(void)
{
	esp_log_level_set("*", ESP_LOG_DEBUG); // ESP_LOG_VERBOSE

	spi_host_device_t spi_host = init_spi();
	i2c_master_bus_handle_t i2c_host = init_i2c();

	// BME280_I2C bme(i2c_host);
	BME280_SPI bme(spi_host, GPIO_NUM_2);

	vTaskDelay(pdMS_TO_TICKS(100));

	bme.init(true);
	bme.settings_filt(BME280_FILTER_2);
	bme.settings_press(BME280_SAMPLES_1);
	bme.settings_temp(BME280_SAMPLES_1);
	bme.settings_hum(BME280_SAMPLES_1);
	bme.settings_sby(BME280_TSBY_0_5MS);

	bme.apply_settings();

	// bme.continuous_mode(false);

	vTaskDelay(pdMS_TO_TICKS(100));

	printf("\nTemperature calculation (Data displayed are compensated values)\n");
	int8_t idx = 0;

	while (idx < 1)
	{
		BME280::Meas meas;
		bme.measure(meas);

		printf("Temperature[%d]: %lf *C\n", idx, meas.temperature);
		printf("Pressure[%d]:    %lf Pa\n", idx, meas.pressure);
		printf("Humidity[%d]:    %lf %%RH\n", idx, meas.humidity);

		// printf("Pres.norm.[%d]:  %lf Pa\n", idx, BME280::get_sea_level_pressure(meas, 220));
		// printf("Altitude[%d]:    %lf m\n", idx, BME280::get_sea_level_altitude(meas));

		idx++;
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	vTaskDelay(pdMS_TO_TICKS(1000));

	bme.deinit();

	/*/
		QMC5883L mag(i2c_host);
		mag.init();

		mag.set_mode(QMC5883L_MODE_CNTNS);
		mag.set_rate(QMC5883L_RATE_200HZ);
		mag.set_range(QMC5883L_RANGE_8G);
		mag.set_samples(QMC5883L_SAMPLES_64);

		while (true)
		{
			mag.read_3d();
			vTaskDelay(pdMS_TO_TICKS(1000));
		}

		mag.deinit();
	//*/
}
