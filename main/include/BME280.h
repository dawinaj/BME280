#ifndef BME280_H
#define BME280_H

#include <cmath>
#include <cstdint>
#include <limits>
#include <array>
#include <bit>

#include <esp_log.h>
#include <esp_check.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <rom/ets_sys.h>

#include "BME280_SensorAPI.h"

//

#define BME_RETURN_ON_ERROR(x, log_tag)                                             \
	do                                                                              \
	{                                                                               \
		uint8_t err_rc_ = (x);                                                      \
		if (unlikely(err_rc_ != BME280_OK))                                         \
		{                                                                           \
			ESP_LOGE(log_tag, "%s(%d): %s %s", __FUNCTION__, __LINE__,              \
					 BME280::bmeerr_name(err_rc_), esp_err_to_name(dev.intf_rslt)); \
			return BME280::bmeerr_to_esp(err_rc_);                                  \
		}                                                                           \
	} while (0)

//

enum bme280_samples_t : uint8_t
{
	BME280_SAMPLES_OFF = BME280_NO_OVERSAMPLING,
	BME280_SAMPLES_MIN = BME280_OVERSAMPLING_1X,
	BME280_SAMPLES_1 = BME280_OVERSAMPLING_1X,
	BME280_SAMPLES_2 = BME280_OVERSAMPLING_2X,
	BME280_SAMPLES_4 = BME280_OVERSAMPLING_4X,
	BME280_SAMPLES_8 = BME280_OVERSAMPLING_8X,
	BME280_SAMPLES_16 = BME280_OVERSAMPLING_16X,
	BME280_SAMPLES_MAX = BME280_OVERSAMPLING_16X,
};

enum bme280_tsby_t : uint8_t
{
	BME280_TSBY_MIN = BME280_STANDBY_TIME_0_5_MS,
	BME280_TSBY_0_5MS = BME280_STANDBY_TIME_0_5_MS,
	BME280_TSBY_10MS = BME280_STANDBY_TIME_10_MS,
	BME280_TSBY_20MS = BME280_STANDBY_TIME_20_MS,
	BME280_TSBY_62_5MS = BME280_STANDBY_TIME_62_5_MS,
	BME280_TSBY_125MS = BME280_STANDBY_TIME_125_MS,
	BME280_TSBY_250MS = BME280_STANDBY_TIME_250_MS,
	BME280_TSBY_500MS = BME280_STANDBY_TIME_500_MS,
	BME280_TSBY_1000MS = BME280_STANDBY_TIME_1000_MS,
	BME280_MODE_MAX = BME280_STANDBY_TIME_1000_MS,
};

enum bme280_filter_t : uint8_t
{
	BME280_FILTER_MIN = BME280_FILTER_COEFF_OFF,
	BME280_FILTER_OFF = BME280_FILTER_COEFF_OFF,
	BME280_FILTER_1 = BME280_FILTER_COEFF_OFF,
	BME280_FILTER_2 = BME280_FILTER_COEFF_2,
	BME280_FILTER_4 = BME280_FILTER_COEFF_4,
	BME280_FILTER_8 = BME280_FILTER_COEFF_8,
	BME280_FILTER_16 = BME280_FILTER_COEFF_16,
	BME280_FILTER_MAX = BME280_FILTER_COEFF_16,
};

struct bme280_config_t
{
	bme280_samples_t osr_p;
	bme280_samples_t osr_t;
	bme280_samples_t osr_h;
	bme280_filter_t filter;
	bme280_tsby_t sby_time;
};

//

class BME280
{
protected:
	static constexpr const char *const TAG = "BME280";

public:
#ifdef BME280_DOUBLE_ENABLE
	struct Meas
	{
		double temperature;
		double pressure;
		double humidity;
	};
#else
	struct Meas
	{
		float temperature;
		float pressure;
		float humidity;
	};
#endif

protected:
	enum class Register : uint8_t
	{
		CLB00 = 0x88,
		CLB25 = 0xA1,

		CHID = 0xD0,
		RESET = 0xE0,

		CLB26 = 0xE1,
		CLB41 = 0xF0,

		CTRLH = 0xF2,
		STTS = 0xF3,
		CTRLM = 0xF4,
		CNFG = 0xF5,

		PRESH = 0xF7,
		PRESM = 0xF8,
		PRESL = 0xF9,

		TEMPH = 0xFA,
		TEMPM = 0xFB,
		TEMPL = 0xFC,

		HUMH = 0xFD,
		HUML = 0xFE,
	};

protected:
	bme280_dev dev = {};

	bme280_settings settings_current = {};
	bme280_settings settings_desired = {};

	uint32_t period = 0;
	bool continuous = false;

public:
	BME280() = default;
	~BME280() = default;

	esp_err_t init()
	{
		BME_RETURN_ON_ERROR(
			bme280_init(&dev),
			TAG);

		BME_RETURN_ON_ERROR(
			bme280_get_sensor_settings(&settings_current, &dev),
			TAG);
		settings_desired = settings_current;

		return ESP_OK;
	}
	esp_err_t deinit()
	{
		return ESP_OK;
	}

	//

	esp_err_t continuous_mode(bool c)
	{
		continuous = c;

		if (continuous)
			BME_RETURN_ON_ERROR(
				bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev),
				TAG);
		else
			BME_RETURN_ON_ERROR(
				bme280_set_sensor_mode(BME280_POWERMODE_SLEEP, &dev),
				TAG);

		return ESP_OK;
	}

	const bme280_settings &settings_clear()
	{
		return settings_desired = {};
	}
	const bme280_settings &settings_press(bme280_samples_t val)
	{
		settings_desired.osr_p = val;
		return settings_desired;
	}
	const bme280_settings &settings_temp(bme280_samples_t val)
	{
		settings_desired.osr_t = val;
		return settings_desired;
	}
	const bme280_settings &settings_hum(bme280_samples_t val)
	{
		settings_desired.osr_h = val;
		return settings_desired;
	}
	const bme280_settings &settings_filt(bme280_filter_t val)
	{
		settings_desired.filter = val;
		return settings_desired;
	}
	const bme280_settings &settings_sby(bme280_tsby_t val)
	{
		settings_desired.standby_time = val;
		return settings_desired;
	}

	esp_err_t apply_settings()
	{
		BME_RETURN_ON_ERROR(
			bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings_desired, &dev),
			TAG);
		settings_current = settings_desired;

		BME_RETURN_ON_ERROR(
			bme280_cal_meas_delay(&period, &settings_current),
			TAG);

		return continuous_mode(continuous); // reapply mode
	}

	//

	esp_err_t measure(Meas &out)
	{
		uint8_t status_reg;
		bme280_data comp_data;

		out = {};

		if (!continuous)
			BME_RETURN_ON_ERROR(
				bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &dev),
				TAG);

		dev.delay_us(period, dev.intf_ptr);

		BME_RETURN_ON_ERROR(
			bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &dev),
			TAG);
		if (status_reg & BME280_STATUS_MEAS_DONE) // shouldnt really happen but it was in the original example so whatever
			return ESP_ERR_NOT_FINISHED;

		// do // alternatively to throwing error, wait until completion
		// {
		// 	BME_RETURN_ON_ERROR(
		// 		bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &dev),
		// 		TAG);
		// } while (status_reg & BME280_STATUS_MEAS_DONE);

		BME_RETURN_ON_ERROR(
			bme280_get_sensor_data(BME280_ALL, &comp_data, &dev),
			TAG);

		out = bme_data_to_meas(comp_data);
		return ESP_OK;
	}

	//

	static float get_sea_level_pressure(const Meas &meas, float h = 0)
	{
		return meas.pressure * std::pow(1 - h * (g / cp / T0), -cp * M / R0);
	}

	static float get_sea_level_altitude(const Meas &meas, float p0 = 101325)
	{
		return cp * T0 / g * (1 - std::pow(meas.pressure / p0, R0 / cp / M));
	}

protected:
	static esp_err_t bme280_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
	static esp_err_t bme280_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
	static void bme280_delay_us(uint32_t period, void *intf_ptr)
	{
		ets_delay_us(period);
	}

protected:
	static const char *bmeerr_name(int8_t rslt)
	{
		switch (rslt)
		{
		case BME280_OK:
			return "BME280_OK";
		case BME280_E_NULL_PTR:
			return "BME280_E_NULL_PTR";
		case BME280_E_COMM_FAIL:
			return "BME280_E_COMM_FAIL";
		case BME280_E_INVALID_LEN:
			return "BME280_E_INVALID_LEN";
		case BME280_E_DEV_NOT_FOUND:
			return "BME280_E_DEV_NOT_FOUND";
		case BME280_E_SLEEP_MODE_FAIL:
			return "BME280_E_SLEEP_MODE_FAIL";
		case BME280_E_NVM_COPY_FAILED:
			return "BME280_E_NVM_COPY_FAILED";
		case BME280_W_INVALID_OSR_MACRO:
			return "BME280_W_INVALID_OSR_MACRO";
		default:
			return "BME280_E_UNKNOWN";
		}
	}

	static esp_err_t bmeerr_to_esp(int8_t rslt)
	{
		switch (rslt)
		{
		case BME280_OK:
			return ESP_OK;
		case BME280_E_NULL_PTR:
			return ESP_ERR_INVALID_ARG;
		case BME280_E_COMM_FAIL:
			return ESP_ERR_NOT_FINISHED;
		case BME280_E_INVALID_LEN:
			return ESP_ERR_INVALID_SIZE;
		case BME280_E_DEV_NOT_FOUND:
			return ESP_ERR_NOT_FOUND;
		case BME280_E_SLEEP_MODE_FAIL:
			return ESP_ERR_INVALID_RESPONSE;
		case BME280_E_NVM_COPY_FAILED:
			return ESP_ERR_INVALID_STATE;
		case BME280_W_INVALID_OSR_MACRO:
			return ESP_ERR_INVALID_ARG;
		default:
			return ESP_FAIL;
		}
	}

private:
#ifdef BME280_DOUBLE_ENABLE
	Meas bme_data_to_meas(const bme280_data &data)
	{
		Meas meas;
		meas.temperature = data.temperature; // DegC
		meas.pressure = data.pressure;		 // Pa
		meas.humidity = data.humidity;		 // %RH
		return meas;
	}
#else
	Meas bme_data_to_meas(const bme280_data &data)
	{
		Meas meas;
		meas.temperature = data.temperature * 0.01f; // 0.01 DegC
#ifdef BME280_32BIT_ENABLE
		meas.pressure = data.pressure; // Pa
#elifdef BME280_64BIT_ENABLE
		meas.pressure = data.pressure * 0.01f; // 0.01 Pa
#else
#error "No data type has been selected (BME280_*_ENABLE)"
#endif
		meas.humidity = data.humidity * (1.0f / 1024); // %RH
		return meas;
	}
#endif

private:
	static constexpr float cp = 1004.68506;	 // J/(kg*K)	Constant-pressure specific heat
	static constexpr float T0 = 288.15;		 // K			Sea level standard temperature
	static constexpr float g = 9.80665;		 // m/s2		Earth surface gravitational acceleration
	static constexpr float M = 0.02896968;	 // kg/mol		Molar mass of dry air
	static constexpr float R0 = 8.314462618; // J/(mol*K)	Universal gas constant

	// static bme280_settings config_enum_to_macro(const bme280_config_t &cfg)
	// {
	// 	bme280_settings sett = {};
	// 	sett.osr_p = static_cast<uint8_t>(cfg.osr_p);
	// 	sett.osr_t = static_cast<uint8_t>(cfg.osr_t);
	// 	sett.osr_h = static_cast<uint8_t>(cfg.osr_h);
	// 	sett.filter = static_cast<uint8_t>(cfg.filter);
	// 	sett.standby_time = static_cast<uint8_t>(cfg.sby_time);
	// 	return sett;
	// }
};

///

class BME280_I2C : public BME280
{

private:
	i2c_master_bus_handle_t i2c_host;
	uint16_t address;
	uint32_t clk_hz;
	i2c_master_dev_handle_t i2c_hdl = nullptr;

public:
	BME280_I2C(i2c_master_bus_handle_t ih, uint16_t a = 0b0, uint32_t chz = 400'000) : i2c_host(ih), address(0x76 | (a & 0b1)), clk_hz(chz)
	{
		ESP_LOGI(TAG, "Constructed with address: %d", address); // port: %d, , ih
	}
	~BME280_I2C() = default;

	esp_err_t init()
	{
		assert(!i2c_hdl);

		i2c_device_config_t dev_cfg = {
			.dev_addr_length = I2C_ADDR_BIT_LEN_7,
			.device_address = address,
			.scl_speed_hz = clk_hz,
			.scl_wait_us = 0,
			.flags = {
				.disable_ack_check = false,
			},
		};

		ESP_RETURN_ON_ERROR(
			i2c_master_bus_add_device(i2c_host, &dev_cfg, &i2c_hdl),
			TAG, "Failed to i2c_master_bus_add_device!");

		dev.intf = BME280_I2C_INTF;
		dev.intf_ptr = static_cast<void *>(i2c_hdl);

		dev.read = bme280_read;
		dev.write = bme280_write;
		dev.delay_us = bme280_delay_us;

		return BME280::init();
	}

	esp_err_t deinit()
	{
		assert(i2c_hdl);

		ESP_RETURN_ON_ERROR(
			i2c_master_bus_rm_device(i2c_hdl),
			TAG, "Failed to i2c_master_bus_add_device!");

		i2c_hdl = nullptr;

		return BME280::deinit();
	}

	//

private:
	static esp_err_t bme280_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
	{
		i2c_master_dev_handle_t i2c_hdl = static_cast<i2c_master_dev_handle_t>(intf_ptr);

		ESP_RETURN_ON_ERROR(
			i2c_master_transmit_receive(i2c_hdl, &reg_addr, 1, reg_data, len, -1),
			TAG, "Failed to i2c_master_transmit_receive!");

		return ESP_OK;
	}

	static esp_err_t bme280_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
	{
		i2c_master_dev_handle_t i2c_hdl = static_cast<i2c_master_dev_handle_t>(intf_ptr);

		std::array<uint8_t, 2 * BME280_MAX_LEN> txbuf = {static_cast<uint8_t>(reg_addr)};
		std::copy(reg_data, reg_data + len, txbuf.begin() + 1);

		ESP_RETURN_ON_ERROR(
			i2c_master_transmit(i2c_hdl, txbuf.data(), len + 1, -1),
			TAG, "Failed to i2c_master_transmit!");

		return ESP_OK;
	}
};

//

class BME280_SPI : public BME280
{

private:
	spi_host_device_t spi_host;
	gpio_num_t cs_gpio;
	int clk_hz;
	spi_device_handle_t spi_hdl = nullptr;

public:
	BME280_SPI(spi_host_device_t sh, gpio_num_t csg, int chz = 10'000'000) : spi_host(sh), cs_gpio(csg), clk_hz(chz)
	{
		ESP_LOGI(TAG, "Constructed with host: %d, pin: %d", spi_host, cs_gpio);
	}
	~BME280_SPI() = default;

	esp_err_t init(bool tw = false)
	{
		assert(!spi_hdl);
		assert(tw == false); // threewire does not work for now

		const spi_device_interface_config_t dev_cfg = {
			.command_bits = 0,
			.address_bits = 8,
			.dummy_bits = 0,
			.mode = 0,
			.clock_source = SPI_CLK_SRC_DEFAULT,
			.duty_cycle_pos = 0,
			.cs_ena_pretrans = 0,
			.cs_ena_posttrans = 0,
			.clock_speed_hz = clk_hz,
			.input_delay_ns = 0,
			.spics_io_num = cs_gpio,
			.flags = SPI_DEVICE_HALFDUPLEX | (tw ? SPI_DEVICE_3WIRE : 0u),
			.queue_size = 1,
			.pre_cb = NULL,
			.post_cb = NULL,
		};

		ESP_RETURN_ON_ERROR(
			spi_bus_add_device(spi_host, &dev_cfg, &spi_hdl),
			TAG, "Error in spi_bus_add_device!");

		dev.intf = BME280_SPI_INTF;
		dev.intf_ptr = static_cast<void *>(spi_hdl);

		dev.read = bme280_read;
		dev.write = bme280_write;
		dev.delay_us = bme280_delay_us;

		if (tw)
		{
			uint8_t reg_addr = BME280_REG_CONFIG;
			uint8_t reg_data = 0b1;

			BME_RETURN_ON_ERROR(
				bme280_set_regs(&reg_addr, &reg_data, 1, &dev), // set 3-wire mode
				TAG);
		}

		return BME280::init();
	}

	esp_err_t deinit()
	{
		assert(spi_hdl);

		ESP_RETURN_ON_ERROR(
			spi_bus_remove_device(spi_hdl),
			TAG, "Error in spi_bus_remove_device!");

		spi_hdl = nullptr;

		return BME280::deinit();
	}

	//

private:
	static esp_err_t bme280_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
	{
		static spi_transaction_t trx = {};
		spi_device_handle_t spi_hdl = static_cast<spi_device_handle_t>(intf_ptr);

		trx.addr = reg_addr;
		trx.rx_buffer = reg_data;
		trx.rxlength = len * 8;

		ESP_RETURN_ON_ERROR(
			spi_device_polling_transmit(spi_hdl, &trx),
			TAG, "Failed to spi_device_polling_transmit!");

		return ESP_OK;
	}

	static esp_err_t bme280_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
	{
		static spi_transaction_t trx = {};
		spi_device_handle_t spi_hdl = static_cast<spi_device_handle_t>(intf_ptr);

		trx.addr = reg_addr;
		trx.tx_buffer = reg_data;
		trx.length = len * 8;

		ESP_RETURN_ON_ERROR(
			spi_device_polling_transmit(spi_hdl, &trx),
			TAG, "Failed to spi_device_polling_transmit!");

		return ESP_OK;
	}
};

#endif
