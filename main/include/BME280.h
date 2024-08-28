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


#define BME280_INTF_RET_TYPE esp_err_t
#include "BME280_SensorAPI.h"

// #include "bitwise_print.h"

//

#define BME_RETURN_ON_ERROR(x, log_tag)                                                           \
	do                                                                                            \
	{                                                                                             \
		uint8_t err_rc_ = (x);                                                                    \
		esp_err_t err_rc_esp_ = bme280_err_to_esp_err(err_rc_);                                   \
		if (unlikely(err_rc_ != BME280_OK))                                                       \
		{                                                                                         \
			ESP_LOGE(log_tag, "%s(%d): %s", __FUNCTION__, __LINE__, bme280_err_to_name(err_rc_)); \
			return err_rc_esp_;                                                                   \
		}                                                                                         \
	} while (0)

//

enum bme280_mode_t : uint8_t
{
	BME280_MODE_SLEEP = BME280_POWERMODE_SLEEP,
	BME280_MODE_FORCED = BME280_POWERMODE_FORCED,
	BME280_MODE_NORMAL = BME280_POWERMODE_NORMAL,
};

// enum bme280_status_t : uint8_t
// {
// 	BME280_STATUS_IMUPD = BME280_STATUS_IM_UPDATE,
// 	BME280_STATUS_DONE = BME280_STATUS_MEAS_DONE,
// };

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

// struct bme280_config_t
// {
// 	bme280_samples_t osr_p;
// 	bme280_samples_t osr_t;
// 	bme280_samples_t osr_h;
// 	bme280_filter_t filter;
// 	bme280_tsby_t sby_time;
// };
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

	uint32_t period;

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

		/* Always set the power mode after setting the configuration */
		BME_RETURN_ON_ERROR(
			bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev),
			TAG);

		return ESP_OK;
	}

	//

	esp_err_t measure(Meas &out)
	{
		uint8_t status_reg;
		bme280_data comp_data;

		BME_RETURN_ON_ERROR(
			bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &dev),
			TAG);

		if (status_reg & BME280_STATUS_MEAS_DONE)
		{
			dev.delay_us(period, dev.intf_ptr);

			BME_RETURN_ON_ERROR(
				bme280_get_sensor_data(BME280_ALL, &comp_data, &dev),
				TAG);

			out = bme_data_to_meas(comp_data);
			return ESP_OK;
		}
		return ESP_FAIL;
	}

	//
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

	static float get_sea_level_pressure(const Meas &meas, float h = 0)
	{
		return meas.pressure * std::pow(1 - h * (g / cp / T0), -cp * M / R0);
	}

	static float get_sea_level_altitude(const Meas &meas, float p0 = 101325)
	{
		return cp * T0 / g * (1 - std::pow(meas.pressure / p0, R0 / cp / M));
	}

	//
	/*/
		void debug() // Meas &pack
		{
			int8_t rslt;
			uint32_t periodi;

			rslt = bme280_cal_meas_delay(&periodi, &settings_current);

			printf("\nTemperature calculation (Data displayed are compensated values)\n");
			printf("Measurement time: %lu us\n\n", (long unsigned int)periodi);

			int8_t idx = 0;
			uint8_t status_reg;
			bme280_data comp_data;

			while (idx < 5)
			{
				rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, &dev);

				if (status_reg & BME280_STATUS_MEAS_DONE)
				{
					dev.delay_us(periodi, dev.intf_ptr);

					rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

					printf("Temperature[%d]: %lf *C\n", idx, comp_data.temperature);
					printf("Pressure[%d]:    %lf Pa\n", idx, comp_data.pressure);
					printf("Humidity[%d]:    %lf %%RH\n", idx, comp_data.humidity);

					idx++;
				}
				vTaskDelay(pdMS_TO_TICKS(1000));
			}
		}
		//*/

protected:
	static BME280_INTF_RET_TYPE bme280_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
	static BME280_INTF_RET_TYPE bme280_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

	static void bme280_delay_us(uint32_t period, void *intf_ptr)
	{
		ets_delay_us(period);
	}

	static const char *bme280_err_to_name(int8_t rslt)
	{
		switch (rslt)
		{
		case BME280_OK:
			return "BME280_OK";
		case BME280_E_NULL_PTR:
			return "BME280_E_NULL_PTR";
		case BME280_E_COMM_FAIL:
			return "BME280_E_COMM_FAIL";
		case BME280_E_DEV_NOT_FOUND:
			return "BME280_E_DEV_NOT_FOUND";
		case BME280_E_INVALID_LEN:
			return "BME280_E_INVALID_LEN";
		default:
			return "BME280_E_UNKNOWN";
		}
	}

	static esp_err_t bme280_err_to_esp_err(int8_t rslt)
	{
		switch (rslt)
		{
		case BME280_OK:
			return ESP_OK;
		case BME280_E_NULL_PTR:
			return ESP_ERR_INVALID_ARG;
		case BME280_E_COMM_FAIL:
			return ESP_ERR_NOT_FINISHED;
		case BME280_E_DEV_NOT_FOUND:
			return ESP_ERR_NOT_FOUND;
		case BME280_E_INVALID_LEN:
			return ESP_ERR_INVALID_SIZE;
		default:
			return ESP_FAIL;
		}
	}

	// virtual esp_err_t write_reg(Register reg, uint8_t val) = 0;
	// virtual esp_err_t read_reg(Register reg, uint8_t &out) = 0;

	// virtual esp_err_t read_regs(Register regb, Register rege, uint8_t *out) = 0;

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
///
///
///

class BME280_I2C : public BME280
{

private:
	i2c_master_bus_handle_t i2c_host;
	uint16_t address;
	uint32_t clk_hz;
	i2c_master_dev_handle_t i2c_hdl = nullptr;

public:
	BME280_I2C(i2c_master_bus_handle_t ih, uint16_t a = 0b0, uint32_t chz = 100'000) : i2c_host(ih), address(0x76 | (a & 0b1)), clk_hz(chz)
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
	/*/
		esp_err_t write_reg(Register reg, uint8_t val) override
		{
			assert(i2c_hdl);

			std::array<uint8_t, 2> txbuf = {static_cast<uint8_t>(reg), val};

			ESP_RETURN_ON_ERROR(
				i2c_master_transmit(i2c_hdl, txbuf.data(), txbuf.size(), -1),
				TAG, "Failed to i2c_master_transmit!");

			return ESP_OK;
		}

		esp_err_t read_reg(Register reg, uint8_t &out) override
		{
			assert(i2c_hdl);

			std::array<uint8_t, 1> txbuf = {static_cast<uint8_t>(reg)};

			ESP_RETURN_ON_ERROR(
				i2c_master_transmit_receive(i2c_hdl, txbuf.data(), txbuf.size(), &out, 1, -1),
				TAG, "Failed to i2c_master_transmit_receive!");

			return ESP_OK;
		}

		esp_err_t read_regs(Register regb, Register rege, uint8_t *out) override
		{
			assert(i2c_hdl);

			std::array<uint8_t, 1> txbuf = {static_cast<uint8_t>(regb)};
			size_t len = static_cast<size_t>(rege) - static_cast<size_t>(regb) + 1;

			ESP_RETURN_ON_ERROR(
				i2c_master_transmit_receive(i2c_hdl, txbuf.data(), txbuf.size(), out, len, -1),
				TAG, "Failed to i2c_master_transmit_receive!");

			return ESP_OK;
		}
	//*/

	static BME280_INTF_RET_TYPE bme280_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
	{
		i2c_master_dev_handle_t i2c_hdl = static_cast<i2c_master_dev_handle_t>(intf_ptr);

		ESP_RETURN_ON_ERROR(
			i2c_master_transmit_receive(i2c_hdl, &reg_addr, 1, reg_data, len, -1),
			TAG, "Failed to i2c_master_transmit_receive!");

		return ESP_OK;
	}

	static BME280_INTF_RET_TYPE bme280_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
	{
		assert(len == 1); // Seems so in the API. If not, Gods help us.
		// Because of `multiple byte write (using pairs of register addresses and register data)` would need interleaving.

		i2c_master_dev_handle_t i2c_hdl = static_cast<i2c_master_dev_handle_t>(intf_ptr);

		std::array<uint8_t, 2> txbuf = {static_cast<uint8_t>(reg_addr), reg_data[0]};

		ESP_RETURN_ON_ERROR(
			i2c_master_transmit(i2c_hdl, txbuf.data(), txbuf.size(), -1),
			TAG, "Failed to i2c_master_transmit!");

		return ESP_OK;
	}
};

#endif
