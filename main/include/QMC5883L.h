#ifndef QMC5883L_H
#define QMC5883L_H

#include <cstdint>
#include <limits>
#include <array>

#include <esp_log.h>
#include <esp_check.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>

enum qmc5883l_samples_t : uint8_t
{
	QMC5883L_SAMPLES_MIN = 0b11,
	QMC5883L_SAMPLES_64 = 0b11,
	QMC5883L_SAMPLES_128 = 0b10,
	QMC5883L_SAMPLES_256 = 0b01,
	QMC5883L_SAMPLES_512 = 0b00,
	QMC5883L_SAMPLES_MAX = 0b00,
};

enum qmc5883l_rate_t : uint8_t
{
	QMC5883L_RATE_MIN = 0b00,
	QMC5883L_RATE_10HZ = 0b00,
	QMC5883L_RATE_50HZ = 0b01,
	QMC5883L_RATE_100HZ = 0b10,
	QMC5883L_RATE_200HZ = 0b11,
	QMC5883L_RATE_MAX = 0b11,
};

enum qmc5883l_range_t : uint8_t
{
	QMC5883L_RANGE_MIN = 0b00,
	QMC5883L_RANGE_2G = 0b00,
	QMC5883L_RANGE_8G = 0b01,
	QMC5883L_RANGE_MAX = 0b01,
	QMC5883L_RANGE_INVALID1 = 0b10,
	QMC5883L_RANGE_INVALID2 = 0b11,
};

enum qmc5883l_mode_t : uint8_t
{
	QMC5883L_MODE_STNDBY = 0b00,
	QMC5883L_MODE_CNTNS = 0b01,
	QMC5883L_MODE_INVALID1 = 0b10,
	QMC5883L_MODE_INVALID2 = 0b11,
};

struct Vector
{
	float XAxis;
	float YAxis;
	float ZAxis;
};

class QMC5883L
{
	static constexpr const char *const TAG = "QMC5883L";

	// static constexpr std::array<float, 8> Ga_per_LSb_LUT = {0.73 * 0.001, 0.92 * 0.001, 1.22 * 0.001, 1.52 * 0.001, 2.27 * 0.001, 2.56 * 0.001, 3.03 * 0.001, 4.35 * 0.001}; // from datasheet

private:
	using sout_t = int16_t;
	using uout_t = uint16_t;

public:
	using out_t = sout_t;
	static constexpr uint8_t bits = 16;

	static constexpr size_t ref = 1u << (bits - 1); // for sign
	static constexpr out_t max = std::numeric_limits<out_t>::max();
	static constexpr out_t min = std::numeric_limits<out_t>::min();

private:
	enum class Register : uint8_t
	{
		OUTXL = 0x00,
		OUTXH = 0x01,
		OUTYL = 0x02,
		OUTYH = 0x03,
		OUTZL = 0x04,
		OUTZH = 0x05,
		STTS = 0x06,
		OUTTL = 0x07,
		OUTTH = 0x08,
		CTRL1 = 0x09,
		CTRL2 = 0x0A,
		SRPR = 0x0B,
		RSVD = 0x0C,
		CHID = 0x0D,
	};

private:
	i2c_master_bus_handle_t i2c_host;
	uint16_t address;
	uint32_t clk_hz;
	i2c_master_dev_handle_t i2c_hdl = nullptr;

	// float Ga_per_LSb = Ga_per_LSb_LUT[1];
	Vector v;
	int xOffset, yOffset, zOffset;

public:
	QMC5883L(i2c_master_bus_handle_t ih, uint16_t a = 0x00, uint32_t chz = 100'000) : i2c_host(ih), address(0x0D), clk_hz(chz)
	{
		ESP_LOGI(TAG, "Constructed with address: %d", address); // port: %d, , ih
	}
	~QMC5883L() = default;

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

		uint8_t check;
		ESP_RETURN_ON_ERROR(
			read_reg(Register::CHID, check), // 9.2.6
			TAG, "Failed to read_reg!");

		if (check != 0xFF)
		{
			ESP_LOGE(TAG, "The connected chip is not QMC5883L!");
			return ESP_ERR_INVALID_RESPONSE;
		}

		ESP_RETURN_ON_ERROR(
			write_reg(Register::SRPR, 0x01), // 9.2.5
			TAG, "Failed to write_reg!");

		return ESP_OK;
	}

	esp_err_t deinit()
	{
		assert(i2c_hdl);

		ESP_RETURN_ON_ERROR(
			i2c_master_bus_rm_device(i2c_hdl),
			TAG, "Failed to i2c_master_bus_add_device!");

		i2c_hdl = nullptr;

		return ESP_OK;
	}

	//

	Vector read_3d()
	{
		out_t x, y, z;

		read_raw(Register::OUTXL, x);
		read_raw(Register::OUTYL, y);
		read_raw(Register::OUTZL, z);

		ESP_LOGI(TAG, "READ:\t%" PRId16 "\t%" PRId16 "\t%" PRId16, x, y, z);

		// v.XAxis = readRegister16(QMC5883L_REG_OUT_X_M) - xOffset;
		// v.YAxis = readRegister16(QMC5883L_REG_OUT_Y_M) - yOffset;
		// v.ZAxis = readRegister16(QMC5883L_REG_OUT_Z_M) - zOffset;

		return v;
	}

	Vector readNormalize()
	{
		// v.XAxis = ((float)readRegister16(QMC5883L_REG_OUT_X_M) - xOffset) * mgPerDigit;
		// v.YAxis = ((float)readRegister16(QMC5883L_REG_OUT_Y_M) - yOffset) * mgPerDigit;
		// v.ZAxis = ((float)readRegister16(QMC5883L_REG_OUT_Z_M) - zOffset) * mgPerDigit;

		return v;
	}

	void setOffset(int xo, int yo, int zo)
	{
		xOffset = xo;
		yOffset = yo;
		zOffset = zo;
	}

	//

	esp_err_t get_samples(qmc5883l_samples_t &samples)
	{
		uint8_t val;

		ESP_RETURN_ON_ERROR(
			read_reg(Register::CTRL1, val),
			TAG, "Failed to read_reg!");

		samples = static_cast<qmc5883l_samples_t>((val >> 6) & 0b11);

		return ESP_OK;
	}

	esp_err_t set_samples(qmc5883l_samples_t samples)
	{
		uint8_t val;

		ESP_RETURN_ON_ERROR(
			read_reg(Register::CTRL1, val),
			TAG, "Failed to read_reg!");

		val &= ~static_cast<uint8_t>(0b11 << 6);
		val |= static_cast<uint8_t>(samples) << 6;

		ESP_RETURN_ON_ERROR(
			write_reg(Register::CTRL1, val),
			TAG, "Failed to write_reg!");

		return ESP_OK;
	}

	//

	esp_err_t get_range(qmc5883l_range_t &range)
	{
		uint8_t val;

		ESP_RETURN_ON_ERROR(
			read_reg(Register::CTRL1, val),
			TAG, "Failed to read_reg!");

		range = static_cast<qmc5883l_range_t>((val >> 4) & 0b11);

		return ESP_OK;
	}

	esp_err_t set_range(qmc5883l_range_t range)
	{
		uint8_t val;

		ESP_RETURN_ON_ERROR(
			read_reg(Register::CTRL1, val),
			TAG, "Failed to read_reg!");

		val &= ~static_cast<uint8_t>(0b11 << 4);
		val |= static_cast<uint8_t>(range) << 4;

		ESP_RETURN_ON_ERROR(
			write_reg(Register::CTRL1, val),
			TAG, "Failed to write_reg!");

		return ESP_OK;
	}

	//

	esp_err_t get_rate(qmc5883l_rate_t &rate)
	{
		uint8_t val;

		ESP_RETURN_ON_ERROR(
			read_reg(Register::CTRL1, val),
			TAG, "Failed to read_reg!");

		rate = static_cast<qmc5883l_rate_t>((val >> 2) & 0b11);

		return ESP_OK;
	}

	esp_err_t set_rate(qmc5883l_rate_t rate)
	{
		uint8_t val;

		ESP_RETURN_ON_ERROR(
			read_reg(Register::CTRL1, val),
			TAG, "Failed to read_reg!");

		val &= ~static_cast<uint8_t>(0b11 << 2);
		val |= static_cast<uint8_t>(rate) << 2;

		ESP_RETURN_ON_ERROR(
			write_reg(Register::CTRL1, val),
			TAG, "Failed to write_reg!");

		return ESP_OK;
	}

	//

	esp_err_t get_mode(qmc5883l_mode_t &mode)
	{
		uint8_t val;

		ESP_RETURN_ON_ERROR(
			read_reg(Register::CTRL1, val),
			TAG, "Failed to read_reg!");

		mode = static_cast<qmc5883l_mode_t>((val >> 0) & 0b11);

		return ESP_OK;
	}

	esp_err_t set_mode(qmc5883l_mode_t mode)
	{
		uint8_t val;

		ESP_RETURN_ON_ERROR(
			read_reg(Register::CTRL1, val),
			TAG, "Failed to read_reg!");

		val &= ~static_cast<uint8_t>(0b11 << 0);
		val |= static_cast<uint8_t>(mode) << 0;

		ESP_RETURN_ON_ERROR(
			write_reg(Register::CTRL1, val),
			TAG, "Failed to write_reg!");

		return ESP_OK;
	}

	//

	Vector self_test()
	{
		Vector value;
		return value;
	}

private:
	esp_err_t write_reg(Register reg, uint8_t val)
	{
		assert(i2c_hdl);

		std::array<uint8_t, 2> txbuf = {static_cast<uint8_t>(reg), val};
		std::array<uint8_t, 0> rxbuf;

		ESP_RETURN_ON_ERROR(
			i2c_master_transmit(i2c_hdl, txbuf.data(), txbuf.size(), -1),
			TAG, "Failed to i2c_master_transmit!");

		return ESP_OK;
	}

	esp_err_t read_reg(Register reg, uint8_t &out)
	{
		assert(i2c_hdl);

		std::array<uint8_t, 1> txbuf = {static_cast<uint8_t>(reg)};
		std::array<uint8_t, 1> rxbuf;

		ESP_RETURN_ON_ERROR(
			i2c_master_transmit_receive(i2c_hdl, txbuf.data(), txbuf.size(), rxbuf.data(), rxbuf.size(), -1),
			TAG, "Failed to i2c_master_transmit_receive!");

		out = rxbuf[0];

		return ESP_OK;
	}

	//

	esp_err_t read_raw(Register reg, out_t &out)
	{
		assert(i2c_hdl);
		assert(reg == Register::OUTXL || reg == Register::OUTYL || reg == Register::OUTZL || reg == Register::OUTTL);

		std::array<uint8_t, 1> txbuf = {static_cast<uint8_t>(reg)};
		std::array<uint8_t, 2> rxbuf;

		ESP_RETURN_ON_ERROR(
			i2c_master_transmit_receive(i2c_hdl, txbuf.data(), txbuf.size(), rxbuf.data(), rxbuf.size(), -1),
			TAG, "Failed to i2c_master_transmit_receive!");

		// out = std::bit_cast<out_t>(rxbuf);
		out = rxbuf[1] << 8 | rxbuf[0];

		return ESP_OK;
	}
};

#endif
