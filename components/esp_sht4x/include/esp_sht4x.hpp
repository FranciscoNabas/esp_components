#ifndef __ESP_SHT4X_H__
#define __ESP_SHT4X_H__

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <esp_check.h>

#define SHT4X_DEVICE_ADDRESS              0x44  // Device I2C address.

#define SHT4X_DATA_SIZE                   6     // The size of the data buffer for reading operations. It's the same for all commands that generate data.
#define SHT4X_I2C_OP_TIMEOUT_MS           1000  // The timeout for I2C transmit/receive operations.

#define SHT4X_CMD_HEATER_ON_20_100        0x15  // Activate heater with 20mW for 0.1s including a high precision measurement just before deactivation..
#define SHT4X_CMD_HEATER_ON_20_1000       0x1E  // Activate heater with 20mW for 1s including a high precision measurement just before deactivation.
#define SHT4X_CMD_HEATER_ON_110_100       0x24  // Activate heater with 110mW for 0.1s including a high precision measurement just before deactivation.
#define SHT4X_CMD_HEATER_ON_110_1000      0x2F  // Activate heater with 110mW for 1s including a high precision measurement just before deactivation.
#define SHT4X_CMD_HEATER_ON_200_100       0x32  // Activate heater with 200mW for 0.1s including a high precision measurement just before deactivation.
#define SHT4X_CMD_HEATER_ON_200_1000      0x39  // Activate heater with 200mW for 1s, including a high precision measurement just before deactivation.
#define SHT4X_CMD_READ_SERIAL_NUMBER      0x89  // Read serial number.
#define SHT4X_CMD_SOFT_RESET              0x94  // Software reset.
#define SHT4X_CMD_MEASURE_LOW_PRECISION   0xE0  // Measure T & RH with lowest precision (low repeatability).
#define SHT4X_CMD_MEASURE_MED_PRECISION   0xF6  // Measure T & RH with medium precision (medium repeatability).
#define SHT4X_CMD_MEASURE_HIGH_PRECISION  0xFD  // Measure T & RH with high precision (high repeatability).

#define SHT4X_POWERUP_TIME_MS             1     // Time between VDD reaching VPOR and sensor entering idle state.
#define SHT4X_SOFTRES_TIME_MS             1     // Time between ACK of soft reset command and sensor entering idle state.
#define SHT4X_MEASURE_LOW_TIME_MS         2     // Low repeatability reading time.
#define SHT4X_MEASURE_MED_TIME_MS         5     // Medium repeatability reading time.
#define SHT4X_MEASURE_HIGH_TIME_MS        9     // High repeatability reading time.
#define SHT4X_HEATER_ON_TIME_SHORT_MS     110   // After that time the heater is automatically switched off.
#define SHT4X_HEATER_ON_TIME_LONG_MS      1100  // After that time the heater is automatically switched off.

#ifdef CONFIG_SHT4X_CLK_1000000
    #define CONFIG_SHT4X_CLK_SPEED_HZ 1000000
#elif defined(CONFIG_SHT4X_CLK_400000)
    #define CONFIG_SHT4X_CLK_SPEED_HZ 400000
#elif defined(CONFIG_SHT4X_CLK_100000)
    #define CONFIG_SHT4X_CLK_SPEED_HZ 100000
#else
    #error Invalid clock speed.
#endif

enum class sht4x_read_flags_t : uint8_t
{
    LOW_PRECISION     = 0xE0,
    MEDIUM_PRECISION  = 0xF6,
    HIGH_PRECISION    = 0xFD,
};

enum class sht4x_heater_flags_t : uint8_t
{
    P20_T100          = 0x15,
    P20_T1000         = 0x1E,
    P110_T100         = 0x24,
    P110_T1000        = 0x2F,
    P200_T100         = 0x32,
    P200_T1000        = 0x39,
};

struct sht4x_result_t
{
    float   temperature;
    float   relative_humidity;

    sht4x_result_t();
    sht4x_result_t(float temp, float rh);
};

class sht4x_sensor_t
{
public:
    sht4x_sensor_t();
    sht4x_sensor_t(i2c_master_bus_handle_t master_bus_handle);
    ~sht4x_sensor_t();

    const i2c_master_bus_handle_t& bus_handle();
    const i2c_master_dev_handle_t& device_handle();

    esp_err_t reset();
    esp_err_t get_serial_number(uint32_t& sn);
    esp_err_t read(sht4x_result_t& result, const sht4x_read_flags_t flags = sht4x_read_flags_t::HIGH_PRECISION);
    esp_err_t trigger_heater(sht4x_result_t& result, const sht4x_heater_flags_t flags = sht4x_heater_flags_t::P20_T100);

private:
    static const char* s_tag;

    bool m_owns_bus;
    i2c_master_bus_handle_t m_master_bus_handle;
    i2c_master_dev_handle_t m_device_handle;

    static inline void get_result_from_data(const uint8_t* data, sht4x_result_t& result) _NOTHROW
    {
        uint16_t raw_tem  = data[0] * 256 + data[1];
        uint16_t raw_rh   = data[3] * 256 + data[4];

        result.temperature        =  -45 + 175 * static_cast<float>(raw_tem) / 65535;
        result.relative_humidity  =  -6 + 125 * static_cast<float>(raw_rh) / 65535;

        if (result.relative_humidity > 100)
            result.relative_humidity = 100;
        if (result.relative_humidity < 0)
            result.relative_humidity = 0;
    }
};

#endif // __ESP_SHT4X_H__