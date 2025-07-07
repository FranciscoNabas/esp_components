#include "esp_sht4x.hpp"

sht4x_result_t::sht4x_result_t()
    : temperature(0), relative_humidity(0) { }

sht4x_result_t::sht4x_result_t(float temp, float rh)
    : temperature(temp), relative_humidity(rh) { }

sht4x_sensor_t::sht4x_sensor_t()
{
    esp_err_t ret = ESP_OK;

    if (CONFIG_SHT4X_SCL_GPIO < 0) {
        ESP_LOGE(s_tag, "SCL GPIO number cannot be smaller than zero. Please change the value in the SDK Config.");
        return;
    }

    if (CONFIG_SHT4X_SDA_GPIO < 0) {
        ESP_LOGE(s_tag, "SDA GPIO number cannot be smaller than zero. Please change the value in the SDK Config.");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(SHT4X_POWERUP_TIME_MS));

    i2c_master_bus_config_t config{ };
    config.clk_source                    = I2C_CLK_SRC_DEFAULT;
    config.i2c_port                      = I2C_NUM_0;
    config.scl_io_num                    = static_cast<gpio_num_t>(CONFIG_SHT4X_SCL_GPIO);
    config.sda_io_num                    = static_cast<gpio_num_t>(CONFIG_SHT4X_SDA_GPIO);
    config.glitch_ignore_cnt             = 7;
    config.flags.enable_internal_pullup  = true;

    ret = i2c_new_master_bus(&config, &m_master_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(s_tag, "Failed creating the I2C master bus - [%s]", esp_err_to_name(ret));
        return;
    }

    i2c_device_config_t device_config{ };
    device_config.dev_addr_length  = I2C_ADDR_BIT_LEN_7;
    device_config.device_address   = SHT4X_DEVICE_ADDRESS;
    device_config.scl_speed_hz     = CONFIG_SHT4X_CLK_SPEED_HZ;

    ret = i2c_master_bus_add_device(m_master_bus_handle, &device_config, &m_device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(s_tag, "Failed adding device to the bus - [%s]", esp_err_to_name(ret));
        return;
    }

    uint32_t sn = 0;
    while (get_serial_number(sn) != ESP_OK) {
        ESP_LOGW(s_tag, "Sensor initializing...");
        // Sensor initializing.
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(s_tag, "Sensor serial number: %u.", sn);

    m_owns_bus = true;
}

sht4x_sensor_t::sht4x_sensor_t(i2c_master_bus_handle_t master_bus_handle)
{
    if (!master_bus_handle) {
        ESP_LOGE(s_tag, "Master bus handle cannot be null.");
        return;
    }

    m_master_bus_handle = master_bus_handle;
    vTaskDelay(pdMS_TO_TICKS(SHT4X_POWERUP_TIME_MS));

    i2c_device_config_t device_config{ };
    device_config.dev_addr_length  = I2C_ADDR_BIT_LEN_7;
    device_config.device_address   = SHT4X_DEVICE_ADDRESS;
    device_config.scl_speed_hz     = CONFIG_SHT4X_CLK_SPEED_HZ;

    esp_err_t ret = i2c_master_bus_add_device(m_master_bus_handle, &device_config, &m_device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(s_tag, "Failed adding device to the bus - [%s]", esp_err_to_name(ret));
        return;
    }

    uint32_t sn = 0;
    while (get_serial_number(sn) != ESP_OK) {
        // Sensor initializing.
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    m_owns_bus = false;
}

sht4x_sensor_t::~sht4x_sensor_t()
{
    i2c_master_bus_rm_device(m_device_handle);
    if (m_owns_bus) {
        i2c_del_master_bus(m_master_bus_handle);
        m_master_bus_handle = nullptr;
    }
    
    m_device_handle = nullptr;
}

const char* sht4x_sensor_t::s_tag = "SHT4X_SENSOR";

const i2c_master_bus_handle_t& sht4x_sensor_t::bus_handle() { return m_master_bus_handle; }

const i2c_master_dev_handle_t& sht4x_sensor_t::device_handle() { return m_device_handle; }

esp_err_t sht4x_sensor_t::reset()
{
    uint8_t cmd = SHT4X_CMD_SOFT_RESET;
    esp_err_t ret = i2c_master_transmit(m_device_handle, &cmd, 1, SHT4X_I2C_OP_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(SHT4X_SOFTRES_TIME_MS));

    return ret;
}

esp_err_t sht4x_sensor_t::get_serial_number(uint32_t& sn)
{
    uint8_t cmd = SHT4X_CMD_READ_SERIAL_NUMBER;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(m_device_handle, &cmd, 1, SHT4X_I2C_OP_TIMEOUT_MS), s_tag, "Failed sending serial number command to the sensor.");
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t data[6]{ };
    ESP_RETURN_ON_ERROR(i2c_master_receive(m_device_handle, data, SHT4X_DATA_SIZE, SHT4X_I2C_OP_TIMEOUT_MS), s_tag, "Failed reading sensor serial number.");

    sn = (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) | (static_cast<uint32_t>(data[3]) << 8) | data[4];

    return ESP_OK;
}

esp_err_t sht4x_sensor_t::read(sht4x_result_t& result, const sht4x_read_flags_t flags)
{
    uint8_t code = static_cast<uint8_t>(flags);
    ESP_RETURN_ON_ERROR(i2c_master_transmit(m_device_handle, &code, 1, SHT4X_I2C_OP_TIMEOUT_MS), s_tag, "Failed sending command to the sensor.");
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t data[6]{ };
    ESP_RETURN_ON_ERROR(i2c_master_receive(m_device_handle, data, SHT4X_DATA_SIZE, SHT4X_I2C_OP_TIMEOUT_MS), s_tag, "Failed reading data from the sensor.");
    get_result_from_data(data, result);

    return ESP_OK;
}

esp_err_t sht4x_sensor_t::trigger_heater(sht4x_result_t& result, const sht4x_heater_flags_t flags)
{
    uint8_t command = static_cast<uint8_t>(flags);
    ESP_RETURN_ON_ERROR(i2c_master_transmit(m_device_handle, &command, 1, SHT4X_I2C_OP_TIMEOUT_MS), s_tag, "Failed sending command to the sensor.");
    switch (flags)
    {
    case sht4x_heater_flags_t::P20_T100:
    case sht4x_heater_flags_t::P110_T100:
    case sht4x_heater_flags_t::P200_T100:
        vTaskDelay(pdMS_TO_TICKS(SHT4X_HEATER_ON_TIME_SHORT_MS));
        break;
    
    default:
        vTaskDelay(pdMS_TO_TICKS(SHT4X_HEATER_ON_TIME_LONG_MS));
        break;
    }

    uint8_t data[6]{ };
    ESP_RETURN_ON_ERROR(i2c_master_receive(m_device_handle, data, SHT4X_DATA_SIZE, SHT4X_I2C_OP_TIMEOUT_MS), s_tag, "Failed reading data from the sensor.");
    get_result_from_data(data, result);

    return ESP_OK;
}