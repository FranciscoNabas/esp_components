#include "esp_lcd_i2c.hpp"

i2c_lcd_display_t::i2c_lcd_display_t()
    : m_owns_bus(true), m_rows(CONFIG_I2C_LCD_ROWS), m_columns(CONFIG_I2C_LCD_COLUMNS) {
    m_backlight_val  = LCD_HD44780_BACKLIGHT_OFF;
    m_bus_handle     = nullptr;
    
    ESP_RETURN_VOID_ON_ERROR(i2c_init(CONFIG_HD44780_LCD_DEVICE_ADDRESS), s_tag, "Failed to initialize I2C Master Bus.");
    ESP_RETURN_VOID_ON_ERROR(display_init(), s_tag, "Failed initializing the display.");

    m_current_row = 0;
}

i2c_lcd_display_t::i2c_lcd_display_t(const i2c_master_bus_handle_t i2c_bus_handle)
    : m_owns_bus(false) {
    m_backlight_val  = LCD_HD44780_BACKLIGHT_OFF;
    m_bus_handle     = i2c_bus_handle;
    
    ESP_RETURN_VOID_ON_ERROR(i2c_init(CONFIG_HD44780_LCD_DEVICE_ADDRESS), s_tag, "Failed to initialize I2C Master Bus.");
    ESP_RETURN_VOID_ON_ERROR(display_init(), s_tag, "Failed initializing the display.");

    m_current_row = 0;
}

i2c_lcd_display_t::~i2c_lcd_display_t()
{
    i2c_master_bus_rm_device(m_device_handle);
    if (m_owns_bus)
        i2c_del_master_bus(m_bus_handle);

    m_bus_handle     = nullptr;
    m_device_handle  = nullptr;
}

const char* i2c_lcd_display_t::s_tag = "I2C_LCD";

void i2c_lcd_display_t::clear()
{
    send_command(LCD_HD44780_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void i2c_lcd_display_t::home()
{
    send_command(LCD_HD44780_CMD_HOME);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void i2c_lcd_display_t::display_enable(const bool enable)
{
    if (enable) {
        m_display_control |= LCD_HD44780_CTRL_ON;
        send_command(LCD_HD44780_CMD_DISPLAY_CTRL | m_display_control);
        
        return;
    }

    m_display_control &= ~LCD_HD44780_CTRL_OFF;
    send_command(LCD_HD44780_CMD_DISPLAY_CTRL | m_display_control);
}

void i2c_lcd_display_t::backlight_enable(const bool enable)
{
    if (enable) m_backlight_val = LCD_HD44780_BACKLIGHT_ON;
    else m_backlight_val = LCD_HD44780_BACKLIGHT_OFF;

    transmit(0);
}

void i2c_lcd_display_t::blink_enable(const bool enable)
{
    if (enable) {
        m_display_control |= LCD_HD44780_CTRL_BLINK_ON;
        send_command(LCD_HD44780_CMD_DISPLAY_CTRL | m_display_control);
        
        return;
    }

    m_display_control &= ~LCD_HD44780_CTRL_BLINK_OFF;
    send_command(LCD_HD44780_CMD_DISPLAY_CTRL | m_display_control);
}

void i2c_lcd_display_t::cursor_enable(const bool enable)
{
    if (enable) {
        m_display_control |= LCD_HD44780_CTRL_CURSOR_ON;
        send_command(LCD_HD44780_CMD_DISPLAY_CTRL | m_display_control);
        
        return;
    }

    m_display_control &= ~LCD_HD44780_CTRL_CURSOR_OFF;
    send_command(LCD_HD44780_CMD_DISPLAY_CTRL | m_display_control);
}

void i2c_lcd_display_t::auto_scroll_enable(const bool enable)
{
    if (enable) {
        m_display_mode |= LCD_HD44780_ENTRY_SHIFT_INC;
        send_command(LCD_HD44780_CMD_ENTRY_MODE_SET | m_display_mode);
        
        return;
    }

    m_display_mode &= ~LCD_HD44780_ENTRY_SHIFT_INC;
    send_command(LCD_HD44780_CMD_ENTRY_MODE_SET | m_display_mode);
}

void i2c_lcd_display_t::scroll_left()
{
    send_command(LCD_HD44780_CMD_CURSOR_SHIFT | LCD_HD44780_CS_DISPLAY_MOVE | LCD_HD44780_CS_MOVE_LEFT);
}

void i2c_lcd_display_t::scroll_right()
{
    send_command(LCD_HD44780_CMD_CURSOR_SHIFT | LCD_HD44780_CS_DISPLAY_MOVE | LCD_HD44780_CS_MOVE_RIGHT);
}

void i2c_lcd_display_t::left_to_right()
{
    m_display_mode |= LCD_HD44780_ENTRY_LEFT;
    send_command(LCD_HD44780_CMD_ENTRY_MODE_SET | m_display_mode);
}

void i2c_lcd_display_t::right_to_left()
{
    m_display_mode &= ~LCD_HD44780_ENTRY_LEFT;
    send_command(LCD_HD44780_CMD_ENTRY_MODE_SET | m_display_mode);
}

void i2c_lcd_display_t::create_char(uint8_t index, const uint8_t char_data[])
{
    index &= 0x07;
    send_command(LCD_HD44780_CMD_SET_CGRAM_ADDR | (index << 3));
    for (uint8_t i = 0; i < 8; i++)
        write(char_data[i]);
}

void i2c_lcd_display_t::set_cursor(uint8_t column, uint8_t row)
{
    static const int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	
    // Original from LiquidCrystal_I2C.h. If the row we want is bigger than the number of rows
    // we have we set it to the last one. But this doesn't account if 'row' == 'm_rows', in which
    // case we'll index outside the boundaries of the 'row_offsets'.
    // It also doesn't account for column overflow.
    // if (row > m_rows)
    //     row = m_rows - 1;    // we count rows starting w/0

    // In our implementation if the user sets the location beyond the number of rows or columns we have
    // we go around until we land in a value within boundaries.
    uint8_t ref_row = row + 1;
    if (ref_row > m_rows) {
        row = 0;
        for (uint8_t i = 0; i < ref_row - m_rows; i++) {
            if (row >= m_rows)
                row = 0;

            row++;
        }

        row--;
    }

    uint8_t ref_col = column + 1;
    if (ref_col > m_columns) {
        column = 0;
        for (uint8_t i = 0; i < ref_col - m_columns; i++) {
            if (column >= m_columns)
                column = 0;

            column++;
        }

        column--;
    }

    // Saving the current row for the 'println' function.
    m_current_row = row;

	send_command(LCD_HD44780_CMD_SET_DDRAM_ADDR | (column + row_offsets[row]));
}

size_t i2c_lcd_display_t::print(char* format, ...)
{
    va_list args;
    va_start(args, format);

    int size = vsnprintf(nullptr, 0, format, args) + 1;
    if (size == 1)
        return 0;

    char* buffer = new char[size];

    vsnprintf(buffer, size, format, args);
    va_end(args);

    size_t written = write_buffer(reinterpret_cast<const uint8_t*>(buffer), size - 1);

    delete[] buffer;

    return written;
}

size_t i2c_lcd_display_t::print(const char buffer[])
{
    if (!buffer)
        return 0;
    
    return write_buffer(reinterpret_cast<const uint8_t*>(buffer), strlen(buffer));
}

size_t i2c_lcd_display_t::print(char c)
{
    write(static_cast<uint8_t>(c));

    return 1;
}

size_t i2c_lcd_display_t::print(unsigned char byte, int base)
{
    return print(static_cast<unsigned long>(byte), base);
}

size_t i2c_lcd_display_t::print(int num, int base)
{
    return print(static_cast<long>(num), base);
}

size_t i2c_lcd_display_t::print(unsigned int num, int base)
{
    return print(static_cast<unsigned long>(num), base);
}

size_t i2c_lcd_display_t::print(long num, int base)
{
    if (base == 0) {
        write(num);
        return 1;
    }
    else if (base == 10) {
        if (num < 0) {
            int t = print('-');
            num = -num;
        
            return print_number(num, 10) + t;
        }

        return print_number(num, 10);
    } 
    else {
        return print_number(num, base);
    }
}

size_t i2c_lcd_display_t::print(unsigned long num, int base)
{
    if (base == 0) {
        write(num);
        return 1;
    }
    else {
        return print_number(num, base);
    }
}

size_t i2c_lcd_display_t::print(double num, int digits)
{
    return print_float(num, digits);
}

size_t i2c_lcd_display_t::println(void)
{
    // The matrix LCD display doesn't interpret '\r' and '\n' as line feed and carriage return.
    // Instead will try to print a character it doesn't have.
    // So in order to simulate LF + CR we set the cursor position to the next line at the first column.
    // return print("\r\n");

    set_cursor(0, m_current_row + 1);

    return 2;
}

size_t i2c_lcd_display_t::println(const char buffer[])
{
    size_t num = print(buffer);
    num += println();

    return num;
}

size_t i2c_lcd_display_t::println(char c)
{
    size_t num = print(c);
    num += println();
    
    return num;
}

size_t i2c_lcd_display_t::println(char* format, ...)
{
    va_list args;
    va_start(args, format);

    int size = vsnprintf(nullptr, 0, format, args) + 1;
    if (size == 1)
        return 0;

    char* buffer = new char[size];

    vsnprintf(buffer, size, format, args);
    va_end(args);

    size_t written = write_buffer(reinterpret_cast<const uint8_t*>(buffer), size - 1);
    written += println();

    delete[] buffer;

    return written;
}

size_t i2c_lcd_display_t::println(unsigned char byte, int base)
{
    size_t num = print(byte, base);
    num += println();
    
    return num;
}

size_t i2c_lcd_display_t::println(int num, int base)
{
    size_t n = print(num, base);
    n += println();
    
    return n;
}

size_t i2c_lcd_display_t::println(unsigned int num, int base)
{
    size_t n = print(num, base);
    n += println();
    
    return n;
}

size_t i2c_lcd_display_t::println(long num, int base)
{
    size_t n = print(num, base);
    n += println();
    
    return n;
}

size_t i2c_lcd_display_t::println(unsigned long num, int base)
{
    size_t n = print(num, base);
    n += println();
    
    return n;
}

size_t i2c_lcd_display_t::println(double num, int digits)
{
    size_t n = print(num, digits);
    n += println();
 
    return n;
}

size_t i2c_lcd_display_t::print_number(unsigned long num, uint8_t base)
{
    char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';

    // prevent crash if called with base == 1
    if (base < 2)
        base = 10;

    do {
        char c = num % base;
        num /= base;

        *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while(num);

    return print(str);
}

size_t i2c_lcd_display_t::print_float(double num, uint8_t digits)
{
    size_t n = 0;
  
    if (num > 4294967040.0) return print ("ovf");  // constant determined empirically
    if (num <-4294967040.0) return print ("ovf");  // constant determined empirically
  
    // Handle negative numbers
    if (num < 0.0) {
        n += print('-');
        num = -num;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i=0; i<digits; ++i)
        rounding /= 10.0;
  
    num += rounding;

    // Extract the integer part of the num and print it
    unsigned long int_part = (unsigned long)num;
    double remainder = num - (double)int_part;
    n += print(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0) {
        n += print('.'); 
    }

    // Extract digits from the remainder one at a time
    while (digits-- > 0) {
        remainder *= 10.0;
        unsigned int toPrint = (unsigned int)(remainder);
        n += print(toPrint);
        remainder -= toPrint; 
    } 
  
    return n;
}

size_t i2c_lcd_display_t::write_buffer(const uint8_t* buffer, size_t size)
{
    size_t num = 0;
    while (size--) {
        write(*buffer++);
        num++;
    }
    
    return num;
}

esp_err_t i2c_lcd_display_t::i2c_init(const uint8_t device_address)
{
    if (!m_bus_handle) {
        if (CONFIG_SHT4X_SCL_GPIO < 0) {
            ESP_LOGE(s_tag, "SCL GPIO number cannot be smaller than zero. Please change the value in the SDK Config.");
            return ESP_ERR_INVALID_ARG;
        }

        if (CONFIG_SHT4X_SDA_GPIO < 0) {
            ESP_LOGE(s_tag, "SDA GPIO number cannot be smaller than zero. Please change the value in the SDK Config.");
            return ESP_ERR_INVALID_ARG;
        }

        i2c_master_bus_config_t bus_config{ };
        bus_config.i2c_port                      = I2C_NUM_0;
        bus_config.clk_source                    = I2C_CLK_SRC_DEFAULT;
        bus_config.sda_io_num                    = static_cast<gpio_num_t>(CONFIG_HD44780_LCD_SDA_GPIO);
        bus_config.scl_io_num                    = static_cast<gpio_num_t>(CONFIG_HD44780_LCD_SCL_GPIO);
        bus_config.glitch_ignore_cnt             = 7;
        bus_config.flags.enable_internal_pullup  = true;

        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &m_bus_handle), s_tag, "Failed creating I2C Master Bus.");
    }

    i2c_device_config_t device_config{ };
    device_config.dev_addr_length  = I2C_ADDR_BIT_LEN_7;
    device_config.device_address   = device_address;
    device_config.scl_speed_hz     = CONFIG_HD44780_LCD_CLK_SPEED_HZ;

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(m_bus_handle, &device_config, &m_device_handle), s_tag, "Failed adding device to I2C Master Bus.");

    return ESP_OK;
}

esp_err_t i2c_lcd_display_t::display_init()
{
    m_display_function = LCD_HD44780_FS_MODE_4BIT | LCD_HD44780_FS_1LINE | LCD_HD44780_FS_DOTS_5X8;
    if (m_rows > 1)
        m_display_function |= LCD_HD44780_FS_2LINE;

    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_RETURN_ON_ERROR(i2c_master_transmit(m_device_handle, &m_backlight_val, 1, LCD_HD44780_I2C_TRANS_TIMEOUT_MS), s_tag, "Failed changing backlight state.");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Hitachi HD44780 figure 24, page 46.
    write_nibble(0x03 << 4);
    ets_delay_us(4500);

    write_nibble(0x03 << 4);
    ets_delay_us(4500);

    write_nibble(0x03 << 4);
    ets_delay_us(150);

    write_nibble(0x02 << 4);

    send_command(LCD_HD44780_CMD_FUNCTION_SET | m_display_function);
    m_display_control = LCD_HD44780_CTRL_ON | LCD_HD44780_CTRL_CURSOR_OFF | LCD_HD44780_CTRL_BLINK_OFF;

    display_enable();
    clear();

    m_display_mode = LCD_HD44780_ENTRY_LEFT | LCD_HD44780_ENTRY_SHIFT_DEC;
    send_command(LCD_HD44780_CMD_ENTRY_MODE_SET | m_display_mode);

    home();

    return ESP_OK;
}

esp_err_t i2c_lcd_display_t::write_nibble(const uint8_t value)
{
    ESP_RETURN_ON_ERROR(transmit(value), s_tag, "Failed transmiting nibble to display.");
    ESP_RETURN_ON_ERROR(pulse_enable(value), s_tag, "Failed pulsing enable bit.");

    return ESP_OK;
}

esp_err_t i2c_lcd_display_t::write_byte(const uint8_t value, const uint8_t mode)
{
    uint8_t nib_high  = value & 0xF0;
    uint8_t nib_low   = (value << 4) & 0xF0;
    ESP_RETURN_ON_ERROR(write_nibble((nib_high) | mode), s_tag, "Failed transmiting byte to display.");
    ESP_RETURN_ON_ERROR(write_nibble((nib_low) | mode), s_tag, "Failed transmiting byte to display.");

    return ESP_OK;
}

esp_err_t i2c_lcd_display_t::transmit(const uint8_t data)
{
    uint8_t value = data | m_backlight_val;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(m_device_handle, &value, 1, LCD_HD44780_I2C_TRANS_TIMEOUT_MS), s_tag, "Failed transmiting data to display.");

    return ESP_OK;
}

esp_err_t i2c_lcd_display_t::pulse_enable(const uint8_t data)
{
    ESP_RETURN_ON_ERROR(transmit(data | LCD_HD44780_BIT_ENABLE), s_tag, "Failed pulsing enable bit.");
    ets_delay_us(1);

    ESP_RETURN_ON_ERROR(transmit(data & ~LCD_HD44780_BIT_ENABLE), s_tag, "Failed pulsing enable bit.");
    ets_delay_us(50);

    return ESP_OK;
}