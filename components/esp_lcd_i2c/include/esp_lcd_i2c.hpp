#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <esp_check.h>
#include <rom/ets_sys.h>

// Commands.
#define LCD_I2C_CMD_CLEAR               0x01
#define LCD_I2C_CMD_HOME                0x02
#define LCD_I2C_CMD_ENTRY_MODE_SET      0x04
#define LCD_I2C_CMD_DISPLAY_CTRL        0x08
#define LCD_I2C_CMD_CURSOR_SHIFT        0x10
#define LCD_I2C_CMD_FUNCTION_SET        0x20
#define LCD_I2C_CMD_SET_CGRAM_ADDR      0x40
#define LCD_I2C_CMD_SET_DDRAM_ADDR      0x80

// Display entry mode flags.
#define LCD_I2C_ENTRY_RIGHT             0x00
#define LCD_I2C_ENTRY_LEFT              0x02
#define LCD_I2C_ENTRY_SHIFT_INC         0x01
#define LCD_I2C_ENTRY_SHIFT_DEC         0x00

// Display control flags.
#define LCD_I2C_CTRL_ON                 0x04
#define LCD_I2C_CTRL_OFF                0x00
#define LCD_I2C_CTRL_CURSOR_ON          0x02
#define LCD_I2C_CTRL_CURSOR_OFF         0x00
#define LCD_I2C_CTRL_BLINK_ON           0x01
#define LCD_I2C_CTRL_BLINK_OFF          0x00

// Cursor shift flags.
#define LCD_I2C_CS_DISPLAY_MOVE         0x08
#define LCD_I2C_CS_CURSOR_MOVE          0x00
#define LCD_I2C_CS_MOVE_RIGHT           0x04
#define LCD_I2C_CS_MOVE_LEFT            0x00

// Function set flags.
#define LCD_I2C_FS_MODE_8BIT            0x10
#define LCD_I2C_FS_MODE_4BIT            0x00
#define LCD_I2C_FS_2LINE                0x08
#define LCD_I2C_FS_1LINE                0x00
#define LCD_I2C_FS_DOTS_5X10            0x04
#define LCD_I2C_FS_DOTS_5X8             0x00

// Backlight control flags.
#define LCD_I2C_BACKLIGHT_ON            0x08
#define LCD_I2C_BACKLIGHT_OFF           0x00

#define LCD_I2C_BIT_RS                  0x01
#define LCD_I2C_BIT_RW                  0x02
#define LCD_I2C_BIT_ENABLE              0x04

#define LCD_I2C_I2C_TRANS_TIMEOUT_MS    2000

#ifdef CONFIG_I2C_LCD_CLK_1000000
    #define CONFIG_I2C_LCD_CLK_SPEED_HZ 1000000
#elif defined(CONFIG_I2C_LCD_CLK_400000)
    #define CONFIG_I2C_LCD_CLK_SPEED_HZ 400000
#elif defined(CONFIG_I2C_LCD_CLK_100000)
    #define CONFIG_I2C_LCD_CLK_SPEED_HZ 100000
#else
    #error Invalid clock speed.
#endif


class i2c_lcd_display_t
{
public:
    i2c_lcd_display_t();
    i2c_lcd_display_t(const i2c_master_bus_handle_t i2c_bus_handle);
    ~i2c_lcd_display_t();

    void clear();
    void home();
    void display_enable(const bool enable = true);
    void backlight_enable(const bool enable = true);
    void blink_enable(const bool enable = true);
    void cursor_enable(const bool enable = true);
    void auto_scroll_enable(const bool enable = true);
    void scroll_left();
    void scroll_right();
    void left_to_right();
    void right_to_left();
    void create_char(uint8_t index, const uint8_t char_data[]);
    void set_cursor(uint8_t column, uint8_t row);
    inline void send_command(uint8_t command) {
        write_byte(command, 0);
    }
    
    inline void write(uint8_t value) {
        write_byte(value, LCD_HD44780_BIT_RS);
    }

    size_t print(char* format, ...);
    size_t print(const char buffer[]);
    size_t print(char c);
    size_t print(unsigned char byte, int base = 10);
    size_t print(int num, int base = 10);
    size_t print(unsigned int num, int base = 10);
    size_t print(long num, int base = 10);
    size_t print(unsigned long num, int base = 10);
    size_t print(double num, int digits = 2);

    size_t println(char* format, ...);
    size_t println(const char buffer[]);
    size_t println(char c);
    size_t println(unsigned char byte, int base = 10);
    size_t println(int num, int base = 10);
    size_t println(unsigned int num, int base = 10);
    size_t println(long num, int base = 10);
    size_t println(unsigned long num, int base = 10);
    size_t println(double num, int digits = 2);
    size_t println(void);

private:
    static const char* s_tag;

    const bool m_owns_bus;
    const uint8_t m_rows = CONFIG_I2C_LCD_ROWS;
    const uint8_t m_columns = CONFIG_I2C_LCD_COLUMNS;

    i2c_master_dev_handle_t m_device_handle;
    i2c_master_bus_handle_t m_bus_handle;
    
    uint8_t m_current_row;
    uint8_t m_display_mode;
    uint8_t m_backlight_val;
    uint8_t m_display_control;
    uint8_t m_display_function;

    size_t print_number(unsigned long num, uint8_t base);
    size_t print_float(double num, uint8_t digits);

    size_t write_buffer(const uint8_t* buffer, size_t size);

    esp_err_t i2c_init(const uint8_t device_address);
    esp_err_t display_init();
    esp_err_t write_nibble(const uint8_t value);
    esp_err_t write_byte(const uint8_t value, const uint8_t mode);
    esp_err_t transmit(const uint8_t data);
    esp_err_t pulse_enable(const uint8_t data);
};