#pragma once

#include <stdlib.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_check.h>

#define INIT_STEP      0x03
#define MAX_SEQ_STEPS  4

enum class rotary_encoder_direction_t : uint8_t
{
    IDLE,
    ACTIVE,
    CLOCKWISE,
    COUNTERCLOCKWISE,
};

#pragma pack(1)
struct encoder_state_t
{
    bool switch_state;
    rotary_encoder_direction_t rotation_state;
};

// No need to pass the structure as reference, it's small enough and we don't care about it once we send it.
typedef void (*encoder_state_cb_t)(const encoder_state_t state);

class esp_rotary_encoder_t
{
public:
    esp_rotary_encoder_t(gpio_isr_t state_callback);
    ~esp_rotary_encoder_t();

    bool get_switch_state();
    rotary_encoder_direction_t get_rotation();

private:
    gpio_num_t m_a_gpio;
    gpio_num_t m_b_gpio;
    gpio_num_t m_sw_gpio;

    volatile uint8_t v_state;
    volatile uint8_t v_state_prev;
    volatile uint8_t v_sequence_step;
    volatile int64_t v_last_sequence_start_us;
    volatile rotary_encoder_direction_t v_direction;
    volatile rotary_encoder_direction_t v_last_result;

    static const uint8_t s_sequence_cw[MAX_SEQ_STEPS];
    static const uint8_t s_sequence_ccw[MAX_SEQ_STEPS];

    static const char* s_re_tag;

    rotary_encoder_direction_t check_rotation();

    inline void set_state(const uint8_t state) { v_state = state; }
};