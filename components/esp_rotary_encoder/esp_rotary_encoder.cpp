#include "esp_rotary_encoder.hpp"

esp_rotary_encoder_t::esp_rotary_encoder_t(gpio_isr_t callback)
    : m_a_gpio(static_cast<gpio_num_t>(CONFIG_ROTARY_ENC_A_GPIO)), m_b_gpio(static_cast<gpio_num_t>(CONFIG_ROTARY_ENC_B_GPIO)), m_sw_gpio(static_cast<gpio_num_t>(CONFIG_ROTARY_ENC_SW_GPIO)),
        v_state(0xFF), v_state_prev(INIT_STEP), v_sequence_step(), v_last_sequence_start_us(esp_timer_get_time()), v_direction(rotary_encoder_direction_t::IDLE), v_last_result(rotary_encoder_direction_t::IDLE) {

    ESP_RETURN_VOID_ON_FALSE((m_a_gpio >= 0 && m_b_gpio >= 0 && m_sw_gpio >= 0), ESP_ERR_INVALID_ARG, s_re_tag, "Rotary encoder GPIO pins need to be set in the SDK Config.");

    gpio_config_t io_config{ };

    // 'A' GPIO.
    io_config.intr_type     = GPIO_INTR_ANYEDGE;
    io_config.pin_bit_mask  = 1ULL << m_a_gpio;
    io_config.mode          = GPIO_MODE_INPUT;
    io_config.pull_up_en    = GPIO_PULLUP_ENABLE;
    io_config.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    ESP_RETURN_VOID_ON_ERROR(gpio_config(&io_config), s_re_tag, "Failed configuring GPIO for pin 'A'.");

    // 'B' GPIO.
    io_config.pin_bit_mask  = 1ULL << m_b_gpio;
    ESP_RETURN_VOID_ON_ERROR(gpio_config(&io_config), s_re_tag, "Failed configuring GPIO for pin 'B'.");

    // 'SW' GPIO.
    io_config.pin_bit_mask  = 1ULL << m_sw_gpio;
    ESP_RETURN_VOID_ON_ERROR(gpio_config(&io_config), s_re_tag, "Failed configuring GPIO for pin 'SW'.");

    // ISR config.
    ESP_RETURN_VOID_ON_ERROR(gpio_install_isr_service(0), s_re_tag, "Failed installing GPIO ISR service.");
    ESP_RETURN_VOID_ON_ERROR(gpio_isr_handler_add(m_a_gpio, callback, this), s_re_tag, "Failed adding GPIO ISR handler for pin 'A'.");
    ESP_RETURN_VOID_ON_ERROR(gpio_isr_handler_add(m_b_gpio, callback, this), s_re_tag, "Failed adding GPIO ISR handler for pin 'B'.");
    ESP_RETURN_VOID_ON_ERROR(gpio_isr_handler_add(m_sw_gpio, callback, this), s_re_tag, "Failed adding GPIO ISR handler for pin 'SW'.");
}

esp_rotary_encoder_t::~esp_rotary_encoder_t()
{
    gpio_isr_handler_remove(m_a_gpio);
    gpio_isr_handler_remove(m_b_gpio);
    gpio_isr_handler_remove(m_sw_gpio);

    gpio_uninstall_isr_service();
}

const char* esp_rotary_encoder_t::s_re_tag = "ROTARY_ENCODER";

const uint8_t esp_rotary_encoder_t::s_sequence_cw[MAX_SEQ_STEPS]   = { 0x01, 0x00, 0x02, INIT_STEP };
const uint8_t esp_rotary_encoder_t::s_sequence_ccw[MAX_SEQ_STEPS]  = { 0x02, 0x00, 0x01, INIT_STEP };

bool esp_rotary_encoder_t::get_switch_state()
{
    // The sw GPIO is pulled up, goes down when the sw is pressed.
    return !gpio_get_level(m_sw_gpio);
}

rotary_encoder_direction_t esp_rotary_encoder_t::get_rotation()
{
    set_state((gpio_get_level(m_a_gpio) << 1) | gpio_get_level(m_b_gpio));
    return check_rotation();
}

rotary_encoder_direction_t esp_rotary_encoder_t::check_rotation()
{
    rotary_encoder_direction_t result = rotary_encoder_direction_t::IDLE;

    // Check if the state changed.
    if (v_state != v_state_prev) {

        // Check if it's the begin of rotation.
        if (v_sequence_step == 0) {

            // Begin of clockwise.
            if (v_state == s_sequence_cw[0]) {
                v_direction               = rotary_encoder_direction_t::CLOCKWISE;
                v_sequence_step           = 1;
                v_last_sequence_start_us  = esp_timer_get_time();
            }

            // Begin of counter-clockwise.
            if (v_state == s_sequence_ccw[0]) {
                v_direction               = rotary_encoder_direction_t::COUNTERCLOCKWISE;
                v_sequence_step           = 1;
                v_last_sequence_start_us  = esp_timer_get_time();
            }
        }
        else {
            switch (v_direction) {
                case rotary_encoder_direction_t::CLOCKWISE: {
                    if (v_state == s_sequence_cw[v_sequence_step]) {
                        v_sequence_step = v_sequence_step + 1; // v_sequence_step++ is deprecated since C++ 20.

                        // Checking if sequence has finished.
                        if (v_sequence_step >= MAX_SEQ_STEPS) {
                            result           = rotary_encoder_direction_t::CLOCKWISE;
                            v_last_result    = rotary_encoder_direction_t::CLOCKWISE;
                            v_direction      = rotary_encoder_direction_t::IDLE;
                            v_sequence_step  = 0;
                        }
                        else {
                            result = rotary_encoder_direction_t::ACTIVE;
                        }
                    }
                    else {
                        // Invalid sequence.
                        if (v_state == INIT_STEP) {

                            // Reset sequence in INIT state.
                            v_direction      = rotary_encoder_direction_t::IDLE;
                            v_sequence_step  = 0;
                        }
                    }
                } break;

                case rotary_encoder_direction_t::COUNTERCLOCKWISE: {
                    if (v_state == s_sequence_ccw[v_sequence_step]) {
                        v_sequence_step = v_sequence_step + 1; // v_sequence_step++ is deprecated since C++ 20.

                        // Checking if sequence has finished.
                        if (v_sequence_step >= MAX_SEQ_STEPS) {
                            result           = rotary_encoder_direction_t::COUNTERCLOCKWISE;
                            v_last_result    = rotary_encoder_direction_t::COUNTERCLOCKWISE;
                            v_direction      = rotary_encoder_direction_t::IDLE;
                            v_sequence_step  = 0;
                        }
                        else {
                            result = rotary_encoder_direction_t::ACTIVE;
                        }
                    }
                    else {
                        // Invalid sequence.
                        if (v_state == INIT_STEP) {

                            // Reset sequence in INIT state.
                            v_direction      = rotary_encoder_direction_t::IDLE;
                            v_sequence_step  = 0;
                        }
                    }
                } break;

                default:
                    break;
            }

            v_state_prev = v_state;
        }
    }

    return result;
}