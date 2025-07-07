#include "esp_pid.hpp"

/**
 * Config.
 */

esp_pid_config_t::esp_pid_config_t(double _kp, double _ki, double _kd, double min, double max)
    : kp(_kp), ki(_ki), kd(_kd), minimum(min), maximum(max), flags({ esp_pid_direction_t::direct, esp_pid_prop_t::on_error }) { }

esp_pid_config_t::esp_pid_config_t(double _kp, double _ki, double _kd, double min, double max, esp_pid_direction_t direction, esp_pid_prop_t prop)
    : kp(_kp), ki(_ki), kd(_kd), minimum(min), maximum(max), flags({ direction, prop }) { }

esp_pid_config_t::~esp_pid_config_t() { }


/**
 * PID.
 */

esp_pid_t::esp_pid_t(const esp_pid_config_t& config)
    : m_input(), m_output(), m_setpoint(), m_last_input(), m_output_sum(), m_last_time(timer_get_time_ms() - 100), m_sample_time(100),  m_config(config) { }

esp_pid_t::~esp_pid_t() { }

const char* esp_pid_t::s_tag = "ESP_PID";

double esp_pid_t::input() { return m_input; }
double esp_pid_t::output() { return m_output; }
double esp_pid_t::setpoint() { return m_setpoint; }

void esp_pid_t::set_input(const double input) { m_input = input; }
void esp_pid_t::set_output(const double output) { m_output = output; }
void esp_pid_t::set_setpoint(const double setpoint) { m_setpoint = setpoint; }
void esp_pid_t::set_proportional(const esp_pid_prop_t prop) { m_config.flags.prop = prop; }

void esp_pid_t::set_direction(const esp_pid_direction_t direction)
{
    if (direction != m_config.flags.direction)
        reverse_params();

    m_config.flags.direction = direction;
}

void esp_pid_t::set_sample_time(const uint32_t time)
{
    double ratio = static_cast<double>(time) / static_cast<double>(m_sample_time);

    m_config.ki *= ratio;
    m_config.kd /= ratio;
    m_sample_time = time;
}

esp_err_t esp_pid_t::set_tunning(double kp, double ki, double kd)
{
    ESP_RETURN_ON_FALSE((kp < 0 || ki < 0 || kd < 0), ESP_ERR_INVALID_ARG, s_tag, "Tunning parameters can't be smaller than zero.");

    double sample_time_sec = static_cast<double>(m_sample_time) / 1000;
    m_config.kp = kp;
    m_config.ki = ki * sample_time_sec;
    m_config.kd = kd / sample_time_sec;

    if (m_config.flags.direction == esp_pid_direction_t::reverse)
        reverse_params();

    return ESP_OK;
}

esp_err_t esp_pid_t::set_output_limit(double lower, double higher)
{
    ESP_RETURN_ON_FALSE((lower < higher), ESP_ERR_INVALID_ARG, s_tag, "Lower output limit is less than or equal to higher limit. Low: %.2f; High: %.2f.", lower, higher);

    m_config.minimum = lower;
    m_config.maximum = higher;

    m_output      = get_bound_value(m_output);
    m_output_sum  = get_bound_value(m_output_sum);

    return ESP_OK;
}

double esp_pid_t::compute()
{
    int64_t current_time = timer_get_time_ms();
    int64_t delta = current_time - m_last_time;
    if (delta < m_sample_time)
        return m_output;

    double input        = m_input;
    double error        = m_setpoint - input;
    double delta_input  = input - m_last_input;

    m_output_sum += m_config.ki * error;

    if (is_pom())
        m_output_sum -= m_config.kp * delta_input;

    m_output_sum = get_bound_value(m_output_sum);
    m_output = is_pom() ? 0 : m_config.kp * error;

    m_output += m_output_sum - m_config.kd * delta_input;
    m_output = get_bound_value(m_output);

    m_last_input  = input;
    m_last_time   = current_time;

    return m_output;
}