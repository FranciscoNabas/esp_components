#pragma once

#include <stdlib.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_check.h>

enum class esp_pid_direction_t : uint8_t
{
    direct,
    reverse,
};

enum class esp_pid_prop_t : uint8_t
{
    on_error,
    on_measure,
};

struct esp_pid_config_t
{
    double kp;
    double ki;
    double kd;

    double minimum;
    double maximum;

    struct {
        esp_pid_direction_t direction;
        esp_pid_prop_t prop;
    } flags;

    esp_pid_config_t(double _kp, double _ki, double _kd, double min, double max);
    esp_pid_config_t(double _kp, double _ki, double _kd, double min, double max, esp_pid_direction_t direction, esp_pid_prop_t prop);
    ~esp_pid_config_t();
};

class esp_pid_t
{
public:
    esp_pid_t(const esp_pid_config_t& config);
    ~esp_pid_t();

    double input();
    double output();
    double setpoint();

    void set_input(const double input);
    void set_output(const double output);
    void set_setpoint(const double setpoint);
    void set_direction(const esp_pid_direction_t direction);
    void set_proportional(const esp_pid_prop_t prop);
    void set_sample_time(const uint32_t time);
    esp_err_t set_tunning(double kp, double ki, double kd);
    esp_err_t set_output_limit(double lower, double higher);

    double compute();

private:
    static const char* s_tag;

    double m_input;
    double m_output;
    double m_setpoint;

    double m_last_input;
    double m_output_sum;
    uint32_t m_last_time;
    uint32_t m_sample_time;

    esp_pid_config_t m_config;

    inline int64_t timer_get_time_ms() { return esp_timer_get_time() / 1000; }
    inline bool is_pom() { return m_config.flags.prop == esp_pid_prop_t::on_measure; }
    inline double get_bound_value(double value) { return value > m_config.maximum ? m_config.maximum : value < m_config.minimum ? m_config.minimum : value; }

    inline void reverse_params()
    {
        m_config.kp = (0 - m_config.kp);
        m_config.ki = (0 - m_config.ki);
        m_config.kd = (0 - m_config.kd);
    }
};