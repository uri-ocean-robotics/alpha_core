/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Authors: 
      Emir Cem Gezer <emircem.gezer@gmail.com> 
      Lin Zhao <linzhao@uri.edu>
    Year: 2022-2023

    Copyright (C) 2022-2023 Smart Ocean Systems Laboratory
*/

#ifndef ALPHA_MCU_PWM_CONTROLLER_H
#define ALPHA_MCU_PWM_CONTROLLER_H

#include "alpha/common/dictionary.h"
#include "cmath"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#include "common.h"
#include "parameters.h"
#include "iostream"

#include "nmea/nmea.h"

class PwmController {
private:
    uint16_t m_freq;

    uint32_t m_slice_num;

    uint16_t m_pulse_width;

    int m_mode;

    uint16_t m_top;

    int m_channel;

    absolute_time_t m_last_comm;

    int m_pin;

    int m_pwm_channel;

    float m_desired;

    float m_current;

    bool m_is_enabled;

    const uint16_t m_limiter_period = 4; // ms

    // struct repeating_timer m_safety_checker_timer;

    struct repeating_timer m_limiter_timer;

    uart_inst_t* m_uart_id;

    void f_change_pulse(uint16_t pulse);

    static bool f_reporter(struct repeating_timer *t);

    static bool f_limiter(struct repeating_timer *t);

    void f_change_magnitude(float magnitude);

    void f_change_magnitude_limited(float magnitude);

    void f_send(const std::string &str, bool debug=false);

public:

    explicit PwmController(int pin, int channel, int mode = PwmMode::Undefined);

    void initialize();

    void set_pwm(float signal);

    void enable();

    void disable();

    // get the the time duration since last communication (e.g., pwm command)
    int64_t get_comm_duration();

    void set_mode(int mode);

    int get_channel();

    float get_current();

    int get_mode();

    bool get_enable();
};

#endif //ALPHA_MCU_PWM_CONTROLLER_H
