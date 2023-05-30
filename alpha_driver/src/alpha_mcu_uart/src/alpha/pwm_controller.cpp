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

#include "pwm_controller.h"

#include <cmath>


PwmController::PwmController(int pin, int channel, int mode) {
    m_pin = pin;

    m_pwm_channel = pwm_gpio_to_channel(m_pin);

    m_channel = channel;

    m_freq = 50;

    m_pulse_width = PWM_PULSE_CTR;

    m_mode = mode;

    m_current = 0;

    m_desired = 0;

    m_is_enabled = false;

    // setup main communication
    if (UART_ID == 0) {
      m_uart_id = uart0;
    }
    else if (UART_ID == 1) {
      m_uart_id = uart1;
    }    
}

void PwmController::initialize() {

    // setup timer
    add_repeating_timer_ms(PWM_REPORT_PERIOD, f_reporter, this, &m_reporter_timer);

    add_repeating_timer_ms(m_limiter_period, f_limiter, this, &m_limiter_timer);

    gpio_set_function(m_pin, GPIO_FUNC_PWM);

    // setup PWM 
    m_slice_num = pwm_gpio_to_slice_num(m_pin);

    uint32_t f_sys = clock_get_hz(clk_sys);

    float divider = f_sys / 1000000UL;

    pwm_set_clkdiv(m_slice_num, divider);

    m_top = 1000000UL / m_freq - 1;

    pwm_set_wrap(m_slice_num, m_top);

    // initialize PWM
    set_mode(PwmMode::Thruster);

    enable();
}

bool PwmController::f_reporter(struct repeating_timer *t) {

    auto self = (PwmController *) t->user_data;

    if(!self->m_is_enabled) {
        return true;
    }

    NMEA *msg = new NMEA();
    msg->construct(NMEA_FORMAT_PWM_REPORT,
                   NMEA_PWM_REPORT,
                   self->m_channel,
                   self->m_current,
                   self->m_mode,
                   self->m_is_enabled
    );

    std::string str = msg->get_raw();
    self->f_send(str);

    //! DEBUG:
    std::cout<<str<<std::endl;

    delete msg;

    return true;
}
#pragma clang diagnostic pop

void PwmController::set_pwm(float signal) {

    m_last_comm = get_absolute_time();

    if (m_mode == PwmMode::Thruster) {
        f_change_magnitude_limited(signal);
    } else {
        f_change_magnitude(signal);
    }
}

void PwmController::f_change_pulse(uint16_t pulse) {

    m_pulse_width = pulse;

    if(m_is_enabled) {
        pwm_set_chan_level(m_slice_num, m_pwm_channel, m_pulse_width);
    }
}

void PwmController::f_change_magnitude(float magnitude) {
    if(m_mode == PwmMode::Thruster) {
        magnitude = magnitude < -1 ? -1 : magnitude;
        magnitude = magnitude > 1 ? 1 : magnitude;

        m_current = magnitude;
        f_change_pulse(static_cast<uint16_t>
            (std::round(magnitude * ((PWM_PULSE_MAX - PWM_PULSE_MIN) / 2.0) + (PWM_PULSE_MAX + PWM_PULSE_MIN) / 2.0))
        );
    } else if (m_mode == PwmMode::Pure) {
        magnitude = magnitude < 0 ? 0 : magnitude;
        magnitude = magnitude > 1 ? 1 : magnitude;

        m_current = magnitude;
        f_change_pulse(static_cast<uint16_t>
            (std::round(magnitude * (PWM_PULSE_MAX - PWM_PULSE_MIN) + PWM_PULSE_MIN))
        );

    }
}

void PwmController::f_change_magnitude_limited(float magnitude) {
    m_desired = magnitude;
}

bool PwmController::f_limiter(struct repeating_timer *t) {
    auto self = (PwmController*)t->user_data;

    if(self->m_mode == PwmMode::Pure) {
        return false;
    }

    if(!self->m_is_enabled) {
        return true;
    }

    auto diff = self->m_desired - self->m_current;
    if(diff != 0) {

        float dmdt = diff / (static_cast<float>(self->m_limiter_period) / 1000.0f);

        if (std::fabs(dmdt) > 5 /* slope */ ) {
            self->m_current = self->m_current + sgn(diff) * 0.01f; // increment
        } else {
            self->m_current = self->m_desired;
        }

        self->f_change_magnitude(self->m_current);
    }

    return true;
}

void PwmController::enable() {

    sleep_ms(1);

    m_is_enabled = true;

    m_pulse_width = m_mode == PwmMode::Pure ? PWM_PULSE_MIN : PWM_PULSE_CTR;

    pwm_set_chan_level(m_slice_num, m_pwm_channel, m_pulse_width);

    pwm_set_enabled(m_slice_num, true);

    add_repeating_timer_ms(100, f_safety_checker, this, &m_safety_checker_timer);
}

void PwmController::disable() {
    m_is_enabled = false;
    pwm_set_enabled(m_slice_num, false);
}

void PwmController::set_mode(int mode) {
    m_mode = mode;
}

bool PwmController::f_safety_checker(struct repeating_timer *t) {
    auto self = (PwmController*)t->user_data;

    if(is_nil_time(self->m_last_comm)) {
        return true;
    }

    if(absolute_time_diff_us(self->m_last_comm, get_absolute_time()) > 2999999) {
        self->set_pwm(0);
    }

    return true;
}

void PwmController::f_send(const std::string &str) {
    std::string str_out = str + "\r\n";

    if(UART_ID < 0) {
        // use USB
        std::cout << str_out;
    }
    else {
        // use UART
        if(uart_is_writable(m_uart_id)) {
            uart_puts(m_uart_id, str_out.c_str());
        }
    }

}