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
}

void PwmController::initialize() {

    // setup timer
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

    sleep_ms(1000);

    m_is_enabled = true;

    m_pulse_width = m_mode == PwmMode::Pure ? PWM_PULSE_MIN : PWM_PULSE_CTR;

    pwm_set_chan_level(m_slice_num, m_pwm_channel, m_pulse_width);

    pwm_set_enabled(m_slice_num, true);

    // add_repeating_timer_ms(100, f_safety_checker, this, &m_safety_checker_timer);
}

void PwmController::disable() {
    m_is_enabled = false;
    pwm_set_enabled(m_slice_num, false);
}

void PwmController::set_mode(int mode) {
    m_mode = mode;
}

// bool PwmController::f_safety_checker(struct repeating_timer *t) {
//     auto self = (PwmController*)t->user_data;

//     if(is_nil_time(self->m_last_comm)) {
//         return true;
//     }

    // if(absolute_time_diff_us(self->m_last_comm, get_absolute_time()) > 2999999) {
//         self->set_pwm(0);
//         // //! DEBUG:
//         // self->f_send("ch: " + 
//         //         std::to_string(self->m_channel) + 
//         //         ", diff= " + 
//         //         std::to_string(absolute_time_diff_us(self->m_last_comm, get_absolute_time())));
//     }

//     return true;
// }

int64_t PwmController::get_comm_duration() {
    // Determine if the given timestamp is nil
    if(is_nil_time(m_last_comm)) {
        return 0;
    }

    // check the duration since last pwm command
    // return absolute_time_diff_us(m_last_comm, get_absolute_time());
    return m_last_comm;
    // return get_absolute_time();
}

void PwmController::f_send(const std::string &str, bool debug) {
    std::string str_line = str + "\r\n";

    if(!debug) {
        // use the normal comm port
        if(COMM_NORMAL_TYPE==0){
            // use UART0
            if(uart_is_writable(uart0)) {
                uart_puts(uart0, str_line.c_str());
            }            
        }
        else if(COMM_NORMAL_TYPE==1) {
            // use UART1
            if(uart_is_writable(uart1)) {
                uart_puts(uart1, str_line.c_str());
            }            
        }
        else {
            // use USB
            std::cout << str_line;
        }
    }
    else {
        // use the debug comm port
        if(COMM_DEBUG_TYPE==0){
            // use UART0
            if(uart_is_writable(uart0)) {
                uart_puts(uart0, str_line.c_str());
            }            
        }
        else if(COMM_DEBUG_TYPE==1) {
            // use UART1
            if(uart_is_writable(uart1)) {
                uart_puts(uart1, str_line.c_str());
            }            
        }
        else {
            // use USB
            std::cout << str_line;
        }        
    }
}

int PwmController::get_channel() {
    return m_channel;
}

float PwmController::get_current() {
    return m_current;
}

int PwmController::get_mode() {
    return m_mode;
}

bool PwmController::get_enable() {
    return m_is_enabled;
}