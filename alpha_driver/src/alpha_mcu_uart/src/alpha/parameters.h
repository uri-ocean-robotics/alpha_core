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
      Lin Zhao <linzhao@uri.edu>
    Year: 2023-2023

    Copyright (C) 2023-2023 Smart Ocean Systems Laboratory
*/

#ifndef ALPHA_PARAMETERS_H
#define ALPHA_PARAMETERS_H

// UART Main Communication
#define UART_ID 0
#define UART_PIN_TX 0
#define UART_PIN_RX 1
#define UART_BAUDRATE 115200

// PWM
#define PWM_CHANNEL_PIN_0 24
#define PWM_CHANNEL_PIN_1 13
#define PWM_CHANNEL_PIN_2 18
#define PWM_CHANNEL_PIN_3 16
#define PWM_CHANNEL_PIN_4 22
#define PWM_CHANNEL_PIN_5 20

#define PWM_PULSE_MAX 1900
#define PWM_PULSE_MIN 1100
#define PWM_PULSE_CTR (PWM_PULSE_MAX + PWM_PULSE_MIN) / 2.0

#define PWM_REPORT_PERIOD 100 // default: 150 ?


// #define REPORT_BAROMETER_PERIOD 200

// #define REPORT_MULTIMETER_PERIOD 100
// #define MULTIMETER_ALERT_PIN 31

// #define ALPHA_I2C_SDA_PIN 16
// #define ALPHA_I2C_SCL_PIN 17

// #define DROP_WEIGHT_PIN 22
// #define REPORT_SAFETY_PERIOD 1000

// #define STROBE_PIN 21
// #define REPORT_STROBE_PERIOD 1000

#endif // ALPHA_PARAMETERS_H