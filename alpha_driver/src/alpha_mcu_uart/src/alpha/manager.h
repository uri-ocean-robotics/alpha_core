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

#ifndef ALPHA_MCU_MANAGER_H
#define ALPHA_MCU_MANAGER_H

// c++
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <queue>
#include <cstring>
// pico
#include "hardware/uart.h"
#include "hardware/gpio.h"

// commom lib
#include "alpha/common/dictionary.h"
#include "alpha/common/types.h"
#include "nmea/nmea.h"
#include "common.h"
#include "parameters.h"

// device lib
#include "pwm_controller.h"


class Manager{
private:
    /** 
     * PWM devices
    */

    std::vector<PwmController*> m_pwm_channels;

    /**
     * UART related
    */

    uart_inst_t* m_uart_id;

    std::string m_str;

    std::queue<std::string> m_str_queue;

    /**
     * timer related
    */

    struct repeating_timer m_reporter_timer;

    struct repeating_timer m_safety_timer;
    
    /**
     * Parse related
    */

    void ParseMsg(const std::string &str);

    bool SendMsg(const std::string &str, bool debug = false);

    bool SendMsgLine(const std::string &str, bool debug = false);

    bool SetPWMInitialized(int channel, int mode);

    static bool ReportPWM(struct repeating_timer *t);

    static bool CheckSafety(struct repeating_timer *t);

public:

    Manager();

    ~Manager();

    void ReceiveMsg();
};

#endif //ALPHA_MCU_MANAGER_H
