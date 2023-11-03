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

#include "manager.h"

Manager::Manager() {
    // setup uart parameters
    if (UART_ID == 0) {
      m_uart_id = uart0;
    }
    else if (UART_ID == 1) {
      m_uart_id = uart1;
    }

    // setup uart
    gpio_set_function(UART_PIN_TX, GPIO_FUNC_UART);
    gpio_set_function(UART_PIN_RX, GPIO_FUNC_UART);
    uart_init(m_uart_id, UART_BAUDRATE);

    // setup devices
    m_pwm_chan0 = new PwmController(PWM_CHANNEL_PIN_0, 0);
    m_pwm_chan1 = new PwmController(PWM_CHANNEL_PIN_1, 1);
    m_pwm_chan2 = new PwmController(PWM_CHANNEL_PIN_2, 2);
    m_pwm_chan3 = new PwmController(PWM_CHANNEL_PIN_3, 3);
    m_pwm_chan4 = new PwmController(PWM_CHANNEL_PIN_4, 4);
    m_pwm_chan5 = new PwmController(PWM_CHANNEL_PIN_5, 5);

    m_pwm_chan0->initialize();
    m_pwm_chan1->initialize();
    m_pwm_chan2->initialize();
    m_pwm_chan3->initialize();
    m_pwm_chan4->initialize();
    m_pwm_chan5->initialize();
}

Manager::~Manager() {
    delete m_pwm_chan0;
    delete m_pwm_chan1;
    delete m_pwm_chan2;
    delete m_pwm_chan3;
    delete m_pwm_chan4;
    delete m_pwm_chan5;
}

void Manager::ReceiveMsg() {
    while(true) {
        while (uart_is_readable(m_uart_id)) {
            // get each char
            uint8_t c = uart_getc(m_uart_id);

            if(c == '\n') {
                m_str.push_back(c);
                m_str_queue.push(m_str);
                m_str.clear();
            }
            else {
                m_str.push_back(c);
            }

            // check if the string can be sent
            if(m_str_queue.size() > 0) {
                // parse the coming msg into specific cmd
                ParseMsg(m_str_queue.front());
                // delete the used msg
                m_str_queue.pop();
            } 
        }       
    }
}

void Manager::ParseMsg(const std::string &str) {
    //! DEBUG:
    // SendMsg(str);
    // std::cout<<str;

    //! TODO: send bad msg back to pi: $PCIO,str*00\r\n

    NMEA msg(str.c_str());

    msg.parse();

    if(!msg.get_valid()) {
        auto str_invaild = "X: " + str; 
        SendMsg(str_invaild);
        return;
    }

    if (strcmp(msg.get_cmd(), NMEA_PWM_CMD) == 0) {
        // bad
        if(msg.get_argc() != 2) {
            return;
        }

        // get info from parsed msg
        int channel;
        float signal;
        sscanf(msg.get_data(), "%*[^,],%d,%f", &channel, &signal);

        // do something
        SetPWM(channel, signal);
    }
    // else if (strcmp(msg.get_cmd(), NMEA_PWM_INITIALIZE) == 0) {
    //     // bad
    //     if(msg.get_argc() != 2) {
    //         return;
    //     }

    //     // get info from parsed msg
    //     int channel;
    //     int mode;
    //     sscanf(msg.get_data(), "%*[^,],%d,%d", &channel, &mode);

    //     // do something
    //     SetPWMInitialized(channel, mode); 
    // }    
    else {
        // report 
        auto str = "Wrong";
        SendMsgLine(str);
    }
}

bool Manager::SetPWMInitialized(int channel, int mode) {
    // report
    auto str = "Ini: " + std::to_string(channel) + 
               " Mode: " + std::to_string(mode);
    SendMsgLine(str);

    switch (channel) {
        case 0:
            m_pwm_chan0->set_mode(mode);
            m_pwm_chan0->enable();
            break;
        case 1:
            m_pwm_chan1->set_mode(mode);
            m_pwm_chan1->enable();
            break;
        case 2:
            m_pwm_chan2->set_mode(mode);
            m_pwm_chan2->enable();
            break;
        case 3:
            m_pwm_chan3->set_mode(mode);
            m_pwm_chan3->enable();
            break;
        case 4:
            m_pwm_chan4->set_mode(mode);
            m_pwm_chan4->enable();
            break;     
        case 5:
            m_pwm_chan5->set_mode(mode);
            m_pwm_chan5->enable();
            break;                   
        default:
            return false;
    }

    return true;
}

bool Manager::SetPWM(int channel, float signal) {
    switch (channel) {
        case 0:
            m_pwm_chan0->set_pwm(signal);
            break;
        case 1:
            m_pwm_chan1->set_pwm(signal);
            break;
        case 2:
            m_pwm_chan2->set_pwm(signal);
            break;
        case 3:
            m_pwm_chan3->set_pwm(signal);
            break;
        case 4:
            m_pwm_chan4->set_pwm(signal);
            break;  
        case 5:
            m_pwm_chan5->set_pwm(signal);
            break;                      
        default:
            break;
    }
    return true;
}

bool Manager::SendMsg(const std::string &str) {
    bool flag = false;

    if(UART_ID < 0) {
        // use USB
        std::cout << str;
        flag = true;
    }
    else {
        // use UART
        if(uart_is_writable(m_uart_id)) {
            uart_puts(m_uart_id, str.c_str());
            flag = true;
        }
    }

    return flag;
}

bool Manager::SendMsgLine(const std::string &str) {
    bool flag = false;

    std::string str_line = str + "\r\n";

    if(UART_ID < 0) {
        // use USB
        std::cout << str_line;
        flag = true;
    }
    else {
        // use UART
        if(uart_is_writable(m_uart_id)) {
            uart_puts(m_uart_id, str_line.c_str());
            flag = true;
        }
    }    

    return flag;
}
