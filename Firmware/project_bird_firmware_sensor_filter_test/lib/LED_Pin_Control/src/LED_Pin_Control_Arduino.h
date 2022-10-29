#pragma once

#include <Arduino.h>
#include <LED_Pin_Control.h>

class LED_Pin_Control_Arduino : public LED_Pin_Control
{
private:
    /* data */
protected:
    void setDigitalHal(bool state, unsigned int pin_number);
    void setPwmHal(unsigned char state, unsigned int pin_number);
    void setupPin(unsigned int pin_number);
    unsigned long getCurrentTime();
public:
    LED_Pin_Control_Arduino(int pin_number);
    ~LED_Pin_Control_Arduino() = default;
};
