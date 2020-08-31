#include "LED_Pin_Control_Arduino.h"


void LED_Pin_Control_Arduino::setDigitalHal(bool state, unsigned int pin_number) {
    digitalWrite(pin_number,state);
}

void LED_Pin_Control_Arduino::setPwmHal(unsigned char state, unsigned int pin_number) {
    analogWrite(pin_number,state);
}

void LED_Pin_Control_Arduino::setupPin(unsigned int pin_number) {
    pinMode(pin_number,OUTPUT);
}

unsigned long LED_Pin_Control_Arduino::getCurrentTime() {
    return millis();
}

LED_Pin_Control_Arduino::LED_Pin_Control_Arduino(int pin_number) : LED_Pin_Control(pin_number)
{
    
}
