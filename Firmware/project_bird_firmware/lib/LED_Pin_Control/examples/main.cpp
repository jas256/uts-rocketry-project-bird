#include "Arduino.h"
#include <LED_Pin_Control_Arduino.h>


LED_Pin_Control_Arduino led1(5);
LED_Pin_Control_Arduino led2(6);
LED_Pin_Control_Arduino led3(9);


void setup() {
  // put your setup code here, to run once:
  led1.init();
  led2.init();
  led3.init();
  
  led1.setMode(LED_Pin_Control_Arduino::PinModes::Strobe);
  led2.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
  led3.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
  
  led1.setBlinkingPeriod(100,500);
  led2.pulse(1000);
  led3.setPWMBlikingState(200,10);
  led3.setBlinkingPeriod(1000,100);

}

void loop() {
  // put your main code here, to run repeatedly:
  led1.update();
  led2.update();
  led3.update();
}