#pragma once

class LED_Pin_Control
{
public:
    LED_Pin_Control(unsigned int pin_number);
    ~LED_Pin_Control() = default;

    enum class PinModes
    {
        //Primitives
        Disabled,
        Solid,
        Blinking,
        PWM,

        //Functional
        Strobe, // On for x Seconds. Off for y Seconds
        PWM_Strobe, // On with PWM Value for x Seconds, Off with PWM Value for y Seconds
        PWM_Fading // Fade Between Values based on blinking period
    };

    void setMode(PinModes mode);
    void setBlinkingPeriod(unsigned long blink_ms);
    void setBlinkingPeriod(unsigned long blink_ms_on, unsigned long blink_ms_off);
    
    void setPWMBlikingState(unsigned char fade_state);    
    void setPWMBlikingState(unsigned char fade_state_on,unsigned char fade_state_off);

    bool setDigitalState(bool onState);
    bool setPWMState(unsigned char brightness);

    void pulse(unsigned long pulse_duration);

    void update();
    void init();

protected:
    unsigned char pwm_blink_on_state_;
    unsigned char pwm_blink_off_state_;
    bool fade_mode_state;

    unsigned long blink_on_period_;
    unsigned long blink_off_period_;

    unsigned long pulse_period_;
    bool pulse_activation_;

    bool digital_state_;
    unsigned char pwm_state_;
    unsigned int pin_number_;
    PinModes current_mode_ = PinModes::Disabled;
    unsigned long blink_fade_timer_;

    void updatePWM();
    void updateDigital();

    // Hardware Specific abstraction
    virtual void setDigitalHal(bool state, unsigned int pin_number) = 0;
    virtual void setPwmHal(unsigned char state, unsigned int pin_number) = 0;
    virtual void setupPin(unsigned int pin_number) = 0;
    virtual unsigned long getCurrentTime() = 0;
};
