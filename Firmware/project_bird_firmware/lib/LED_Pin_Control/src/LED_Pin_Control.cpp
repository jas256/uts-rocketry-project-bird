#include "LED_Pin_Control.h"

LED_Pin_Control::LED_Pin_Control(unsigned int pin_number) : pin_number_(pin_number)
{
}

void LED_Pin_Control::setMode(PinModes mode)
{
    current_mode_ = mode;
}

void LED_Pin_Control::setBlinkingPeriod(unsigned long blink_ms_on, unsigned long blink_ms_off)
{
    blink_on_period_ = blink_ms_on;
    blink_off_period_ = blink_ms_off;
}

void LED_Pin_Control::setPWMBlikingState(unsigned char fade_state_on, unsigned char fade_state_off)
{
    pwm_blink_on_state_ = fade_state_on;
    pwm_blink_off_state_ = fade_state_off;
}

void LED_Pin_Control::setBlinkingPeriod(unsigned long blink_ms)
{
    setBlinkingPeriod(blink_ms, blink_ms);
}

void LED_Pin_Control::setPWMBlikingState(unsigned char fade_state)
{
    setPWMBlikingState(fade_state, 0);
}

bool LED_Pin_Control::setDigitalState(bool onState)
{
    if (current_mode_ != PinModes::Solid)
        return false;
    digital_state_ = onState;
    return true;
}

bool LED_Pin_Control::setPWMState(unsigned char brightness)
{
    if (current_mode_ != PinModes::PWM)
        return false;
    pwm_state_ = brightness;
    return true;
}

void LED_Pin_Control::pulse(unsigned long pulse_duration)
{
    pulse_activation_ = true;
    pulse_period_ = pulse_duration;
    blink_fade_timer_ = getCurrentTime();
}

void LED_Pin_Control::update()
{
    switch (current_mode_)
    {
    case PinModes::Solid:
        updateDigital();
        break;
    case PinModes::Blinking:
        updateDigital();
        break;
    case PinModes::PWM:
        updatePWM();
        break;
    case PinModes::Strobe:
        updateDigital();
        break;
    case PinModes::PWM_Strobe:
        updatePWM();
        break;
    case PinModes::PWM_Fading:
        updatePWM();
        break;
    case PinModes::Disabled:
        break;
    default:
        break;
    }
}

void LED_Pin_Control::updatePWM()
{
    switch (current_mode_)
    {
    case PinModes::PWM:
        if (pulse_activation_ == true)
        {
            setPWMState(pwm_blink_on_state_);
            if (getCurrentTime() - blink_fade_timer_ >= pulse_period_)
            {
                setPWMState(pwm_blink_off_state_);
                pulse_activation_ = false;
            }
        }
        break;
    case PinModes::PWM_Strobe:
        if (pwm_state_ == pwm_blink_on_state_)
        {
            if (getCurrentTime() - blink_fade_timer_ >= blink_on_period_)
            {
                pwm_state_ = pwm_blink_off_state_;
                blink_fade_timer_ = getCurrentTime();
            }
        }
        else
        {
            if (getCurrentTime() - blink_fade_timer_ >= blink_off_period_)
            {
                pwm_state_ = pwm_blink_on_state_;
                blink_fade_timer_ = getCurrentTime();
            }
        }
        break;
    case PinModes::PWM_Fading:
        float delta_state = pwm_blink_on_state_ - pwm_blink_off_state_;
        if (delta_state < 0)
            delta_state *= -1; //absolute value it

        if (fade_mode_state == true)
        { 
            float current_percentage = float(getCurrentTime() - blink_fade_timer_) / float(blink_on_period_);
            pwm_state_ = pwm_blink_off_state_ + (current_percentage * float(delta_state));
            if (getCurrentTime() - blink_fade_timer_ >= blink_on_period_)
            {
                fade_mode_state = false;
                blink_fade_timer_ = getCurrentTime();
            }
        }
        else
        {
            float current_percentage = float(getCurrentTime() - blink_fade_timer_) / float(blink_off_period_);
            pwm_state_ = pwm_blink_on_state_ - (current_percentage * float(delta_state));
            if (getCurrentTime() - blink_fade_timer_ >= blink_off_period_)
            {
                fade_mode_state = true;
                blink_fade_timer_ = getCurrentTime();
            }
        }
        break;
    case PinModes::Disabled:
        return;
        break;
    default:
        return;
        break;
    }
    setPwmHal(pwm_state_, pin_number_);
}

void LED_Pin_Control::updateDigital()
{
    switch (current_mode_)
    {
    case PinModes::Solid:
        if (pulse_activation_ == true)
        {
            setDigitalState(true);
            if (getCurrentTime() - blink_fade_timer_ >= pulse_period_)
            {
                setDigitalState(false);
                pulse_activation_ = false;
            }
        }
        break;
    case PinModes::Blinking:
        if (getCurrentTime() - blink_fade_timer_ >= blink_on_period_)
        {
            digital_state_ = !digital_state_;
            blink_fade_timer_ = getCurrentTime();
        }
        break;
    case PinModes::Strobe:
        if (digital_state_ == true)
        {
            if (getCurrentTime() - blink_fade_timer_ >= blink_on_period_)
            {
                digital_state_ = false;
                blink_fade_timer_ = getCurrentTime();
            }
        }
        else
        {
            if (getCurrentTime() - blink_fade_timer_ >= blink_off_period_)
            {
                digital_state_ = true;
                blink_fade_timer_ = getCurrentTime();
            }
        }
        break;
    case PinModes::Disabled:
        return;
        break;
    default:
        return;
        break;
    }
    setDigitalHal(digital_state_, pin_number_);
}

void LED_Pin_Control::init()
{
    setupPin(pin_number_);
}
