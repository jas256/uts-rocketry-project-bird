#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <board_definition.h>

#include "RunningAverage.h"
#include <MS5xxx.h>
#include <Adafruit_LIS331DLH.h>
#include <Adafruit_Sensor.h>
#include <Bounce2.h>
#include <LED_Pin_Control_Arduino.h>
#include <CircularBuffer.h>

#define CIRCULAR_BUFFER_INT_SAFE

MS5xxx barometer(&Wire);
Adafruit_LIS331DLH accelerometer = Adafruit_LIS331DLH();

RunningAverage barometer_temperature_avg(4);
RunningAverage barometer_pressure_avg(4); // must use average due to possible ejection charges
float barometer_temperature;
float barometer_pressure;
float armed_pressure;

LED_Pin_Control_Arduino status_led(LED1_STATUS);
LED_Pin_Control_Arduino armed_led(LED2_ARMED);

Button button = Button();
Bounce cut_wire = Bounce();

char current_flight_record;

enum class SystemState
{
    DISARMED,
    INSTALL,
    ARMED,
    FLIGHT,
    LANDED,
    PC_DATA_TRANSFER
} current_state;

enum class FlightState
{
    NO_FLIGHT,
    LAUNCHED,
    APOGEE_REACHED,
    DROUGE_DEPLOYED,
    MAIN_DEPLOYED,
    LANDED
} flight_state;

struct FlightInfoFrame
{
    enum EventCode : byte
    {
        NO_EVENT,
        LAUNCHED_OFF_RAIL,
        APOGEE,
        LANDED
    };
    enum DepolymentEvent : byte
    {
        DEPLOY_NO_EVENT,
        DEPLOY_MAIN_DEPLOYED,
        DEPLOY_DROUGUE_DEPLOYED,
        DEPLOY_MAIN2_DEPLOYED,
        DEPLOY_DROUGUE2_DEPLOYED
    };

    uint32_t timestamp;
    EventCode event;
    DepolymentEvent deployment_event;
    float ground_pressure;
    float absolute_pressure;
    float temperature;
    float acceleration_data[3]; // x, y, z
};

byte updateBarometerData()
{
    barometer.ReadProm();
    barometer.Readout();
    ;
    barometer_pressure_avg.addValue(barometer.GetPres());
    barometer_temperature_avg.addValue(barometer.GetTemp());
    barometer_pressure = barometer_pressure_avg.getFastAverage();
    barometer_temperature = barometer_temperature_avg.getFastAverage();

    return 0x00;
}

byte readSensorData(FlightInfoFrame *dataframe)
{
    static unsigned long last_event_time;
    sensors_event_t event;
    accelerometer.getEvent(&event);

    if (event.timestamp != last_event_time)
    {
        dataframe->timestamp = event.timestamp;
        dataframe->ground_pressure = armed_pressure;
        dataframe->absolute_pressure = barometer_temperature;
        dataframe->temperature = barometer_pressure;
        dataframe->acceleration_data[0] = event.acceleration.x;
        dataframe->acceleration_data[1] = event.acceleration.y;
        dataframe->acceleration_data[2] = event.acceleration.z;
        last_event_time = event.timestamp;
        return 0x00;
    }
    return 0x01;
}

byte sensorSetup()
{
    barometer.setI2Caddr(BAROMETER_ADDRESS);
    if (barometer.connect() > 0)
        return 0x01;
    if (!accelerometer.begin_I2C(ACCELEROMETER_ADDRESS, &Wire))
        return 0x02;
    accelerometer.configIntDataReady(digitalPinToInterrupt(ACCELEROMETER_INT1), true, true);
    accelerometer.setDataRate(LIS331_DATARATE_50_HZ);
    FlightInfoFrame test_frame;
    if (readSensorData(&test_frame) > 0)
        return 0x03;
    for (int ix = 0; ix < 4; ix++)
    {
        barometer_pressure_avg.addValue(0);
        barometer_temperature_avg.addValue(0);
    }
    return 0x00;
}

byte outputSetup()
{
    digitalWrite(LOW, CHARGE_DETONATE_CH1);
    pinMode(CHARGE_DETONATE_CH1, OUTPUT);
    digitalWrite(LOW, CHARGE_DETONATE_CH1);

    digitalWrite(LOW, CHARGE_DETONATE_CH2);
    pinMode(CHARGE_DETONATE_CH2, OUTPUT);
    digitalWrite(LOW, CHARGE_DETONATE_CH2);

    digitalWrite(LOW, CHARGE_DETONATE_CH3);
    pinMode(CHARGE_DETONATE_CH3, OUTPUT);
    digitalWrite(LOW, CHARGE_DETONATE_CH3);

    digitalWrite(LOW, CHARGE_DETONATE_CH4);
    pinMode(CHARGE_DETONATE_CH4, OUTPUT);
    digitalWrite(LOW, CHARGE_DETONATE_CH4);

    status_led.init();
    armed_led.init();

    pinMode(BUZZER, OUTPUT);
    digitalWrite(LOW, BUZZER);

    pinMode(FLASH_CHIP_WP, OUTPUT);
    digitalWrite(LOW, FLASH_CHIP_WP);

    return 0x00;
}

byte inputSetup()
{
    button.attach(PUSHBUTTON, INPUT_PULLUP);
    button.setPressedState(false);
    cut_wire.attach(WIRE_CUT_SENSE, INPUT_PULLUP);

    pinMode(CHARGE_CONTINUITY_CH1, INPUT);
    pinMode(CHARGE_CONTINUITY_CH2, INPUT);
    pinMode(CHARGE_CONTINUITY_CH3, INPUT);
    pinMode(CHARGE_CONTINUITY_CH4, INPUT);
    pinMode(ACCELEROMETER_INT1, INPUT);
    pinMode(ACCELEROMETER_INT2, INPUT);

    return 0x00;
}

float calculateAltitude(float current_pressure, float ground_pressure, float ground_temperature) // Only for Serial Terminal due to long calculation time
{
    // From https://www.mide.com/air-pressure-at-altitude-calculator
    float temp_kelvin = ground_temperature + 273.15;
    float tb_lb = temp_kelvin / -0.0065;
    const float constant_exponent = 0.1902632365; // calculated from (-8.31432 * -0.0065)/(9.80665*0.0289644)
    float p_pb = pow((current_pressure / ground_pressure), constant_exponent) - 1;
    return tb_lb * p_pb;
}

// IO Updaters

void ledUpdate()
{
    switch (current_state)
    {
    case SystemState::PC_DATA_TRANSFER:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        status_led.setPWMBlikingState(255,50);
        status_led.setBlinkingPeriod(500);

        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        armed_led.setPWMBlikingState(255,50);
        armed_led.setBlinkingPeriod(500);
        break;
    case SystemState::DISARMED:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM);
        status_led.setPWMState(50);
        
        static unsigned long blink_timer;
        static char blink_counts;
        static bool blink_pause;
        if (millis() - blink_timer >= 500 && blink_pause == false)
        {
            if (blink_counts >= current_flight_record)
            {
                status_led.pulse(100);
                blink_counts = 0;
                blink_pause = true;
            }
            else {
                status_led.pulse(100);
                blink_counts++;
            }
            
            blink_timer = millis();
        }
        else if (millis() - blink_timer >= 3000 && blink_pause == true)
        {
            blink_pause = false;
            blink_timer = millis();
        }

        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
        armed_led.setDigitalState(LOW);
        break;
    case SystemState::INSTALL:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        status_led.setPWMBlikingState(255);
        status_led.setBlinkingPeriod(1000);
        
        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        armed_led.setPWMBlikingState(100);
        armed_led.setBlinkingPeriod(1000);

        break;
    case SystemState::ARMED:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        status_led.setPWMBlikingState(255);
        status_led.setBlinkingPeriod(1000);

        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::Strobe);
        armed_led.setBlinkingPeriod(250);
        break;
    case SystemState::FLIGHT:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        status_led.setPWMBlikingState(255);
        status_led.setBlinkingPeriod(250);

        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
        armed_led.setDigitalState(HIGH);
        break;
    case SystemState::LANDED:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        status_led.setPWMBlikingState(255);
        status_led.setBlinkingPeriod(250);

        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::Strobe);
        armed_led.setBlinkingPeriod(100,1000);
        break;
    default:
        break;
    }

    status_led.update();
    armed_led.update();
}

void buzzerUpdate()
{
    static unsigned long buzzer_timer;
    static bool buzzer_state;
    static unsigned char buzzer_mode;

    switch (current_state)
    {
    case SystemState::PC_DATA_TRANSFER:
        noTone(BUZZER);
        break;
    case SystemState::DISARMED:
        noTone(BUZZER);
        break;
    case SystemState::INSTALL:
        if (millis() - buzzer_timer >= 5000)
        {
            tone(BUZZER,1000,100);
            buzzer_timer = millis();
        }
        break;
    case SystemState::ARMED:
        if (millis() - buzzer_timer >= 1000)
        {
            tone(BUZZER,1500,250);
            buzzer_timer = millis();
        }
        break;
    case SystemState::FLIGHT:
        if (millis() - buzzer_timer >= 2000)
        {
            tone(BUZZER,1500,1000);
            buzzer_timer = millis();
        }
        break;
    case SystemState::LANDED:
        if (millis() - buzzer_timer >= 2000 && buzzer_mode == 0)
        {
            tone(BUZZER,1500,1000);
            buzzer_mode = 1;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 1)
        {
            buzzer_mode = 2;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 2)
        {
            tone(BUZZER,500,1000);
            buzzer_mode = 3;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 3)
        {
            buzzer_mode = 4;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 4)
        {
            tone(BUZZER,500,1000);
            buzzer_mode = 0;
            buzzer_timer = millis();
        }
        break;
    default:
        break;
    }
}

void inputUpdate()
{
    button.update();
    cut_wire.update();
}

bool vbusActive()
{
    // from http://msalino-hugg.blogspot.com/2014/02/arduino-micro-leonardo-detecting-if-you.html
    if (USBSTA & (1 << VBUS))
    { //checks state of VBUS
        return true;
    }
    return false;
}

// State Programs

void disarmedState()
{
    if (vbusActive())
    {
        current_state = SystemState::PC_DATA_TRANSFER;
    }
    else if (button.pressed())
    {
        current_state = SystemState::INSTALL;
    }
}

void installState()
{
    if (!cut_wire.read()) //since cut disconnects GND
    {
        current_state = SystemState::ARMED;
        return;
    }

    static unsigned long start_pressed_time; // Cancel Install Process
    static bool pressed;
    if (button.pressed() && pressed == false)
    {
        start_pressed_time = millis();
        pressed = true;
    }
    else if (button.released() && pressed == true && (millis() - start_pressed_time >= 5000))
    {
        current_state = SystemState::DISARMED;
    }
    else if (button.released() && pressed == true && (millis() - start_pressed_time < 5000))
    {
        pressed = false;
    } 

}

CircularBuffer<FlightInfoFrame, 20> flight_data_loop_buffer;

void armedState()
{
    // reading data but not storing in flash (storing in ram buffer until launch)
    FlightInfoFrame loop_data;
    readSensorData(&loop_data);
    flight_data_loop_buffer.push(loop_data);
    
    




    // Abort Armed State Programs
    static unsigned long start_pressed_time; // Cancel Armed (Will need to reconnect wire)
    static bool pressed;
    if (button.pressed() && pressed == false && cut_wire.read())
    {
        start_pressed_time = millis();
        pressed = true;
    }
    else if (button.released() && pressed == true && (millis() - start_pressed_time >= 5000) && cut_wire.read())
    {
        current_state = SystemState::DISARMED;
    }
    else if (button.released() && pressed == true && (millis() - start_pressed_time < 5000) && cut_wire.read())
    {
        pressed = false;
    } 

}

void flightState()
{
    FlightInfoFrame loop_data;
}

void landedState()
{
}

void pcTransferState()
{
}

void setup()
{
    outputSetup();
    inputSetup();

    status_led.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
    armed_led.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
    status_led.setDigitalState(HIGH);
    armed_led.setDigitalState(HIGH);
    status_led.update();
    armed_led.update();
    // Setup and Self Test
    USBCON |= (1 << OTGPADE); // Enable detection of VBUS to determine power source
    delay(1000);
    sensorSetup();
    if (vbusActive())
    {
        current_state = SystemState::PC_DATA_TRANSFER;
    }
    else
    {
        current_state = SystemState::DISARMED;
    }
    // Setup and Self Test Completed
}

void loop()
{
    inputUpdate();
    updateBarometerData();
    // State Machine
    switch (current_state)
    {
    case SystemState::PC_DATA_TRANSFER:
        pcTransferState();
        break;
    case SystemState::DISARMED:
        disarmedState();
        break;
    case SystemState::INSTALL:
        installState();
        break;
    case SystemState::ARMED:
        armedState();
        break;
    case SystemState::FLIGHT:
        flightState();
        break;
    case SystemState::LANDED:
        landedState();
        break;
    default:
        break;
    }
    ledUpdate();
    buzzerUpdate();
}