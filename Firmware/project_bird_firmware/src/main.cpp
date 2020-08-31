#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <board_definition.h>
#include <EEPROM.h>

#include "RunningAverage.h"
#include <MS5xxx.h>
#include <Adafruit_LIS331DLH.h>
#include <Adafruit_Sensor.h>
#include <Bounce2.h>
#include <LED_Pin_Control_Arduino.h>
#include <CircularBuffer.h>
#include <SPIMemory.h>

#define CIRCULAR_BUFFER_INT_SAFE

MS5xxx barometer(&Wire);
Adafruit_LIS331DLH accelerometer = Adafruit_LIS331DLH();
SPIFlash flash_chip(FLASH_CHIP_SS);

RunningAverage barometer_temperature_avg(4);
RunningAverage barometer_pressure_avg(4); // must use average due to possible ejection charges
float barometer_temperature;
float barometer_pressure;

// Current Flight Temp Storage
float armed_pressure;
float armed_temperature;
unsigned long data_write_idx;
unsigned long current_flight_number;
unsigned long flight_start_time;
unsigned long apogee_reached_time;
unsigned long drogue_deployed_time;
unsigned long drogue2_deployed_time;
unsigned long main_deployed_time;
unsigned long main2_deployed_time;

LED_Pin_Control_Arduino status_led(LED1_STATUS);
LED_Pin_Control_Arduino armed_led(LED2_ARMED);

LED_Pin_Control_Arduino detonate_ch1(CHARGE_DETONATE_CH1);
LED_Pin_Control_Arduino detonate_ch2(CHARGE_DETONATE_CH2);
LED_Pin_Control_Arduino detonate_ch3(CHARGE_DETONATE_CH3);
LED_Pin_Control_Arduino detonate_ch4(CHARGE_DETONATE_CH4);


Button button = Button();
Bounce cut_wire = Bounce();

// Data Structures

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

enum class EventCode : byte
{
    NO_EVENT = 0b00000000,
    LAUNCHED_OFF_RAIL = 0b00000001,
    APOGEE = 0b00000010,
    LANDED = 0b00000100
};

enum class DepolymentEvent : byte
{
    DEPLOY_NO_EVENT = 0b00000000,
    DEPLOY_MAIN_DEPLOYED = 0b00010000,
    DEPLOY_DROUGUE_DEPLOYED = 0b00100000,
    DEPLOY_MAIN2_DEPLOYED = 0b01000000,
    DEPLOY_DROUGUE2_DEPLOYED = 0b10000000
};

struct FlightInfoFrame
{
    byte event_store; // High Nibble = DeploymentEvent | Low Nibble = EventCode
    byte reserved[7];
    short ground_temperature_x100;
    short temperature_x100;
    float ground_pressure;
    float absolute_pressure;
    float acceleration_data[3]; // x, y, z
    uint32_t timestamp;
};

// 32MBit Flash Chip has 4194304 Bytes // for this, assume data is 36 bytes max, therefore @ 50hz, 1800 bytes/second and 108000 bytes/minute.
// therefore ~12.5mins per flight max with 3 flight profiles
// Proflie    Start Add  End Add , Start Idx    End Idx
// flight0 -> 0       -> 1349964 , 0         -> 37499
// flight1 -> 1350000 -> 2699964 , 37500     -> 74999
// flight2 -> 2700000 -> 4049964 , 75000     -> 112499

struct FlashMemoryMap
{
    unsigned long flight_record_start_file_address[3] = {0, 37500, 75000}; // in multiples of 36 bytes, address spaces {0,1350000,2700000} ending @ 4049999
    unsigned long flight_record_end_file_address[3] = {37499, 74999, 112499};
};

struct SystemConfig
{
    //Data Options (Fixed)
    unsigned char current_flight_record;
    byte recording_rate_divider = 2;
    bool flight_record_contains_data[3];
    FlashMemoryMap memory_map;

    // Flight Info
    float max_altitudes[3];
    unsigned long flight_times[3];
    bool flight_in_progress[3];

    // User Deployment Charge Settings (delays in ms)
    bool main_parachute_absolute_altitude_deploy = false;
    float main_parachute_deploy_altitude = 100;
    float main_parachute_deploy_altitude_difference = 20;
    unsigned long drouge_delay = 0;
    unsigned long main_delay = 0;
    unsigned long redundant_delay = 1000;

    // Armed to Flight Trigger
    enum TriggerOption
    {
        ALTITUDE_OR_ACCELERATION,
        ALTITUDE_AND_ACCELERATION,
        ACCELERATION_ONLY,
        ALTITUDE_ONLY,
    };
    TriggerOption trigger_option = ALTITUDE_OR_ACCELERATION;
    float trigger_altitude = 2.0;
    float trigger_acceleration = 9.81 * 3;

} current_config;

// Sensing Functions

byte updateBarometerData()
{
    barometer.ReadProm();
    barometer.Readout();

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

    if (uint32_t(event.timestamp) != last_event_time)
    {
        dataframe->timestamp = event.timestamp;
        dataframe->ground_pressure = armed_pressure;
        dataframe->ground_temperature_x100 = static_cast<int>(round(armed_temperature * 100));
        dataframe->absolute_pressure = barometer_pressure;
        dataframe->temperature_x100 = static_cast<int>(round(barometer_temperature * 100));
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

// IO Setup Functions

byte outputSetup()
{
    // digitalWrite(LOW, CHARGE_DETONATE_CH1);
    // pinMode(CHARGE_DETONATE_CH1, OUTPUT);
    // digitalWrite(LOW, CHARGE_DETONATE_CH1);

    // digitalWrite(LOW, CHARGE_DETONATE_CH2);
    // pinMode(CHARGE_DETONATE_CH2, OUTPUT);
    // digitalWrite(LOW, CHARGE_DETONATE_CH2);

    // digitalWrite(LOW, CHARGE_DETONATE_CH3);
    // pinMode(CHARGE_DETONATE_CH3, OUTPUT);
    // digitalWrite(LOW, CHARGE_DETONATE_CH3);

    // digitalWrite(LOW, CHARGE_DETONATE_CH4);
    // pinMode(CHARGE_DETONATE_CH4, OUTPUT);
    // digitalWrite(LOW, CHARGE_DETONATE_CH4);

    detonate_ch1.init();
    detonate_ch1.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
    detonate_ch1.setDigitalState(false);

    detonate_ch2.init();
    detonate_ch2.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
    detonate_ch2.setDigitalState(false);

    detonate_ch3.init();
    detonate_ch3.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
    detonate_ch3.setDigitalState(false);

    detonate_ch4.init();
    detonate_ch4.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
    detonate_ch4.setDigitalState(false);

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

// IO Updaters

void ledUpdate()
{
    switch (current_state)
    {
    case SystemState::PC_DATA_TRANSFER:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        status_led.setPWMBlikingState(255, 50);
        status_led.setBlinkingPeriod(500);

        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        armed_led.setPWMBlikingState(255, 50);
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
            if (blink_counts >= current_config.current_flight_record)
            {
                status_led.pulse(100);
                blink_counts = 0;
                blink_pause = true;
            }
            else
            {
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
        armed_led.setBlinkingPeriod(100, 1000);
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
            tone(BUZZER, 1000, 100);
            buzzer_timer = millis();
        }
        break;
    case SystemState::ARMED:
        if (millis() - buzzer_timer >= 1000)
        {
            tone(BUZZER, 1500, 250);
            buzzer_timer = millis();
        }
        break;
    case SystemState::FLIGHT:
        if (millis() - buzzer_timer >= 2000)
        {
            tone(BUZZER, 1500, 1000);
            buzzer_timer = millis();
        }
        break;
    case SystemState::LANDED:
        if (millis() - buzzer_timer >= 2000 && buzzer_mode == 0)
        {
            tone(BUZZER, 1500, 1000);
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
            tone(BUZZER, 500, 1000);
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
            tone(BUZZER, 500, 1000);
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

// Serial PC Data Transfer Functions

// Flash Functions

byte setupFlash()
{
    if (!flash_chip.begin(MB(FLASH_SIZE_MB)))
        return 0x01;
    return 0x00;
}

byte writeDataToFlash(unsigned char flight_number, unsigned long idx, FlightInfoFrame dataframeToWrite)
{

    return 0x00;
}

byte readDataFromFlash(unsigned char flight_number, unsigned long idx, FlightInfoFrame *dataframeStorage)
{
    return 0x00;
}

byte eraseFlightRecord(unsigned char flight_number)
{

    return 0x00;
}

// EEPROM Configuration Functions

void readConfigFromStore(SystemConfig *config)
{
    EEPROM.begin();
    EEPROM.get(0, config);
}

void storeConfigInStore(SystemConfig *config)
{
    EEPROM.begin();
    EEPROM.put(0, config);
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
        armed_pressure = barometer_pressure;
        armed_temperature = barometer_temperature;
        return;
    }

    // Abort Install
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

void armedState()
{
    // reading data but not storing in flash (storing in ram buffer until launch)
    static CircularBuffer<FlightInfoFrame, 20> flight_data_loop_buffer;
    FlightInfoFrame loop_data;
    loop_data.ground_pressure = armed_pressure;
    loop_data.ground_temperature_x100 = armed_temperature * 100;
    if (readSensorData(&loop_data) == 0)
    {
        static byte data_divider_counter;
        if (data_divider_counter + 1 >= current_config.recording_rate_divider)
        {
            data_divider_counter = 0;
            flight_data_loop_buffer.push(loop_data);
        }
        else
        {
            data_divider_counter++;
        }
        
        // Launch Detection
        bool launch_detected = false;
        float resultant_accel = sqrt(pow(loop_data.acceleration_data[0], 2) + pow(loop_data.acceleration_data[0], 2) + pow(loop_data.acceleration_data[0], 2));
        switch (current_config.trigger_option)
        {
        case SystemConfig::TriggerOption::ALTITUDE_OR_ACCELERATION:
            launch_detected = (resultant_accel >= current_config.trigger_acceleration) || (calculateAltitude(loop_data.absolute_pressure, loop_data.ground_pressure, float(loop_data.ground_temperature_x100 / 100)) >= current_config.trigger_altitude);
            break;
        case SystemConfig::TriggerOption::ALTITUDE_AND_ACCELERATION:
            launch_detected = (resultant_accel >= current_config.trigger_acceleration) && (calculateAltitude(loop_data.absolute_pressure, loop_data.ground_pressure, float(loop_data.ground_temperature_x100 / 100)) >= current_config.trigger_altitude);
            break;
        case SystemConfig::TriggerOption::ALTITUDE_ONLY:
            launch_detected = (calculateAltitude(loop_data.absolute_pressure, loop_data.ground_pressure, float(loop_data.ground_temperature_x100 / 100)) >= current_config.trigger_altitude);
            break;
        case SystemConfig::TriggerOption::ACCELERATION_ONLY:
            launch_detected = (resultant_accel >= current_config.trigger_acceleration);
            break;
        default:
            break;
        }

        // Launch Occured
        if (launch_detected)
        {
            current_flight_number = current_config.current_flight_record;
            current_config.current_flight_record++;
            data_write_idx = current_config.memory_map.flight_record_start_file_address[current_flight_number];
            // write buffered data to flash
            for (int i = 0; i < flight_data_loop_buffer.size(); i++)
            {
                if (i >= flight_data_loop_buffer.size() - 1)
                {
                    FlightInfoFrame last_data = flight_data_loop_buffer.last();
                    last_data.timestamp -= flight_start_time;
                    last_data.event_store = static_cast<byte>(EventCode::LAUNCHED_OFF_RAIL);
                    writeDataToFlash(current_flight_number, data_write_idx, last_data);
                    data_write_idx++;
                }
                else
                {
                    if (i == 0)
                    {
                        flight_start_time = flight_data_loop_buffer[i].timestamp;
                    }
                    flight_data_loop_buffer[i].timestamp -= flight_start_time;
                    writeDataToFlash(current_flight_number, data_write_idx, flight_data_loop_buffer[i]);
                    data_write_idx++;
                }
            }
            current_config.flight_in_progress[current_flight_number] = true;
            storeConfigInStore(&current_config);
            current_state = SystemState::FLIGHT;
            flight_state = FlightState::LAUNCHED;
        }
    }

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
    static CircularBuffer<float, 20> pressure_long_term;
    static byte pressure_long_term_counter;
    static FlightInfoFrame loop_data;

    loop_data.ground_pressure = armed_pressure;
    loop_data.ground_temperature_x100 = armed_temperature * 100;

    static float previous_flight_pressure;
    static float min_flight_pressure; // max altitude
    static byte pressure_increasing_count;

    // New Data
    if (readSensorData(&loop_data) == 0)
    {
        // Altitude Checker and Landing Detect
        if (pressure_long_term_counter > 10) // converts 50hz to 5hz
        {
            pressure_long_term.push(loop_data.absolute_pressure);
            pressure_long_term_counter = 0;

            // Landed Detection
            if (pressure_long_term.size() >= 20 && (flight_state == FlightState::APOGEE_REACHED || flight_state == FlightState::DROUGE_DEPLOYED || flight_state == FlightState::MAIN_DEPLOYED))
            {
                float long_term_alt_avg = abs(calculateAltitude(pressure_long_term.first(), loop_data.ground_pressure, float(loop_data.ground_temperature_x100 / 100)) - calculateAltitude(pressure_long_term.last(), loop_data.ground_pressure, float(loop_data.ground_temperature_x100 / 100)));
                if (long_term_alt_avg < 0.5) // if the absolute altitude change over 4 seconds is less than 0.5
                {
                    loop_data.event_store |= static_cast<byte>(EventCode::LANDED);

                    current_state = SystemState::LANDED;
                    flight_state = FlightState::LANDED;
                }
            }
        }
        else
        {
            pressure_long_term_counter++;
        }

        // Altitude Increasing
        if (loop_data.absolute_pressure < min_flight_pressure)
        {
            min_flight_pressure = loop_data.absolute_pressure;
            pressure_increasing_count = 0;
        }

        // Altitude Decreasing
        if (loop_data.absolute_pressure > previous_flight_pressure)
        {
            pressure_increasing_count++;
        }

        // Apogee Detection
        if (pressure_increasing_count > 25 && flight_state == FlightState::LAUNCHED) // was launched and been decending for 0.5 seconds
        {
            loop_data.event_store |= static_cast<byte>(EventCode::APOGEE);
            flight_state = FlightState::APOGEE_REACHED;
            apogee_reached_time = millis();

            // delay main if rocket is under deployment altitude if drouge is active
            if (false && (calculateAltitude(loop_data.absolute_pressure, loop_data.ground_pressure, float(loop_data.ground_temperature_x100 / 100)) <= current_config.main_parachute_deploy_altitude))
            {
                // deploy Drouge immiediately and delay main for 2 seconds
            }
            else
            {
                // delay + deploy Drouge
            }
        }

        //Write Data to Flash
        static byte data_divider_counter;
        if (data_divider_counter + 1 >= current_config.recording_rate_divider)
        {
            data_divider_counter = 0;
            if (data_write_idx <= current_config.memory_map.flight_record_end_file_address[current_flight_number])
            {
                writeDataToFlash(current_flight_number, data_write_idx, loop_data);
                data_write_idx++;
            }
        }
        else
        {
            data_divider_counter++;
        }
        
        // Landing Detected
        if (current_state == SystemState::LANDED && flight_state == FlightState::LANDED)
        {
            current_config.flight_in_progress[current_flight_number] = false;
            storeConfigInStore(&current_config);
        }

        previous_flight_pressure = loop_data.absolute_pressure;
    }
}

void landedState()
{
    // Recovered
    static unsigned long start_pressed_time;
    static bool pressed;
    if (button.pressed() && pressed == false)
    {
        start_pressed_time = millis();
        pressed = true;
    }
    else if (button.released() && pressed == true && (millis() - start_pressed_time >= 2000))
    {
        current_state = SystemState::DISARMED;
    }
    else if (button.released() && pressed == true && (millis() - start_pressed_time < 2000))
    {
        pressed = false;
    }
}

void pcTransferState()
{
    if (!vbusActive())
    {
        current_state = SystemState::DISARMED;
    }
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
    readConfigFromStore(&current_config);
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