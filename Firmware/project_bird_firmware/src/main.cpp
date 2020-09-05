#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <board_definition.h>
#include <EEPROM.h>

#include "RunningAverage.h"
#include <MS5xxx.h>
#include "SparkFun_LIS331.h"
#include <Bounce2.h>
#include <LED_Pin_Control_Arduino.h>
#include <CircularBuffer.h>
#include <SPIMemory.h>
#include <FlightRecorder.h>

#define CIRCULAR_BUFFER_INT_SAFE

MS5xxx barometer(&Wire);
LIS331 accelerometer; //Adafruit_LIS331HH accelerometer = Adafruit_LIS331HH();

MemoryMap4M memory_map;
FlightRecorder recorder(MB(4), FLASH_CHIP_SS, memory_map);

RunningAverage barometer_temperature_avg(4);
RunningAverage barometer_pressure_avg(4); // must use average due to possible ejection charges
float barometer_temperature;
float barometer_pressure;

// Current Flight Temp Storage
float armed_pressure;
float armed_temperature;
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
    PC_DATA_TRANSFER,
    ERROR
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

struct SystemConfig
{
    //Data Options (Fixed)
    byte recording_rate_divider = 2;
    byte written = 101;

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
    float trigger_altitude = 2;
    float trigger_acceleration = 9.81 * 3;

    float landed_altitude_change = 2;
    int accelerometer_max_range = 24;

} current_config;

// Sensing Functions

byte updateBarometerData()
{
    barometer.Readout();
    barometer_pressure_avg.addValue(barometer.GetPres());
    barometer_temperature_avg.addValue(barometer.GetTemp() * 0.01); // Lib provide temp in 0.01 multiples
    barometer_pressure = barometer_pressure_avg.getAverage();
    barometer_temperature = barometer_temperature_avg.getAverage();

    return 0x00;
}

byte readSensorData(FlightInfoFrame *dataframe)
{
    if (accelerometer.newXData() || accelerometer.newYData() || accelerometer.newZData())
    {
        int16_t x, y, z;
        accelerometer.readAxes(x, y, z);
        dataframe->timestamp = millis();
        dataframe->ground_pressure = armed_pressure;
        dataframe->ground_temperature = armed_temperature;
        dataframe->absolute_pressure = barometer_pressure;
        dataframe->temperature = barometer_temperature;
        dataframe->acceleration_data[0] = accelerometer.convertToG(current_config.accelerometer_max_range, x);
        dataframe->acceleration_data[1] = accelerometer.convertToG(current_config.accelerometer_max_range, y);
        dataframe->acceleration_data[2] = accelerometer.convertToG(current_config.accelerometer_max_range, z);
        return 0x00;
    }
    return 0x01;
}

byte sensorSetup()
{
    Wire.begin();
    barometer.setI2Caddr(BAROMETER_ADDRESS);
    if (barometer.connect() > 0)
    {
        return 0x01;
    }
    barometer.ReadProm();

    accelerometer.setI2CAddr(ACCELEROMETER_ADDRESS);
    accelerometer.begin(LIS331::USE_I2C);

    accelerometer.intSrcConfig(LIS331::DRDY, 1);
    accelerometer.setIntDuration(5, 1);
    accelerometer.setODR(LIS331::DR_50HZ);
    accelerometer.setFullScale(LIS331::HIGH_RANGE);

    accelerometer.enableHPF(true);
    accelerometer.setHighPassCoeff(LIS331::HPC_8);

    status_led.setDigitalState(LOW);
    armed_led.setDigitalState(HIGH);
    status_led.update();
    armed_led.update();

    FlightInfoFrame test_frame;
    //if (readSensorData(&test_frame) > 0)
    //    return 0x03;
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
    //pinMode(ACCELEROMETER_INT1, INPUT);
    //pinMode(ACCELEROMETER_INT2, INPUT);

    return 0x00;
}

// IO Updaters

void ledUpdate()
{
    switch (current_state)
    {
    case SystemState::ERROR:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::Strobe);
        status_led.setBlinkingPeriod(100);
        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::Strobe);
        armed_led.setBlinkingPeriod(100);
        break;
    case SystemState::PC_DATA_TRANSFER:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        status_led.setPWMBlikingState(100, 20);
        status_led.setBlinkingPeriod(500);

        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        armed_led.setPWMBlikingState(100, 20);
        armed_led.setBlinkingPeriod(500);
        break;
    case SystemState::DISARMED:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM);
        status_led.setPWMBlikingState(255, 50);
        status_led.setPWMState(50);

        static unsigned long blink_timer;
        static char blink_counts;
        static bool blink_pause;
        if (millis() - blink_timer >= 500 && blink_pause == false)
        {
            // if (blink_counts > recorder.getMetadata().current_flight_record)
            // {
            //     status_led.pulse(100);
            //     blink_counts = 0;
            //     blink_pause = true;
            // }
            // else
            // {
            //     status_led.pulse(100);
            //     blink_counts++;
            // }
            if (blink_counts > recorder.getMetadata().current_flight_record)
            {
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
        status_led.setPWMBlikingState(254);
        status_led.setBlinkingPeriod(1000);

        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        armed_led.setPWMBlikingState(100);
        armed_led.setBlinkingPeriod(1000);

        break;
    case SystemState::ARMED:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        status_led.setPWMBlikingState(254, 0);
        status_led.setBlinkingPeriod(1000);

        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::Strobe);
        armed_led.setBlinkingPeriod(250);
        break;
    case SystemState::FLIGHT:
        if (flight_state == FlightState::LAUNCHED)
        {
            status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
            status_led.setPWMBlikingState(254, 0);
            status_led.setBlinkingPeriod(250);
        }
        else
        {
            status_led.setMode(LED_Pin_Control_Arduino::PinModes::Strobe);
            status_led.setBlinkingPeriod(250);
        }

        armed_led.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
        armed_led.setDigitalState(HIGH);
        break;
    case SystemState::LANDED:
        status_led.setMode(LED_Pin_Control_Arduino::PinModes::PWM_Fading);
        status_led.setPWMBlikingState(254, 0);
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
            tone(BUZZER, 500);
            buzzer_mode = 1;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 1)
        {
            noTone(BUZZER);
            buzzer_mode = 2;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 2)
        {
            tone(BUZZER, 1000);
            buzzer_mode = 3;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 3)
        {
            noTone(BUZZER);
            buzzer_mode = 4;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 4)
        {
            tone(BUZZER, 1500);
            buzzer_mode = 5;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 5)
        {
            noTone(BUZZER);
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

// EEPROM Configuration Functions

void storeConfigInStore(SystemConfig *config)
{
    EEPROM.begin();
    EEPROM.put(sizeof(RecorderMetadata) + 1, config);
}

void readConfigFromStore(SystemConfig *config)
{
    EEPROM.begin();
    EEPROM.get(sizeof(RecorderMetadata) + 1, config);
    if (config->written != 221)
    {
        SystemConfig new_conf;
        storeConfigInStore(&new_conf);
        EEPROM.get(sizeof(RecorderMetadata) + 1, config);
    }
}

// Serial PC Data Transfer Functions

void printMainMenu()
{
    Serial.println(F("UTS Rocketry - Project Bird, Micro Flight Controller V1.0"));
    Serial.println(F("Please select an option (Enter Number Followed By Enter using CL & LF):"));
    Serial.println();
    // Menu Options
    Serial.println(F("1. Print Data from Last Flight (csv Format)"));
    Serial.println(F("2. Print Data from Any Flight (csv Format)"));
    Serial.println(F("3. Read Sensor Values"));
    Serial.println(F("4. Read Configuration Settings"));
    Serial.println(F("5. Set Deployment Charge Options"));
    Serial.println(F("6. Exit From PC Transfer Mode"));
    Serial.println(F("10. DANGER - Trigger and Test Deployment Charges"));
}

// State Programs

void disarmedState()
{
    if (Serial)
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
    if (cut_wire.read())
    {
        current_state = SystemState::ARMED;
        armed_pressure = barometer_pressure;
        armed_temperature = barometer_temperature;
        recorder.beginFlight(true);
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
    loop_data.ground_temperature = armed_temperature;
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
            launch_detected = (resultant_accel >= current_config.trigger_acceleration) || (recorder.calculateAltitude(loop_data.absolute_pressure, loop_data.ground_pressure, loop_data.ground_temperature) >= current_config.trigger_altitude);
            break;
        case SystemConfig::TriggerOption::ALTITUDE_AND_ACCELERATION:
            launch_detected = (resultant_accel >= current_config.trigger_acceleration) && (recorder.calculateAltitude(loop_data.absolute_pressure, loop_data.ground_pressure, loop_data.ground_temperature) >= current_config.trigger_altitude);
            break;
        case SystemConfig::TriggerOption::ALTITUDE_ONLY:
            launch_detected = (recorder.calculateAltitude(loop_data.absolute_pressure, loop_data.ground_pressure, loop_data.ground_temperature) >= current_config.trigger_altitude);
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
            for (int i = 0; i < flight_data_loop_buffer.size(); i++)
            {
                if (i >= flight_data_loop_buffer.size() - 1)
                {
                    FlightInfoFrame last_data = flight_data_loop_buffer.last();
                    last_data.flight_event = static_cast<byte>(EventCode::LAUNCHED_OFF_RAIL);
                    last_data.deployment_event = static_cast<byte>(DepolymentEvent::DEPLOY_NO_EVENT);
                    recorder.saveDataframe(last_data);
                }
                else
                {
                    recorder.saveDataframe(flight_data_loop_buffer[i]);
                }
            }
            current_state = SystemState::FLIGHT;
            flight_state = FlightState::LAUNCHED;
        }
    }

    // Abort Armed State Programs
    static unsigned long start_pressed_time; // Cancel Armed (Will need to reconnect wire)
    static bool pressed;
    if (button.pressed() && pressed == false && !cut_wire.read())
    {
        start_pressed_time = millis();
        pressed = true;
    }
    else if (button.released() && pressed == true && (millis() - start_pressed_time >= 5000) && !cut_wire.read())
    {
        current_state = SystemState::DISARMED;
    }
    else if (button.released() && pressed == true && (millis() - start_pressed_time < 5000) && !cut_wire.read())
    {
        pressed = false;
    }
}

void flightState()
{
    static CircularBuffer<float, 20> pressure_long_term;
    static byte pressure_long_term_counter;
    bool bypass_divider;
    FlightInfoFrame loop_data;

    loop_data.ground_pressure = armed_pressure;
    loop_data.ground_temperature = armed_temperature;
    loop_data.flight_event = static_cast<byte>(EventCode::NO_EVENT);
    loop_data.deployment_event = static_cast<byte>(DepolymentEvent::DEPLOY_NO_EVENT);

    static float previous_flight_pressure;
    static float min_flight_pressure; // max altitude
    static byte pressure_increasing_count;

    // New Data
    if (readSensorData(&loop_data) == 0)
    {
        // Altitude Checker and Landing Detect
        if (pressure_long_term_counter > 10) // converts 50hz to 10hz
        {
            pressure_long_term.push(loop_data.absolute_pressure);
            pressure_long_term_counter = 0;
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
            loop_data.flight_event = static_cast<byte>(EventCode::APOGEE);
            flight_state = FlightState::APOGEE_REACHED;
            bypass_divider = true;
            apogee_reached_time = millis();

            // delay main if rocket is under deployment altitude if drouge is active
            if (false && (recorder.calculateAltitude(loop_data.absolute_pressure, loop_data.ground_pressure, loop_data.ground_temperature) <= current_config.main_parachute_deploy_altitude))
            {
                // deploy Drouge immiediately and delay main for 5 seconds
            }
            else
            {
                // delay + deploy Drouge
            }
        }

        // Landed Detection
        if ((flight_state == FlightState::APOGEE_REACHED || flight_state == FlightState::DROUGE_DEPLOYED || flight_state == FlightState::MAIN_DEPLOYED) && pressure_long_term.size() >= 19)
        {
            float long_term_alt_avg = abs(recorder.calculateAltitude(pressure_long_term.first(), loop_data.ground_pressure, loop_data.ground_temperature) - recorder.calculateAltitude(pressure_long_term.last(), loop_data.ground_pressure, loop_data.ground_temperature));

            if (long_term_alt_avg <= current_config.landed_altitude_change) // if the absolute altitude change over 2 seconds is less than 3m
            {
                loop_data.flight_event = static_cast<byte>(EventCode::LANDED);
                flight_state = FlightState::LANDED;
                bypass_divider = true;
            }
        }

        //Write Data to Flash
        static byte data_divider_counter;
        if (data_divider_counter + 1 >= current_config.recording_rate_divider || bypass_divider)
        {
            if (data_divider_counter + 1 >= current_config.recording_rate_divider)
            {
                data_divider_counter = 0;
            }
            recorder.saveDataframe(loop_data);
        }
        else
        {
            data_divider_counter++;
        }

        // Landing Detected
        if (flight_state == FlightState::LANDED)
        {
            current_state = SystemState::LANDED;
            recorder.endFlight();
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

void printFlightData(int flight_no)
{
    RecorderMetadata recorder_metadata = recorder.getMetadata();
    uint8_t flight_number = flight_no;

    if (flight_number == 99)
    {
        Serial.println("No Flights Recorded.");
        break;
    }

    Serial.println();
    for (int i = 0; i < 100; i++)
    {
        Serial.print('-');
    }
    Serial.println();

    Serial.println();
    Serial.print(F("Flight "));
    Serial.println(flight_number);

    Serial.print(F("Maximum Altitude = "));
    Serial.print(recorder_metadata.max_altitudes[flight_number]);
    Serial.println("m");

    Serial.print(F("Flight Time = "));
    Serial.print(float(recorder_metadata.flight_times[flight_number]) / 1000.0);
    Serial.println(F("sec"));
    Serial.println();

    Serial.println(F("Time,Alt(m),Flight Event,Dep Event,Ground P(Pa),Ground T(C),Pressure (Pa),Temperature (C),Ax (m/s^2),Ay (m/s^2),Az (m/s^2)"));

    for (unsigned long record_iter = 0; record_iter < recorder_metadata.flight_rows_used[flight_number]; record_iter++)
    {
        FlightInfoFrame record_frame;
        recorder.loadDataframe(flight_number, record_iter, &record_frame);
        Serial.print(record_frame.timestamp);
        Serial.print(",");
        Serial.print(recorder.calculateAltitude(record_frame.absolute_pressure, record_frame.ground_pressure, record_frame.ground_temperature));
        Serial.print(",");
        switch (static_cast<EventCode>(record_frame.flight_event))
        {
        case EventCode::NO_EVENT:

            break;
        case EventCode::LAUNCHED_OFF_RAIL:
            Serial.print(F("Launched"));
            break;
        case EventCode::APOGEE:
            Serial.print(F("Apogee"));
            break;
        case EventCode::LANDED:
            Serial.print(F("Landed"));
            break;
        default:
            break;
        }
        Serial.print(",");
        switch (static_cast<DepolymentEvent>(record_frame.deployment_event))
        {
        case DepolymentEvent::DEPLOY_NO_EVENT:

            break;
        case DepolymentEvent::DEPLOY_MAIN_DEPLOYED:
            Serial.print(F("Main"));
            break;
        case DepolymentEvent::DEPLOY_MAIN2_DEPLOYED:
            Serial.print(F("Main 2"));
            break;
        case DepolymentEvent::DEPLOY_DROUGUE_DEPLOYED:
            Serial.print(F("Drougue"));
            break;
        case DepolymentEvent::DEPLOY_DROUGUE2_DEPLOYED:
            Serial.print(F("Drougue 2"));
            break;
        default:
            break;
        }
        Serial.print(",");
        Serial.print(record_frame.ground_pressure);
        Serial.print(",");
        Serial.print(record_frame.ground_temperature);
        Serial.print(",");
        Serial.print(record_frame.absolute_pressure);
        Serial.print(",");
        Serial.print(record_frame.temperature);
        Serial.print(",");
        Serial.print(record_frame.acceleration_data[0]);
        Serial.print(",");
        Serial.print(record_frame.acceleration_data[1]);
        Serial.print(",");
        Serial.print(record_frame.acceleration_data[2]);
        Serial.println();
    }

    Serial.println();
    Serial.println(F("End of Flight Report"));
    Serial.println();
    for (int i = 0; i < 100; i++)
    {
        Serial.print('-');
    }
    Serial.println();
}

void pcTransferState()
{
    // Serial Connect detect
    static bool old_serial_connect;
    static bool reprint_main_menu;

    bool serial_connected;

    if (!Serial)
    {
        serial_connected = false;
    }
    else if (Serial)
    {
        serial_connected = true;
    }

    // Connect Print Menu
    if (old_serial_connect != serial_connected || reprint_main_menu)
    {
        printMainMenu();
        reprint_main_menu = false;
    }

    // Input Retrive and Command Entered
    if (Serial.available() > 0)
    {
        int option = Serial.parseInt();
        while (Serial.available() > 0)
        {
            Serial.read();
        }

        if (option == 255) // Device Control 1 & 2
        {
            // Binary Control
        }
        else
        {
            // Interactive Terminal Control
            int selected_option = option;

            switch (selected_option)
            {
            case 1: // 1. Print Data from Last Flight (csv Format)

                RecorderMetadata recorder_metadata = recorder.getMetadata();
                uint8_t flight_number = recorder_metadata.last_successful_flight;
                printFlightData(flight_number);
                reprint_main_menu = true;

                break;
            case 2: // 2. Print Data from Any Flight (csv Format)
                /* code */
                break;
            case 3: // 3. Read Sensor Values
                /* code */
                break;
            case 4: // 4. Read Configuration Settings
                /* code */
                break;
            case 5: // 5. Set Deployment Charge Options
                /* code */
                break;
            case 6: // 6. Exit From PC Transfer Mode
                /* code */
                break;
            case 10: // 10. DANGER - Trigger and Test Deployment Charges
                /* code */
                break;

            default:
                Serial.println(F("Invalid Input, Please Enter A Correct Value."));
                break;
            }
        }
    }

    // USB Disconnect
    if (!vbusActive())
    {
        current_state = SystemState::DISARMED;
    }
    old_serial_connect = serial_connected;
}

// Main Program
void setup()
{
    delay(2000); // Bootloader Delay
    outputSetup();
    inputSetup();

    status_led.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
    armed_led.setMode(LED_Pin_Control_Arduino::PinModes::Solid);
    status_led.setDigitalState(HIGH);
    armed_led.setDigitalState(HIGH);
    status_led.update();
    armed_led.update();

    readConfigFromStore(&current_config);

    USBCON |= (1 << OTGPADE); // Enable detection of VBUS to determine power source
    // Setup and Self Test

    bool recorder_setup_success = recorder.begin();

    byte sensor_setup_val = sensorSetup();
    bool sensor_setup_success = (sensor_setup_val == 0x00);

    status_led.setDigitalState(LOW);
    armed_led.setDigitalState(LOW);
    status_led.update();
    armed_led.update();
    if (!(recorder_setup_success && sensor_setup_success))
    {
        // Setup Failed
        current_state = SystemState::ERROR;
    }
    else
    {
        // Setup and Self Test Completed
        if (Serial)
        {
            current_state = SystemState::PC_DATA_TRANSFER;
            Serial.begin(115200);
        }
        else
        {
            current_state = SystemState::DISARMED;
        }
    }
}

void loop()
{
    inputUpdate();
    if (current_state != SystemState::ERROR)
    {
        updateBarometerData();
    }
    // State Machine
    switch (current_state)
    {
    case SystemState::ERROR:

        break;
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