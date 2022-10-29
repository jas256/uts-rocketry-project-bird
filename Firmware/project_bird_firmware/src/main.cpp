#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <board_definition.h>
#include <EEPROM.h>
#include <NewTone.h>
#include <Streaming.h>

#include <SimpleKalmanFilter.h>
#include <MS5xxx.h>
#include <SparkFun_LIS331.h>
#include <Bounce2.h>
#include <LED_Pin_Control_Arduino.h>
#include <CircularBuffer.h>
#include <SPIMemory.h>
#include <FlightRecorder.h>

#define CIRCULAR_BUFFER_INT_SAFE

#define CONTINUITY_VOLTAGE 5.0
#define DETONATE_PULSE_TIME 2000

MS5xxx barometer(&Wire);
LIS331 accelerometer; //Adafruit_LIS331HH accelerometer = Adafruit_LIS331HH();

MemoryMap4M memory_map;
FlightRecorder recorder(MB(4), FLASH_CHIP_SS, memory_map);

SimpleKalmanFilter pressureKalmanFilter(100, 100, 0.5);
float barometer_temperature;
float raw_pressure;
float barometer_pressure;
float current_altitude;

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

float continuity_voltages[4];
bool main_det = false;
bool same_det = false;
bool drogue_det = false;

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

#define WRITTEN_CHECK 160

struct SystemConfig
{
    //Data Options (Fixed)
    byte recording_rate_divider = 2;
    byte written = WRITTEN_CHECK;

    // User Deployment Charge Settings (delays in ms)
    bool main_parachute_absolute_altitude_deploy = false;
    float main_parachute_deploy_altitude = 100;
    unsigned long drouge_delay = 0;
    unsigned long main_delay = 0;
    unsigned long redundant_delay = 500;
    unsigned long dual_delay = 1000;

    // Armed to Flight Trigger
    enum TriggerOption
    {
        ALTITUDE_OR_ACCELERATION,
        ALTITUDE_AND_ACCELERATION,
        ACCELERATION_ONLY,
        ALTITUDE_ONLY,
    };
    TriggerOption trigger_option = ALTITUDE_ONLY;
    float trigger_altitude = 15.0;
    float trigger_acceleration = 9.81 * 3;

    float landed_altitude_change = 2;
    int accelerometer_max_range = 8;

} current_config;

// Sensing Functions

byte updateBarometerData()
{
    barometer.Readout();
    raw_pressure = barometer.GetPres();
    barometer_pressure = pressureKalmanFilter.updateEstimate(raw_pressure);
    barometer_temperature = barometer.GetTemp() * 0.01;
    current_altitude = recorder.calculateAltitude(barometer_pressure, armed_pressure, armed_temperature);

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
        dataframe->acceleration_data[0] = accelerometer.convertToG(current_config.accelerometer_max_range, x) * 9.81;
        dataframe->acceleration_data[1] = accelerometer.convertToG(current_config.accelerometer_max_range, y) * 9.81;
        dataframe->acceleration_data[2] = accelerometer.convertToG(current_config.accelerometer_max_range, z) * 9.81;
        return 0x00;
    }
    return 0x01;
}

void readDeploymentChargeVoltage()
{
    continuity_voltages[0] = float(analogRead(CHARGE_CONTINUITY_CH1)) * float(CHARGE_DIVIDER_RATIO_MULTIPLIER);
    continuity_voltages[1] = float(analogRead(CHARGE_CONTINUITY_CH2)) * float(CHARGE_DIVIDER_RATIO_MULTIPLIER);
    continuity_voltages[2] = float(analogRead(CHARGE_CONTINUITY_CH3)) * float(CHARGE_DIVIDER_RATIO_MULTIPLIER);
    continuity_voltages[3] = float(analogRead(CHARGE_CONTINUITY_CH4)) * float(CHARGE_DIVIDER_RATIO_MULTIPLIER);
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

    status_led.setDigitalState(LOW);
    armed_led.setDigitalState(HIGH);
    status_led.update();
    armed_led.update();
    delay(1000);
    accelerometer.setI2CAddr(ACCELEROMETER_ADDRESS);
    accelerometer.begin(LIS331::USE_I2C);

    accelerometer.intSrcConfig(LIS331::DRDY, 1);
    accelerometer.setIntDuration(5, 1);
    accelerometer.setODR(LIS331::DR_50HZ);
    accelerometer.setFullScale(LIS331::HIGH_RANGE);

    accelerometer.enableHPF(true);
    accelerometer.setHighPassCoeff(LIS331::HPC_8);

    // for (int ix = 0; ix < 4; ix++)
    // {
    //     barometer_pressure_avg.addValue(0);
    //     barometer_temperature_avg.addValue(0);
    // }
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

    detonate_ch1.update();
    detonate_ch2.update();
    detonate_ch3.update();
    detonate_ch4.update();
}

void buzzerUpdate()
{
    static unsigned long buzzer_timer;
    static unsigned char buzzer_mode;

    switch (current_state)
    {
    case SystemState::PC_DATA_TRANSFER:
        noNewTone(BUZZER);
        break;
    case SystemState::DISARMED:
        noNewTone(BUZZER);
        break;
    case SystemState::INSTALL:
        if (millis() - buzzer_timer >= 5000)
        {
            NewTone(BUZZER, 1000, 100);
            buzzer_timer = millis();
        }
        break;
    case SystemState::ARMED:
        if (millis() - buzzer_timer >= 1000)
        {
            NewTone(BUZZER, 1500, 250);
            buzzer_timer = millis();
        }
        break;
    case SystemState::FLIGHT:
        if (millis() - buzzer_timer >= 2000)
        {
            NewTone(BUZZER, 1500, 1000);
            buzzer_timer = millis();
        }
        break;
    case SystemState::LANDED:
        if (millis() - buzzer_timer >= 2000 && buzzer_mode == 0)
        {
            NewTone(BUZZER, 500);
            buzzer_mode = 1;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 1)
        {
            noNewTone(BUZZER);
            buzzer_mode = 2;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 2)
        {
            NewTone(BUZZER, 1000);
            buzzer_mode = 3;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 3)
        {
            noNewTone(BUZZER);
            buzzer_mode = 4;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 4)
        {
            NewTone(BUZZER, 1500);
            buzzer_mode = 5;
            buzzer_timer = millis();
        }
        else if (millis() - buzzer_timer >= 2000 && buzzer_mode == 5)
        {
            noNewTone(BUZZER);
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
    readDeploymentChargeVoltage();
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
    if (config->written != WRITTEN_CHECK)
    {
        SystemConfig new_conf;
        storeConfigInStore(&new_conf);
        EEPROM.get(sizeof(RecorderMetadata) + 1, config);
    }
}

// Serial PC Data Transfer Functions

void printMainMenu()
{
    Serial << F("UTS Rocketry - Project Bird V1.0") << endl;
    Serial << F("Please select an option: ") << endl;
    Serial << endl;
    // Menu Options
    Serial << F("1-4. Print from Flight x") << endl;
    Serial << F("5. Print from Last Flight") << endl;
    Serial << F("80. Start Sensor Print") << endl;
    Serial << F("81. Stop Sensor Print") << endl;
}

void printFlightData(int flight_no)
{
    RecorderMetadata recorder_metadata = recorder.getMetadata();
    uint8_t flight_number = flight_no;

    if (flight_number == 99)
    {
        Serial << F("No Flight Recorded.") << endl;
        return;
    }

    Serial.println();
    for (int i = 0; i < 100; i++)
    {
        Serial << '-';
    }
    Serial << endl
           << endl;

    Serial << F("Flight ") << (flight_number + 1) << endl;
    Serial << F("Max Alt = ") << recorder_metadata.max_altitudes[flight_number] << 'm' << endl;
    Serial << F("Flt Time = ") << (float(recorder_metadata.flight_times[flight_number]) / 1000.0) << 's' << endl;

    Serial << F("Time,Alt(m),Flt Event,Dep Event,G P(Pa),G T(C),P (Pa),T (C),Ax (m/s^2),Ay (m/s^2),Az (m/s^2)") << endl;

    for (unsigned long record_iter = 0; record_iter < recorder_metadata.flight_rows_used[flight_number]; record_iter++)
    {
        FlightInfoFrame record_frame;
        recorder.loadDataframe(flight_number, record_iter, &record_frame);

        Serial << record_frame.timestamp << "," << recorder.calculateAltitude(record_frame.absolute_pressure, record_frame.ground_pressure, record_frame.ground_temperature) << ",";

        switch (static_cast<EventCode>(record_frame.flight_event))
        {
        case EventCode::NO_EVENT:

            break;
        case EventCode::LAUNCHED_OFF_RAIL:
            Serial << F("Launched");
            break;
        case EventCode::APOGEE:
            Serial << F("Apogee @ -6");
            break;
        case EventCode::LANDED:
            Serial << F("Landed");
            break;
        default:
            break;
        }
        Serial << ',';
        switch (static_cast<DepolymentEvent>(record_frame.deployment_event))
        {
        case DepolymentEvent::DEPLOY_NO_EVENT:

            break;
        case DepolymentEvent::DEPLOY_MAIN_DEPLOYED:
            Serial << F("Main");
            break;
        case DepolymentEvent::DEPLOY_MAIN2_DEPLOYED:
            Serial << F("Main 2");
            break;
        case DepolymentEvent::DEPLOY_DROUGUE_DEPLOYED:
            Serial << F("Drougue");
            break;
        case DepolymentEvent::DEPLOY_DROUGUE2_DEPLOYED:
            Serial << F("Drougue 2");
            break;
        default:
            break;
        }
        Serial << ',' << record_frame.ground_pressure << ',' << record_frame.ground_temperature << ',' << record_frame.absolute_pressure << ',' << record_frame.temperature << ',';
        Serial << record_frame.acceleration_data[0] << ',' << record_frame.acceleration_data[1] << ',' << record_frame.acceleration_data[2] << endl;
    }

    Serial << endl
           << F("End of Flight Report") << endl;
    for (int i = 0; i < 100; i++)
    {
        Serial << '-';
    }
    Serial << endl;
}

// State Programs

DepolymentEvent detonateMachine()
{
    DepolymentEvent event_return = DepolymentEvent::DEPLOY_NO_EVENT;

    // static bool previous_main_detonate;
    // static bool previous_drogue_detonate;
    // static bool previous_same_detonate;

    static bool detonateLockout[4];

    static unsigned long detonate_main_start;
    static unsigned long detonate_drogue_start;

    // unsigned long same_delay;

    if (!drogue_det)
    {
        detonate_drogue_start = millis();
    }

    if (drogue_det)
    {
        if (millis() - detonate_drogue_start >= current_config.drouge_delay && !detonateLockout[0]) // Primary Drogue
        {
            event_return = DepolymentEvent::DEPLOY_DROUGUE_DEPLOYED;
            if ((continuity_voltages[0] >= CONTINUITY_VOLTAGE))
            {
                detonate_ch1.pulse(DETONATE_PULSE_TIME);
            }
            detonateLockout[0] = true;
        }
        if (millis() - detonate_drogue_start >= current_config.drouge_delay + current_config.redundant_delay && !detonateLockout[1]) // Secondary Drogue
        {
            event_return = DepolymentEvent::DEPLOY_DROUGUE2_DEPLOYED;
            if ((continuity_voltages[3] >= CONTINUITY_VOLTAGE))
            {
                detonate_ch4.pulse(DETONATE_PULSE_TIME);
            }
            detonateLockout[1] = true;
        }
    }

    if (!main_det || (millis() - detonate_drogue_start <= (current_config.drouge_delay + current_config.redundant_delay + current_config.dual_delay)))
    {
        detonate_main_start = millis();
    }

    if (main_det && ((millis() - detonate_drogue_start > (current_config.drouge_delay + current_config.redundant_delay + current_config.dual_delay)) || (same_det && millis() - detonate_drogue_start > current_config.dual_delay)))
    {
        if (millis() - detonate_main_start >= current_config.main_delay && !detonateLockout[2]) // Primary Main
        {
            event_return = DepolymentEvent::DEPLOY_MAIN_DEPLOYED;
            if((continuity_voltages[1] >= CONTINUITY_VOLTAGE))
            {
                detonate_ch2.pulse(DETONATE_PULSE_TIME);
                
            }
            detonateLockout[2] = true;
        }
        if (millis() - detonate_main_start >= current_config.main_delay + current_config.redundant_delay && !detonateLockout[3]) // Secondary Main
        {
            event_return = DepolymentEvent::DEPLOY_MAIN2_DEPLOYED;
            if ((continuity_voltages[2] >= CONTINUITY_VOLTAGE) )
            {
                detonate_ch3.pulse(DETONATE_PULSE_TIME);
            }
            detonateLockout[3] = true;
        }
    }

    detonate_ch1.update();
    detonate_ch2.update();
    detonate_ch3.update();
    detonate_ch4.update();
    // previous_drogue_detonate = drogue_det;
    // previous_main_detonate = main_det;
    // previous_same_detonate = same_det;

    return event_return;
}

void disarmedState()
{
    if (button.pressed())
    {
        current_state = SystemState::INSTALL;
    }
    else if (Serial)
    {
        current_state = SystemState::PC_DATA_TRANSFER;
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
            launch_detected = (resultant_accel >= current_config.trigger_acceleration) || (current_altitude >= current_config.trigger_altitude);
            break;
        case SystemConfig::TriggerOption::ALTITUDE_AND_ACCELERATION:
            launch_detected = (resultant_accel >= current_config.trigger_acceleration) && (current_altitude >= current_config.trigger_altitude);
            break;
        case SystemConfig::TriggerOption::ALTITUDE_ONLY:
            launch_detected = current_altitude >= current_config.trigger_altitude;
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
            unsigned char abort_fail = 0;
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
                    abort_fail = recorder.saveDataframe(flight_data_loop_buffer[i]);
                }
            }
            if (abort_fail != 0)
            {
                current_state = SystemState::ERROR;
            }
            else
            {
                current_state = SystemState::FLIGHT;
                flight_state = FlightState::LAUNCHED;
            }
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
    static CircularBuffer<float, 20> altitude_long_term;
    static byte altitude_long_term_counter;
    bool bypass_divider;
    FlightInfoFrame loop_data;

    loop_data.ground_pressure = armed_pressure;
    loop_data.ground_temperature = armed_temperature;
    loop_data.flight_event = static_cast<byte>(EventCode::NO_EVENT);
    loop_data.deployment_event = static_cast<byte>(DepolymentEvent::DEPLOY_NO_EVENT);

    static float previous_altitude;
    static float max_altitude; // max altitude
    static byte altitude_decreasing_count;

    // New Data
    if (readSensorData(&loop_data) == 0)
    {
        // Altitude Checker and Landing Detect
        if (altitude_long_term_counter > 17) // New reading at approx 30ms, want 10000 ms alt stability with 20 readings. need to take readings every 500ms. 500/30 = 17
        {
            altitude_long_term.push(current_altitude);
            altitude_long_term_counter = 0;
        }
        else
        {
            altitude_long_term_counter++;
        }

        // Altitude Increasing
        if (current_altitude > max_altitude)
        {
            max_altitude = current_altitude;
            altitude_decreasing_count = 0;
        }

        // Altitude Decreasing
        if (current_altitude < previous_altitude)
        {
            altitude_decreasing_count++;
        }

        // Apogee Detection
        if (altitude_decreasing_count > 5 && flight_state == FlightState::LAUNCHED) // was launched and been decending for 0.5 seconds
        {
            loop_data.flight_event = static_cast<byte>(EventCode::APOGEE);
            flight_state = FlightState::APOGEE_REACHED;
            bypass_divider = true;
            apogee_reached_time = millis();

            // delay main if rocket is under deployment altitude if drouge is active
            if ((current_altitude <= current_config.main_parachute_deploy_altitude))
            {
                same_det = true;
                drogue_det = true;
                main_det = true;
            }
            else
            {
                drogue_det = true;
            }
        }

        if (drogue_det && current_altitude <= current_config.main_parachute_deploy_altitude)
        {
            main_det = true;
        }

        loop_data.deployment_event = static_cast<byte>(detonateMachine());

        // Landed Detection
        if ((flight_state == FlightState::APOGEE_REACHED || flight_state == FlightState::DROUGE_DEPLOYED || flight_state == FlightState::MAIN_DEPLOYED) && altitude_long_term.size() >= 19)
        {
            float long_term_alt_avg = abs(altitude_long_term.first() - altitude_long_term.last());

            if (long_term_alt_avg <= current_config.landed_altitude_change) // if the absolute altitude change over 5 seconds is less than 3m
            {
                loop_data.flight_event = static_cast<byte>(EventCode::LANDED);
                flight_state = FlightState::LANDED;
                bypass_divider = true;
                altitude_long_term.clear();
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
        previous_altitude = current_altitude;
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
    FlightInfoFrame loop_data;
    readSensorData(&loop_data);
    loop_data.ground_pressure = armed_pressure;
    loop_data.ground_temperature = armed_temperature;

    // Serial Connect detect
    static bool old_serial_connect;
    static bool reprint_main_menu;
    static bool binary_control_mode;
    static bool print_sensor_data;

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
        byte ident = Serial.peek();
        if (ident == 0x11 && !binary_control_mode) // Device Control 1
        {
            // Binary Control
            while (Serial.available() > 0)
            {
                Serial.read();
            }
            binary_control_mode = true;
        }
        else if (ident == 0x12 && binary_control_mode) // Device Control 2
        {
            // Binary Control
            while (Serial.available() > 0)
            {
                Serial.read();
            }
            binary_control_mode = false;
        }

        if (binary_control_mode)
        {
        }

        if (!binary_control_mode)
        {
            // interactive control
            long option = Serial.parseInt();
            Serial << F("RX: ") << _DEC(option) << endl;
            while (Serial.available() > 0)
            {
                Serial.read();
            }
            RecorderMetadata recorder_metadata = recorder.getMetadata();
            switch (option)
            {
            case 1:
                printFlightData(0);
                break;
            case 2:
                printFlightData(1);
                break;
            case 3:
                printFlightData(2);
                break;
            case 4:
                printFlightData(3);
                break;
            case 5:
                printFlightData(recorder_metadata.last_successful_flight);
                break;
            case 80:
                armed_pressure = barometer_pressure;
                armed_temperature = barometer_temperature;
                Serial << F("Printing Sensor Data") << endl;
                print_sensor_data = true;
                break;
            case 81:
                Serial << F("Stopping Sensor Data") << endl;
                print_sensor_data = false;
                break;
            default:
                Serial << F("Invalid Input.") << endl;
                break;
            }
            reprint_main_menu = true;
        }
    }

    static unsigned long sensor_print;
    float raw_alt;
    if (print_sensor_data)
    {
        raw_alt = recorder.calculateAltitude(raw_pressure, armed_pressure, armed_temperature);
    }
    if (millis() - sensor_print >= 50 && print_sensor_data)
    {
        Serial << F("Sense Data- BP: ") << barometer_pressure << F(" ,BT: ") << barometer_temperature << F(" ,RALT: ") << raw_alt << F(" ,ALT: ") << current_altitude;
        Serial << F(" ,AX: ") << loop_data.acceleration_data[0] << F(" ,AY: ") << loop_data.acceleration_data[1] << F(" ,AZ: ") << loop_data.acceleration_data[2] << endl;
        sensor_print = millis();
    }

    // USB Disconnect
    if (!serial_connected)
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