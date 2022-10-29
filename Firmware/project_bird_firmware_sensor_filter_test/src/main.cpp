#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <board_definition.h>
#include <EEPROM.h>
#include <NewTone.h>

#include <SimpleKalmanFilter.h>
#include <MS5xxx.h>
#include <SparkFun_LIS331.h>
#include <Bounce2.h>
#include <LED_Pin_Control_Arduino.h>
#include <CircularBuffer.h>
#include <SPIMemory.h>
#include <FlightRecorder.h>

#include <Filters.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Notch.hpp>

// Sampling frequency
const double f_s = 250; // Hz
// Notch frequency (-∞ dB)
const double f_c = 50; // Hz
// Normalized notch frequency
const double f_n = 2 * f_c / f_s;

// Sample timer
Timer<micros> timer_sample = std::round(1e6 / f_s);
auto filter1 = simpleNotchFIR(f_n);     // fundamental

#define CIRCULAR_BUFFER_INT_SAFE

#define CONTINUITY_VOLTAGE 5.0
#define DETONATE_PULSE_TIME 2000

MS5xxx barometer(&Wire);
LIS331 accelerometer; //Adafruit_LIS331HH accelerometer = Adafruit_LIS331HH();

MemoryMap4M memory_map;
FlightRecorder recorder(MB(4), FLASH_CHIP_SS, memory_map);

SimpleKalmanFilter pressureKalmanFilter(50, 50, 0.15);
float barometer_temperature;
float barometer_pressure;
float kal_filter_barometer_pressure;
float notch_filter_barometer_pressure;

float ground_pressure;
float ground_temperature;

// Sensing Functions

byte updateBarometerData()
{
    
    barometer.Readout();

    barometer_pressure = barometer.GetPres();
    barometer_temperature = barometer.GetTemp() * 0.01;
    kal_filter_barometer_pressure = pressureKalmanFilter.updateEstimate(barometer_pressure);

    if (timer_sample) {
       notch_filter_barometer_pressure = filter1(barometer_pressure);
    }

    return 0x00;
}

float filterAltitude(float raw_alt)
{
    static float prev_alt;
    if (isfinite(raw_alt))
    {
        prev_alt = pressureKalmanFilter.updateEstimate(raw_alt);
        return prev_alt;
    }
    return prev_alt;
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

    return 0x00;
}

// Main Program
void setup()
{
    delay(2000); // Bootloader Delay
    byte sensor_setup_val = sensorSetup();
    bool sensor_setup_success = (sensor_setup_val == 0x00);
    barometer.Readout();

    for (int i = 0; i < 100; i++)
    {
        updateBarometerData();
    }
    
    ground_pressure = kal_filter_barometer_pressure;
    ground_temperature = barometer_temperature;
    while (!Serial)
    {
        /* code */
    }
    Serial.begin(115200);
}

float calculateAltitude(float current_pressure, float ground_pressure, float ground_temperature)
{
    // From https://www.mide.com/air-pressure-at-altitude-calculator
    float temp_kelvin = ground_temperature + 273.15;
    float tb_lb = temp_kelvin / -0.0065;
    const float constant_exponent = 0.1902632365; // calculated from (-8.31432 * -0.0065)/(9.80665*0.0289644)
    float p_pb = pow((current_pressure / ground_pressure), constant_exponent) - 1;
    return tb_lb * p_pb;
}

int16_t x, y, z;
float x_g, y_g, z_g;
unsigned long timer;

void loop()
{
    if (accelerometer.newXData() || accelerometer.newYData() || accelerometer.newZData())
    {
        accelerometer.readAxes(x, y, z);
    }
    updateBarometerData();

    x_g = accelerometer.convertToG(24, x);
    y_g = accelerometer.convertToG(24, y);
    z_g = accelerometer.convertToG(24, z);

    if (millis() - timer >= 10)
    {
        Serial.print(calculateAltitude(barometer_pressure, ground_pressure, ground_temperature));
        Serial.print(" ");
        Serial.print(calculateAltitude(kal_filter_barometer_pressure, ground_pressure, ground_temperature));
        //Serial.print(" ");
        //Serial.print(calculateAltitude(notch_filter_barometer_pressure, ground_pressure, ground_temperature));
        //Serial.print(" ");
        //Serial.print(x_g);
        //Serial.print(" ");
        //Serial.print(y_g);
        //Serial.print(" ");
        //Serial.print(z_g);
        Serial.println();
    }
}