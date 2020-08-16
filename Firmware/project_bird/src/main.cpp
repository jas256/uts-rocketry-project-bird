#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <board_definition.h>

#include <MS5xxx.h>
#include <Adafruit_LIS331DLH.h>
#include <Adafruit_Sensor.h>

MS5xxx barometer(&Wire);
Adafruit_LIS331DLH accelerometer = Adafruit_LIS331DLH();

float armed_pressure;

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

byte sensor_setup()
{
    barometer.setI2Caddr(BAROMETER_ADDRESS);
    if (barometer.connect() > 0) 
        return 0x01;
    if (!accelerometer.begin_I2C(ACCELEROMETER_ADDRESS,&Wire))
        return 0x02;
    accelerometer.configIntDataReady(digitalPinToInterrupt(ACCELEROMETER_INT1), true, true);
    accelerometer.setDataRate(LIS331_DATARATE_50_HZ)
    return 0x00;
}

byte readSensorData(FlightInfoFrame* dataframe)
{
    static unsigned long last_event_time;
    sensors_event_t event;
    lis.getEvent(&event);

    if (event.timestamp != last_event_time) 
    {
        dataframe->timestamp = event.timestamp;
        dataframe->ground_pressure = armed_pressure;
        dataframe->absolute_pressure = barometer.GetPres();
        dataframe->temperature = barometer.GetTemp();
        dataframe->acceleration_data[0] = event.acceleration.x;
        dataframe->acceleration_data[1] = event.acceleration.y;
        dataframe->acceleration_data[2] = event.acceleration.z;
        last_event_time = event.timestamp;
    }
    return 0x00;
}

void setup() {

}

void loop() {
    
}