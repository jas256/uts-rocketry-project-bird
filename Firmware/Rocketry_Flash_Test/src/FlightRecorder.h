#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <SPIMemory.h>

struct FlashMemoryMap
{
    uint8_t number_of_records = 0;
    uint16_t number_of_rows_in_record = 0;
    uint32_t bytes_in_record = 0;
    uint16_t sector_64k_in_boundary = 0;
};

struct MemoryMap4M : FlashMemoryMap // Calculated with the Excel Spreadsheet
{
    uint8_t number_of_records = 4;
    uint16_t number_of_rows_in_record = 29127;
    uint32_t bytes_in_record = 1048576;
    uint16_t sector_64k_in_boundary = 16;
    //uint32_t record_boundry_start_addresses[4] = {0,1048576,2097152,3145728};
    //uint32_t record_boundry_end_addresses[4] = {1048575,2097151,3145727,4194303};
    //uint16_t record_64k_start_sector_boundary[4] = {0,16,32,48};
    //uint16_t record_64k_end_sector_boundary[4] = {15,31,47,63};
};

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

struct alignas(4) FlightInfoFrame
{
    byte deployment_event;
    byte flight_event;
    byte reserved[2];
    float ground_temperature;
    float temperature;
    float ground_pressure;
    float absolute_pressure;
    float acceleration_data[3]; // x, y, z
    uint32_t timestamp;
};

struct alignas(4) RecorderMetadata 
{
    uint8_t current_flight_record;
    uint8_t reserved[3];

    bool flight_record_contains_data[4];
    bool flight_in_progress[4];
    uint32_t flight_rows_used[4];

    float max_altitudes[4];
    unsigned long flight_times[4];
};

class FlightRecorder
{
protected:
    SPIFlash* flash_;
    int flash_ss_;
    uint32_t flash_size_;
    FlashMemoryMap* memory_map_;
    RecorderMetadata current_metadata_;
    const int eeprom_address_ = 0;

    uint8_t active_flight_record_ = 255;
    uint32_t current_write_idx_ = 0;
    uint32_t current_write_row_idx_ = 0;
    uint32_t start_flight_time_ = 0;

    float min_pressure_;
    float start_pressure_;
    float start_temperature_;

    bool storeBytes(uint32_t start_address, void* data, size_t data_size);
    bool readBytes(uint32_t start_address, void* data, size_t data_size);

    bool erase64KSector(uint32_t sector);

public:
    FlightRecorder(uint32_t flash_size, int flash_ss_pin, FlashMemoryMap* memory_map);
    bool begin();

    void saveMetadata();
    void loadMetadata();

    byte beginFlight(bool override); // returns the flight number used
    byte saveDataframe(FlightInfoFrame data);
    byte endFlight();

    bool loadDataframe(uint8_t flight_number, uint32_t row_index, FlightInfoFrame* data_frame_out);
    bool eraseFlightRecord(uint8_t flight_number);

    ~FlightRecorder();
};
