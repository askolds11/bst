#ifndef AHT10_HPP
#define AHT10_HPP

#include <driver/i2c_master.h>

namespace sensor
{
    // https://esphome.io/api/aht10_8cpp_source

    // response
    // 8bit each
    // 0:state
        //  7 - busy (1 - busy, 0 - measurment state, is idle)
        //  6:5 - working mode (00 NOR(mal), 01 CYC, 1x CMD)
        //  4 - reserved
        //  3 - calibrated (1 - yes, 0 - no)
        //  2:0 - reserved
    // 1[7:0]:humidity
    // 2[7:0]:humidity data
    // 3[7:4]:humidity data
    // 3[3:0]:temperature data
    // 4[7:0]:temperature data
    // 5[7:0]:temperature
    class AHT10
    {
    public:
        AHT10(i2c_master_bus_handle_t master_bus_handle);
        ~AHT10();

        esp_err_t initialize();
        esp_err_t read();

        float humidity_f;
        float temperature_f;

    private:
        static constexpr char TAG[6] = "AHT10";
        static const uint8_t i2c_address = 0x38;

        // attempts
        static const uint8_t calibration_attempts = 10;

        // bit masks
        static const uint8_t busy = 0x80; // bit 7
        static const uint8_t normal_calibrated = 0x68; // bits 6:5, bit 3

        // bit mask results
        static const uint8_t calibrated = 0x08; // bit 3

        // Delays
        static const uint8_t default_delay = 5; //ms, for initialization and temperature measurement
        static const uint8_t read_delay = 80; // ms, time to wait for conversion result
        static const uint8_t soft_reset_delay = 30; // ms

        // Individual Commands
        static const uint8_t initialization_cmd = 0xE1; // datasheet
        static const uint8_t calibration_cmd = 0x08; // chinese datasheet
        static const uint8_t soft_reset_cmd = 0xBA; // datasheet
        static const uint8_t trigger_measurement_cmd = 0xAC; // datasheet
        static const uint8_t measurement_first_byte = 0x33; // chinese datasheet
        static const uint8_t no_cmd = 0x00; // chinese datasheet

        // Command sequences
        static constexpr uint8_t initialize_seq[] = {initialization_cmd, calibration_cmd, no_cmd};
        static constexpr uint8_t measure_seq[] = {trigger_measurement_cmd, measurement_first_byte, no_cmd};
        static constexpr uint8_t soft_reset_seq[] = {soft_reset_cmd};

        // Bus handles
        const i2c_master_bus_handle_t master_bus_handle;
        i2c_master_dev_handle_t device_handle;
    };
}

#endif
