#include "aht10.hpp"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <esp_log.h>

namespace sensor
{
    /// @brief
    /// @param i2c_port Which I2C controller (port) to use
    AHT10::AHT10(i2c_master_bus_handle_t master_bus_handle) : master_bus_handle(master_bus_handle) {}

    AHT10::~AHT10()
    {
        i2c_master_bus_rm_device(device_handle);
        // i2c_del_master_bus(master_bus_handle);
    }

    esp_err_t AHT10::initialize()
    {
        ESP_LOGI(TAG, "Starting AHT10");
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = i2c_address,
            .scl_speed_hz = 100000,
        };

        esp_err_t error;
        error = i2c_master_bus_add_device(master_bus_handle, &dev_cfg, &device_handle);
        if (error != ESP_OK)
        {
            return error;
        }

        // soft reset
        ESP_LOGV(TAG, "Soft resetting AHT10");
        error = i2c_master_transmit(device_handle, soft_reset_seq, sizeof(soft_reset_seq), -1);
        if (error != ESP_OK)
        {
            ESP_LOGE(TAG, "Communication failed with AHT10!");
            return error;
        }
        vTaskDelay(pdMS_TO_TICKS(soft_reset_delay));

        // initialize
        ESP_LOGV(TAG, "Initializing AHT10");
        error = i2c_master_transmit(device_handle, initialize_seq, sizeof(initialize_seq), -1);
        if (error != ESP_OK)
        {
            ESP_LOGE(TAG, "Communication failed with AHT10!");
            return error;
        }
        vTaskDelay(pdMS_TO_TICKS(default_delay));
        
        // calibrate
        ESP_LOGV(TAG, "Calibrating AHT10");
        // only need status - only read first byte, ignore the rest
        uint8_t data = 0x00;
        bool success = false;
        for (int i = 0; i < calibration_attempts; i++) {
            error = i2c_master_receive(device_handle, &data, sizeof(data), -1);
            if (error != ESP_OK)
            {
                ESP_LOGE(TAG, "Communication failed with AHT10!");
                return error;
            }

            // busy bit should be 0 for success
            if ((data & busy) != busy) {
                success = true;
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(default_delay));
        }

        if (!success) {
            ESP_LOGE(TAG, "AHT10 was busy!");
            return ESP_FAIL;
        }

        // calibrated bit should be 1
        if ((data & normal_calibrated) != calibrated) {
            ESP_LOGE(TAG, "Failed to calibrate AHT10!");
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "AHT10 Initialized!");
        return ESP_OK;
    }

    esp_err_t AHT10::read() {
        esp_err_t error;
        // initialize
        error = i2c_master_transmit(device_handle, measure_seq, sizeof(measure_seq), -1);
        if (error != ESP_OK)
        {
            ESP_LOGE(TAG, "Communication failed with AHT10!");
            return error;
        }
        vTaskDelay(pdMS_TO_TICKS(read_delay));

        uint8_t data[6];
        error = i2c_master_receive(device_handle, data, sizeof(data), -1);
        if (error != ESP_OK)
        {
            ESP_LOGE(TAG, "Communication failed with AHT10!");
            return error;
        }

        if ((data[0] & busy) == busy) {
            return ESP_FAIL; // TODO: Restart read
        }

        uint32_t humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;
        uint32_t temperature = (((data[3] & 0x0F)<< 16) | (data[4] << 8) | data[5]);

        humidity_f = humidity / 1048576.0 * 100; // 2^20 (20 bits)
        temperature_f = temperature / 1048576.0 * 200 - 50; // 2^20 (20 bits), formula datasheet

        // ESP_LOGI(TAG, "Humidity: %f, Temperature: %f", humidity_f, temperature_f);

        return ESP_OK;
    }

    
}