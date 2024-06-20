#include <stdio.h>
#include "sensor/aht10.hpp"
#include "i2c/i2c.hpp"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "freertos/semphr.h"
#include "nrf/nrf.hpp"
#include "common.h"
#include "driver/gpio.h"
#include "esp_timer.h"
extern "C" {
#include "sd/sd.h"
#include "sntp/sntp.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_event.h"
#include "web/file_server.c"
}

#define SENSOR_NUM 1010

SemaphoreHandle_t xSemaphore;
sdmmc_card_t *sdCard;

static void task_aht10(void *)
{
    ESP_LOGI(pcTaskGetName(0), "TASK STARTING");
    i2c_master_bus_handle_t master_bus_handle = i2c::new_master_bus();
    ESP_LOGI(pcTaskGetName(0), "master bus created");
    sensor::AHT10 aht10 = sensor::AHT10(master_bus_handle);
    aht10.initialize();
    ESP_LOGI(pcTaskGetName(0), "aht10 created");

    comm_data_t* data = new comm_data_t();
    data->sensor_num = SENSOR_NUM;

    while (true)
    {
        aht10.read();
        data->temperature = aht10.temperature_f;
        data->humidity = aht10.humidity_f;
        
        data->millis = (int) (esp_timer_get_time() / 1000);

        if (xSemaphoreTake(xSemaphore, (TickType_t) 1000) == pdFALSE) {
            ESP_LOGW(pcTaskGetName(0), "Couldn't obtain semaphore");
        } else {
            sd_append_sensor(data);

            xSemaphoreGive(xSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

extern "C" void app_main()
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    sntp_main();

    xSemaphore = xSemaphoreCreateMutex();
    // init_sd(sdCard);
    // sdmmc_card_print_info(stdout, sdCard);
    setup_sd();

    example_start_file_server(MOUNT_POINT);

    gpio_pullup_en(GPIO_NUM_12);
    gpio_pullup_en(GPIO_NUM_13);
    gpio_pullup_en(GPIO_NUM_14);
    gpio_pullup_en(GPIO_NUM_15);
    // bare_minimum();
    xTaskCreate(task_aht10, "aht10", 4096, NULL, configMAX_PRIORITIES - 2, NULL);
    // xTaskCreate(&receiver, "RECEIVER", 1024 * 3, NULL, 2, NULL);
    xTaskCreate(&task_nrf24l01, "NFR24L01", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    
}