#include "nrf24l01.h"
#include "nrf.hpp"
#include <esp_log.h>
#include "driver/gpio.h"
#include <string.h>
extern "C" {
#include "sd/sd.h"
#include "sntp/sntp.h"
#include "common.h"
}

int previousResponses[10][3] = {-1};

bool previousResponsesContains(int sensor_num, int millis) {
    int array_num = sensor_num - 1001;
    if (previousResponses[array_num][0] == millis ||
        previousResponses[array_num][1] == millis ||
        previousResponses[array_num][2] == millis
    ) {
        return true;
    }
    return false;
}

void task_nrf24l01(void *) {
    NRF24L01 nrf = NRF24L01(46, 16);

    nrf.open_tx_pipe((uint8_t *) "\xe1\xf0\xf0\xf0\xf0", 5);
    nrf.open_rx_pipe(1, (uint8_t *) "\xd2\xf0\xf0\xf0\xf0", 5);
    nrf.set_power_speed(0, 2);

    nrf.start_listening();

    nrf.printAll();

    uint8_t buf[32];
    comm_data_t* test;
    response_t* resp = new response_t();
    resp->pad1 = 0;
    resp->pad2 = 0;

    esp_err_t ret;

    while (1) {
        if (nrf.any()) {
            while (nrf.any()) {
                nrf.recv(buf);
                test = (comm_data_t*) buf;
                ESP_LOGI(pcTaskGetName(0), "Test: From %i, At %i, temp %f, rh %f", test->sensor_num, test->millis, test->temperature, test->humidity);
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // Give initiator time to get into receive mode.
            vTaskDelay(pdMS_TO_TICKS(15));
            nrf.stop_listening();


            // only allow 10 sensors
            if (!(test->sensor_num > 1000 && test->sensor_num <= 1010)) {
                // invalid data
                ESP_LOGW(pcTaskGetName(0), "Invalid sensor num");
            } else if (previousResponsesContains(test->sensor_num, test->millis)) {
                // already received
                ESP_LOGW(pcTaskGetName(0), "Already received");
                // reply to make it shut up, don't care about result
                resp->millis = test->millis;
                resp->sensor_num = test->sensor_num;
                bool sent = nrf.send((uint8_t*) resp);
            } else {
                // new data, remove older from previous responses
                if (xSemaphoreTake(xSemaphore, (TickType_t) 1000) == pdFALSE) {
                    ESP_LOGW(pcTaskGetName(0), "Couldn't obtain semaphore");
                } else {
                    ret = sd_append_sensor(test);

                    xSemaphoreGive(xSemaphore);

                    if (ret != ESP_OK) {
                        ESP_LOGW(pcTaskGetName(0), "Failed to write to sd");
                    } else {
                        int arrayNum = test->sensor_num - 1001;
                        previousResponses[arrayNum][0] = previousResponses[arrayNum][1];
                        previousResponses[arrayNum][1] = previousResponses[arrayNum][2];
                        previousResponses[arrayNum][2] = test->millis;

                        resp->millis = test->millis;
                        resp->sensor_num = test->sensor_num;
                        bool sent = nrf.send((uint8_t*) resp);
                        if (sent) {
                            ESP_LOGI(pcTaskGetName(0), "Responded succesfully");
                        } else {
                            ESP_LOGW(pcTaskGetName(0), "Sent fail");
                        }
                    }
                }
            }

            
            nrf.start_listening();
            ESP_LOGI(pcTaskGetName(0), "Listening...");
        }
        vTaskDelay(1);
    }
    delete resp;
}