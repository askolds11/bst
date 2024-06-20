#ifndef APP_COMMON_H_
#define APP_COMMON_H_

#include "sdmmc_cmd.h"

extern SemaphoreHandle_t xSemaphore;
extern sdmmc_card_t *sdCard;

typedef struct {
    int sensor_num;
    int millis;
    float temperature;
    float humidity;
} comm_data_t;

#endif /* APP_COMMON_H_ */