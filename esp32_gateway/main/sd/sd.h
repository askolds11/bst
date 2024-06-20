#ifndef MY_SD_H_
#define MY_SD_H_

#define MOUNT_POINT "/sdcard"

#include "common.h"

esp_err_t sd_append_sensor(comm_data_t* response);
esp_err_t sd_append_file(const char *path, char *data);
void setup_sd(); 

#endif /* MY_SD_H_ */