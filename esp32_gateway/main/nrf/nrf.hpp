#ifndef NRF_TASK_H_
#define NRF_TASK_H_

struct response_t {
    int sensor_num;
    int millis;
    int pad1 = 0;
    int pad2 = 0;
};

bool previousResponsesContains(int sensor_num, int millis);
void task_nrf24l01(void *);

#endif /* NRF_TASK_H_ */