#ifndef SNTP_STUFF_H_
#define SNTP_STUFF_H_

void sntp_main(void);
void get_current_date(char* formatted_date);
int64_t get_current_time_us();

#endif /* SNTP_STUFF_H_ */