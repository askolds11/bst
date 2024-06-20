#include <STM32RTC.h>
#include <rtc.h>
#include <STM32LowPower.h>
#include <low_power.h>

#include "MirfHardwareSpiDriver.h"
#include "nrf24l01.h"
#include <Wire.h>
#include <ArtronShop_SHT3x.h>

#define SENSOR_NUM 1002

struct comm_data_t {
    int sensor_num;
    int millis;
    float temperature;
    float humidity;
};

struct response_t {
    int sensor_num;
    int millis;
    int pad1 = 0;
    int pad2 = 0;
};

comm_data_t mydata;
uint8_t buf[32];
response_t resp;

NRF24L01 nrf = NRF24L01((uint8_t) 46, (uint8_t) 16);
ArtronShop_SHT3x sht3x(0x44, &Wire);

// HardwareSerial Serial2(PA3, PA2);

// void Nrf24_print_byte_register(const char* name, uint8_t reg)
// {
// 	Serial2.printf("%s\t =", name);
//     uint8_t buffer = nrf.reg_read(reg);
//     Serial2.printf(" 0x%02x", buffer);
// 	Serial2.printf("\r\n");
// }

// void Nrf24_print_address_register(const char* name, uint8_t reg)
// {
// 	Serial2.printf("%s\t =",name);
// 		//uint8_t buffer[addr_width];
// 		uint8_t buffer[5];
// 		nrf.reg_read_multiple(reg++, buffer, sizeof(buffer));

// 		Serial2.printf(" 0x");
// 		for(int i=0;i<5;i++) {
// 			Serial2.printf("%02x", buffer[i]);
// 		}
// 	Serial2.printf("\r\n");
// }

// void printAll()
// {
//     Nrf24_print_byte_register("CONFIG\t", CONFIG);
//     Nrf24_print_byte_register("EN_AA\t", EN_AA);
//     Nrf24_print_byte_register("EN_RXADDR\t", EN_RXADDR);
//     Nrf24_print_byte_register("SETUP_AW\t", SETUP_AW);
//     Nrf24_print_byte_register("SETUP_RETR\t", SETUP_RETR);
//     Nrf24_print_byte_register("RF_CH\t", RF_CH);
//     Nrf24_print_byte_register("RF_SETUP\t", RF_SETUP);
//     Nrf24_print_byte_register("STATUS\t", STATUS);
//     Nrf24_print_byte_register("OBSERVE_TX\t", OBSERVE_TX);
//     Nrf24_print_byte_register("CD\t", CD);

//     Nrf24_print_address_register("RX_ADDR_P0\t", RX_ADDR_P0);
//     Nrf24_print_address_register("RX_ADDR_P1\t", RX_ADDR_P1);
// 	Nrf24_print_byte_register("RX_ADDR_P2\t", RX_ADDR_P2);
//     Nrf24_print_byte_register("RX_ADDR_P3\t", RX_ADDR_P3);
//     Nrf24_print_byte_register("RX_ADDR_P4\t", RX_ADDR_P4);
//     Nrf24_print_byte_register("RX_ADDR_P5\t", RX_ADDR_P5);
// 	Nrf24_print_address_register("TX_ADDR\t", TX_ADDR);

// 	Nrf24_print_byte_register("RX_PW_P0\t", RX_PW_P0);
//     Nrf24_print_byte_register("RX_PW_P1\t", RX_PW_P1);
//     Nrf24_print_byte_register("RX_PW_P2\t", RX_PW_P2);
//     Nrf24_print_byte_register("RX_PW_P3\t", RX_PW_P3);
//     Nrf24_print_byte_register("RX_PW_P4\t", RX_PW_P4);
//     Nrf24_print_byte_register("RX_PW_P5\t", RX_PW_P5);

//     Nrf24_print_byte_register("FIFO_STATUS\t", FIFO_STATUS);
	
// 	Nrf24_print_byte_register("DYNPD/FEATURE", DYNPD);
// }

// the setup function runs once when you press reset or power the board
void setup() {
  // Serial2.begin(115200);

  Wire.begin();
  while (!sht3x.begin()) {
    // Serial2.println("SHT3x not found !");
    delay(1000);
  }
  
  nrf.setup(&MirfHardwareSpi);
  // printAll();

  // initialize digital pin LED_BUILTIN as an output.
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);
  LowPower.begin();

  nrf.open_tx_pipe((uint8_t*)"\xe1\xf0\xf0\xf0\xf0", 5);
  nrf.open_rx_pipe(1, (uint8_t*)"\xd2\xf0\xf0\xf0\xf0", 5);
  nrf.set_power_speed(0, 2);

  mydata.sensor_num = SENSOR_NUM;
}

// the loop function runs over and over again forever
void loop() {
  nrf.power_up();
  //  if (sht3x.measure()) {
  //   Serial2.print("Temperature: ");
  //   Serial2.print(sht3x.temperature(), 1);
  //   Serial2.print(" *C\tHumidity: ");
  //   Serial2.print(sht3x.humidity(), 1);
  //   Serial2.print(" %RH");
  //   Serial2.println();
  // } else {
  //   Serial2.println("SHT3x read error");
  // }
  sht3x.measure();

  mydata.temperature = sht3x.temperature();
  mydata.humidity = sht3x.humidity();
  mydata.millis = millis();

  short send_attempts = 0;
  bool success = false;

  while (send_attempts < 10 && !success) {
    send_attempts++;
    // send
    if (!nrf.send((uint8_t*) &mydata)) {
      LowPower.sleep(2000);
      continue;
    }

    // start listening
    nrf.start_listening();

    long start_time = millis();
    bool timeout = false;
    while (!nrf.any() && !timeout) {
      if (millis() - start_time > 250) {
        timeout = true;
      }
    }

    // if timeout, try again
    if (timeout) {
      nrf.stop_listening();
      continue;
    } else {
      // received
      nrf.recv(buf);
      nrf.stop_listening();
      resp = *((response_t*) buf);

      if (resp.sensor_num == SENSOR_NUM and resp.millis == mydata.millis) {
        success = true;
      }
    }
  }

  nrf.power_down();
  if (success) {
    LowPower.deepSleep(60000);
  } else {
    LowPower.deepSleep(25000);
  }
}