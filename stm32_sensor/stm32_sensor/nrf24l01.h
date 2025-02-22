#ifndef MAIN_MIRF_H_
#define MAIN_MIRF_H_

#include <Arduino.h>

#include "MirfHardwareSpiDriver.h"

/* Memory Map */
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define CD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D

/* Bit Mnemonics */
#define MASK_RX_DR 6
#define MASK_TX_DS 5
#define MASK_MAX_RT 4
#define EN_CRC 3
#define CRCO 2
#define PWR_UP 1
#define PRIM_RX 0
#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0
#define ERX_P5 5
#define ERX_P4 4
#define ERX_P3 3
#define ERX_P2 2
#define ERX_P1 1
#define ERX_P0 0
#define AW 0
#define ARD 4
#define ARC 0
#define RF_DR_LOW 5
#define PLL_LOCK 4
#define RF_DR_HIGH 3
#define RF_PWR 1
#define LNA_HCURR 0
#define RX_DR 6
#define TX_DS 5
#define MAX_RT 4
#define RX_P_NO 1
#define TX_FULL 0
#define PLOS_CNT 4
#define ARC_CNT 0
#define TX_REUSE 6
#define FIFO_FULL 5
#define TX_EMPTY 4
#define RX_FULL 1
#define RX_EMPTY 0

/* Instruction Mnemonics */
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF

/* Non-P omissions */
#define LNA_HCURR 0

/* P model memory Map */
#define RPD 0x09
#define W_TX_PAYLOAD_NO_ACK 0xB0

/* P model bit Mnemonics */
#define RF_DR_LOW 5
#define RF_DR_HIGH 3
#define RF_PWR_LOW 1
#define RF_PWR_HIGH 2

/* Device addrees length:3~5 bytes */
#define mirf_ADDR_LEN 5
typedef struct
{
    uint8_t PTX;     // In sending mode.
    uint8_t  cePin;   // CE Pin controls RX / TX, default 8.
    uint8_t  csnPin;  // CSN Pin Chip Select Not, default 7.
    uint8_t channel; // Channel 0 - 127 or 0 - 84 in the US.
    uint8_t payload; // Payload width in bytes default 16 max 32.
    MirfHardwareSpiDriver *spi;
    uint8_t status; // Receive status
} NRF24_t;

class NRF24L01
{
    static const int SPI_Frequency = 4000000; // Stable even with a long jumper cable
//static const int SPI_Frequency = 6000000;
//static const int SPI_Frequency = 8000000; // Requires a short jumper cable
//static const int SPI_Frequency = 10000000; // Unstable even with a short jumper cable

    uint8_t *buf;
    NRF24_t dev;
    uint8_t *pipe0_read_addr;
    uint8_t pipe0_read_addr_len;

public:
    NRF24L01(uint8_t channel, uint8_t payload);
    void setup(MirfHardwareSpiDriver *spi);
    void power_down();
    void power_up();
    uint8_t reg_read(uint8_t reg);
    void reg_read_multiple(uint8_t reg, uint8_t* value, uint8_t len);
    uint8_t reg_write_bytes(uint8_t reg, uint8_t* buf, uint8_t len);
    uint8_t reg_write(uint8_t reg, uint8_t value);
    void flush_rx();
    void flush_tx();
    void set_power_speed(uint8_t power, uint8_t speed);
    void set_crc(uint8_t length);
    void set_channel(uint8_t channel);
    void open_tx_pipe(uint8_t* address, uint8_t len);
    void open_rx_pipe(uint8_t pipe_id, uint8_t* address, uint8_t len);
    void start_listening();
    void stop_listening();
    bool any();
    void recv(uint8_t * data);
    bool send(uint8_t* buf, uint32_t timeout = 500);
    void send_start(uint8_t * buf);
    int8_t send_done();
    void printAll(NRF24_t * dev);
    void spi_csnHi();
    void spi_csnLow();
    void Nrf24_ceHi();
    void Nrf24_ceLow();
    uint8_t spi_transfer(uint8_t address);
    bool spi_write_byte(uint8_t* Dataout, size_t DataLength);
    bool spi_read_byte(uint8_t* Datain, uint8_t* Dataout, size_t DataLength);
    // void Nrf24_print_byte_register(const char* name, uint8_t reg);
    // void Nrf24_print_address_register(const char* name, uint8_t reg);
    // void printAll();
};

#endif /* MAIN_MIRF_H_ */