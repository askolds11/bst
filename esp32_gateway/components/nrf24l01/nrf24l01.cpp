#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"
#include "nrf24l01.h"

#define TAG "NRF24"
#define HOST_ID SPI3_HOST

NRF24L01::NRF24L01(uint8_t channel, uint8_t payload) {
    esp_err_t ret;

    ESP_LOGI(TAG, "CONFIG_MISO_GPIO=%d", CONFIG_MISO_GPIO);
    ESP_LOGI(TAG, "CONFIG_MOSI_GPIO=%d", CONFIG_MOSI_GPIO);
    ESP_LOGI(TAG, "CONFIG_SCLK_GPIO=%d", CONFIG_SCLK_GPIO);
    ESP_LOGI(TAG, "CONFIG_CE_GPIO=%d", CONFIG_CE_GPIO);
    ESP_LOGI(TAG, "CONFIG_CSN_GPIO=%d", CONFIG_CSN_GPIO);

    //gpio_pad_select_gpio(CONFIG_CE_GPIO);
    gpio_reset_pin((gpio_num_t) CONFIG_CE_GPIO);
    gpio_set_direction((gpio_num_t) CONFIG_CE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) CONFIG_CE_GPIO, 0);

    //gpio_pad_select_gpio(CONFIG_CSN_GPIO);
    gpio_reset_pin((gpio_num_t) CONFIG_CSN_GPIO);
    gpio_set_direction((gpio_num_t) CONFIG_CSN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) CONFIG_CSN_GPIO, 1);

    spi_bus_config_t spi_bus_config = {0};

    spi_bus_config.sclk_io_num = CONFIG_SCLK_GPIO;
    spi_bus_config.mosi_io_num = CONFIG_MOSI_GPIO;
    spi_bus_config.miso_io_num = CONFIG_MISO_GPIO;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;

    ret = spi_bus_initialize( SPI3_HOST, &spi_bus_config, SPI_DMA_CH2 );
    ESP_LOGI(TAG, "spi_bus_initialize=%d",ret);
    assert(ret==ESP_OK);

    spi_device_interface_config_t devcfg;
    memset( &devcfg, 0, sizeof( spi_device_interface_config_t ) );
    devcfg.clock_speed_hz = SPI_Frequency;
    // It does not work with hardware CS control.
    //devcfg.spics_io_num = csn_pin;
    // It does work with software CS control.
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 7;
    devcfg.mode = 0;
    devcfg.flags = SPI_DEVICE_NO_DUMMY;

    spi_device_handle_t handle;
    ret = spi_bus_add_device( HOST_ID, &devcfg, &handle);
    ESP_LOGI(TAG, "spi_bus_add_device=%d",ret);
    assert(ret==ESP_OK);

    dev.cePin = (gpio_num_t) CONFIG_CE_GPIO;
    dev.csnPin = (gpio_num_t) CONFIG_CSN_GPIO;
    dev.channel = channel;
    dev.payload = payload;
    dev._SPIHandle = handle;

    Nrf24_ceLow();
    spi_csnHi();
    // self.buf = bytearray(1)
    // self.pipe0_read_addr = None
    vTaskDelay(pdMS_TO_TICKS(5));

    // set address width to 5 bytes and check for device present
    reg_write(SETUP_AW, 0b11);
    if (reg_read(SETUP_AW) != 0b11) {
        // throw new Exception("nRF24L01+ Hardware not responding");
        return;
    }

    // disable dynamic payloads
    reg_write(DYNPD, 0);

    // auto retransmit delay: 1750us
    // auto retransmit count: 8
    reg_write(SETUP_RETR, (6 << 4) | 8);

    // set rf power and speed
    set_power_speed(0, 2);  // Best for point to point links

    // init CRC
    set_crc(2);

    // clear status flags
    reg_write(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

    // set channel
    set_channel(channel);

    // flush buffers
    flush_rx();
    flush_tx();
}

uint8_t NRF24L01::reg_read(uint8_t reg) {
    uint8_t rv;
    spi_csnLow();
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    spi_read_byte(&rv, &rv, 1);
    spi_csnHi();
    return rv;
}

void NRF24L01::reg_read_multiple(uint8_t reg, uint8_t* value, uint8_t len) {
    spi_csnLow();
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
	spi_read_byte(value, value, len);
    spi_csnHi();
}

uint8_t NRF24L01::reg_write_bytes(uint8_t reg, uint8_t* buf, uint8_t len) {
    spi_csnLow();
    spi_transfer(0x20 | reg);
    spi_write_byte(buf, len);
    spi_csnHi();
    return buf[0];
}

uint8_t NRF24L01::reg_write(uint8_t reg, uint8_t value) {
    spi_csnLow();
    uint8_t retVal = spi_transfer(0x20 | reg);
    spi_transfer(value);
    spi_csnHi();
    return retVal;
}

void NRF24L01::flush_rx() {
    spi_csnLow();
    spi_transfer(FLUSH_RX);
    spi_csnHi();
}
    

void NRF24L01::flush_tx() {
    spi_csnLow();
    spi_transfer(FLUSH_TX); // Write cmd to flush tx fifo
    spi_csnHi();
}

// power is one of POWER_x defines; speed is one of SPEED_x defines
// speed: 0=1Mbps, 1=2Mbps, 2=250Kbps
// power: 0=-18dBm,1=-12dBm,2=-6dBm,3=0dBm
void NRF24L01::set_power_speed(uint8_t power, uint8_t speed) {
    uint8_t setup = reg_read(RF_SETUP) & 0b11000001; // 0b11010001
    uint8_t speedB = 0b00000000;
    if (speed == 2) {
        speedB |= 0b00100000;
    } else if (speed == 1) {
        speedB |= 0b00001000;
    }
    reg_write(RF_SETUP, setup | (power << 1) | (speedB));
}

// length in bytes: 0, 1 or 2
void NRF24L01::set_crc(uint8_t length) {
    uint8_t config = reg_read(CONFIG) & ~((1 << CRCO) | (1 << EN_CRC));
    if (length == 0) {
        //
    }
    else if(length == 1) {
        config |= (1 << EN_CRC);
    }
    else {
        config |= (1 << EN_CRC) | (1 << CRCO);
    }
    reg_write(CONFIG, config);
}

void NRF24L01::set_channel(uint8_t channel) {
    if (channel > 125) {
        reg_write(RF_CH, 125);
    } else {
        reg_write(RF_CH, channel);
    }
}

// address should be a bytes object 5 bytes long
void NRF24L01::open_tx_pipe(uint8_t* address, uint8_t len) {
    assert(len == 5);
    reg_write_bytes(RX_ADDR_P0, address, len);
    reg_write_bytes(TX_ADDR, address, len);
    reg_write(RX_PW_P0, dev.payload);
}

// address should be a bytes object 5 bytes long
// pipe 0 and 1 have 5 byte address
// pipes 2-5 use same 4 most-significant bytes as pipe 1, plus 1 extra byte
void NRF24L01::open_rx_pipe(uint8_t pipe_id, uint8_t* address, uint8_t len) {
    assert(len == 5);
    assert(0 <= pipe_id);
    assert(pipe_id <= 5);
    if (pipe_id == 0) {
        pipe0_read_addr = address;
        pipe0_read_addr_len = len;
    }
    if (pipe_id < 2) {
        reg_write_bytes(RX_ADDR_P0 + pipe_id, address, len);
    }
    else {
        reg_write(RX_ADDR_P0 + pipe_id, address[0]);
    }
    reg_write(RX_PW_P0 + pipe_id, dev.payload);
    reg_write(EN_RXADDR, reg_read(EN_RXADDR) | (1 << pipe_id));
}

void NRF24L01::start_listening() {
    reg_write(CONFIG, reg_read(CONFIG) | (1 << PWR_UP) | (1 << PRIM_RX));
    reg_write(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

    if (pipe0_read_addr != NULL) {
        reg_write_bytes(RX_ADDR_P0, pipe0_read_addr, pipe0_read_addr_len);
    }

    flush_rx();
    flush_tx();
    Nrf24_ceHi();
    esp_rom_delay_us(130);
}

void NRF24L01::stop_listening() {
    Nrf24_ceLow();
    flush_tx();
    flush_rx();
}
    

// returns True if any data available to recv
bool NRF24L01::any() {
    return !(reg_read(FIFO_STATUS) & (1 << RX_EMPTY));
}

void NRF24L01::recv(uint8_t * data) {
    // get the data
    spi_csnLow();
    spi_transfer(R_RX_PAYLOAD);
    spi_read_byte(data, data, dev.payload);
    spi_csnHi();
    // clear RX ready flag
    reg_write(STATUS, (1 << RX_DR));
}

// blocking wait for tx complete
// buf must be same as payload!
// true if success, false otherwise
bool NRF24L01::send(uint8_t* buf, uint32_t timeout) {
    send_start(buf);
    TickType_t startTick = xTaskGetTickCount();
    int8_t result = -1;
    while (result == -1 && (xTaskGetTickCount() - startTick) * portTICK_PERIOD_MS < timeout) {
        result = send_done();  // 1 == success, 2 == fail
    }
    if (result == 2) {
        return false;
    } else {
        return true;
    }
}

// non-blocking tx
void NRF24L01::send_start(uint8_t * buf) {
    // power up
    reg_write(CONFIG, (reg_read(CONFIG) | (1 << PWR_UP)) & ~(1 << PRIM_RX));
    esp_rom_delay_us(150);
    // send the data
    spi_csnLow();
    spi_transfer(W_TX_PAYLOAD);
    spi_write_byte(buf, dev.payload); // Write payload
    // if len(buf) < self.payload_size:
    //     self.spi.write(b"\x00" * (self.payload_size - len(buf)))  # pad out data
    spi_csnHi();

    // enable the chip so it can send the data
    Nrf24_ceHi();
    esp_rom_delay_us(15);  // needs to be >10us
    Nrf24_ceLow();
}

// returns -1 if send still in progress, 1 for success, 2 for fail
int8_t NRF24L01::send_done() {
    if (!(reg_read(STATUS) & ((1 << TX_DS)  | (1 << MAX_RT)))) {
        return -1;  // tx not finished
    }

    bool tx_ds = reg_read(STATUS) & (1 << TX_DS);
    bool max_rt = reg_read(STATUS) & (1 << MAX_RT);

    // either finished or failed: get and clear status flags, power down
    reg_write(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));
    reg_write(CONFIG, reg_read(CONFIG) & ~(1 << PWR_UP));
    if (max_rt) {
        ESP_LOGW(TAG, "Failed because of MAX_RT");
    }
    if (tx_ds) {
        return 1;
    }
    return 2;
}

void NRF24L01::Nrf24_print_byte_register(const char* name, uint8_t reg)
{
	printf("%s\t =", name);
    uint8_t buffer = reg_read(reg);
    printf(" 0x%02x", buffer);
	printf("\r\n");
}

void NRF24L01::Nrf24_print_address_register(const char* name, uint8_t reg)
{
	printf("%s\t =",name);
		//uint8_t buffer[addr_width];
		uint8_t buffer[5];
		reg_read_multiple(reg++, buffer, sizeof(buffer));

		printf(" 0x");
		for(int i=0;i<5;i++) {
			printf("%02x", buffer[i]);
		}
	printf("\r\n");
}

void NRF24L01::printAll()
{
    Nrf24_print_byte_register("CONFIG\t", CONFIG);
    Nrf24_print_byte_register("EN_AA\t", EN_AA);
    Nrf24_print_byte_register("EN_RXADDR\t", EN_RXADDR);
    Nrf24_print_byte_register("SETUP_AW\t", SETUP_AW);
    Nrf24_print_byte_register("SETUP_RETR\t", SETUP_RETR);
    Nrf24_print_byte_register("RF_CH\t", RF_CH);
    Nrf24_print_byte_register("RF_SETUP\t", RF_SETUP);
    Nrf24_print_byte_register("STATUS\t", STATUS);
    Nrf24_print_byte_register("OBSERVE_TX\t", OBSERVE_TX);
    Nrf24_print_byte_register("CD\t", CD);

    Nrf24_print_address_register("RX_ADDR_P0\t", RX_ADDR_P0);
    Nrf24_print_address_register("RX_ADDR_P1\t", RX_ADDR_P1);
	Nrf24_print_byte_register("RX_ADDR_P2\t", RX_ADDR_P2);
    Nrf24_print_byte_register("RX_ADDR_P3\t", RX_ADDR_P3);
    Nrf24_print_byte_register("RX_ADDR_P4\t", RX_ADDR_P4);
    Nrf24_print_byte_register("RX_ADDR_P5\t", RX_ADDR_P5);
	Nrf24_print_address_register("TX_ADDR\t", TX_ADDR);

	Nrf24_print_byte_register("RX_PW_P0\t", RX_PW_P0);
    Nrf24_print_byte_register("RX_PW_P1\t", RX_PW_P1);
    Nrf24_print_byte_register("RX_PW_P2\t", RX_PW_P2);
    Nrf24_print_byte_register("RX_PW_P3\t", RX_PW_P3);
    Nrf24_print_byte_register("RX_PW_P4\t", RX_PW_P4);
    Nrf24_print_byte_register("RX_PW_P5\t", RX_PW_P5);

    Nrf24_print_byte_register("FIFO_STATUS\t", FIFO_STATUS);
	
	Nrf24_print_byte_register("DYNPD/FEATURE", DYNPD);
}

void NRF24L01::spi_csnHi() {
    gpio_set_level( dev.csnPin, 1 );
}

void NRF24L01::spi_csnLow() {
    gpio_set_level( dev.csnPin, 0 );
}

void NRF24L01::Nrf24_ceHi() {
    gpio_set_level( dev.cePin, 1 );
}

void NRF24L01::Nrf24_ceLow() {
    gpio_set_level( dev.cePin, 0 );
}

uint8_t NRF24L01::spi_transfer(uint8_t address) {
	uint8_t datain[1];
	uint8_t dataout[1];
	dataout[0] = address;
	//spi_write_byte(dev, dataout, 1 );
	spi_read_byte(datain, dataout, 1 );
	return datain[0];
}

bool NRF24L01::spi_write_byte(uint8_t* Dataout, size_t DataLength)
{
	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Dataout;
		SPITransaction.rx_buffer = NULL;
		spi_device_transmit( dev._SPIHandle, &SPITransaction );
	}

	return true;
}

bool NRF24L01::spi_read_byte(uint8_t* Datain, uint8_t* Dataout, size_t DataLength)
{
	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Dataout;
		SPITransaction.rx_buffer = Datain;
		spi_device_transmit( dev._SPIHandle, &SPITransaction );
	}

	return true;
}
