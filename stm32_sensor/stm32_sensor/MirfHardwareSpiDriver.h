//#include "MirfSpiDriver.h"
#ifndef __MIRF_HARDWARE_SPI_DRIVER
#define __MIRF_HARDWARE_SPI_DRIVER

#include "SPI.h"

#define RF24_SPI_SPEED      8 * 1000000 // 8Mhz

class MirfHardwareSpiDriver  {

  public:
    SPISettings  spiSettings;
    virtual uint8_t transfer(uint8_t data);
    virtual void begin();
    virtual void end();
};

extern MirfHardwareSpiDriver MirfHardwareSpi;

#endif