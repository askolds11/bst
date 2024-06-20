#ifndef I2C_HPP
#define I2C_HPP

#include <driver/i2c_master.h>

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22

namespace i2c
{
    i2c_master_bus_handle_t new_master_bus();
}

#endif