#include "i2c.hpp"

namespace i2c
{
    i2c_master_bus_handle_t new_master_bus()
    {
        i2c_master_bus_config_t i2c_master_bus_config = {
            .i2c_port = I2C_PORT,
            .sda_io_num = I2C_SDA_PIN,
            .scl_io_num = I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags {
                .enable_internal_pullup = true
            }
        };

        i2c_master_bus_handle_t master_bus_handle;
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &master_bus_handle));

        return master_bus_handle;
    }
}