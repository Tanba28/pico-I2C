#ifndef __PICO_I2C_DRIVER__
#define __PICO_I2C_DRIVER__

#include "pico/stdlib.h"
#include "hardware/i2c.h"

class PicoI2C{
    public:
        PicoI2C(i2c_inst_t *_i2c,uint _sda_pin,uint _scl_pin ,uint _baudrate);
        bool write(uint8_t slave_address,uint8_t *reg,const uint8_t *src,size_t len);
        bool read(uint8_t slave_address,uint8_t *reg,uint8_t *dst,size_t len);

    private:
        i2c_inst_t *i2c;
        uint sda_pin;
        uint scl_pin;
        uint baudrate;
};

#endif