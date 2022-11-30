#include "pico_i2c.hpp"

PicoI2C::PicoI2C(i2c_inst_t *_i2c,uint _sda_pin,uint _scl_pin ,uint _baudrate):
i2c(_i2c),sda_pin(_sda_pin),scl_pin(_scl_pin),baudrate(_baudrate){
    i2c_init(i2c, baudrate);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
}

bool PicoI2C::write(uint8_t slave_address,uint8_t *reg,const uint8_t *src,size_t len){
    i2c_write_blocking(i2c,slave_address,reg,1,false);
    i2c_write_blocking(i2c,slave_address,src,len,false);
    return true;
}

bool PicoI2C::read(uint8_t slave_address,uint8_t *reg,uint8_t *dst,size_t len){  
    i2c_write_blocking(i2c,slave_address,reg,1,true);
    i2c_read_blocking(i2c,slave_address,dst,len,false);
    return true;
}