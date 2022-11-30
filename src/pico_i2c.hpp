#ifndef __PICO_I2C_DRIVER__
#define __PICO_I2C_DRIVER__

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

class PicoI2C{
    public:
        PicoI2C(i2c_inst_t *_i2c,uint _sda_pin,uint _scl_pin ,uint _baudrate);
        virtual bool write(uint8_t slave_address,uint8_t reg,const uint8_t *src,size_t len);
        virtual bool read(uint8_t slave_address,uint8_t reg,uint8_t *dst,size_t len);
    protected:
        i2c_inst_t *i2c;
        uint sda_pin;
        uint scl_pin;
        uint baudrate;
};

class PicoDmaI2C : public PicoI2C{
    public:
        PicoDmaI2C(i2c_inst_t *_i2c,uint _sda_pin,uint _scl_pin ,uint _baudrate);
        bool write(uint8_t slave_address,uint8_t reg,const uint8_t *src,size_t len) override;
        bool read(uint8_t slave_address,uint8_t reg,uint8_t *dst,size_t len) override;

        // Singleton Class
        static void createInstance(i2c_inst_t *_i2c,uint _sda_pin,uint _scl_pin ,uint _baudrate);
        static PicoDmaI2C* getInstance();
        
        void i2cHandler();

    private:
        static PicoDmaI2C *pico_dma_i2c;
        int tx_dma_chan;
        int rx_dma_chan;

        SemaphoreHandle_t irq_semaphor;
        SemaphoreHandle_t mutex;

        void setTarget(uint8_t slave_address);
};
#endif