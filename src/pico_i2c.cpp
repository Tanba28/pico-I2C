#include "pico_i2c.hpp"
#include "hardware/dma.h"
#include "hardware/irq.h"

PicoI2C::PicoI2C(i2c_inst_t *_i2c,uint _sda_pin,uint _scl_pin ,uint _baudrate):
i2c(_i2c),sda_pin(_sda_pin),scl_pin(_scl_pin),baudrate(_baudrate){
    i2c_init(i2c, baudrate);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
}

bool PicoI2C::write(uint8_t slave_address,uint8_t reg,const uint8_t *src,size_t len){
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = *src;
    // i2c_write_blocking(i2c,slave_address,reg,1,true);
    i2c_write_blocking(i2c,slave_address,buf,2,false);
    return true;
}

bool PicoI2C::read(uint8_t slave_address,uint8_t reg,uint8_t *dst,size_t len){  
    i2c_write_blocking(i2c,slave_address,&reg,1,true);
    i2c_read_blocking(i2c,slave_address,dst,len,false);
    return true;
}

PicoDmaI2C* PicoDmaI2C::pico_dma_i2c = NULL;

void PicoDmaI2C::createInstance(i2c_inst_t *_i2c,uint _sda_pin,uint _scl_pin ,uint _baudrate){
    if(pico_dma_i2c==NULL){
        pico_dma_i2c = new PicoDmaI2C(_i2c,_sda_pin,_scl_pin,_baudrate);
    }
}

PicoDmaI2C* PicoDmaI2C::getInstance(){
    return pico_dma_i2c;
}

void PicoDmaI2C::i2cHandler(){
    uint32_t status = i2c_get_hw(i2c)->intr_stat;

    if(status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS){
        i2c_get_hw(i2c)->clr_tx_abrt;
    }
    
    if(status & I2C_IC_INTR_STAT_R_STOP_DET_BITS){
        i2c->hw->clr_stop_det;

        BaseType_t higher_priority_task_woken = pdFALSE;   

        xSemaphoreGiveFromISR(pico_dma_i2c->irq_semaphor, &higher_priority_task_woken);
        
        portEND_SWITCHING_ISR( higher_priority_task_woken );    
    }
}

static void i2cHandlerWrapper(){
    PicoDmaI2C::getInstance()->i2cHandler();
}

PicoDmaI2C::PicoDmaI2C(i2c_inst_t *_i2c,uint _sda_pin,uint _scl_pin ,uint _baudrate):
PicoI2C(_i2c,_sda_pin,_scl_pin,_baudrate){
    irq_semaphor = xSemaphoreCreateBinary();
    mutex = xSemaphoreCreateBinary();

    tx_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_transfer_data_size(&c,DMA_SIZE_16);
    channel_config_set_read_increment(&c,true);
    channel_config_set_write_increment(&c,false);
    channel_config_set_dreq(&c,i2c_get_dreq(i2c,true));

    dma_channel_configure(
        tx_dma_chan,
        &c,
        &i2c_get_hw(i2c)->data_cmd,
        NULL,
        0,
        false
    );

    rx_dma_chan = dma_claim_unused_channel(true);

    c = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_transfer_data_size(&c,DMA_SIZE_8);
    channel_config_set_read_increment(&c,false);
    channel_config_set_write_increment(&c,true);
    channel_config_set_dreq(&c,i2c_get_dreq(i2c,false));

    dma_channel_configure(
        rx_dma_chan,
        &c,
        NULL,
        &i2c_get_hw(i2c)->data_cmd,
        0,
        false
    );
    
    i2c->hw->enable = 0;
    i2c_get_hw(i2c)->intr_mask = I2C_IC_INTR_MASK_M_STOP_DET_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS;
    i2c->hw->enable = 1;

    irq_set_exclusive_handler(I2C1_IRQ,i2cHandlerWrapper);
    irq_set_enabled(I2C1_IRQ,true);

    xSemaphoreGive(mutex);
}

void PicoDmaI2C::setTarget(uint8_t slave_address){
    i2c->hw->enable = 0;
    i2c->hw->tar = slave_address;
    i2c->hw->enable = 1;
}

bool PicoDmaI2C::write(uint8_t slave_address,uint8_t reg,const uint8_t *src,size_t len){
    xSemaphoreTake(mutex,portMAX_DELAY);

    uint16_t buf[64] = {0};
    buf[0] = reg | I2C_IC_DATA_CMD_RESTART_BITS;
    for(size_t  i=0;i<len;i++){
        buf[i+1] = src[i];
    }
    buf[len] = buf[len] | I2C_IC_DATA_CMD_STOP_BITS;

    setTarget(slave_address);

    dma_channel_set_read_addr(tx_dma_chan,buf,false);
    dma_channel_set_trans_count(tx_dma_chan,len+1,true);

    xSemaphoreTake(irq_semaphor,portMAX_DELAY);
    xSemaphoreGive(mutex);

    return true;
}
bool PicoDmaI2C::read(uint8_t slave_address,uint8_t reg,uint8_t *dst,size_t len){
    xSemaphoreTake(mutex,portMAX_DELAY);

    uint16_t buf[64] = {0};
    buf[0] = reg | I2C_IC_DATA_CMD_RESTART_BITS;
    for(size_t  i=0;i<len;i++){
        buf[i+1] = (i==0?I2C_IC_DATA_CMD_RESTART_BITS:0x00) | I2C_IC_DATA_CMD_CMD_BITS;
    }
    buf[len] = buf[len] | I2C_IC_DATA_CMD_STOP_BITS;

    setTarget(slave_address);

    dma_channel_set_write_addr(rx_dma_chan,dst,false);
    dma_channel_set_trans_count(rx_dma_chan,len,true);
    dma_channel_set_read_addr(tx_dma_chan,buf,false);
    dma_channel_set_trans_count(tx_dma_chan,len+1,true);

    xSemaphoreTake(irq_semaphor,portMAX_DELAY);
    xSemaphoreGive(mutex);

    return true;
}