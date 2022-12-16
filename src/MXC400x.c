/**
 * @file MXC400x.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/MXC400x.h"

static int16_t MXC400xReadData(const MXC400x* const dev, const MXC400xReg address) {

    uint8_t buffer[2];
    dev->hal.i2c_reg_read(MXC400xADDRESS, address, buffer, 2);
    
    int16_t out = (buffer[0] << 8) + buffer[1];
    return out;

}

static float MXC400xScaleData(const MXC400x* const dev, const uint16_t input) {

    int32_t output = input; // input is a left justified signed 12 bit value
    output  /=  dev->range == RANGE_PM_2G? 8:
                dev->range == RANGE_PM_4G? 4:
                dev->range == RANGE_PM_8G? 2: 1;

    return output / 2048.0f; 

}



uint8_t MXC400xRead(const MXC400x* const dev, const MXC400xReg reg) {

    uint8_t data = 0;
    dev->hal.i2c_reg_read(MXC400xADDRESS, reg, &data, 1);
    return data;

}

uint8_t MXC400xWrite(const MXC400x* const dev, const MXC400xReg reg, const uint8_t value)  { return dev->hal.i2c_reg_write(MXC400xADDRESS, reg, &value, 1); }

int16_t MXC400xReadXRaw(const MXC400x* const dev) { return MXC400xReadData(dev, XOUT_H); }

int16_t MXC400xReadYRaw(const MXC400x* const dev) { return MXC400xReadData(dev, YOUT_H); }

int16_t MXC400xReadZRaw(const MXC400x* const dev) { return MXC400xReadData(dev, ZOUT_H); }

float MXC400xReadX(const MXC400x* const dev) { return MXC400xScaleData(dev, MXC400xReadXRaw(dev)); }

float MXC400xReadY(const MXC400x* const dev) { return MXC400xScaleData(dev, MXC400xReadYRaw(dev)); }

float MXC400xReadZ(const MXC400x* const dev) { return MXC400xScaleData(dev, MXC400xReadZRaw(dev)); }


