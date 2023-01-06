/**
 * @file MXC400x.c
 * @author Orion Serup (oserup@proton.me)
 * @brief Contains the functionality implementation for the MXC400
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../include/MXC400x.h"
#include <string.h>

static int16_t MXC400xReadData(const MXC400x* const dev, const MXC400xReg address) {

    #ifdef DEBUG
    if(dev == NULL)
        return 0;
    #endif

    uint8_t buffer[2];
    dev->hal.i2c_reg_read(MXC400xADDRESS, address, buffer, 2);
    
    int16_t out = ((buffer[0] << 8) | buffer[1]);
    return out;

}

static float MXC400xScaleData(const MXC400x* const dev, const int16_t input) {

    #ifdef DEBUG
    if(dev == NULL)
        return 0.0f;
    #endif

    int32_t output = input; // input is a left justified signed 12 bit value
    output  /=  dev->range == RANGE_PM_2G? 8: // scale it so it matches up with the range
                dev->range == RANGE_PM_4G? 4:
                dev->range == RANGE_PM_8G? 2: 1;

    return output / 2048.0f; // normalize the value to be a float

}

MXC400x* MXC400xInit(MXC400x* const dev, const MXC400xHAL* const hal, const MXC400xRange range) {

    if(dev == NULL || hal->i2c_reg_read == NULL || hal->i2c_reg_write == NULL || range > RANGE_UNDEFINED)
        return NULL;

    dev->hal = *hal;
    dev->range = range;
    
    uint8_t config = (range & 0x3) << 5;
    if(MXC400xWrite(dev, CONTROL, config) != 1)
        return NULL;

    return dev;

}

void MXC400xDeinit(MXC400x* const dev) {

    #ifdef DEBUG
    if(dev == NULL)
        return;
    #endif

    MXC400xWrite(dev, CONTROL, 0x61); // put the device to sleep and clear any config, replace with undefined values
    memset(&dev->hal, 0, sizeof(MXC400xHAL));
    dev->range = RANGE_UNDEFINED;

}

uint8_t MXC400xRead(const MXC400x* const dev, const MXC400xReg reg) {

    #ifdef DEBUG
    if(dev == NULL)
        return 0;
    #endif

    uint8_t data = 0;
    dev->hal.i2c_reg_read(MXC400xADDRESS, reg, &data, 1);
    return data;

}

uint8_t MXC400xWrite(const MXC400x* const dev, const MXC400xReg reg, const uint8_t value)  { 

    #ifdef DEBUG
    if(dev == NULL)
        return 0;
    #endif

    return dev->hal.i2c_reg_write(MXC400xADDRESS, reg, &value, 1); 
    
}

uint16_t MXC400xReadInt(const MXC400x* const dev) {

    #ifdef DEBUG
    if(dev == NULL)
        return 0;
    #endif

    uint8_t buffer[2] = {0};
    dev->hal.i2c_reg_read(MXC400xADDRESS, INT_SRC0, buffer, 2);
    
    return buffer[0] + (buffer[1] << 8);
}

uint8_t MXC400xClearInt(const MXC400x* const dev)  {

    #ifdef DEBUG
    if(dev == NULL)
        return;
    #endif

    uint8_t buffer[] = {0xff, 0x1};
    return (uint8_t)dev->hal.i2c_reg_write(MXC400xADDRESS, INT_SRC0, buffer, 2);
    
}

uint8_t MXC400xEnableInt(const MXC400x* const dev, const uint16_t int_mask) {

    #ifdef DEBUG
    if(dev == NULL)
        return 0;
    #endif

    uint8_t buffer[2];
    if(dev->hal.i2c_reg_read(MXC400xADDRESS, INT_MASK0, buffer, 2) == 0)
        return 0;

    buffer[0] |= (int_mask & 0xff);
    buffer[1] |= (int_mask >> 8);

    return dev->hal.i2c_reg_write(MXC400xADDRESS, INT_MASK0, buffer, 2);
    
}

uint8_t MXC400xDisableInt(const MXC400x* const dev, const uint16_t int_mask) {

    #ifdef DEBUG
    if(dev == NULL)
        return 0;
    #endif

    uint8_t buffer[2];
    if(dev->hal.i2c_reg_read(MXC400xADDRESS, INT_MASK0, buffer, 2) == 0)
        return 0;

    buffer[0] &= ~(int_mask & 0xff);
    buffer[1] &= ~(int_mask >> 8);

    return dev->hal.i2c_reg_write(MXC400xADDRESS, INT_MASK0, buffer, 2);

}

uint8_t MXC400xSetDetection(const MXC400x* const dev, const MXC400xShakeMode mode, const MXC400xShakeSpeed speed, const MXC400xShakeThresh thresh, const MXC400xOrientChange chor) {

    uint8_t value = ((mode & 1) << 7) | ((thresh & 0x7) << 4) | ((speed & 0x3) << 2) | (chor & 0x3);
    return MXC400xWrite(dev, DETECTION, value);

}

int16_t MXC400xReadXRaw(const MXC400x* const dev) { return MXC400xReadData(dev, XOUT_H); }

int16_t MXC400xReadYRaw(const MXC400x* const dev) { return MXC400xReadData(dev, YOUT_H); }

int16_t MXC400xReadZRaw(const MXC400x* const dev) { return MXC400xReadData(dev, ZOUT_H); }

int8_t MXC400xReadTempRaw(const MXC400x* const dev) { 

    #ifdef DEBUG
    if(dev == NULL)
        return INT8_MIN;
    #endif

    int8_t temp = 0;
    dev->hal.i2c_reg_read(MXC400xADDRESS, TOUT, &temp, 1);
    return temp;

}

float MXC400xReadX(const MXC400x* const dev) { return MXC400xScaleData(dev, MXC400xReadXRaw(dev)); }

float MXC400xReadY(const MXC400x* const dev) { return MXC400xScaleData(dev, MXC400xReadYRaw(dev)); }

float MXC400xReadZ(const MXC400x* const dev) { return MXC400xScaleData(dev, MXC400xReadZRaw(dev)); }

float MXC400xReadTemp(const MXC400x* const dev) { return MXC400xReadTempRaw(dev) * .586f; }

uint8_t MXC400xReadRawData(const MXC400x* const dev, MXC400xRawData* const data) {

    #ifdef DEBUG
    if(dev == NULL || data == NULL)
        return 0;
    #endif

    return dev->hal.i2c_reg_read(MXC400xADDRESS, XOUT_H, data, sizeof(MXC400xRawData));

}

uint8_t MXC400xReadRealData(const MXC400x* const dev, MXC400xRealData* const data)  {

    MXC400xRawData raw_data;
    if(MXC400xReadRawData(dev, &raw_data) == 0)
        return 0;

    data->x_accel = MXC400xScaleData(dev, raw_data.x_accel);
    data->y_accel = MXC400xScaleData(dev, raw_data.y_accel);
    data->z_accel = MXC400xScaleData(dev, raw_data.z_accel);
    data->temperature = raw_data.temperature * .586f;

    return sizeof(MXC400xRawData);

}


