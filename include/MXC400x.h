/**
 * @file MXC400x.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdint.h>

#define MXC400xADDRESS 0x15

typedef enum MXC400XREG {

    INT_SRC0    = 0x00,
    INT_CLR0    = 0x00,
    INT_SRC1    = 0x01,
    INT_CLR1    = 0x01,
    STATUS      = 0x02,
    XOUT_H      = 0x03,
    XOUT_L      = 0x04,
    YOUT_H      = 0x05,
    YOUT_L      = 0x06,
    ZOUT_H      = 0x07,
    ZOUT_L      = 0x08,
    TOUT        = 0x09,
    INT_MASK0   = 0x0A,
    INT_MASK1   = 0x0B,
    DETECTION   = 0x0C,
    CONTROL     = 0x0D,
    ID          = 0x0F

} MXC400xReg;

typedef struct MXC400XDATA {

    uint16_t x_accel;
    uint16_t x_accel;
    uint16_t z_accel;
    uint8_t temperature;

} MXC400xData;

typedef struct MXC400X {

    /// @brief Function to write to an I2C Device from a Master
    uint32_t (*i2c_reg_write)(const uint8_t address, const uint8_t reg, const void* const data, const uint32_t size);
    /// @brief Function to read from an I2C Device to a Master
    uint32_t (*i2c_reg_read)(const uint8_t address, const uint8_t reg, void* const data, const uint32_t size);

} MXC400x;


/**
 * @brief 
 * 
 * @param dev 
 * @param register 
 * @return uint8_t 
 */
uint8_t MXC400xRead(const MXC400x* const dev, const MXC400xReg register);

/**
 * @brief 
 * 
 * @param dev 
 * @param regsiter 
 * @param value 
 */
void MXC400xWrite(const MXC400x* const dev, const MXC400xReg regsiter, const uint8_t value);

/**
 * @brief 
 * 
 * @param dev 
 * @return uint16_t 
 */
uint16_t MXC400xReadX(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return uint16_t 
 */
uint16_t MXC400xReadY(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return uint16_t 
 */
uint16_t MXC400xReadZ(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 */
void MXC400xReadData(const MXC400x* const dev, MXC400xData* const data);

