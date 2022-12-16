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
#define MXC400xDATAWIDTH 12

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

typedef enum MXC400XRANGE {

    RANGE_PM_2G     = 0x0,
    RANGE_PM_4G     = 0x1,
    RANGE_PM_8G     = 0x2,
    RANGE_UNDEFINED = 0x3

} MXC400xRange;

typedef struct MXC400XRAWDATA {

    uint16_t x_accel;
    uint16_t y_accel;
    uint16_t z_accel;
    uint8_t temperature;

} MXC400xRawData;


typedef struct MXC400XREALDATA {

    float x_accel;
    float y_accel;
    float z_accel;
    float temperature;

} MXC400xRealData;

typedef struct MXC400XHAL {

    /// @brief Function to write to an I2C Device from a Master
    uint32_t (*i2c_reg_write)(const uint8_t address, const uint8_t reg, const void* const data, const uint32_t size);
    /// @brief Function to read from an I2C Device to a Master
    uint32_t (*i2c_reg_read)(const uint8_t address, const uint8_t reg, void* const data, const uint32_t size);

} MXC400xHAL;

typedef struct MXC400X {

    MXC400xHAL hal;
    MXC400xRange range;

} MXC400x;

/**
 * @brief 
 * 
 * @param dev 
 * @param hal 
 * @param range 
 * @return MXC400x* 
 */
MXC400x* MXC400xInit(MXC400x* const dev, const MXC400xHAL* const hal, const MXC400xRange range);

/**
 * @brief 
 * 
 * @param dev 
 */
void MXC400xDeinit(MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @param register 
 * @return uint8_t 
 */
uint8_t MXC400xRead(const MXC400x* const dev, const MXC400xReg reg);

/**
 * @brief 
 * 
 * @param dev 
 * @param regsiter 
 * @param value 
 */
uint8_t MXC400xWrite(const MXC400x* const dev, const MXC400xReg reg, const uint8_t value);

/**
 * @brief 
 * 
 * @param dev 
 * @return uint16_t 
 */
int16_t MXC400xReadXRaw(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return float 
 */
float MXC400xReadX(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return uint16_t 
 */
int16_t MXC400xReadYRaw(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return float 
 */
float MXC400xReadY(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return uint16_t 
 */
int16_t MXC400xReadZRaw(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return int8_t 
 */
int8_t MXC400xReadTRaw(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return float 
 */
float MXC400xReadT(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 */
uint8_t MXC400xReadRawData(const MXC400x* const dev, MXC400xRawData* const data);

/**
 * @brief 
 * 
 * @param dev 
 * @param data 
 * @return uint8_t 
 */
uint8_t MXC400xReadRealData(const MXC400x* const dev, MXC400xRealData* const data);


