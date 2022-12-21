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

typedef enum MXC400XRANGE {

    RANGE_PM_2G     = 0x0,
    RANGE_PM_4G     = 0x1,
    RANGE_PM_8G     = 0x2,
    RANGE_UNDEFINED = 0x3

} MXC400xRange;

typedef enum MXC400XINTSOURCE {

    INT_SHAKE_XP    = 0x0001,
    INT_SHAKE_XM    = 0x0002,
    INT_SHAKE_YP    = 0x0004,
    INT_SHAKE_YM    = 0x0008,
    INT_CH_ORIENT_XY= 0x0040,
    INT_CH_ORIENT_Z = 0x0080,
    INT_TILT        = 0x8000,
    INT_ORIENT_Z    = 0x4000,
    INT_ORIENT_XY1  = 0x2000,
    INT_ORIENT_XY0  = 0x1000,
    INT_DATA_READY  = 0x0100

} MXC400xIntSource;

typedef enum MXC400XORIENTCHANGE {

    ORIENT_CHANGE_FAST      = 0x00,
    ORIENT_CHANGE_MEDIUM    = 0x01,
    ORIENT_CHANGE_SLOW      = 0x02,
    ORIENT_CHANGE_SLOWEST   = 0x03

} MXC400xOrientChange;

typedef enum MXC400XSHAKESPEED {

    SHAKE_FAST      = 0x00,
    SHAKE_MEDIUM    = 0x01,
    SHAKE_SLOW      = 0x02,
    SHAKE_SLOWEST   = 0x03

} MXC400xShakeSpeed;

typedef enum MXC400XSHAKETHRESH {

    SHAKE_THRESH_G25    = 0x00,
    SHAKE_THRESH_G50    = 0x01,
    SHAKE_THRESH_1G     = 0x02,
    SHAKE_THRESH_1G25   = 0x03,
    SHAKE_THRESH_1G50   = 0x04,
    SHAKE_THRESH_1G75   = 0x05,
    SHAKE_THRESH_2G     = 0x06,
    SHAKE_THRESH_2G25   = 0x07

} MXC400xShakeThresh;

typedef enum MXC400XSHAKEMODE {

    SHAKE_MODE_NOTHRESH = 0x00,
    SHAK_MODE_THRESH    = 0x01

} MXC400xShakeMode;

#pragma pack(push, 1)
typedef struct MXC400XRAWDATA {

    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int8_t temperature;

} MXC400xRawData;

#pragma pack(pop)

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
uint16_t MXC400xReadInt(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @param int_mask 
 * @return uint16_t 
 */
uint16_t MXC400xEnableInt(const MXC400x* const dev, const uint16_t int_mask);

/**
 * @brief 
 * 
 * @param dev 
 * @param int_mask 
 * @return uint16_t 
 */
uint16_t MXC400xDisableInt(const MXC400x* const dev, const uint16_t int_mask);

/**
 * @brief 
 * 
 * @param dev 
 */
void MXC400xClearInt(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev
 * @param mode 
 * @param speed 
 * @param thresh 
 * @param chor 
 * @return uint8_t 
 */
uint8_t MXC400xSetDetection(const MXC400x* const dev, const MXC400xShakeMode mode, const MXC400xShakeSpeed speed, const MXC400xShakeThresh thresh, const MXC400xOrientChange chor);

/**
 * @brief 
 * 
 * @param dev 
 * @return uint8_t 
 */
uint8_t MXC400xReset(const MXC400x* const dev);

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
int8_t MXC400xReadTempRaw(const MXC400x* const dev);

/**
 * @brief 
 * 
 * @param dev 
 * @return float 
 */
float MXC400xReadTemp(const MXC400x* const dev);

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

/**
 * @brief 
 * 
 * @param dev 
 * @return uint8_t 
 */
uint8_t MXC400xReadID(const MXC400x* const dev) { return MXC400xRead(dev, ID);}

