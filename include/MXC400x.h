/**
 * @file MXC400x.h
 * @author Orion Serup (oserup@proton.me)
 * @brief Contains the definitions and declarations for the MXC400X functionality
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#ifndef MXC400_H
#define MXC400_H

#include <stdint.h>

// ---------------------------------- Global Defines ---------------------------------- //

#define MXC400xADDRESS 0x15 /// The I2C Address for the device

// ---------------------------------- Register Definitions --------------------------- //

/// @brief All of the register addresses in the device
typedef enum MXC400XREG {

    INT_SRC0    = 0x00, ///< Read Register and it gives you some of the interrupt sources
    INT_CLR0    = 0x00, ///< Writing the Register and it clears the interrupt flags
    INT_SRC1    = 0x01, ///< Reading this Register gets more of the sources of the interrupts
    INT_CLR1    = 0x01, ///< Writing this register clears more of the interrupt flags
    STATUS      = 0x02, ///< Status register for the device
    XOUT_H      = 0x03, ///< The High byte of the X acceleration
    XOUT_L      = 0x04, ///< The Low Byte of the X Acceleration
    YOUT_H      = 0x05, ///< The High Byte of the Y acceleration
    YOUT_L      = 0x06, ///< The Low Byte of the Y acceleration
    ZOUT_H      = 0x07, ///< The High Byte of the Z Acceleration    
    ZOUT_L      = 0x08, ///< The Low Byte of the Z acceleration
    TOUT        = 0x09, ///< The Temperature value
    INT_MASK0   = 0x0A, ///< The First byte of the interrupt mask
    INT_MASK1   = 0x0B, ///< The Second Byte of the interrupt mask
    DETECTION   = 0x0C, ///< The detection parameter register
    CONTROL     = 0x0D, ///< The Control and configuration register
    ID          = 0x0F  ///< The Identification register

} MXC400xReg;

// ------------------------------------ Range Definitions -------------------------------- //

/// @brief The Range of values that can be read, determines resolution
typedef enum MXC400XRANGE {

    RANGE_PM_2G     = 0x0,  ///< The range is from -2G to 2G
    RANGE_PM_4G     = 0x1,  ///< The range is from -4G to 4G
    RANGE_PM_8G     = 0x2,  ///< The range is from -8G to 8G
    RANGE_UNDEFINED = 0x3   ///< Undefined range, wont work

} MXC400xRange;

// ----------------------------------- Interrupt Definitions ---------------------------------- //

/// @brief The Interrupt sources for the device
typedef enum MXC400XINTSOURCE {

    // All of the interrupts that can be enabled

    INT_SHAKE_XP    = 0x0001,   ///< Interrupt on a shake in the Positive X Direction
    INT_SHAKE_XM    = 0x0002,   ///< Interrupt on a shake in the Negative X direction
    INT_SHAKE_YP    = 0x0004,   ///< Interrupt on a shake in the Positive Y direction
    INT_SHAKE_YM    = 0x0008,   ///< Interrupt on a shake in the Negative Y direction
    INT_CH_ORIENT_XY= 0x0040,   ///< Interrupt on a change in orientation in the X/Y Direction
    INT_CH_ORIENT_Z = 0x0080,   ///< Interrupt on a change in orientation in the Z Direction
    INT_DATA_READY  = 0x0100,   ///< Interrupt when the Data is ready to read
    
    // Readable interrupt flags only

    INT_TILT        = 0x8000,   ///< Interrupt on a tilt in Z
    INT_ORIENT_Z    = 0x4000,   ///< Interrupt on orientaion in the Z direction

    // Mutually Exclusive, can only pick one

    INT_ORIENT_XP   = 0x0000,   ///< Interrupt on orientation in Positive X direction (+1G)
    INT_ORIENT_YP   = 0x1000,   ///< Interrupt on orientation in Positive Y Direction (+1G)
    INT_ORIENT_XM   = 0x2000,   ///< Interrupt on orientation in Negative X Direction (-1G)
    INT_ORIENT_YM   = 0x3000,   ///< Interrupt on orientation in Negative X Direction (-1G)

} MXC400xIntSource;

// -------------------------- Orientation Change Detection Configuration -------------------------------- //

/// @brief How Fast of an orientation change to detect
typedef enum MXC400XORIENTCHANGE {

    ORIENT_CHANGE_FAST      = 0x00, ///< A Fast Change will be detected
    ORIENT_CHANGE_MEDIUM    = 0x01, ///< A Medium speed change will be detected
    ORIENT_CHANGE_SLOW      = 0x02, ///< A Slow speed change will be detected
    ORIENT_CHANGE_SLOWEST   = 0x03  ///< A super slow change will be detected

} MXC400xOrientChange;

// ----------------------------- Shake Interrupt Configuration Types ---------------------------------- //

/// @brief How fast of of a shake
typedef enum MXC400XSHAKESPEED {

    SHAKE_FAST      = 0x00, ///< Fast shakes will be detected
    SHAKE_MEDIUM    = 0x01, ///< Medium fast shakes will be detected
    SHAKE_SLOW      = 0x02, ///< Slow Shakes will be detected
    SHAKE_SLOWEST   = 0x03  ///< The slowest shakes will be detected

} MXC400xShakeSpeed;

/// @brief What threshold of force in the shake to detect
typedef enum MXC400XSHAKETHRESH {

    SHAKE_THRESH_G25    = 0x00, ///< Shake threshold is .25G
    SHAKE_THRESH_G50    = 0x01, ///< Shake threshold is .5G
    SHAKE_THRESH_1G     = 0x02, ///< Shake threshold is 1G
    SHAKE_THRESH_1G25   = 0x03, ///< Shake threshold is 1.25G
    SHAKE_THRESH_1G50   = 0x04, ///< Shake threshold is 1.5G
    SHAKE_THRESH_1G75   = 0x05, ///< Shake threshold is 1.75G
    SHAKE_THRESH_2G     = 0x06, ///< Shake threshold is 2G
    SHAKE_THRESH_2G25   = 0x07  ///< Shake threshold is 2.25G

} MXC400xShakeThresh;

/// @brief How to detect shakes and throw interrupts
typedef enum MXC400XSHAKEMODE {

    SHAKE_MODE_NOTHRESH = 0x00, ///< Shake will be detected with speed
    SHAK_MODE_THRESH    = 0x01  ///< Shake will be detected with speed and force

} MXC400xShakeMode;

// ---------------------------------------- Data Structures ---------------------------------------- //

#pragma pack(push, 1)

/// @brief All of the raw integral data from the device
typedef struct MXC400XRAWDATA {

    int16_t x_accel;    ///< X Acceleration value
    int16_t y_accel;    ///< Y Acceleration Value
    int16_t z_accel;    ///< Z Acceleration value
    int8_t temperature; ///< Temperature value

} MXC400xRawData;

#pragma pack(pop)

/// @brief The Actual data from the device, in actual units
typedef struct MXC400XREALDATA {

    float x_accel;      ///< The X Acceleration in Gs
    float y_accel;      ///< The Y Acceleration in Gs
    float z_accel;      ///< The Z Acceleration in Gs
    float temperature;  ///< The Temperature in Celsius

} MXC400xRealData;

// ---------------------------------------- HAL and Device Structuress ------------------------------ //

/// @brief All of the functions we need to 
typedef struct MXC400XHAL {

    /// @brief Function to write to an I2C Device from a Master
    uint32_t (*i2c_reg_write)(const uint8_t address, const uint8_t reg, const void* const data, const uint32_t size);
    /// @brief Function to read from an I2C Device to a Master
    uint32_t (*i2c_reg_read)(const uint8_t address, const uint8_t reg, void* const data, const uint32_t size);

} MXC400xHAL;

/// @brief A device struct to represent the device, contains everything needed to use the device
typedef struct MXC400X {

    MXC400xHAL hal;     ///< Functions to use 
    MXC400xRange range; ///< What range we are using 

} MXC400x;

// ------------------------------------------ Initialization and Deinitialization ---------------------------------------------- //

/**
 * @brief Initializes an accelerometer object
 * 
 * @param[out] dev: Device to Initialize 
 * @param[in] hal: Functionality to give the device
 * @param[in] range: Range to set the for the device
 * @return MXC400x*: The inited object, NULL if it fails to init
 */
MXC400x* MXC400xInit(MXC400x* const dev, const MXC400xHAL* const hal, const MXC400xRange range);

/**
 * @brief Deinitializes the object
 * 
 * @param[in] dev: Device to deinit and clear 
 */
void MXC400xDeinit(MXC400x* const dev);

// --------------------------------------------- Basic Read/Write ------------------------------------------- //

/**
 * @brief Reads from a register on the device
 * 
 * @param[in] dev: Device to read from
 * @param[in] register: Register to read 
 * @return uint8_t: The value in the register
 */
uint8_t MXC400xRead(const MXC400x* const dev, const MXC400xReg reg);

/**
 * @brief Writes to a regsiter on the device
 * 
 * @param[in] dev: Device to write to 
 * @param[in] regsiter: Register to write into 
 * @param[in] value: Value to write in to the register 
 */
uint8_t MXC400xWrite(const MXC400x* const dev, const MXC400xReg reg, const uint8_t value);

// --------------------------------------- Interrupt Functions ------------------------------------ //

/**
 * @brief Reads the interrupt sources register
 * 
 * @param[in] dev: Device to read from 
 * @return uint16_t: Interrupt Detection Mask 
 */
uint16_t MXC400xReadInt(const MXC400x* const dev);

/**
 * @brief Clears all of the pending interrupts
 * 
 * @param[in] dev: Device to clear the interrupts on 
 */
uint8_t MXC400xClearInt(const MXC400x* const dev);

/**
 * @brief Enables Interrupts on the device
 * 
 * @param[in] dev: Device to enable interrupts on
 * @param[in] int_mask: Interrupt Mask, a bunch of ORed MXC400XInterrupts
 * @return uint16_t: Status, 0 if there was an error
 */
uint8_t MXC400xEnableInt(const MXC400x* const dev, const uint16_t int_mask);

/**
 * @brief Disables a sequence of interrupts 
 * 
 * @param[in] dev: Device to disable the Interrupts on
 * @param[in] int_mask: The interrupts to disable 
 * @return uint16_t: Status, if there was an error
 */
uint8_t MXC400xDisableInt(const MXC400x* const dev, const uint16_t int_mask);

// -------------------------------------------- Utility Functions ----------------------------------- //

/**
 * @brief Software resets the device, clears all of the registers and sets them to their default values
 * 
 * @param[in] dev: Device to reset 
 * @return uint8_t: Status, 0 if there is an error 
 */
uint8_t MXC400xReset(const MXC400x* const dev);

/**
 * @brief Reads the unique device ID
 * 
 * @param[in] dev: Device to read the ID from 
 * @return uint8_t: The ID, 0 if there was an error
 */
inline uint8_t MXC400xReadID(const MXC400x* const dev) { return MXC400xRead(dev, ID);}

// ---------------------------------------------- Detection Setting -------------------------------------- //

/**
 * @brief Sets the detection parameters for shaking and orientation detection
 * 
 * @param[in] dev: Device to configure
 * @param[in] mode: Shake Detection Mode
 * @param[in] speed: Shake Speed to be Detected 
 * @param[in] thresh: How hard of a shake to detect 
 * @param[in] chor: How much orientation change to detect 
 * @return uint8_t: Status, 0 if there was an error 
 */
uint8_t MXC400xSetDetection(const MXC400x* const dev, const MXC400xShakeMode mode, const MXC400xShakeSpeed speed, const MXC400xShakeThresh thresh, const MXC400xOrientChange chor);

// -------------------------------------------- Raw Data Reading Functions ---------------------------------------------- //

/**
 * @brief Reads the raw int16 X Acceleration value from the device
 * 
 * @param[in] dev: Device to read the data from 
 * @return int16_t: The Raw X acceleration Value
 */
int16_t MXC400xReadXRaw(const MXC400x* const dev);

/**
 * @brief Reads the Raw int16 Y Value from the device
 * 
 * @param[in] dev: Device to read from  
 * @return int16_t: The Raw Y acceleration value 
 */
int16_t MXC400xReadYRaw(const MXC400x* const dev);

/**
 * @brief Reads the raw int16 Z acceleration value from the device
 * 
 * @param[in] dev: Device to read from 
 * @return int16_t: The raw z acceleration value  
 */
int16_t MXC400xReadZRaw(const MXC400x* const dev);

/**
 * @brief Reads the raw temperature value from the device
 * 
 * @param[in] dev: Device to read from  
 * @return int8_t: the raw temperature value 
 */
int8_t MXC400xReadTempRaw(const MXC400x* const dev);

/**
 * @brief Reads the X, Y, and Z acceleration as well as the temperature
 * 
 * @param[in] dev: Device to read from
 * @param[out] data: Where to put the read data
 * @return uint8_t: Status, 0 if there was an error 
 */
uint8_t MXC400xReadRawData(const MXC400x* const dev, MXC400xRawData* const data);

// ----------------------------------------------- Real Data Reading Functions --------------------------------------- //

/**
 * @brief Reads and scales the raw X value to be in actual units
 * 
 * @param[in] dev: Device to read from 
 * @return float: The X Acceleration in Gs 
 */
float MXC400xReadX(const MXC400x* const dev);

/**
 * @brief Reads the Y acceleration value in Gs
 * 
 * @param[in] dev: Device to read from 
 * @return float: The Y Acceleration  
 */
float MXC400xReadY(const MXC400x* const dev);

/**
 * @brief Reads the Z acceleration value in Gs
 * 
 * @param[in] dev: Device to read from 
 * @return float: The Z Acceleration 
 */
float MXC400xReadZ(const MXC400x* const dev);

/**
 * @brief Reads the temperature in celsius
 * 
 * @param[in] dev: Device to read from 
 * @return float: The Temperature 
 */
float MXC400xReadTemp(const MXC400x* const dev);

/**
 * @brief Reads the X, Y and Z Acceleration as well as the temp
 * 
 * @param[in] dev: Device to read from
 * @param[out] data: Where to put the data 
 * @return uint8_t: Status, 0 if there was an error 
 */
uint8_t MXC400xReadRealData(const MXC400x* const dev, MXC400xRealData* const data);


#endif

