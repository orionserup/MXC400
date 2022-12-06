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

uint16_t MXC400xReadX(const MXC400x* const dev) { 
    
    uint16_t x;
    dev->i2c_reg_read(MXC400xADDRESS, XOUT_H, &x, 2);
    
    return x >> 4;
}