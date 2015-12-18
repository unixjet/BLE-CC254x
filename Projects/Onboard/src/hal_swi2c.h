#ifndef _HAL_SWI2C_H_
#define _HAL_SWI2C_H_

#include "hal_types.h"

void  HalI2CHWInit() ;

void  HalI2CWrite( uint8 slaveAddr, uint8 regAddr,uint8 regData);

uint8 HalI2CRead( uint8 slaveAddr,uint8 regAddr );

void  HalI2CMulWrite( uint8 slaveAddr,uint8 regAddr,uint8 * buf,uint8 n );

void  HalI2CMulRead( uint8 slaveAddr,uint8 regAddr, uint8 * buf,uint8 n );	

#endif /* _HAL_SWI2C_H_ */