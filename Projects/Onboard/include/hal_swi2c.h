#ifndef _HAL_SWI2C_H_
#define _HAL_SWI2C_H_

#include "hal_types.h"

void  halI2CHWInit ( void ) ;

void  halI2CWrite( uint8 slaveAddr, uint8 regAddr,uint8 regData);

uint8 halI2CRead( uint8 slaveAddr,uint8 regAddr );

void  halI2CMulWrite( uint8 slaveAddr,uint8 regAddr,uint8 * buf,uint8 n );

void  halI2CMulRead( uint8 slaveAddr,uint8 regAddr, uint8 * buf,uint8 n );	

#endif /* _HAL_SWI2C_H_ */