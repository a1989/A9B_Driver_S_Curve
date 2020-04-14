#ifndef __AT24C512_H
#define __AT24C512_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

extern uint8_t eep_i2c_status;

uint8_t I2C_EE_ByteWrite (uint8_t SlaveAddr, uint16_t WriteAddr, uint8_t WriteData);
uint32_t I2C_EE_ByteRead (uint8_t SlaveAddr, uint16_t ReadAddr);
void EEP_I2C_Test (void);
void At24c512_Init (void);
void  At24c512_WriteByte (uint16_t address, uint8_t data);
uint8_t At24c512_ReadByte (uint16_t address);
#endif
