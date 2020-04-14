#include "at24c512.h"
#include "includes.h"

uint8_t eep_i2c_status = 0;

uint8_t I2C_EE_ByteWrite (uint8_t SlaveAddr, uint16_t WriteAddr, uint8_t WriteData)
{
    HAL_StatusTypeDef status = HAL_OK;

    status = HAL_I2C_Mem_Write (&EEPROM_I2C, SlaveAddr, WriteAddr, I2C_MEMADD_SIZE_16BIT, &WriteData, 1, 1000);
    /* Check the communication status */
    if(status != HAL_OK)
    {
        /* Execute user timeout callback */
        //I2Cx_Error(Addr);
        printf ("\r\n read status:%d", status);
    }
	eep_i2c_status = status;
    while (HAL_I2C_GetState (&EEPROM_I2C) != HAL_I2C_STATE_READY);
    /* Check if the EEPROM is ready for a new operation */
    while (HAL_I2C_IsDeviceReady (&EEPROM_I2C, SlaveAddr, EEPROM_MAX_TRIALS, I2Cx_TIMEOUT_MAX) == HAL_TIMEOUT);
    /* Wait for the end of the transfer */
    while (HAL_I2C_GetState (&EEPROM_I2C) != HAL_I2C_STATE_READY);
    return status;
}

uint32_t I2C_EE_ByteRead (uint8_t SlaveAddr, uint16_t ReadAddr)
{
	uint8_t  value;
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Read (&EEPROM_I2C, SlaveAddr, ReadAddr, I2C_MEMADD_SIZE_16BIT, &value, 1, 1000);
	if (status != HAL_OK)
	{
		printf ("\r\n read status:%d", status);
	}
	eep_i2c_status = status;
	return value;
}

void EEP_I2C_Test(void)
{
	uint8_t rdata;

	I2C_EE_ByteWrite (EEPROM_ADDRESS, 0x38, 0x54);
	I2C_EE_ByteWrite (EEPROM_ADDRESS, 0x1, 0xa2);
	I2C_EE_ByteWrite (EEPROM_ADDRESS, 0x2, 0xa3);
	I2C_EE_ByteWrite (EEPROM_ADDRESS, 0x3, 0xa4);
	rdata = I2C_EE_ByteRead (EEPROM_ADDRESS, 0x38);
	printf ("\r\n rdata:%x", rdata);
	rdata = I2C_EE_ByteRead (EEPROM_ADDRESS, 0x1);
	printf ("\r\n rdata:%x", rdata);
	rdata = I2C_EE_ByteRead (EEPROM_ADDRESS, 0x2);
	printf ("\r\n rdata:%x", rdata);
	rdata =I2C_EE_ByteRead (EEPROM_ADDRESS, 0x3);
	printf ("\r\n rdata:%x", rdata);
}

void At24c512_Init(void)
{
   // Eeprom_Parameter_Write();
   // EEP_I2C_Test();
}

void  At24c512_WriteByte (uint16_t address, uint8_t data)
{
    I2C_EE_ByteWrite (EEPROM_ADDRESS, address, data);
}

uint8_t At24c512_ReadByte (uint16_t address)
{
    uint8_t  value;

    value = I2C_EE_ByteRead (EEPROM_ADDRESS, address);
    return value;
}
