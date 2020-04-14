#ifndef __DRV8711_H
#define __DRV8711_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#define CTRL_Register_ADDR               0x00
#define TORQUE_Register_ADDR             0x01
#define OFF_Register_ADDR                0x02
#define BLANK_Register_ADDR              0x03
#define DECAY_Register_ADDR              0x04
#define STALL_Register_ADDR              0x05
#define DRIVE_Register_ADDR              0x06
#define STATUS_Register_ADDR             0x07
#define Dummy_Byte                       0xFF

#define SPI_DRV8711_CS_HIGH()   HAL_GPIO_WritePin (SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)
#define SPI_DRV8711_CS_LOW()    HAL_GPIO_WritePin (SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)

#define DRV8711_SLEEP_PIN_HIGH()   HAL_GPIO_WritePin (Motor_nSLEEP_GPIO_Port, Motor_nSLEEP_Pin, GPIO_PIN_SET)
#define DRV8711_SLEEP_PIN_LOW()    HAL_GPIO_WritePin (Motor_nSLEEP_GPIO_Port, Motor_nSLEEP_Pin, GPIO_PIN_RESET)

#define DRV8711_RESET_PIN_HIGH()   HAL_GPIO_WritePin (Motor_RESET_GPIO_Port, Motor_RESET_Pin, GPIO_PIN_SET)
#define DRV8711_RESET_PIN_LOW()    HAL_GPIO_WritePin (Motor_RESET_GPIO_Port, Motor_RESET_Pin, GPIO_PIN_RESET)

extern uint16_t drv8711_ctrl_value;
extern uint16_t drv8711_torque_value;
extern uint16_t drv8711_off_value;
extern uint16_t drv8711_blank_value;
extern uint16_t drv8711_decay_value;
extern uint16_t eep_drv8711_stall_value;
extern uint16_t drv8711_drive_value;
extern uint16_t drv8711_status_value;

extern uint8_t  drv8711_error_status;
extern uint8_t  motor_step_value;

void SPI_DRV8711_Write (uint8_t addr, uint16_t data);
uint16_t SPI_DRV8711_Read (uint8_t addr);
uint8_t SPI_DRV8711_SendByte (uint8_t byte);
uint8_t SPI_DRV8711_ReadByte (void);
void Drv8711_Init (void);
void Drv8711_Reset (void);
void Drv8711_Sleep_Enable (void);
void Drv8711_Sleep_Disable (void);
uint16_t Get_Drv8711_Statu (void);
uint16_t Drv8711_Step_Set (uint8_t value);
void Drv8711_Reg_Read (void);
uint16_t Drv8711_TORQUE_Set (uint8_t value);
uint16_t Drv8711_BLANKTIME_Set (uint8_t value);
uint16_t Drv8711_ISGAIN_Set (uint8_t value);
float Drv8711_Ifs_Set (uint8_t torque);
#endif
