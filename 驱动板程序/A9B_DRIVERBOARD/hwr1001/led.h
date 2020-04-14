#ifndef __LED_H
#define __LED_H

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#define    ON 0
#define    OFF 1

#define LEDR(a) if (a)  \
                    HAL_GPIO_WritePin(SIGNAL_LED_R_GPIO_Port, SIGNAL_LED_R_Pin,GPIO_PIN_SET);\
                    else        \
                    HAL_GPIO_WritePin(SIGNAL_LED_R_GPIO_Port, SIGNAL_LED_R_Pin,GPIO_PIN_RESET)
#define LEDB(a) if (a)  \
                    HAL_GPIO_WritePin(SIGNAL_LED_B_GPIO_Port, SIGNAL_LED_B_Pin,GPIO_PIN_SET);\
                    else        \
                    HAL_GPIO_WritePin(SIGNAL_LED_B_GPIO_Port, SIGNAL_LED_B_Pin,GPIO_PIN_RESET)
#define LEDG(a) if (a)  \
                    HAL_GPIO_WritePin(SIGNAL_LED_G_GPIO_Port, SIGNAL_LED_G_Pin,GPIO_PIN_SET);\
                    else        \
                     HAL_GPIO_WritePin(SIGNAL_LED_G_GPIO_Port, SIGNAL_LED_G_Pin,GPIO_PIN_RESET)
#define LED_RUN_TOGGLE                HAL_GPIO_TogglePin (SIGNAL_LED_G_GPIO_Port, SIGNAL_LED_G_Pin)
#define LEDB_TOGGLE                   HAL_GPIO_TogglePin (SIGNAL_LED_B_GPIO_Port, SIGNAL_LED_B_Pin)
#define LEDR_TOGGLE                   HAL_GPIO_TogglePin (SIGNAL_LED_R_GPIO_Port, SIGNAL_LED_R_Pin)

void LED_Running (void);
void Get_Driver_status (void);
void Signal_LED_Control (void);
#endif
