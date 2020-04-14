#ifndef __CONTROL__H
#define __CONTROL__H
#include "includes.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

/*等待超时时间*/
#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))
#define I2Cx_TIMEOUT_MAX                300
#define EEPROM_MAX_TRIALS               300

#define ECDPeriod      0xffff
#define ECDPrescaler   0

#define prd     ECDPeriod
#define Vbreak  ECDPeriod / 2

#define PWMPeriod      0xFFFF         //10k HZ
#define PWMLOSS        65000    //最大90%占空比

/*****DRV8711******/
#define DRV8711_SPI  hspi2

#define StepMotor_TIM  htim2
/*****htimx_Encoder******/
#define htimx_Encoder  htim3

/*****husart_debug******/
#define husart_debug  huart1

/****eeprom*****/
#define EEPROM_I2C     hi2c2
#define EEPROM_ADDRESS 0xA0

#define DRIVER_TO_MAIN_DATA_LEN 8
#define MAIN_TO_DRIVER_DATA_LEN 8

#define UART1_RX_BUFF_LEN 255
#define UART1_TX_BUFF_LEN 255

#define UARTI_DEBUG_DATA_FRAME_START 0xFF
#define UARTI_DEBUG_DATA_FRAME_SECOND 0xFF
#define UARTI_DEBUG_DATA_LEN 0xFF

#define Start_Position_Read() HAL_GPIO_ReadPin (GPIOB, Position_END_Pin)
#define End_Position_Read()   HAL_GPIO_ReadPin (GPIOB, Position_START_Pin)

#define MOTOR_MAXPLUS 4000000
#define MOTOR_MINPLUS -4000000

extern uint8_t can_rbuff[MAIN_TO_DRIVER_DATA_LEN];
extern uint8_t DRIVER_TO_MAIN_DATA[DRIVER_TO_MAIN_DATA_LEN]; // 驱动板到主控板的数据
extern uint8_t MAIN_TO_DRIVER_DATA[MAIN_TO_DRIVER_DATA_LEN]; // 主控板到驱动板的数据 
extern uint8_t UARTI_DEBUG_DATA[UARTI_DEBUG_DATA_LEN];

extern uint8_t uart1_rxbuff[UART1_RX_BUFF_LEN];
extern uint8_t uart1_rdata[UART1_RX_BUFF_LEN];
extern uint8_t uart1_tdata[UART1_TX_BUFF_LEN];
extern uint16_t uart1_rxdata_cnt;
extern uint8_t uart1_debug_data_len;//调试串口数据长度
extern uint8_t uart1_receive_byte;
extern uint8_t uart1_Receive_Right_flag;
extern uint8_t driver_over_current_flag;
extern  float stepmotor_RM_Kp;
extern  float stepmotor_RM_Ki;
extern  float stepmotor_RM_Kd;
extern  float stepmotor_RM_Lim;
extern  uint16_t driver_can_stdid;
extern  uint8_t main_request_data_flag;
extern  uint8_t led_display_cnt_flag;//led闪烁任务
extern  uint8_t can_send_data_cnt_flag;//CAN发送数据标志
extern  uint8_t location_write_cnt_flag;//写电机位置标志
extern  uint8_t soft_reset_flag;
extern  uint8_t motor_maxspeed;
extern  uint8_t motor_stop_flag;
extern  uint16_t eep_location_addr;

extern CanTxMsgTypeDef TxMessage;
extern CanRxMsgTypeDef RxMessage;

extern uint8_t can_Receive_Right_flag;
extern uint8_t bsp_init_flag;
extern uint8_t driver_board_enable_flag;
extern uint8_t driver_error_flag;
extern uint8_t limit_stop_flag;
extern uint8_t motor_enable_flag;
extern uint8_t motor_select_flag;    // 1:丝杆电机  0：关节电机
extern uint8_t percircle_distance;
extern float screw_distance;
extern int32_t Aim_Location;
extern float Current_Location;
extern uint8_t motor_limit_flag;
extern s32 write_temp;
extern s32 loc_temp;

extern uint32_t ADC_Get_Info[3];
extern float ADC_Current_Value;
extern float ADC_Voltage_Value;
extern float ADC_Temperature_Value;
extern s32 Location_Cnt_tmp;
extern uint16_t location_addr;
extern uint8_t limit_start_done;
extern uint8_t limit_end_done;
extern s32 screw_maxdistance;
extern s32 screw_mindistance;
extern uint8_t motor_move_direction_flag;//运动正方向标志位
extern uint8_t encoder_cnt_direction_flag;
extern uint8_t iwdg_system_flag; //系统控制函数看门狗标志位
extern uint8_t iwdg_motor_ctrl_flag;
extern uint8_t motor_limit_cnt_flag;
extern uint8_t motor_speed;

void Iwdg_Updata (void);
void System_Control (void);
void CAN_Receive_Data_Analysis (void); // CAN接收的数据解析
void Check_Location (void);
void CAN_Filter_Config_Scale16_IdList (void);
void RxMes_Init (void);
void CAN_SetMsg (void);
void Send_Driver_To_Mian_Data (void);
void Driver_Can_Init (void);
void Motor_Location_Write (void);
void Motor_Location_Read (void);
void Ina201_ADC_Get (void);
void Driver_Board_Parameter_Init (void);
void Eep_Data_Update (void);
void Limit_Switch_Scanning (void);
void Check_Location (void);
void CAN_SendData_Acquisition (void);
#endif
