#include "drv8711.h"
#include "includes.h"

uint16_t eep_drv8711_ctrl_value;
uint16_t eep_drv8711_torque_value;
uint16_t eep_drv8711_off_value;
uint16_t eep_drv8711_blank_value;
uint16_t eep_drv8711_decay_value;
uint16_t eep_drv8711_stall_value;
uint16_t eep_drv8711_drive_value;
uint16_t eep_drv8711_status_value;

uint16_t drv8711_ctrl_value = 0x0c22;
uint16_t drv8711_torque_value = 0x01ff;
uint16_t drv8711_off_value = 0x0040;//14
uint16_t drv8711_blank_value = 0x0040; //40
uint16_t drv8711_decay_value = 0x0110;
uint16_t drv8711_stall_value = 0x0c10;
uint16_t drv8711_drive_value = 0x0A59;
uint16_t drv8711_status_value = 0x0000;

uint8_t drv8711_error_status = 0;
uint8_t motor_step_value = 32;//细分设置，默认16细分
float drv_motor_ifs = 0; // 电机电流
__IO float drv_r_sense = 0.1; //采样电阻
uint8_t drv_isgain = 5;   // 设置增益
uint8_t drv_torque_value = 0xff;

void Drv8711_Init (void)
{
    Drv8711_Reset(); //上电复位
    Drv8711_Sleep_Disable(); //解除睡眠模式
    //写默认参数
    SPI_DRV8711_Write (CTRL_Register_ADDR, drv8711_ctrl_value);
    Delay_ms(1);
    SPI_DRV8711_Write (TORQUE_Register_ADDR, drv8711_torque_value);
    Delay_ms(1);
    SPI_DRV8711_Write (OFF_Register_ADDR, drv8711_off_value);
    Delay_ms(1);
    SPI_DRV8711_Write (BLANK_Register_ADDR, drv8711_blank_value);
    Delay_ms(1);
    SPI_DRV8711_Write (DECAY_Register_ADDR, drv8711_decay_value);
    Delay_ms(1);
    SPI_DRV8711_Write (STALL_Register_ADDR, drv8711_stall_value);
    Delay_ms(1);
    SPI_DRV8711_Write (DRIVE_Register_ADDR, drv8711_drive_value);
    Delay_ms(1);
    SPI_DRV8711_Write (STATUS_Register_ADDR, drv8711_status_value);

    Drv8711_Step_Set (motor_step_value); // 设置细分
    Delay_ms(1);
    Drv8711_ISGAIN_Set (drv_isgain);  // 设置增益
    Delay_ms(1);
    Drv8711_TORQUE_Set (0); // 设置力矩
    Delay_ms(1);
    //Drv8711_TORQUE_Set(drv_torque_value); // 设置力矩
    //Delay_ms(1);
    Drv8711_Reg_Read(); // 写入的数据在都出来
    //STEPMOTOR_OUTPUT_DISABLE();
    STEPMOTOR_OUTPUT_ENABLE();//这里设置输出使能才能使能力矩
}

//计算drv设置的电流
float Drv8711_Ifs_Set (uint8_t torque)
{
    float ifs = 0;

    ifs = (2.75 * torque) / (256 * drv_isgain * drv_r_sense);
    return ifs;
}
// 设置消隐时间
uint16_t Drv8711_BLANKTIME_Set (uint8_t value)
{
    uint16_t tmp0,tmp1;

    tmp0 = drv8711_blank_value;
    tmp1 = value;
    tmp0 = (tmp0 & 0xff00) | tmp1;
    drv8711_blank_value = tmp0;
    SPI_DRV8711_Write (BLANK_Register_ADDR, tmp0);
    return tmp0;
}

//设置力矩
uint16_t Drv8711_TORQUE_Set (uint8_t value)
{
    uint16_t tmp0;
    uint8_t  tmp1;

    tmp0 = drv8711_torque_value;
    tmp1 = value;
    tmp0 = (tmp0 & 0xff00) | tmp1;
    drv8711_torque_value = tmp0;
    SPI_DRV8711_Write (TORQUE_Register_ADDR, tmp0);
    return tmp0;
}

//设置增益
uint16_t Drv8711_ISGAIN_Set (uint8_t value)
{

    uint16_t tmp0,tmp1;

    tmp0 = drv8711_ctrl_value;
    switch (value)
    {
        case 5:
				tmp1 = 0x0; // 5倍增益
            break;
        case 10:
				tmp1 = 0x1; // 10倍增益
            break;
        case 20:
				tmp1 = 0x2; // 20倍增益
            break;
        case 40:
				tmp1 = 0x3; // 40倍增益
            break;
        default:
				tmp1= tmp1;
    }
    tmp0 = tmp0 & 0xfcff;
    tmp0 = tmp0 | (tmp1 << 8);
    drv8711_ctrl_value = tmp0;
    SPI_DRV8711_Write (CTRL_Register_ADDR, tmp0);
    return tmp0;
}
// 设置细分
uint16_t Drv8711_Step_Set (uint8_t value)
{

    uint16_t tmp0,tmp1;

    tmp0 = drv8711_ctrl_value;
    switch (value)
    {
        case 2:
				tmp1 = 0x01;//2细分
            break;
        case 4:
				tmp1 = 0x02;//4细分
            break;
        case 8:
				tmp1 = 0x03;//8细分
            break;
        case 16:
				tmp1 = 0x04;//16细分
            break;
        case 32:
				tmp1 = 0x05;//32细分
            break;
        case 64:
				tmp1 = 0x06;//64细分
            break;
        case 128:
				tmp1 = 0x07;//128细分
            break;
        case 256:
				tmp1 = 0x08;//256细分
            break;
        default:
				tmp1= 0x05;
    }
    tmp0 = tmp0 & 0xff87;
    tmp0 = tmp0 | (tmp1 << 3);
    drv8711_ctrl_value = tmp0;
	//printf ("\r\n %x", drv8711_ctrl_value);
    SPI_DRV8711_Write (CTRL_Register_ADDR, drv8711_ctrl_value);
    return tmp0;
}

void Drv8711_Reg_Read (void)
{
    uint16_t value;

    value = SPI_DRV8711_Read (CTRL_Register_ADDR);
    printf ("\r\n CTRL:0x%x", value);
    value = SPI_DRV8711_Read (TORQUE_Register_ADDR);
    printf ("\r\n TORQUE:0x%x", value);
    value = SPI_DRV8711_Read (OFF_Register_ADDR);
    printf ("\r\n OFF:0x%x", value);
    value = SPI_DRV8711_Read (BLANK_Register_ADDR);
    printf ("\r\n BLANK:0x%x", value);
    value = SPI_DRV8711_Read (DECAY_Register_ADDR);
    printf ("\r\n DECAY:0x%x", value);
    value = SPI_DRV8711_Read (STALL_Register_ADDR);
    printf ("\r\n STALL:0x%x", value);
    value = SPI_DRV8711_Read (DRIVE_Register_ADDR);
    printf ("\r\n DRIVE:0x%x", value);
    value = SPI_DRV8711_Read (STATUS_Register_ADDR);
    printf ("\r\n STATUS:0x%x", value);
}

//读取状态寄存器
uint16_t Get_Drv8711_Statu (void)
{
    uint8_t value;
    uint8_t bit;

    value = (uint8_t)SPI_DRV8711_Read (STATUS_Register_ADDR);
    if (value != 0) 
	{
//       printf ("\r\n STATUS:0x%x", value);
       bit = ReadByte_Bit (value, 1);
       if (bit)
       {
           //printf ("\r\n A相过流 ");
           driver_over_current_flag = 1;
       }
       bit = ReadByte_Bit (value, 2);
       if (bit)
       {
           //printf ("\r\n B相过流 ");
           driver_over_current_flag = 1;
       }
       bit = ReadByte_Bit (value, 5);
       if(bit)
       {
           //printf ("\r\n 欠压锁定 ");
           drv8711_error_status = 1;
       }
       bit = ReadByte_Bit (value, 7);
       if ((bit) && (motor_stop_flag == 0))
       {
           //printf ("\r\n 电机失速 ");
           drv8711_error_status = 1;
       }
       bit = 0;        
       Delay_ms (1);
       SPI_DRV8711_Write (STATUS_Register_ADDR, 0x0);
	}
    else
    {
        //driver_over_current_flag = 0;
        drv8711_error_status = 0;
    }
    return value;
}
//复位DRV8711
void Drv8711_Reset (void)
{
    DRV8711_RESET_PIN_HIGH();
    Delay_ms (5);
    DRV8711_RESET_PIN_LOW();
    Delay_ms (5);
}
//使能睡眠模式
void Drv8711_Sleep_Enable (void)
{
    Delay_ms (5);
    DRV8711_SLEEP_PIN_LOW();
    Delay_ms (5);
}

//解除睡眠模式
void Drv8711_Sleep_Disable (void)
{
    Delay_ms (5);
    DRV8711_SLEEP_PIN_HIGH();
    Delay_ms (5);
}

/**
  * 函数功能: 从DRV8711读取一个字节数据
  * 输入参数: 无
  * 返 回 值: uint8_t：读取到的数据
  * 说    明：This function must be used only if the Start_Read_Sequence
  *           function has been previously called.
  */
uint8_t SPI_DRV8711_ReadByte (void)
{
    uint8_t d_read,d_send = Dummy_Byte;

    if (HAL_SPI_TransmitReceive (&DRV8711_SPI, &d_send, &d_read, 1, 0xFFFFFF) != HAL_OK)
        d_read = Dummy_Byte;
    return d_read;
}

/**
  * 函数功能: DRV8711读取写入一个字节数据并接收一个字节数据
  * 输入参数: byte：待发送数据
  * 返 回 值: uint8_t：接收到的数据
  * 说    明：无
  */
uint8_t SPI_DRV8711_SendByte (uint8_t byte)
{
    uint8_t d_read,d_send = byte;

    if (HAL_SPI_TransmitReceive (&DRV8711_SPI, &d_send, &d_read, 1, 0xFFFFFF) != HAL_OK)
        d_read = Dummy_Byte;
    return d_read;
}

uint16_t SPI_DRV8711_Read (uint8_t addr)
{
    uint8_t Temp = 0, Temp0 = 0, Temp1 = 0;
    uint16_t Temp2;

    /* Select the FLASH: Chip Select low */
    SPI_DRV8711_CS_HIGH();
    Temp = (0x08  |addr) << 4;
    /* Read a byte from the FLASH */
    Temp0 = SPI_DRV8711_SendByte (Temp);
    /* Read a byte from the FLASH */
    Temp1 = SPI_DRV8711_SendByte (Dummy_Byte);
    /* Deselect the FLASH: Chip Select high */
    SPI_DRV8711_CS_LOW();
    Temp2 = ((Temp0 & 0x0f) << 8) | Temp1;
    return Temp2;
}

void SPI_DRV8711_Write (uint8_t addr, uint16_t data)
{
    uint8_t Temp = 0;
    /* Select the FLASH: Chip Select low */
    SPI_DRV8711_CS_HIGH();
    Temp = ((data >> 8) & 0x0f) | (addr << 4);
    SPI_DRV8711_SendByte (Temp);
    Temp = (uint8_t)(data & 0x00ff);
    SPI_DRV8711_SendByte (Temp);
    /* Deselect the FLASH: Chip Select high */
    SPI_DRV8711_CS_LOW();
}
