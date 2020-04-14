#include "includes.h"
#include "core_technology.h"

//字节转十六进制字符
void ByteToHexStr (unsigned char* source, unsigned char* dest, int sourceLen)
{
    short i;
    unsigned char highByte, lowByte;

    for (i = 0; i < sourceLen; i++)
    {
        highByte = source[i] >> 4;
        lowByte = source[i] & 0x0f ;
        highByte += 0x30;
        if (highByte > 0x39)
            dest[i * 2] = highByte + 0x07;
        else
            dest[i * 2] = highByte;
        lowByte += 0x30;
        if (lowByte > 0x39)
            dest[i * 2 + 1] = lowByte + 0x07;
        else
            dest[i * 2 + 1] = lowByte;
    }
    return ;
}

//十六进制字符转字节流
void HexStrToByte (unsigned char* source, unsigned char* dest, int sourceLen)   //HEXTObyte
{
    short i;
    unsigned char highByte, lowByte;

    for (i = 0; i < sourceLen; i += 2)
    {
        highByte = toupper (source[i]);
        lowByte  = toupper (source[i + 1]);
        if (highByte > 0x39)
            highByte -= 0x37;
        else
            highByte -= 0x30;
        if (lowByte > 0x39)
            lowByte -= 0x37;
        else
            lowByte -= 0x30;
        dest[i / 2] = (highByte << 4) | lowByte;
    }
    return ;
}
//读位
uint8_t ReadByte_Bit (uint8_t data, uint8_t bit)
{
    uint8_t a = 0xFF;
    uint8_t value;

    if (bit <= 32)
    {
        value = (data >> bit) & (~((a >> bit) - 1));
    }
    return value;
}
//写位
uint8_t SetByte_Bit (uint8_t data, uint8_t bit, uint8_t value)
{
    uint8_t n = 1;
    n = n << bit;
    if (bit <= 8)
    {
        if (value == 0)
        {
            data = data & (~n);
        }
        else if (value == 1)
        {
            data = data | n;
        }
    }
    else
        printf ("\r\n value error\r\n");
    return data;
}
//异或校验
uint8_t Xor_Checking (unsigned char *data, uint8_t data_start, uint8_t len)
{
    uint8_t XOR = 0;
    uint8_t i = 0;

    for (i = data_start; i < len - 1; i++)
    {
        XOR ^= data[i];
    }
    return XOR;
}

int fputc (int ch, FILE *f)
{
    /* 发送一个字节数据到串口RS232_USART */
    HAL_UART_Transmit (&Uart_Handle, (uint8_t *)&ch, 1, 1000);
    return (ch);
}

int fgetc (FILE *f)
{
    int ch;
    HAL_UART_Receive (&Uart_Handle, (uint8_t *)&ch, 1, 1000);
    return (ch);
}
