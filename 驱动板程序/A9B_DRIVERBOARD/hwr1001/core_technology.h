#ifndef __CORE_TECHNOLOGY__H
#define __CORE_TECHNOLOGY__H

#include "includes.h"
#include "stm32f1xx_hal.h"

//定义打印串口
#define Uart_Handle huart1
#define max(a, b)           (a>b? a:b)  
#define min(a, b)           (a<b? a:b)  
#define limiter(x, a, b)      (min(max(x, a), b))  
#define PI 3.14159265359
#define mycos(x)  cos((x*PI)/180)
#define mysin(x)  sin((x*PI)/180)

typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

uint8_t ReadByte_Bit (uint8_t data, uint8_t bit); //读位
uint8_t SetByte_Bit (uint8_t data, uint8_t bit, uint8_t value);//写位
void HexStrToByte (unsigned char* source, unsigned char* dest, int sourceLen);   //HEXTObyte
void ByteToHexStr (unsigned char* source, unsigned char* dest, int sourceLen);//ByteTOHEX
uint8_t Xor_Checking (unsigned char *data, uint8_t data_start, uint8_t len);
int fputc (int ch, FILE *f);
int fgetc (FILE *f);
#endif
