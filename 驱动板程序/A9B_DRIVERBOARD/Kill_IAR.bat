echo off
 
set ROOT_PATH=%~dp0
set PRJ_PATH=%ROOT_PATH%Examples\SPI\sc16is750_polling\
 
echo -
echo Please check the path wether is right.
echo %ROOT_PATH%
echo %PRJ_PATH%
 
echo pause
 
cd Examples\UART\Polling\EWARM\Flash\Exe
DEL /F/S/Q *.sim
cd %ROOT_PATH%
 
 
cd %PRJ_PATH%\EWARM\Flash\List
DEL /F/S/Q *.map
cd %ROOT_PATH%
 
 
cd %PRJ_PATH%\EWARM\Flash\Obj
DEL /F/S/Q *.o
DEL /F/S/Q *.pbi
DEL /F/S/Q *.xcl
DEL /F/S/Q *.pbd
DEL /F/S/Q *.browse
DEL /F/S/Q *.linf
DEL /F/S/Q *.pbw
cd %ROOT_PATH%
 
 
cd %PRJ_PATH%\EWARM\settings
DEL /F/S/Q *.wsdt
DEL /F/S/Q *.dbgdt
DEL /F/S/Q *.dnx
cd %ROOT_PATH%
 
 
cd %PRJ_PATH%\EWARM
DEL /F/S/Q *.dep
cd %ROOT_PATH%
 
cd vs
DEL /F/S/Q *.ncb
attrib -h *.suo
DEL /F/S/Q *.suo
cd %ROOT_PATH%
 
cd vs\vs
rd /S/Q Debug
DEL /F/S/Q *.user
cd %ROOT_PATH%
 
 
timeout /t 15
————————————————
版权声明：本文为CSDN博主「zhuohui307317684」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/zhuohui307317684/java/article/details/102677865