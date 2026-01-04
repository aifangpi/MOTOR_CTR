#include "usb.h"
#include "usart.h"
//#include "rtthread.h"
char UsbTxBuffer[200];
void usb_printf(const char *format, ...)
{
    va_list args;
    uint32_t length;
    va_start(args, format);
    length = vsnprintf((char *)UsbTxBuffer, 200, (char *)format, args);
    va_end(args);
    HAL_UART_Transmit(&huart1,UsbTxBuffer,length,0XFF);
}
