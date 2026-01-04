#ifndef __USB_H__
#define __USB_H__

#include "stdio.h"
#include "stdint.h"
#include "stdarg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

void usb_printf(const char *format, ...);
void news_usb_printf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* __MYUSB_H__ */
