#ifndef __DELAY_H
#define __DELAY_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
/* delay function */
void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void delay_sec(uint16_t sec);

#ifdef __cplusplus
}
#endif

#endif

