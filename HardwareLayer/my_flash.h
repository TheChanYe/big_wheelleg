#ifndef __MY_FLASH_H
#define __MY_FLASH_H

#include "main.h"

/* 函数原型 */
int flash_read(uint32_t addr, void *buffer, uint16_t size);
int flash_write(uint32_t addr, uint16_t *data, uint16_t size);
int flash_erasepage(uint32_t page_address);
#endif 

