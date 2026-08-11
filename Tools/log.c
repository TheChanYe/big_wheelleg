/*****************************************************************************
* Copyright: 2019-2020
* File name: log.c
* Description: 日志模块函数实现
* Author: 许智豪
* Version: v1.0
* Date: xxx
*****************************************************************************/
#include "log.h"
#ifdef MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG "log"

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  

#define ITM_PORT8(n)         (*(volatile unsigned char *)(0xe0000000 + 4*(n)))
#define ITM_PORT16(n)        (*(volatile unsigned short *)(0xe0000000 + 4*(n)))
#define ITM_PORT32(n)        (*(volatile unsigned long *)(0xe0000000 + 4*(n)))
#define DEMCR                (*(volatile unsigned long *)(0xE000EDFC))
#define TRCENA               0X01000000

#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;   
FILE __stdin; 
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 
#if 0	
	while((USART2->SR&0X40)==0); 
    USART2->DR = (u8) ch;      
	return ch;	
#else 
	 if(DEMCR & TRCENA)
    {
        while(ITM_PORT32(0) == 0);                                                                                                                                                                                                                                                                                      
        ITM_PORT8(0) = ch;
    }
    return ch;
#endif
}

/*************************************************
* Function: log_init
* Description: 日志系统初始化
* Input : 无
* Output: 无
* Return: 无
* Others: 无
*************************************************/
int log_init(void)
{     
    log_debug("Log sys init successful!");

    return E_OK;
}
