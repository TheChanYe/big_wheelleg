/*****************************************************************************
* Copyright:
* File name: reg.c
* Description: 寄存器操作工具
* Author:许少
* Version: V1.0
* Date: 2022/01/14
*****************************************************************************/

#include "reg.h"

void be_out(gpio_type* gpio,uint32_t pin)
{
	u8 pin_num = 0;

	for(pin_num = 0;1 != pin;pin >>= 1)
	{
		++pin_num;
	}

	if(pin_num <= 7)
	{      
		gpio->cfglr &= ~(0x0000000F << (4 * pin_num));  /*清空功能*/
		gpio->cfglr |= (u32)1 << (4 * pin_num);   /*设置为输出模式*/
	}
	else
	{
        pin_num -= 8;
		gpio->cfghr &= ~(0x0000000F << (4 * pin_num));  /*清空功能*/
		gpio->cfghr |= (u32)1 << (4 * pin_num);   /*设置为输出模式*/
	}
}

void be_in(gpio_type* gpio,uint32_t pin)
{
	u8 pin_num = 0;

	for(pin_num = 0;1 != pin;pin >>= 1)
	{
		++pin_num;
	}

	if(pin_num <= 7)
	{
		gpio->cfglr &= ~(0x0000000F << (4 * pin_num));  /*清空功能*/
		gpio->cfglr |= (u32)8 << (4 * pin_num);   /*设置为输入模式*/
	}
	else
	{
        pin_num -= 8;
		gpio->cfghr &= ~(0x0000000F << (4 * pin_num));  /*清空功能*/
		gpio->cfghr |= (u32)8 << (4 * pin_num);   /*设置为输入模式*/
	}
}

#if 0
//外部中断配置函数
//只针对GPIOA~G;不包括PVD,RTC和USB唤醒这三个
//参数:
//GPIOx:0~6,代表GPIOA~G
//BITx:需要使能的位;
//TRIM:触发模式,1,下升沿;2,上降沿;3，任意电平触发
//该函数一次只能配置1个IO口,多个IO口,需多次调用
//该函数会自动开启对应中断,以及屏蔽线   	    
void exti_cfg(gpio_type* gpio,uint32_t pin,uint32_t mode) 
{
	u8 EXTADDR;
	u8 EXTOFFSET;
    u8 gpiox = 0;
    u8 bitx = 0;
    
    for(bitx = 0;1 != pin;pin >>= 1)
	{
		++bitx;
	}
    
    if(GPIOA == gpio)
    {
        gpiox = 0;
    }
    else if(GPIOB == gpio)
    {
        gpiox = 1;
    }
    else if(GPIOC == gpio)
    {
        gpiox = 2;
    }
    
	EXTADDR=bitx/4;//得到中断寄存器组的编号
	EXTOFFSET=(bitx%4)*4; 
	IOMUX->evtout[EXTADDR]&=~(0x000F<<EXTOFFSET);//清除原来设置！！！
	IOMUX->evtout[EXTADDR]|= gpiox<<EXTOFFSET;//EXTI.BITx映射到GPIOx.BITx 
    
	//自动设置
	//EXTI->IMR|=1<<bitx;//  开启line BITx上的中断
	//EXTI->EMR|=1<<BITx;//不屏蔽line BITx上的事件 (如果不屏蔽这句,在硬件上是可以的,但是在软件仿真的时候无法进入中断!)
    
 	if(GPIO_MODE_IT_FALLING == mode)
    {
        EXTI->FTSR|=1<<bitx;//line BITx上事件下降沿触发
    }
	else if(GPIO_MODE_IT_RISING == mode)
    {
        EXTI->RTSR|=1<<bitx;//line BITx上事件上升降沿触发
    }
    else if(GPIO_MODE_IT_RISING_FALLING == mode)
    {
        EXTI->FTSR|=1<<bitx;//line BITx上事件下降沿触发
        EXTI->RTSR|=1<<bitx;//line BITx上事件上升降沿触发
    }
} 	



void exti_set_follow(uint32_t pin,uint32_t mode)
{
    if(GPIO_MODE_IT_FALLING == mode)
    {
        EXTI->FTSR|=pin;//line BITx上事件下降沿触发
        EXTI->RTSR&=~pin;
    }
	else if(GPIO_MODE_IT_RISING == mode)
    {
        EXTI->RTSR|=pin;//line BITx上事件上升降沿触发
        EXTI->FTSR&=~pin;
    }
    else if(GPIO_MODE_IT_RISING_FALLING == mode)
    {
        EXTI->FTSR|=pin;//line BITx上事件下降沿触发
        EXTI->RTSR|=pin;//line BITx上事件上升降沿触发
    }
}

uint32_t exti_get_follow(uint32_t pin)
{
    if((EXTI->RTSR & pin) && (EXTI->FTSR & pin))
    {
        return GPIO_MODE_IT_RISING_FALLING;
    }
    else if(!(EXTI->RTSR & pin) && (EXTI->FTSR & pin))
    {
        return GPIO_MODE_IT_FALLING;
    }
    else
    {
        return GPIO_MODE_IT_RISING;
    }
}
#endif

