#ifndef __AS5047P_H__
#define __AS5047P_H__
#include "foc_cfg.h"
#include "my_spi.h"



/*内容随外界变化的寄存器 可读不可写*/
#define NOP_ADDR      0x0000 //启动读取过程寄存器地址
#define ERRFL_ADDR    0x0001 //错误寄存器地址
#define PROG_ADDR     0x0003 //编程寄存器地址
#define DIAAGC_ADDR   0x3FFC //诊断和AGC寄存器地址
#define MAG_ADDR      0x3FFD //CORDIC寄存器地址
#define ANGLEUNC_ADDR 0x3FFE //无动态角度误差补偿的测量角度寄存器地址
#define ANGLECOM_ADDR 0x3FFF //带动态角度误差补偿的测量角度寄存器地址
 
/*配置选项寄存器 可读可写*/
#define ZPOSM         0x0016
#define ZPOSL         0x0017
#define SETTINGS1     0x0018
#define SETTINGS2     0x0019
 
/*读取数据命令*/
#define READ_ANGLECOM 0xFFFF
#define READ_NOP      0xC000
#define READ_ERRFL    0x4001



typedef struct __C_AS5047P c_as5047p;
typedef struct __C_AS5047P
{
    void* this;
	int (*get_Data)(const c_as5047p* this,float *Data);
	int (*get_Angle)(const c_as5047p* this,float *Angle);
	int (*get_mech_Angle)(const c_as5047p* this,float *mech_Angle);
	int (*get_Electrical_Angle)(const c_as5047p* this,float *Electrical_Angle);	
}c_as5047p;
c_as5047p as5047p_create(u8 spi_channal,gpio_type* cs_gpio,uint32_t cs_pin);
#endif
