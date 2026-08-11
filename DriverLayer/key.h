/*****************************************************************************
* Copyright:
* File name: switch.h
* Description: 开关模块公开头文件
* Author: 许少
* Version: V1.2
* Date: 2022/04/08
* log:  V1.0  2022/04/08
*       发布：
*
*       V1.1  2022/04/02
*       新增：power接口中 相同功率退出 没有先给this指针赋值再判断。
*
*       V1.2  2022/04/08
*       新增：SWITHC_POWER_INTERVAL 开关频率控制宏
*****************************************************************************/

#ifndef _KEY_H
#define _KEY_H

#include "sys.h"

#define KEY_HIGHT  TRUE
#define KEY_LOW    FALSE

#define KEY_TYPE   flag_status

typedef struct __KEY c_key;

typedef struct __KEY
{
	void*             this;
    
    /************************************************* 
    * Function: set 
    * Description: 设置开关电平状态
    * Input : <c_switch>      开关对象
    *         <GPIO_PinState> IO口状态
    *                         (SWITCH_HIGHT)  高电平
    *                         (SWITCH_LOW)    低电平
    * Output: 无
    * Return: <E_OK>     操作成功
    *         <E_NULL>   空指针
    *         <E_ERROR>  操作失败
    * Others: 无
    * Demo  :
    *         ret = led.set(&led,SWITCH_HIGHT);
    *         if(E_OK != ret )
    *         {
    *             log_error("Swtich set failed.");
    *         }
    *************************************************/      

    /************************************************* 
    * Function: get 
    * Description: 获取开关电平状态
    * Input : <c_switch>      开关对象
    * Output: <GPIO_PinState> IO口状态
    *                         (SWITCH_HIGHT)  高电平
    *                         (SWITCH_LOW)    低电平
    * Return: <E_OK>     操作成功
    *         <E_NULL>   空指针
    *         <E_ERROR>  操作失败
    * Others: 无
    * Demo  :
    *         SWITCH_TYPE state = 0;
    *
    *         ret = led.get(&led,&state);
    *         if(E_OK != ret )
    *         {
    *             log_error("Swtich get failed.");
    *         }
    *************************************************/  
    int   (*get)     (const c_key* this,KEY_TYPE* state);
    
    /************************************************* 
    * Function: flicker 
    * Description: 开关闪烁
    * Input : <c_switch>  开关对象
    *         <time>      闪烁的时间间隔
    * Output: 无
    * Return: <E_OK>     操作成功
    *         <E_NULL>   空指针
    *         <E_ERROR>  操作失败
    * Others: 无
    * Demo  :
    *
    *         ret = led.flicker(&led,100);
    *         if(E_OK != ret )
    *         {
    *             log_error("Swtich set failed.");
    *         }
    *************************************************/  
    
    /************************************************* 
    * Function: flicker 
    * Description: 开关闪烁
    * Input : <c_switch>  开关对象
    *         <power>     功率系数 0~9  0最小，9最大
    * Output: 无
    * Return: <E_OK>     操作成功
    *         <E_NULL>   空指针
    *         <E_ERROR>  操作失败
    * Others: 无
    * Demo  :
    *
    *         ret = led.power(&led,5);
    *         if(E_OK != ret )
    *         {
    *             log_error("Swtich set failed.");
    *         }
    *************************************************/
}c_key;

/************************************************* 
* Function: switch_create 
* Description: 创建一个新的开关对象
* Input : <gpio>  开关所在gpio
*         <pin>   开关所在pin脚
* Output: 无
* Return: <E_OK>     操作成功
*         <E_NULL>   空指针
*         <E_ERROR>  操作失败
* Others: 无
* Demo  :
*         c_switch led = {0}；
*
*         led = switch_create(GPIOC,GPIO_PINS_13);
*         if(NULL == led.this)
*         {
*             log_error("Swtich creat failed.");
*         }
*************************************************/
c_key key_create(gpio_type* gpio,uint32_t pin);


#endif
