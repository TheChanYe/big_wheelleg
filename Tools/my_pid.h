/*************************************************************************
* Copyright:  Coruitech Technology Co. All Rights Reserved
* Author:     jcao@coruitech.com
* Version:    V1.0
* Date:       2018/2/26
* Description:
* 
*PID模块
* 
* 定义标准PID参数和PID计算接口
************************************************************************/


#ifndef __CR_PID_H__
#define __CR_PID_H__

#include "sys.h"

#define SHOW_PID_EFFECT 1 

typedef struct __MY_PID c_my_pid;

typedef struct __MY_PID
{
    void* this;
    int (*realize)(c_my_pid* this,float des_v, float real_v,float* out);
    int (*reset_param)(c_my_pid* this,float kp,float ki,float kd,float i_max,float i_min);
}c_my_pid;

c_my_pid my_pid_create(float kp,float ki,float kd,float i_max,float i_min);

#endif
