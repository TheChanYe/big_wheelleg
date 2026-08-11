#ifndef __MAIN_H
#define __MAIN_H
#include "at32f403a_407.h"
#include "at32f403a_407_clock.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
//#include	"math.h"
#include "arm_math.h"

#include "log.h"
#include "sys_cfg.h"

#define E_OK             0  /*正常*/
#define E_PARAM         -1	/*参数错误*/
#define E_NULL          -2 	/*空指针*/
#define E_OUT_OF_MEMORY -3	/*超出内存*/
#define E_ERROR         -4	/*返回错误*/


#endif


