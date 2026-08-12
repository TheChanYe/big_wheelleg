/**
 * 文件用途：基础系统状态监控接口。
 * 所属层级：AppLayer/system。
 * 主要职责：处理 LED 指示和 MOS 温度读取等不属于控制环的周期业务。
 */

#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include "main.h"

/** 功能：执行一次状态指示和温度读取；参数：无；返回值：无。 */
void SystemMonitor_Process(void);

#endif
