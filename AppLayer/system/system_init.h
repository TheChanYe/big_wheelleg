/**
 * 文件用途：系统基础初始化接口。
 * 所属层级：AppLayer/system。
 * 主要职责：初始化时钟、日志、全局同步资源及任务运行前需要的外设时钟。
 */

#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 功能：完成业务任务启动前的基础初始化。
 * 参数：无。
 * 返回值：E_OK 表示成功，E_OUT_OF_MEMORY 表示互斥锁创建失败。
 */
int System_Init(void);

#ifdef __cplusplus
}
#endif

#endif
