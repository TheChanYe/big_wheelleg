/*****************************************************************************
* Copyright: 2019-2020
* File name: log.h
* Description: 日志模块公开头文件
* Author: 许智豪
* Version: v1.0
* Date: xxx
*****************************************************************************/

#ifndef __LOG_H__
#define __LOG_H__

#include "main.h"

/*
模块名定义，若模块自定义日志时需要在模块公共头文件中重定义模块名
*/
#ifndef MODE_LOG_TAG
#define MODE_LOG_TAG "sys"
#endif
#define common_log(str, format, ...) do{\
											printf("%s %s %d %s:"format"\r\n",\
												   MODE_LOG_TAG,\
												   str,\
												   __LINE__, __FUNCTION__,##__VA_ARGS__);\
									   }while(0)

#define log_inform(format, ...)   common_log("[I]",  format, ##__VA_ARGS__)
#define log_debug(format, ...)   common_log("[D]",  format, ##__VA_ARGS__)
#define log_warning(format, ...)  common_log("[W]", format, ##__VA_ARGS__)
#define log_error(format, ...)    common_log("[E]",   format, ##__VA_ARGS__)

/*************************************************
* Function: log_init
* Description: 日志系统初始化
* Input : 无
* Output: 无
* Return: 无
* Others: 无
*************************************************/
int log_init(void);

#endif


