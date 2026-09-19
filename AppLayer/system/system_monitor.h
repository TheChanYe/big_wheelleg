/**
 * 文件用途：基础系统状态监控接口。
 * 所属层级：AppLayer/system。
 * 主要职责：处理 LED 指示和 MOS 温度读取等不属于控制环的周期业务。
 */

#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include "main.h"

#ifdef __cplusplus

#include "switch.h"
#include "motor_fault.h"

class SystemMonitor final {
    public:
        SystemMonitor();
        ~SystemMonitor();

        SystemMonitor(const SystemMonitor&) = delete;
        SystemMonitor& operator=(const SystemMonitor&) = delete;

        /** 功能：执行一次状态指示和温度读取；参数：无；返回值：无。 */
        void process(void);

    private:
        void initialize();  // 初始化系统监控模块
        void updateLed();  // 更新 LED 指示状态
        void thermalProtect(uint8_t motor_id,
                            Motor_Data *motor,
                            MotorFaultBits fault_bit);  // 热保护处理函数
        
        c_switch led_;  // LED 指示灯对象
        bool initialized_;  // 是否已初始化标志

        uint8_t thermal_sample_count_[2];   // 热采样计数
        uint8_t temp_invalid_count_[2];     // 温度无效计数
        uint8_t temp_fault_count_[2];       // 温度故障计数

        TickType_t thermal_start_tick_;  // 热采样起始时间戳

};

#endif /* __cplusplus */

#endif
