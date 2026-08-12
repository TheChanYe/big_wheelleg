#ifndef FOC_CFG_H
#define FOC_CFG_H
#include "main.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "ramp_generator.h"

#define MOTOR1_CALIB_ADDR 0x080FF000 //电机1校准数据存储地址
#define MOTOR2_CALIB_ADDR 0x080FF400 //电机2校准数据存储地址
#define DATA_BUFEER_SIZE    1024 //存储数据大小

/****************************************************/
// 典型配置（需满足整数倍关系）
#define CURRENT_INTERVAL   1  // 电流环1ms
#define SPEED_INTERVAL    5  // 速度环5ms（电流环的10倍）
#define POSITION_INTERVAL 10  // 位置环100ms（速度环的10倍）
#define MOTOR_SPEED_ESTIMATE_MS 5u
#define MOTOR_SPEED_LPF_TF      0.010f

/*****************定时器参数*******************************/
#define TMR_CLOCK 240000000                                          // 定时器时钟频率 240MHz
#define TMR_PWM 20000                                                // PWM频率 24kHz
#define TMR_PR (TMR_CLOCK / (2 * TMR_PWM * (TMR_DIV + 1)))           // 定时器重装值
#define TMR_DIV 0                                                    // 定时器分频系数 
#define TMR_CLOCK_PERIOD_NS (1000000000.0 / TMR_CLOCK)               // 将定时器时钟频率转换为每纳秒的周期数
#define DEADTIME_NS 180 // 死区时间 100ns
#define DEAD_TIME ((u16)((DEADTIME_NS / TMR_CLOCK_PERIOD_NS) + 0.5)) // 计算死区时间（单位：时钟周期）

/******************电机参数**************************/
#define Udc 24.0f               //总线电压参数
#define	POLE_PAIR_NUM 7 /*电机极对数*/
#define PHASE_IND  0.00002883 //相电感（单位H）
#define PHASE_RES  0.06 //相电阻（单位R）
#define SPEED_MAX  6000 //设计最大转速（单位 r/min）

/*****************数学相关数据*********************/
#define SQRT3           1.7320508f  // √3的值
#define SQRT3_2         0.8660254f  // √3/2的值
#define PI	3.14159265358979f           //圆周率
#define cpr (float)(2.0f*PI)
/*****************电流采样时间点相关计算参数*******************************/
// 时间到计数值的转换（四舍五入）
#define NS_TO_COUNT(time_ns) ((uint32_t)((time_ns) * 240.0f / 1000.0f))

#define TNOISE_NS 800  //每次开关管状态变化时产生的噪音时间
#define TRISE_NS 500    //开关管打开时的上升时间
#define SAMPLING_TIME_NS   5  //ADC采样时间
#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif

#define SAMPLE_TIME_MIN 0.001f // 最小有效速度采样时间（1ms）
#define SAMPLE_TIME_MAX 0.1f   // 最大有效速度采样时间（100ms）

// 计算装载值
#define TW_AFTER   NS_TO_COUNT(DEADTIME_NS + MAX_TNTR_NS)
#define TW_BEFORE  NS_TO_COUNT(SAMPLING_TIME_NS) + 1 // 根据需求决定是否保留 +1

#define PWM2_MODE 0
#define PWM1_MODE 1

/************************ADC配置参数***************/
#define G 80.0f                   //电流放大器增益80v/v
#define Sampling_resistor 0.01f   //采样电阻10mR

/******************* FOC算法数据结构体****************/

/*三相电流*/
typedef struct
{
  float Ia;
  float Ib;
  float Ic;
}CURRENT_ABC_DEF;

/*alpha_beta电流*/
typedef struct
{
  float Ialpha;
  float Ibeta;
}CURRENT_ALPHA_BETA_DEF;

/*alpha_beta电压*/
typedef struct
{
  float Valpha;
  float Vbeta;
}VOLTAGE_ALPHA_BETA_DEF;

/*D Q轴电流*/
typedef struct
{
  float Id;
  float Iq;
}CURRENT_DQ_DEF;
/*D Q轴电压*/
typedef struct
{
  float Vd;
  float Vq;
}VOLTAGE_DQ_DEF;

/***************************************************/
/***************************************************/


/***************************************************/

/*电机数据校准枚举*/
typedef enum {
    WAIT = 1,  // 等待校准
    SUCC,  // 校准完成
    FAIL,  // 校准失败
} Cailb_Type;

/******************* 电机校准数据结构体****************/
typedef struct  {
    float ia_offset;    // IA通道零点偏移
    float ib_offset;    // IB通道零点偏移
    float angle_offset; // 角度传感器偏移
		u8 Current_Cailb_State;//电流校准状态
		u8 Angle_Cailb_State;//角度校准状态
    uint16_t checksum;  // CRC32校验值
} Calib_Data;

/*****************************************/

// 电机选择枚举
typedef enum {
    MOTOR_1 = 1,  // 电机1
    MOTOR_2   // 电机2
} Motor_Type;

/*电机系统状态枚举*/
typedef enum 
{
	INIT= 1, 	//初始化
	CALIB,	//校准
	IDLE ,	//空闲
	STOP, 	//停止
	RUN, 		//运行
	BRAKE, 	//刹车
	FAULT		//错误
} Motor_State;

/*控制模式枚举*/
typedef enum 
{
	Current_loop = 0,//电流环
	Speed_loop, //速度环
	Position_loop, //位置环
} Motor_Mode;

/*控制模式配置*/
typedef struct
{
	bool enable_current;// 电流环控制
	bool enable_speed;  // 位置环控制
	bool enable_position;// 速度环控制

}Control_Mode;

/*--------------------- 电机控制结构体 ---------------------*/
typedef struct {   
    // 各环控制周期（单位：ms）
    uint32_t current_interval;
    uint32_t speed_interval;
    uint32_t position_interval;

    // 各环时间记录
    TickType_t current_last_tick;
    TickType_t speed_last_tick;
    TickType_t position_last_tick;

    // 设定值传递
    float position_target;  // 位置目标（用户设置）
    float speed_target;     // 速度目标（位置环输出）
    float id_current_target;   // ID电流目标（速度环输出）
    float iq_current_target;   // IQ电流目标（速度环输出）

    // 反馈值缓存
    float id_current_feedback;
    float iq_current_feedback;
    float speed_feedback;
    float position_feedback;
    uint8_t speed_startup_boost_active;
    TickType_t speed_startup_tick;
    uint8_t speed_startup_failed;
} MotorControl;

/*电机参数结构体*/
typedef struct{
	tmr_type *tmr;        // 绑定的定时器
	Calib_Data 				calib;     // 指向校准数据的指针
	CURRENT_ABC_DEF			current_abc; //指向三相电流数据的指针
	CURRENT_ALPHA_BETA_DEF  current_alpha_beta; //指向alpha_beta电流数据的指针
	VOLTAGE_ALPHA_BETA_DEF  voltage_alpha_beta;
	CURRENT_DQ_DEF 			current_dq;
	VOLTAGE_DQ_DEF 			voltage_dq;
	MotorControl			control;
	Control_Mode 			mode;
	// 定义滤波器实例
	LPF_Handle Filter_current_id;     // 电流滤波
	LPF_Handle Filter_current_iq;     // 电流滤波
	LPF_Handle Filter_speed;  // 速度滤波
	LPF_Handle Filter_angle;  // 角度滤波
	Ramp_Handle ramp_speed;  // 速度斜坡
	///*PID参数*/
	/* 定义各环PID实例 */
	PID_Handle id_current_pid;   // 电流环PID实例
	PID_Handle iq_current_pid;   // 电流环PID实例
	PID_Handle position_pid;  // 位置环PID实例
	PID_Handle speed_pid;  // 速度环PID实例   

	u8 run_state;   			//电机运行状态
	u8 control_state;   			//电机控制状态	
	float mos_temp;		//mos温度
	float mos_voltage; 	//mos温度采集电压
	int* flash_addr;  		//校准数据读写地址
	u8 pwm4_polarity;   	// PWM4方向模式
	float theta; 			// theta角
    float angle_data;   	// 当前原始角度（rad）
    float filter_angle;     // 当前滤波后的角度（rad）
    float angle_prev;       // 上一次的角度（rad）
	TickType_t current_tick;//当前角度获取时间
	TickType_t last_tick;	//上次角度获取时间
	float delta_raw;		//两次的的角度差
	float angle_dt; 		//两次角度的时间差
    int32_t circle_num;		// 旋转圈数计数器
    int first_run;			//速度环第一次运行的状态
    float filter_speed;     // 滤波后的速度（rad）
    float velocity;         // 当前角速度（rad/s）
    float rpm;              // 当前转速（RPM）

}Motor_Data;


#endif 


