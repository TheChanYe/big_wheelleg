#include "foc.h"
#define MODULE_NAME  "foc"
#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME

extern Motor_Data g_motor1;//电机1全局数据
extern Motor_Data g_motor2;//电机2全局数据
extern SemaphoreHandle_t mutex;//互斥锁句柄

/* PID参数配置示例（需根据实际系统调整） */
static const PID_Params current_params = {
#if 0
    .kp = 0.15f,//整定参数 PHASE_IND * (SPEED_MAX / 60 * POLE_PAIR_NUM) * cpr = 0.00002883*6000/60*7*6.28 = 0.126736f
    .ki = 0.001f,//整定参数 (PHASE_RES * (SPEED_MAX / 60 * POLE_PAIR_NUM) * cpr)/10000(电流环执行频率每秒钟10000次) = 0.06*6000/60*7*6.28/10000 = 0.026376
    .kd = 0.0f,
    .integral_max =3.0f,
    .output_max = 13.9f
#else
    .kp = 0.07f, 
    .ki = 0.0001f,
    .kd = 0.0f,
    .integral_max =6.0f,
    .output_max =13.9f
#endif
};

static const PID_Params velocity_params = {
#if 0
    .kp = 0.15f,
    .ki = 0.1f,
    .kd = 0.01f,
    .integral_max =6.0f,
    .output_max = 100.0f//速度最大输出为 SPEED_MAX / 60 = 100 RPM/S
#else
    .kp = 0.02f,
    .ki = 0.05f,
    .kd = 0.0f,
    .integral_max = 6.0f,
    .output_max = 13.9f
#endif	
};

static const PID_Params position_params = {
    .kp = 30.0f,//此值大小决定位置环响应速度，数值越大，位置响应越快
    .ki = 0.0001f,
    .kd = 0.0f,
    .integral_max = 1.0f,
    .output_max = 150.0f
};



c_as5047p as5047p_1 = {0};
c_drv8301 drv8301_1 = {0};
c_as5047p as5047p_2 = {0};
c_drv8301 drv8301_2 = {0};
//u8 read = 0;
 #define ARRAY_SIZE  50
//  各温度滑动窗口数组大小
float motor1[ARRAY_SIZE] = {0};
float motor2[ARRAY_SIZE] = {0};
int g_cnt_motor1 = 0;
int g_cnt_motor2 = 0;
float new_speed = 0.0f;
/*温度滤波函数*/
float getAvg(float newTemp,float* temps,int* count)
{
    int cnt = ARRAY_SIZE;
    if (*count < ARRAY_SIZE)
    {
        temps[*count] = newTemp;
        cnt = ++*count;
    }
    else
    {
        for (int i = 0; i < ARRAY_SIZE - 1; i++)
        {
            temps[i] = temps[i + 1];   
        }
        temps[ARRAY_SIZE - 1] = newTemp;
    }

    float sum = 0;
    for (int i = 0; i < cnt; i++)
    {
        sum += temps[i];
    }

    float ret = sum / cnt;
    return ret;
}

/*--------------------- 函数声明 ---------------------*/
static int Motor_Control_Init(Motor_Data* motor);
void MotorControl_Run(MotorControl* ctrl);

/*--------------------- 实现 ------------------------*/

/*****************电机数据校准**************************/
// CRC16-CCITT多项式计算函数 (0x1021)
static uint16_t CalculateCRC16(const uint8_t* data, size_t length)
 {
    uint16_t crc = 0xFFFF;   
    for(size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        
        for(int j = 0; j < 8; j++) {
            if(crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }    
    return crc;
}

// 验证CRC校验是否有效
static int ValidateCalibrationParams(Motor_Data* motor)
 {
    // 计算数据部分的CRC
    size_t data_size = offsetof(Calib_Data, checksum);
    uint16_t calculated_crc = CalculateCRC16((uint8_t*)&motor->calib, data_size);    
    return (calculated_crc == motor->calib.checksum);
}
/**
  * @brief  从Flash加载校准数据
  * @param  motor: 电机类型（MOTOR_1/MOTOR_2）
  * @param  calib: 输出参数，指向校准数据的指针
  * @retval 操作结果（0成功，非0失败）
  */
static int LoadCalibrationData(Motor_Data* motor) 
{
		int ret = 0;
    if (!motor) return E_PARAM;   
    ret = flash_read((uint32_t)motor->flash_addr, &motor->calib, sizeof(motor->calib));
    if(ret != E_OK){
	       log_error("Flash read failed!");
        return E_ERROR;
		}
    if(ValidateCalibrationParams(motor)) {
        return E_OK; // 校验成功
    } else {
        log_error("Calibration data corrupted");
        return E_ERROR;
    }
}
/**
  * @brief  将校准数据写入Flash
  * @param  motor: 电机类型（MOTOR_1/MOTOR_2）
  * @param  calib: 输出参数，指向校准数据的指针
  * @retval 操作结果（0成功，非0失败）
  */
static int SaveCalibrationData(Motor_Data* motor)
 {
    if (!motor) return E_PARAM;
		int ret = 0;
    // 计算新校验
    motor->calib.checksum = 0;
		// 使用结构体偏移量计算有效数据范围
    size_t crc_data_size = offsetof(Calib_Data, checksum);
    motor->calib.checksum = CalculateCRC16((uint8_t*)&motor->calib, crc_data_size);
    ret = flash_write((uint32_t)motor->flash_addr, (uint16_t*)&motor->calib, sizeof(motor->calib)/sizeof(uint16_t));
    if(ret != E_OK){
	       log_error("Flash write failed!");
        return E_ERROR;
		}
		else
			return E_OK;
}

/*电机参数校准*/
static int Motor_Calib(Motor_Data* motor)
{
	int ret = 0;
    if(motor == NULL) 
	{
		log_error("Motor param error.");
		return E_PARAM; // 参数错误
	}
	Shut_PWM(motor);//关闭电机PWM输出
	vTaskDelay(2000);//延时等待电机完全停止
	/*修改电流和角度校准状态*/
	motor->calib.Current_Cailb_State = WAIT;
	motor->calib.Angle_Cailb_State = WAIT;
	motor->run_state = CALIB;
	while(motor->calib.Current_Cailb_State == WAIT)//等待电流校准完成
	{
		vTaskDelay(100);
	}
	if(motor->calib.Current_Cailb_State == SUCC)
	{
		//开始校准电机角度
		while(motor->calib.Angle_Cailb_State == WAIT)//等待角度校准完成
		{
			motor->voltage_dq.Vd=0.5f;//D轴给电压 将转子拉到0点
			motor->voltage_dq.Vq=0.0f;                                                                                                                                                                       
			RevParkOperate(motor->voltage_dq.Vd , motor->voltage_dq.Vq, 0 , &motor->voltage_alpha_beta.Valpha, &motor->voltage_alpha_beta.Vbeta );//反Park变换
			SvpwmAlgorithm(motor, motor->voltage_alpha_beta.Valpha, motor->voltage_alpha_beta.Vbeta, Udc, TMR_CLOCK/TMR_PWM );
			vTaskDelay(300);
			/* 获取互斥锁（带超时） */
			if(xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
			{
						log_error("SemaphoreTake Wait timeout!");
				xSemaphoreGive(mutex); // 先释放锁！
						return E_ERROR;
			}
			if(motor->tmr == TMR1)
			{	
				for(u8 i=0;i<5;i++)
				{
					ret = as5047p_1.get_mech_Angle(&as5047p_1,&motor->calib.angle_offset);//获取电机机械角度校准值
					vTaskDelay(50);
				}
			}	
			else if(motor->tmr == TMR8)
			{
				for(u8 i=0;i<5;i++)
				{
					ret = as5047p_2.get_mech_Angle(&as5047p_2,&motor->calib.angle_offset);//获取电机机械角度校准值
					vTaskDelay(50);
				}
			}
			xSemaphoreGive(mutex);//释放互斥锁
			if(ret != E_OK || motor->calib.angle_offset == 0.0f )
			{
				motor->calib.Angle_Cailb_State = FAIL;//电机角度校准失败
				log_error("Motor angle calibration failed.");
				return E_ERROR;					
			}
			Shut_PWM(motor);//关闭电机PWM输出
			
			motor->calib.Angle_Cailb_State = SUCC;//电机角度校准完成
			/*将校准参数写入flash*/
			ret = SaveCalibrationData(motor);
			if(ret != E_OK) 
			{
				log_error("Calib data Save failed");
				return E_ERROR;
			}
			log_inform("Motor calibration end!");
			
		// 配置抢占触发源
//		adc_preempt_conversion_trigger_set(ADC1, ADC12_PREEMPT_TRIG_TMR1CH4, TRUE);//启用或禁用抢占通道的外部触发器，以及设置指定adc外设的外部触发事件
//		adc_preempt_auto_mode_enable(ADC1, TRUE);//启用或禁用指定adc外设的自动抢占组转换

		// 配置中断（可选，根据需求）
//		adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
//		nvic_irq_enable(ADC1_2_IRQn, ADC_1_2_PRO, 0);
			
			return E_OK;
		}	
	}
	else if(motor->calib.Current_Cailb_State == FAIL)
	{
			log_error("Motor current calibration failed.");	
			return E_ERROR;//电流校准失败
	}
	return E_OK;
}

/***************************************end***********************************************/

int Close_Motor(Motor_Data* motor)
{
	int ret = 0;
	// 根据电机选择定时器
	if (motor == NULL)
	{
        log_error("Param error.");
		return E_PARAM; //参数错误
	}
	/*关闭PWM输出*/
	ret = Shut_PWM(motor);
	if (ret != E_OK)
	{
        log_error("shut pwm error.");
		return E_ERROR; //
	}
	vTaskDelay(10);
	if(motor->tmr == TMR1)
	{
		ret = drv8301_1.close_drive(&drv8301_1);
		if(ret != E_OK)
		{
			log_error("drv8301_1 close failed.");
			return E_ERROR;
		} 
	}
	else if(motor->tmr == TMR8)
	{
		ret = drv8301_2.close_drive(&drv8301_2);
		if(ret != E_OK)
		{
			log_error("drv8301_2 close failed.");
			return E_ERROR;
		} 
	}
	return E_OK;
}
int Open_Motor(Motor_Data* motor)
{
	int ret = 0;
	// 根据电机选择定时器
	if (motor == NULL)
	{
        log_error("Param error.");
		return E_PARAM; //参数错误
	}
	vTaskDelay(10);
	if(motor->tmr == TMR1)
	{
		ret = drv8301_1.open_drive(&drv8301_1);
		if(ret != E_OK)
		{
			log_error("drv8301_1 close failed.");
			return E_ERROR;
		} 
	}
	else if(motor->tmr == TMR8)
	{
		ret = drv8301_2.open_drive(&drv8301_2);
		if(ret != E_OK)
		{
			log_error("drv8301_2 close failed.");
			return E_ERROR;
		} 
	}
	return E_OK;
}
/*电机参数初始化*/
int Motor_Init(Motor_Type motor)
{
	int ret = 0;
	if (motor != MOTOR_1 && motor != MOTOR_2 )
	{
		log_error("Motor param error.");
		return E_PARAM; // 参数错误
	}
	Motor_Data* g_motor = NULL;

	g_motor = (motor == MOTOR_1) ? &g_motor1 : &g_motor2;
	g_motor->run_state = INIT;
	g_motor->control_state = IDLE;
  /* 获取互斥锁（带超时） */
  if(xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
	{
        log_error("SemaphoreTake Wait timeout!");
		xSemaphoreGive(mutex); // 先释放锁！
        return E_ERROR;
  }

	/*初始化底层硬件*/
	if(motor == MOTOR_1)
	{
		drv8301_1  = drv8301_create(MY_SPI_3,GPIOC,GPIO_PINS_9,GPIOB,GPIO_PINS_12);
		if(NULL == drv8301_1.this)
		{
			log_error("drv8301_1 creat failed.");
			xSemaphoreGive(mutex);//释放互斥锁
			return E_ERROR;
		} 
		as5047p_1 = as5047p_create(MY_SPI_3,GPIOC,GPIO_PINS_4);
		if(NULL == as5047p_1.this)
		{
			log_error("as5047p_1 creat failed.");
			xSemaphoreGive(mutex);//释放互斥锁
			return E_ERROR;
		} 
		g_motor->flash_addr = (int*)MOTOR1_CALIB_ADDR;
		g_motor->tmr = TMR1;
	}
	else if(motor == MOTOR_2)
	{
		drv8301_2 = drv8301_create(MY_SPI_3,GPIOC,GPIO_PINS_14,GPIOC,GPIO_PINS_13);
		if(NULL == drv8301_2.this)
		{
			log_error("drv8301_2 creat failed.");
			xSemaphoreGive(mutex);//释放互斥锁
			return E_ERROR;
		} 
		as5047p_2 = as5047p_create(MY_SPI_3,GPIOC,GPIO_PINS_3);
		if(NULL == as5047p_2.this)
		{
			log_error("as5047p_2 creat failed.");
			xSemaphoreGive(mutex);//释放互斥锁
			return E_ERROR;
		} 
		g_motor->flash_addr = (int*)MOTOR2_CALIB_ADDR;
		g_motor->tmr = TMR8;
	}
	xSemaphoreGive(mutex);//释放互斥锁

	g_motor->run_state = INIT;
	ret = FOC_ADC_Init(motor); 
	if(ret != E_OK)
	{
		log_error("foc adc init failed.");
		return E_ERROR;
	}   
	FOC_TMR_Init(motor);
	if(ret != E_OK)
	{
		log_error("foc timer init failed.");
		return E_ERROR;
	}  

    // 电流滤波器：时间常数1us（快速响应）
    ret = LPF_Init(&g_motor->Filter_current_id, 0.000000001f); 
	if(ret != E_OK)
	{
		log_error("current LPF init failed.");	
		return E_ERROR;
	}  
    ret = LPF_Init(&g_motor->Filter_current_iq, 0.000000001f); 
	if(ret != E_OK)
	{
		log_error("current LPF init failed.");	
		return E_ERROR;
	}  
    //速度滤波器：时间常数0.01s（中等响应）
    ret = LPF_Init(&g_motor->Filter_speed, 0.000000001f);   
	if(ret != E_OK)
	{
		log_error("speed LPF init failed.");	
		return E_ERROR;
	}  
    // 角度滤波器：时间常数1ns（超快响应）
    ret = LPF_Init(&g_motor->Filter_angle, 0.000000001f);
	if(ret != E_OK)
	{
		log_error("angle LPF init failed.");	
		return E_ERROR;
	}  
	//速度斜坡函数初始化
	ret = Ramp_Init(&g_motor->ramp_speed, 10.0f); // 初始化转速变化率5rpm/s
	if(ret != E_OK)
	{
		log_error("ramp_speed init failed.");	
		return E_ERROR;
	}  
	ret = Motor_Control_Init(g_motor);
	if(ret != E_OK)
	{
		log_error("motor control init failed.");	
		return E_ERROR;
	}  
//	/*读取flash电机参数并校验数据*/
//	if(LoadCalibrationData(g_motor) != E_OK) /*是否需要校准电机参数*/
//	{
//			log_warning("Loading default calibration");
			ret = Motor_Calib(g_motor);/*是，开始执行校准程序*/
			if(ret != E_OK)
			{
				log_error("Motor calibration failed.");
				g_motor->run_state = FAULT;	
				return E_ERROR;//电机参数校准失败
			}
//	}
	g_motor->run_state = IDLE;
	return E_OK;
}
	u16 register1_1=0;
	u16 register2_1=0;
	u16 register1_2=0;
	u16 register2_2=0;
/*获取MOS温度*/
int Get_Mos_Temp(Motor_Data* motor)
{
    float rt;
	float temp = 0;
   if(motor == NULL) 
		return E_PARAM; 
	/*如果电压大于3.13f，断定热敏电阻没有连接 返回硬件错误*/
	if(motor->mos_voltage >=3.13f)
	{
		return E_ERROR;					
	}
	/*计算热敏电阻当前阻值*/
	rt = motor->mos_voltage / ((3.3f - motor->mos_voltage) / 10000.0f);
    temp = queryTemp(rt / 1000);
	if(motor->tmr == TMR1)
	{
		motor->mos_temp = getAvg(temp, motor1, &g_cnt_motor1);
	}
	else if(motor->tmr == TMR8)
	{
		motor->mos_temp = getAvg(temp, motor2, &g_cnt_motor2);
	}
//	if(read == 1)
//	{
//		/* 获取互斥锁（带超时） */
//		if(xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
//		{
//			log_error("SemaphoreTake Wait timeout!");
//			xSemaphoreGive(mutex); // 先释放锁！
//			return E_ERROR;
//		}
//		drv8301_1.get_status_register(&drv8301_1,&register1_1,&register1_2);	
//		drv8301_2.get_status_register(&drv8301_2,&register2_1,&register2_2);
//		xSemaphoreGive(mutex);//释放互斥锁
//		read = 0;	
//	}
	return E_OK;
}

///*获取电压和温度*/
//void get_DC_BUS(void)
//{
//    float temp_voltage;
//    float rt;
//    if (Adc_Data.count > 100)
//    {
//        Adc_Data.DC_BUS = Adc_Data.Bus_Voltage / Adc_Data.count * 11.0f; // 计算总线电压
//        temp_voltage = Adc_Data.Temp_Voltage / Adc_Data.count;
//        rt = temp_voltage / ((3.3f - temp_voltage) / 10000.0f);
//        Adc_Data.Pcb_Temp = queryTemp(rt / 1000.0f);
//        Adc_Data.count = 0;
//        Adc_Data.Bus_Voltage = 0;
//        Adc_Data.Temp_Voltage = 0;
//    }
//    else
//    {
//        Adc_Data.count++;
//        Adc_Data.Bus_Voltage += ((float)Adc_Data.Vbus * 0.00080586f);
//        Adc_Data.Temp_Voltage += ((float)Adc_Data.Temp * 0.00080586f);
//    }
//}

/*获取电机速度*/
/**
  * @brief  获取电机转速（带圈数统计和滤波）
  * @retval 滤波后的转速值（RPM）
  */
static int Get_Motor_Speed(Motor_Data* motor)
{
	int ret = 0;
    /*-- 1. 角度滤波处理 --*/
    // 应用低通滤波器获取平滑角度
     ret = LPF_Update(&motor->Filter_angle, motor->angle_data, &motor->filter_angle);
    /*-- 2. 计算这一次与上一次的角度差 --*/
    motor->delta_raw = motor->filter_angle - motor->angle_prev;
	/*-- 5. 更新上一次的角度数据 --*/
	motor->angle_prev =  motor->filter_angle;    
    // 处理角度突变（超过0.5圈视为跨圈）
    if(fabs(motor->delta_raw) > 0.8f * cpr) {
        // 判断旋转方向
        motor->delta_raw > 0 ? motor->circle_num-- : motor->circle_num++;
    } 

	//处理电机正反转
	if(motor->delta_raw<-PI)
		motor->delta_raw=motor->delta_raw+cpr;
	else if(motor->delta_raw>PI)
		motor->delta_raw=motor->delta_raw-cpr;	

    /*-- 4. 速度计算 --*/
    // 获取时间差（考虑FreeRTOS tick溢出）单位S
    motor->current_tick = xTaskGetTickCount();
    motor->angle_dt = (float)(motor->current_tick - motor->last_tick) * portTICK_PERIOD_MS / 1000.0f;
	//更新时间
		motor->last_tick = motor->current_tick;  	
	// 确保时间间隔合理
	if ( motor->angle_dt <= 0.0f) {
			 motor->angle_dt =0.001f; // 最小时间间隔
	} else if ( motor->angle_dt > 0.1f) { 
			motor->angle_dt = 0.01f;// 最大时间间隔10ms
	}  

    // 时间有效性检查
    if(motor->first_run==0) {
        motor->first_run = 1;
//				motor->velocity = 0.0f;
    } 
	else
	{
		// 计算角速度（rad/s）弧度每秒
//		motor->velocity = motor->delta_raw / motor->angle_dt; 
	motor->velocity=motor->delta_raw*60.0f/PI*200;
		// 转换为RPM：rad/s -> RPM/S = (rad/s) * (60秒/分钟) / (2π弧度/转)
		motor->rpm = motor->velocity / cpr;	
		ret = LPF_Update(&motor->Filter_speed,motor->rpm, &motor->filter_speed);
		if(ret != E_OK)
		{
			log_error("LPF UPdate failed.");	
			return E_ERROR;		
		}		
  }

	
	/*-- 6. 返回滤波后的角速度 --*/

   
	return E_OK;
}
	
/**
  * @brief  级联控制器初始化
  * @param  ctrl 控制器实例指针
  */
static int Motor_Control_Init(Motor_Data* motor)
{
	int ret = 0;

    if(motor == NULL) 
		return E_PARAM; 

    // 默认启用所有控制环
    motor->mode.enable_current = true;
    motor->mode.enable_speed = true;
    motor->mode.enable_position = true;

    // 设置默认控制周期（单位：ms）
    motor->control.current_interval = CURRENT_INTERVAL;    // 1ms电流环
    motor->control.speed_interval = SPEED_INTERVAL;     // 10ms速度环
    motor->control.position_interval = POSITION_INTERVAL; // 100ms位置环

    // 初始化时间戳
    TickType_t now = xTaskGetTickCount();
    motor->control.current_last_tick = now;
    motor->control.speed_last_tick = now;
    motor->control.position_last_tick = now;

    ret = PID_Init(&motor->id_current_pid, (PID_Params*)&current_params);
    if(ret != E_OK)
	{
		log_error("id current pid init failed.");	
		return E_ERROR;		
	}
    ret = PID_Init(&motor->iq_current_pid, (PID_Params*)&current_params);
    if(ret != E_OK)
	{
		log_error("iq current pid init failed.");	
		return E_ERROR;		
	}
    ret = PID_Init(&motor->speed_pid, (PID_Params*)&velocity_params);
     if(ret != E_OK)
	{
		log_error("speed pid init failed.");	
		return E_ERROR;		
	}   
    ret = PID_Init(&motor->position_pid, (PID_Params*)&position_params);
     if(ret != E_OK)
	{
		log_error("position pid init failed.");	
		return E_ERROR;		
	}   
    // 初始值清零
    motor->control.position_target = 0.0f;
    motor->control.speed_target = 0.0f;
    motor->control.id_current_target = 0.0f;
    motor->control.iq_current_target = 0.0f;
	return E_OK;	
}

/**
  * @brief  级联控制主函数（需周期性调用）
  * @param  ctrl 控制器实例指针
  */
int CascadeControl_Run(Motor_Data* motor, Motor_Mode mode, float target)
{
	int ret = 0;
    if(motor == NULL) 
	{
		log_error("Motor param error.");
		return E_PARAM; // 参数错误
	}
	if(mode==Position_loop)
	{
		motor->mode.enable_position = true;
		motor->mode.enable_speed = true;
		motor->mode.enable_current = true;
		motor->control.position_target = target;
	}
	else if(mode==Speed_loop)
	{
		motor->mode.enable_position = false;
		motor->mode.enable_speed = true;
		motor->mode.enable_current = true;
		motor->control.speed_target = target;
	}
	else if(mode==Current_loop)
	{
		motor->mode.enable_position = false;
		motor->mode.enable_speed = false;
		motor->mode.enable_current = true;
		motor->control.id_current_target = 0.0f;//target
		motor->control.iq_current_target = target;//0.0f
	}
  /* 获取互斥锁（带超时） */
  if(xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
	{
        log_error("SemaphoreTake Wait timeout!");
		xSemaphoreGive(mutex); // 先释放锁！
        return E_ERROR;
    }
	if(motor->tmr == TMR1)
	{
		for(u8 i=0;i<3;i++)
		{
			ret = as5047p_1.get_mech_Angle(&as5047p_1,&motor->angle_data);//获取电机机械角度
			if(ret==E_OK)
			{
				break;
			}
		}
		if(ret != E_OK)
		{
			motor->run_state = FAULT;//电机角度获取失败
			Close_Motor(motor);//关闭电机
			log_error("motor1 angle get failed.");

			xSemaphoreGive(mutex);//释放互斥锁
			return E_ERROR;					
		}
	}	
	else if(motor->tmr == TMR8)
	{
		for(u8 i=0;i<3;i++)
		{
			ret = as5047p_2.get_mech_Angle(&as5047p_2,&motor->angle_data);//获取电机机械角度
			if(ret==E_OK)
			{
				break;
			}
		}
		if(ret != E_OK)
		{
			motor->run_state = FAULT;//电机角度获取失败
			Close_Motor(motor);//关闭电机
			log_error("motor2 angle get failed.");
			xSemaphoreGive(mutex);//释放互斥锁
			return E_ERROR;					
		}
	}  
	xSemaphoreGive(mutex);//释放互斥锁
#if 1
//	TickType_t current_tick = xTaskGetTickCount();  

    /*====== 位置环计算 ======*/
    if(motor->mode.enable_position)
    {
			// 检查是否到达执行周期
			if(motor->control.position_interval==0)
			{
					motor->control.position_interval = POSITION_INTERVAL;
					// 获取位置反馈
					motor->control.position_feedback = motor->filter_angle + motor->circle_num*cpr;   					
					// 执行位置环PID计算
					PID_Calc(&motor->position_pid, motor->control.position_target, motor->control.position_feedback, &motor->control.speed_target); 
			}
			else
			motor->control.position_interval--;
    }

    /*====== 速度环计算 ======*/
    if(motor->mode.enable_speed) 
    {
			// 检查是否到达执行周期
			if(motor->control.speed_interval==0)
			{
				motor->control.speed_interval = SPEED_INTERVAL;
				// 获取速度反馈
				Get_Motor_Speed(motor);

				//使用位置环时不需要执行斜坡函数
				if(motor->mode.enable_position)//
				{
						// 执行速度环PID计算
					PID_Calc(&motor->speed_pid, motor->control.speed_target, motor->velocity, &motor->control.iq_current_target);					
				}
				else
				{
					// 执行速度设置斜坡函数
					Ramp_Execute(&motor->ramp_speed, motor->velocity,motor->control.speed_target, &motor->control.speed_feedback);	
					// 执行速度环PID计算
					PID_Calc(&motor->speed_pid, motor->control.speed_feedback, motor->velocity, &motor->control.iq_current_target);				
				}				
			}
			else
			motor->control.speed_interval--;
    }

    /*====== 电流环计算 ======*/
    if(motor->mode.enable_current) 
    {
      // 获取电流反馈
			motor->theta =( motor->angle_data - motor->calib.angle_offset ) * POLE_PAIR_NUM;//计算theta角度
			Clark_Transf(motor->current_abc.Ia,motor->current_abc.Ib,motor->current_abc.Ic,&motor->current_alpha_beta.Ialpha,&motor->current_alpha_beta.Ibeta);//clark变换
			Park_Transf(motor->current_alpha_beta.Ialpha,motor->current_alpha_beta.Ibeta,motor->theta,&motor->current_dq.Id,&motor->current_dq.Iq);//park变换	 

			//低通滤波
			LPF_Update(&motor->Filter_current_id, motor->current_dq.Id, &motor->control.id_current_feedback);
			LPF_Update(&motor->Filter_current_iq, motor->current_dq.Iq, &motor->control.iq_current_feedback);

			PID_Calc(&motor->id_current_pid, motor->control.id_current_target, motor->control.id_current_feedback, &motor->voltage_dq.Vd); 
			PID_Calc(&motor->iq_current_pid, motor->control.iq_current_target, motor->control.iq_current_feedback, &motor->voltage_dq.Vq); 
			
			RevParkOperate(motor->voltage_dq.Vd, motor->voltage_dq.Vq, motor->theta, &motor->voltage_alpha_beta.Valpha, &motor->voltage_alpha_beta.Vbeta);//反Park变换
			SvpwmAlgorithm(motor, motor->voltage_alpha_beta.Valpha,motor->voltage_alpha_beta.Vbeta,Udc,TMR_CLOCK/TMR_PWM);		
    }
		#endif
	return E_OK;
}

