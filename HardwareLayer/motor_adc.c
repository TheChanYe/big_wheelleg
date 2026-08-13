#include "motor_adc.h"
#include "foc.h"
#include "app_tasks.h"
#define MODULE_NAME       "motor_adc"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME

extern Motor_Data g_motor1;//电机1全局数据
extern Motor_Data g_motor2;//电机2全局数据
// ADC采样值数组（每个电机3个通道）
uint16_t adc_preempt_value_motor1[3];
uint16_t adc_preempt_value_motor2[3];

/* PA6 / ADC1_CH6: PVDD 经 R78(10k) / R77(1k) 分压。 */
#define VBUS_ADC_DIVIDER_RATIO       11.0f
#define VBUS_ADC_REF_V               3.3f
#define VBUS_ADC_FULL_SCALE           4095.0f

static float g_bus_voltage = 0.0f;
static uint8_t g_bus_voltage_valid = 0u;

#define min_offset 1.55f//电流校准偏差最小值
#define max_offset 1.7f//电流校准偏差最小值
#define RP  10000
#define T2  (273.15f + 25.0f)
#define BX  3950.0f
#define KA  273.15f

/*温度查表-10度-79度*/
double g_RTTable[] = {56.071, 53.078, 50.263, 74.614, 45.121, 42.774, 40.563, 38.480, 36.517, 34.665,
					  32.919, 31.270, 29.715, 28.246, 26.858, 25.547, 24.307, 23.135, 22.026, 20.977, 
					  19.987, 19.044, 18.154, 17.310, 16.510, 15.752, 15.034, 14.352, 13.705, 13.090, 
					  12.507, 11.953, 11.427, 10.927, 10.452, 10.000, 9.570,  9.161,  8.771,  8.401,  
					  8.048,  7.712,  7.391,  7.086,  6.795,  6.518,  6.254,  6.001,  5.761,  5.531, 
					  5.311,  5.102,  4.902,  4.710,  4.528,  4.353,  4.186,  4.026,  3.874,  3.728,
					  3.588,  3.454,	3.326,	3.203,	3.085,	2.973,	2.865,	2.761,	2.662,	2.567,
					  2.476,  2.388,	2.304,	2.223,	2.146,	2.072,	2.000,	1.932,	1.866,	1.803,
					  1.742,  1.684,	1.627,	1.573,	1.521,	1.471,	1.423,	1.377,	1.332,	1.289};


/*adc校准中间参数结构体*/
typedef struct{
	u32 Cumulative_A;//A相电流校准采集累计值
	u32 Cumulative_B;//B相电流校准采集累计值
	u8 Cumulative_count;//电流校准采集次数计数值
}Adc_Data;

Adc_Data motor_adc1;
Adc_Data motor_adc2;

static float getTemp(int t2, float r1, float r2, float r)
{
    float t;
    t = t2 - (r2 - r) / (r2 - r1);
		if(t2<10)
		{
			t=-(10-t);			
		}
		else
		{
			t-=10;
		}
    return t;
}
//  根据电阻查表获取温度
float queryTemp(float r)
{
    if (r < 1.289f)
		{
			return 79;		
		}
		else if(r > 56.071f)
		{
      return -10;			
		}
    
    float t;
    int i = 0;
    for (; i < 90; i++)
    {
        if (r >= g_RTTable[i])
            break;
    } 
    t = getTemp(i, g_RTTable[i - 1], g_RTTable[i], r);
    return t;
}


// ADC 初始化函数
int FOC_ADC_Init(Motor_Type motor) {
	if (motor != MOTOR_1 && motor != MOTOR_2 )
	{
		log_error("Motor param error.");
		return E_PARAM; // 参数错误
	}
    gpio_init_type gpio_init_struct;
    adc_base_config_type adc_base_struct;
    if (motor == MOTOR_1) {
		// 配置 GPIO 为模拟输入
		gpio_default_para_init(&gpio_init_struct);
		gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
		gpio_init_struct.gpio_pins = GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_1 | GPIO_PINS_6;//电机1 A B相电流 MOS温度 总线电压
		gpio_init(GPIOA, &gpio_init_struct);

		// 配置ADC时钟
		crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
		crm_adc_clock_div_set(CRM_ADC_DIV_6);

		// ADC通道基础配置
		adc_base_default_para_init(&adc_base_struct);
		adc_base_struct.sequence_mode = TRUE;          // 抢占通道不使用序列模式
		adc_base_struct.repeat_mode = FALSE;
		adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
		adc_base_config(ADC1, &adc_base_struct);
		adc_ordinary_channel_set(ADC1, ADC_CHANNEL_6, 1,
			ADC_SAMPLETIME_239_5);
		adc_ordinary_conversion_trigger_set(ADC1,
			ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);

		// 配置 ADC 抢占通道
		adc_preempt_channel_length_set(ADC1, 3);//设置指定adc外设的抢占信道长度
		adc_preempt_channel_set(ADC1, ADC_CHANNEL_4, 1, ADC_SAMPLETIME_7_5);//电机1 A相电流采样通道//一次采样时间为 （7.5+12.5）*1/20=20*0.05us=1us
		adc_preempt_channel_set(ADC1, ADC_CHANNEL_5, 2, ADC_SAMPLETIME_7_5);//电机1 B相电流采样通道
		adc_preempt_channel_set(ADC1, ADC_CHANNEL_1, 3, ADC_SAMPLETIME_7_5);//电机1 MOS温度采样通道
		// 配置抢占触发源
		adc_preempt_conversion_trigger_set(ADC1, ADC12_PREEMPT_TRIG_TMR1CH4, TRUE);//启用或禁用抢占通道的外部触发器，以及设置指定adc外设的外部触发事件
		adc_preempt_auto_mode_enable(ADC1, TRUE);//启用或禁用指定adc外设的自动抢占组转换

		// 配置中断（可选，根据需求）
		adc_interrupt_enable(ADC1, ADC_PCCE_INT, TRUE);
		nvic_irq_enable(ADC1_2_IRQn, ADC_1_2_PRO, 0);

		// 校准 ADC
		adc_enable(ADC1, TRUE);
		adc_calibration_init(ADC1);
		while(adc_calibration_init_status_get(ADC1));
		adc_calibration_start(ADC1);
		while(adc_calibration_status_get(ADC1));

    } else if (motor == MOTOR_2) {
		// 配置 GPIO 为模拟输入
		gpio_default_para_init(&gpio_init_struct);
		gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
		gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 ;//电机2 A B相电流
		gpio_init(GPIOC, &gpio_init_struct);

		gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
		gpio_init_struct.gpio_pins = GPIO_PINS_0;//电机2 MOS温度
		gpio_init(GPIOA, &gpio_init_struct);

	 // 配置ADC时钟
		crm_periph_clock_enable(CRM_ADC3_PERIPH_CLOCK, TRUE);
		crm_adc_clock_div_set(CRM_ADC_DIV_6);

		// ADC通道基础配置
		adc_base_default_para_init(&adc_base_struct);
		adc_base_struct.sequence_mode = TRUE;          // 抢占通道不使用序列模式
		adc_base_struct.repeat_mode = FALSE;
		adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
		adc_base_config(ADC3, &adc_base_struct);

		adc_preempt_channel_length_set(ADC3, 3);//设置指定adc外设的抢占信道长度
		adc_preempt_channel_set(ADC3, ADC_CHANNEL_10, 1, ADC_SAMPLETIME_7_5);//电机1 A相电流采样通道//一次采样时间为 （239.5+12.5）*1/20=252*0.05us=12.6ns
		adc_preempt_channel_set(ADC3, ADC_CHANNEL_11, 2, ADC_SAMPLETIME_7_5);//电机1 B相电流采样通道
		adc_preempt_channel_set(ADC3, ADC_CHANNEL_0, 3, ADC_SAMPLETIME_7_5);//电机2 MOS温度采样通道

		// 配置抢占触发源
		adc_preempt_conversion_trigger_set(ADC3, ADC3_PREEMPT_TRIG_TMR8CH4, TRUE);
		adc_preempt_auto_mode_enable(ADC3, TRUE);//启用或禁用指定adc外设的自动抢占组转换

		// 配置中断（可选，根据需求）
		adc_interrupt_enable(ADC3, ADC_PCCE_INT, TRUE);
		nvic_irq_enable(ADC3_IRQn, ADC_3_PRO, 1);

		// 校准 ADC
		adc_enable(ADC3, TRUE);
		adc_calibration_init(ADC3);
		while(adc_calibration_init_status_get(ADC3));
		adc_calibration_start(ADC3);
		while(adc_calibration_status_get(ADC3));

    } else {
				log_error("Param error.");
        return E_PARAM; //参数错误
    }
		return E_OK;
}

int Motor_ADC_UpdateBusVoltage(void)
{
    uint32_t wait_count;
    uint16_t raw;

    adc_flag_clear(ADC1, ADC_CCE_FLAG);
    adc_ordinary_software_trigger_enable(ADC1, TRUE);

    for (wait_count = 0u; wait_count < 10000u; wait_count++)
    {
        if (adc_flag_get(ADC1, ADC_CCE_FLAG) == SET)
        {
            raw = adc_ordinary_conversion_data_get(ADC1);
            g_bus_voltage = (float)raw * (VBUS_ADC_REF_V / VBUS_ADC_FULL_SCALE)
                * VBUS_ADC_DIVIDER_RATIO;
            g_bus_voltage_valid = 1u;
            return E_OK;
        }
    }

    return E_ERROR;
}

float Motor_ADC_GetBusVoltage(void)
{
    return g_bus_voltage;
}

uint8_t Motor_ADC_BusVoltageValid(void)
{
    return g_bus_voltage_valid;
}

/*ADC1电流采样中断处理程序*/
void ADC1_2_IRQHandler(void)
{
  if( adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET)
  {
		int ret = 0;
		/*获取Ia Ib电流 MOS温度*/
		adc_preempt_value_motor1[0] = adc_preempt_conversion_data_get(ADC1,ADC_PREEMPT_CHANNEL_1);//获取B相电流转换数据
		adc_preempt_value_motor1[1] = adc_preempt_conversion_data_get(ADC1,ADC_PREEMPT_CHANNEL_2);//获取A相电流转换数据
		adc_preempt_value_motor1[2] = adc_preempt_conversion_data_get(ADC1,ADC_PREEMPT_CHANNEL_3);//获取MOS温度转换数据	
		
		if(g_motor1.run_state == CALIB && g_motor1.calib.Current_Cailb_State == WAIT)//参考电压偏置校准
		{
			motor_adc1.Cumulative_A+=adc_preempt_value_motor1[0];
			motor_adc1.Cumulative_B+=adc_preempt_value_motor1[1];
			motor_adc1.Cumulative_count++;
			if(motor_adc1.Cumulative_count>9)
			{
				//计算校准数据
				g_motor1.calib.ia_offset = motor_adc1.Cumulative_A / motor_adc1.Cumulative_count * 0.00080586f ;
				g_motor1.calib.ib_offset = motor_adc1.Cumulative_B / motor_adc1.Cumulative_count * 0.00080586f ;
				/*清除中间参数*/
				motor_adc1.Cumulative_A = 0;
				motor_adc1.Cumulative_B = 0;
				motor_adc1.Cumulative_count = 0;
				
				if(g_motor1.calib.ia_offset > max_offset || g_motor1.calib.ia_offset < min_offset || g_motor1.calib.ib_offset > max_offset || g_motor1.calib.ib_offset < min_offset)
					{	
						g_motor1.calib.Current_Cailb_State = FAIL;//电流校准失败					
					}			
				else
					g_motor1.calib.Current_Cailb_State = SUCC;//电流校准完成
			}
		}
		else if(g_motor1.run_state > CALIB)//正常电流采集(float)adc_value * (3.3f / 4095.0f);
		{
				g_motor1.current_abc.Ia=( ( g_motor1.calib.ia_offset - ( (float)(adc_preempt_value_motor1[0]) * 0.00080586f  )  )  / G ) / Sampling_resistor;//采集A相电流
				g_motor1.current_abc.Ib=( ( g_motor1.calib.ib_offset - ( (float)(adc_preempt_value_motor1[1]) * 0.00080586f  )  )  / G ) / Sampling_resistor;//采集B相电流
				g_motor1.current_abc.Ic = -g_motor1.current_abc.Ia - g_motor1.current_abc.Ib;//计算C相电流
				g_motor1.mos_voltage = (float)adc_preempt_value_motor1[2] * (3.3f / 4095.0f);
#if 0
				ret = CascadeControl_Run(&g_motor1, Speed_loop, 10.0f);
				if(ret != E_OK)
				{
					g_motor1.run_state = FAULT;				
				}
#else
			
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			//向目标任务发送任务通知，已唤醒任务
			vTaskNotifyGiveFromISR(g_motor0_task_handle, &xHigherPriorityTaskWoken);
			// 如果有更高优先级的任务被唤醒，执行上下文切换
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
		}
		adc_flag_clear(ADC1, ADC_PCCE_FLAG);//清除抢占转换通道转换结束标志位
	}
}
/*ADC3电流采样中断处理程序*/
void ADC3_IRQHandler(void)
{
  if( adc_interrupt_flag_get(ADC3, ADC_PCCE_FLAG) != RESET)
  {
		int ret = 0;
	/*获取Ia Ib电流 MOS温度*/
		adc_preempt_value_motor2[0] = adc_preempt_conversion_data_get(ADC3,ADC_PREEMPT_CHANNEL_1);//获取B相电流转换数据
		adc_preempt_value_motor2[1] = adc_preempt_conversion_data_get(ADC3,ADC_PREEMPT_CHANNEL_2);//获取A相电流转换数据
		adc_preempt_value_motor2[2] = adc_preempt_conversion_data_get(ADC3,ADC_PREEMPT_CHANNEL_3);//获取MOS温度转换数据	
		
		if(g_motor2.run_state == CALIB && g_motor2.calib.Current_Cailb_State == WAIT)//参考电压偏置校准
		{
			motor_adc2.Cumulative_A+=adc_preempt_value_motor2[0];
			motor_adc2.Cumulative_B+=adc_preempt_value_motor2[1];
			motor_adc2.Cumulative_count++;
			if(motor_adc2.Cumulative_count>9)
			{
				//计算校准数据
				g_motor2.calib.ia_offset = motor_adc2.Cumulative_A / motor_adc2.Cumulative_count * 0.00080586f ;
				g_motor2.calib.ib_offset = motor_adc2.Cumulative_B / motor_adc2.Cumulative_count * 0.00080586f ;
				/*清除中间参数*/
				motor_adc2.Cumulative_A = 0;
				motor_adc2.Cumulative_B = 0;
				motor_adc2.Cumulative_count = 0;
				
				if(g_motor2.calib.ia_offset > max_offset || g_motor2.calib.ia_offset < min_offset || g_motor2.calib.ib_offset > max_offset || g_motor2.calib.ib_offset < min_offset)
					{	
						g_motor2.calib.Current_Cailb_State = FAIL;//电流校准失败
					}			
				else
					g_motor2.calib.Current_Cailb_State = SUCC;//电流校准完成
			}
		}
		else if(g_motor2.run_state > CALIB)//正常电流采集
		{
				g_motor2.current_abc.Ia=( ( g_motor2.calib.ia_offset - ( (float)(adc_preempt_value_motor2[0]) * 0.00080586f ) )  / G ) / Sampling_resistor;//采集A相电流
				g_motor2.current_abc.Ib=( ( g_motor2.calib.ib_offset - ( (float)(adc_preempt_value_motor2[1]) * 0.00080586f ) )  / G ) / Sampling_resistor;//采集B相电流
				g_motor2.current_abc.Ic = -g_motor2.current_abc.Ia - g_motor2.current_abc.Ib;//计算C相电流
				g_motor2.mos_voltage = (float)adc_preempt_value_motor2[2] * (3.3f / 4095.0f);
#if 0
				ret = CascadeControl_Run(&g_motor2, Speed_loop, 10.0f);
				if(ret != E_OK)
				{
					g_motor2.run_state = FAULT;				
				}
#else
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			//向目标任务发送任务通知，已唤醒任务
			vTaskNotifyGiveFromISR(g_motor1_task_handle, &xHigherPriorityTaskWoken);//
			// 如果有更高优先级的任务被唤醒，执行上下文切换
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
		}
		adc_flag_clear(ADC3, ADC_PCCE_FLAG);//清除抢占转换通道转换结束标志位
	}
}

