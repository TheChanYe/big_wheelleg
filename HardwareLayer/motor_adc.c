#include "motor_adc.h"
#include "foc.h"
#include "app_tasks.h"
#include <math.h>
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
#define NTC_R25_OHM  10000.0f
#define NTC_BETA      3950.0f
#define NTC_T25_K     298.15f


/*adc校准中间参数结构体*/
typedef struct{
	u32 Cumulative_A;//A相电流校准采集累计值
	u32 Cumulative_B;//B相电流校准采集累计值
	u8 Cumulative_count;//电流校准采集次数计数值
}Adc_Data;

Adc_Data motor_adc1;
Adc_Data motor_adc2;

/* r 单位为 kOhm；标准 10k B3950 NTC 可覆盖至少 -40~125°C。 */
float queryTemp(float r)
{
    float temp;

    if (r <= 0.0f)
        return 125.0f;

    temp = 1.0f / ((1.0f / NTC_T25_K)
        + logf((r * 1000.0f) / NTC_R25_OHM) / NTC_BETA) - 273.15f;
    if (temp < -40.0f)
        return -40.0f;
    if (temp > 125.0f)
        return 125.0f;
    return temp;
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

