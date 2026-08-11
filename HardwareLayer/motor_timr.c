#include "motor_timr.h"
#define MODULE_NAME       "motor_timr"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME

// TIM1/TIM8 初始化函数
int FOC_TMR_Init(Motor_Type motor) {
	if (motor != MOTOR_1 && motor != MOTOR_2 )
	{
		log_error("Motor param error.");
		return E_PARAM; // 参数错误
	}
    gpio_init_type gpio_init_struct = {0};
    tmr_output_config_type tmr_output_struct;
    tmr_brkdt_config_type tmr_brkdt_config_struct;

    // 根据电机选择定时器
    tmr_type *tmr;

    if (motor == MOTOR_1) {
        tmr = TMR1;
		// 使能定时器和GPIO时钟
		crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
		// 配置GPIO为复用功能（PWM输出）
		gpio_default_para_init(&gpio_init_struct);
		gpio_init_struct.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
		gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
		gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
		gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
		gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
		gpio_init(GPIOA, &gpio_init_struct);

		gpio_init_struct.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14 | GPIO_PINS_15;
		gpio_init(GPIOB, &gpio_init_struct);

    } else if (motor == MOTOR_2) {
        tmr = TMR8;
		// 使能定时器和GPIO时钟
		crm_periph_clock_enable(CRM_TMR8_PERIPH_CLOCK, TRUE);
		// 配置GPIO为复用功能（PWM输出）
		gpio_default_para_init(&gpio_init_struct);
		gpio_init_struct.gpio_pins = GPIO_PINS_6 | GPIO_PINS_7 | GPIO_PINS_8;
		gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
		gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
		gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
		gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
		gpio_init(GPIOC, &gpio_init_struct);

		gpio_init_struct.gpio_pins = GPIO_PINS_7;
		gpio_init(GPIOA, &gpio_init_struct);

		gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 ;
		gpio_init(GPIOB, &gpio_init_struct);

    } else {
            log_error("Param error.");
						return E_PARAM; //参数错误
    }

    // 配置定时器基础参数
    tmr_base_init(tmr, TMR_PR, TMR_DIV);          // 不分频，周期为 period
    tmr_cnt_dir_set(tmr, TMR_COUNT_TWO_WAY_1); // 中心对齐模式

			// 配置 PWM 输出通道（通道1-3）
		/* 通道1、2、3输出模式下的配置 */
		tmr_output_default_para_init(&tmr_output_struct);					//重置定时器
		tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;//输出通道模式B
		tmr_output_struct.oc_output_state = TRUE;									//启用输出通道
		tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;		//输出通道极性低
		tmr_output_struct.oc_idle_state = FALSE;									//输出通道空闲状态低
		tmr_output_struct.occ_output_state = TRUE;								//启用互补输出通道
		tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_LOW;		//互补输出通道极性低
		tmr_output_struct.occ_idle_state = FALSE;									//互补输出通道空闲状态低

		// 初始化通道1-3
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_2, &tmr_output_struct);
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
		tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_1, 0);
		tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_2, 0);
		tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_3, 0);
			// 配置通道4（用于触发ADC）
		tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;//输出通道模式B
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_4, &tmr_output_struct);
		tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_4, TMR_PR - 1); // 触发点

			// 配置死区时间
		/* 自动输出启用、停止、停滞时间和锁定配置 */
		tmr_brkdt_default_para_init(&tmr_brkdt_config_struct);
	//  tmr_brkdt_config_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_HIGH;//刹车输入信号高电平	
	//  tmr_brkdt_config_struct.brk_enable = TRUE;//启用刹车功能
		tmr_brkdt_config_struct.auto_output_enable = TRUE;//自动输出启用
		tmr_brkdt_config_struct.deadtime = DEAD_TIME;//死区时间设置 20*4.2=84ns
		tmr_brkdt_config_struct.fcsodis_state = TRUE;//禁用定时器输出时的输出通道为无效电平
		tmr_brkdt_config_struct.fcsoen_state = TRUE;//启用定时器时的输出通道为空闲电平
		tmr_brkdt_config_struct.wp_level = TMR_WP_LEVEL_3;//3级写保护
		tmr_brkdt_config(tmr, &tmr_brkdt_config_struct);
		/* output enable */
		tmr_output_enable(tmr, TRUE);
		/* enable tmr1 */
		tmr_counter_enable(tmr, TRUE);


//		Shut_PWM(motor);
	return E_OK;
}

/*关闭PWM输出*/
int Shut_PWM(Motor_Data* motor)
{
	// 根据电机选择定时器
	if (motor == NULL)
	{
        log_error("Param error.");
				return E_PARAM; //参数错误
	}
	tmr_channel_value_set(motor->tmr, TMR_SELECT_CHANNEL_1, 0);
	tmr_channel_value_set(motor->tmr, TMR_SELECT_CHANNEL_2, 0);
	tmr_channel_value_set(motor->tmr, TMR_SELECT_CHANNEL_3, 0);
//  tmr_channel_value_set(motor->tmr, TMR_SELECT_CHANNEL_4, TMR_PR - 1); // 触发点
	return E_OK;
}


