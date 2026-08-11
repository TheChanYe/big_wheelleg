/*****************************************************************************
* Copyright:
* File name: switch.c
* Description: 开关模块函数实现
* Author: 许少
* Version: V1.2
* Date: 2022/04/08
* log:  V1.0  2022/04/08
*       发布：
*
*       V1.1  2022/04/02
*       新增：power接口中 相同功率退出 没有先给this指针赋值再判断。
*
*       V1.2  2022/04/08
*       新增：SWITHC_POWER_INTERVAL 开关频率控制宏
*****************************************************************************/
#include "key.h"
#define MODULE_NAME       "key"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME

typedef struct __M_KEY
{
	gpio_type* m_gpio     ;  /*gpio*/
	uint32_t      m_pin      ;  /*pin*/

}m_key;

static int   get     (const c_key* this,KEY_TYPE* state);

c_key key_create(gpio_type* gpio,uint32_t pin)
{
	c_key new = {0};
	gpio_init_type gpio_init_struct;
	/*为新对象申请内存*/
	new.this = pvPortMalloc(sizeof(m_key));
	if(NULL == new.this)
	{
		log_error("Out of memory");
		return new;
	}
    memset(new.this,0,sizeof(m_key));
    //初始化对应的GPIO
    gpio_default_para_init(&gpio_init_struct);
    
    gpio_init_struct.gpio_pins = pin;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init(gpio, &gpio_init_struct);
    	
	/*保存相关变量*/
	((m_key* )new.this)->m_gpio     = gpio    ;
	((m_key* )new.this)->m_pin      = pin     ;
	new.get = get;   
	return new;
}


static int   get     (const c_key* this,KEY_TYPE* state)
{
    const m_key* m_this = NULL;
    KEY_TYPE   io_state = RESET;

    /*参数检测*/
    if(NULL == this || NULL == this->this || NULL == state)
    {
        log_error("Null pointer.");
        return E_NULL;
    }
    m_this = this->this;

    io_state = gpio_input_data_bit_read(m_this->m_gpio,m_this->m_pin);

    *state = io_state;

    return  E_OK;
}

