#include "drv8301.h" // 栅极驱动器接口保留现有 C ABI。
#include "switch.h"
#define MODULE_NAME       "drv8301"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME

typedef struct __M_DRV8301
{
	  u8            m_spi_channal ;  /*spi??*/
    c_switch      m_dc_cal          ;  /*分流放大器使能脚*/	
    c_switch      m_en_gate          ;  /*栅极驱动器使能脚*/		
    c_switch      m_cs          ;  /*CS片选脚*/
}m_drv8301;
static int m_close_drive(const c_drv8301* self);
static int m_open_drive(const c_drv8301* self);

static int m_write_data(const c_drv8301* self,u8 datasize,const u16* send_data,u16* read_data);
static int m_get_status_register(const c_drv8301* self,u16 *register1,u16 *register2);
static int m_init_cmd(const c_drv8301* self);
c_drv8301 drv8301_create(u8 spi_channal,gpio_type* cs_gpio,uint32_t cs_pin,
																				gpio_type* en_gate_gpio,uint32_t en_gate_pin)
{
	 int ret = 0;
	u16 register1=0;
	u16 register2=0;	
    c_drv8301  instance = {0};
    m_drv8301* m_this = NULL;  

    /*为新对象申请内存*/
    instance.context = pvPortMalloc(sizeof(m_drv8301));
    if(NULL == instance.context)
    {
        log_error("Out of memory");
        return instance;
    }
    memset(instance.context,0,sizeof(m_drv8301));
    m_this = static_cast<m_drv8301*>(instance.context);

    /*初始化相应的SPI*/
    ret = my_spi.init(spi_channal);
    if(E_OK != ret)
    {
        log_error("Spi init failed.");
        goto error_handle;
    }	
    /*引脚配置*/
    m_this->m_en_gate = switch_create(en_gate_gpio,en_gate_pin);
    if(NULL == m_this->m_en_gate.context)
    {
        log_error("Switch creat failed.");
        goto error_handle;
    }  		
    /*使能栅极驱动器*/
    ret = m_this->m_en_gate.set(&m_this->m_en_gate,SWITCH_HIGHT);
    if(E_OK != ret)
    {
        log_error("Switch set failed.");
         goto error_handle;
    }	
		
    /*引脚配置*/
    m_this->m_cs = switch_create(cs_gpio,cs_pin);
    if(NULL == m_this->m_cs.context)
    {
        log_error("Switch creat failed.");
        goto error_handle;
    }  
    /*释放片选*/
    ret = m_this->m_cs.set(&m_this->m_cs,SWITCH_HIGHT);
    if(E_OK != ret)
    {
        log_error("Switch set failed.");
         goto error_handle;
    }
     /*保存相关配置*/ 
    m_this->m_spi_channal = spi_channal;	
	instance.write_data = m_write_data;
	instance.get_status_register = m_get_status_register;
	instance.close_drive = m_close_drive;	
	instance.open_drive = m_open_drive;		
	m_init_cmd(&instance);
	m_get_status_register(&instance,&register1,&register2);		
    return instance;
		
error_handle:
	vPortFree(instance.context); // 释放内存
	instance.context = NULL; // 设置指针为空
    return instance;
}

static int m_close_drive(const c_drv8301* self)
{
    m_drv8301* m_this = NULL; // 内部结构体指针
    int ret = 0; // 返回值
    /* 参数检测 */
    if (NULL == self || NULL == self->context)
    {
        log_error("Null pointer."); // 空指针错误日志
        return E_NULL; // 返回空指针错误
    }
    m_this = static_cast<m_drv8301*>(self->context);

    /*使能栅极驱动器*/
    ret = m_this->m_en_gate.set(&m_this->m_en_gate,SWITCH_LOW);
    if(E_OK != ret)
    {
        log_error("Switch set failed.");
         return E_ERROR;
    }		
	return E_OK;
}
static int m_open_drive(const c_drv8301* self)
{
    m_drv8301* m_this = NULL; // 内部结构体指针
    int ret = 0; // 返回值
    /* 参数检测 */
    if (NULL == self || NULL == self->context)
    {
        log_error("Null pointer."); // 空指针错误日志
        return E_NULL; // 返回空指针错误
    }
    m_this = static_cast<m_drv8301*>(self->context);
    /*使能栅极驱动器*/
    ret = m_this->m_en_gate.set(&m_this->m_en_gate,SWITCH_HIGHT);
    if(E_OK != ret)
    {
        log_error("Switch set failed.");
         return E_ERROR;
    }	
		m_init_cmd(self);		
	return E_OK;
}

static int m_init_cmd(const c_drv8301* self)
{
	u16 send_data=0;
	u16 recv_data=0;
	vTaskDelay(10);
	send_data=WRITE_CMD|WRITE_REGISTER1|CURRENT_17|LIMITING|PWM_6|GATE_RESET|LIMITING_0_25A;
	log_inform("send_data=0X%02X",send_data);
	self->write_data(self,16,&send_data,&recv_data);		
	vTaskDelay(10);	
	send_data=WRITE_CMD|WRITE_REGISTER2|CYCLE_MODE|AMPLIFIER2_IN|AMPLIFIER1_IN|AMPLTFIER_80|REPORT_NOCTW;
	log_inform("send_data=0X%02X",send_data);
	self->write_data(self,16,&send_data,&recv_data);			
	return E_OK;
}

static int m_get_status_register(const c_drv8301* self,u16 *register1,u16 *register2)
{
	u16 send_data=0;
	send_data=READ_CMD|READ_REGISTER1;
//	log_inform("send_data=0X%02X",send_data);
	self->write_data(self,16,&send_data,register1);
	self->write_data(self,16,&send_data,register1);
	log_inform("register1=0X%02X",*register1);	
	send_data=READ_CMD|READ_REGISTER2;
	log_inform("send_data=0X%02X",send_data);	
	self->write_data(self,16,&send_data,register2);
	self->write_data(self,16,&send_data,register2);	
	log_inform("register2=0X%02X",*register2);	
	
	return E_OK;
}

static int m_write_data(const c_drv8301* self,u8 datasize,const u16* send_data,u16* read_data)
{
	int ret = 0;
	m_drv8301* m_this = NULL;
	/*参数检测*/
	if(NULL == self || NULL == self->context)
	{
			log_error("Null pointer.");
			return E_NULL;
	}
	m_this = static_cast<m_drv8301*>(self->context);
	/*选中片选*/
	ret = m_this->m_cs.set(&m_this->m_cs,SWITCH_LOW);
	if(E_OK != ret)
	{
			log_error("Switch set failed.");
			return E_ERROR;
	}    
		/*发送数据*/
	ret = my_spi.transmission(m_this->m_spi_channal,datasize,send_data,read_data,1,FALSE,1000);
	/*复位片选*/
	ret = m_this->m_cs.set(&m_this->m_cs,SWITCH_HIGHT);
	if(E_OK != ret)
	{
			log_error("Switch set failed.");
			return E_ERROR;
	}
	return E_OK;
}
