#include "my_spi.h"

#define MODULE_NAME "spi"

#ifdef MODE_LOG_TAG
#undef MODE_LOG_TAG
#endif
#define MODE_LOG_TAG MODULE_NAME

typedef struct __SPI_CFG {
    spi_type* m_spi_addr;/*spi地址*/
    dma_channel_type* m_send_dma_channel;   /*发送DMA通道*/
    dma_channel_type* m_recv_dma_channel;   /*接收DMA通道*/
    gpio_type* m_spi_gpio;
    u32 m_pin_sck;
    u32 m_pin_miso;
    u32 m_pin_mosi;
} spi_cfg;

typedef struct _M_SPI {
    u8 m_id;
    bool m_init_state;
    spi_init_type m_spi_handle;
    dma_init_type m_tx_dma_handle;
    dma_init_type m_rx_dma_handle;
} m_spi;

u8 dma_recv_state = 0;
u8 dma_send_state = 0;

static int m_init(u8 spi);
static int m_set_speed(u8 spi, u8 speed);
static int m_set_clock_polarity(u8 spi,u8 clock_polarity);
static int m_set_datasize				(u8 spi,u8 datasize);
static int m_transmission(u8 spi, u8 datasize, const u16* send, u16* recv, u32 len, bool incremental, TickType_t time);

const c_spi my_spi = { m_init, m_set_speed, m_transmission };

static const spi_cfg g_cfg[3] = {{SPI1,DMA1_CHANNEL3,DMA1_CHANNEL2,GPIOA,GPIO_PINS_5,GPIO_PINS_6,GPIO_PINS_7},
                                { SPI2,DMA1_CHANNEL5,DMA1_CHANNEL4,GPIOB,GPIO_PINS_13,GPIO_PINS_14,GPIO_PINS_15},
                                { SPI3,DMA2_CHANNEL2,DMA2_CHANNEL1,GPIOC,GPIO_PINS_10,GPIO_PINS_11,GPIO_PINS_12},																
                                };

static m_spi g_my_spi[3];

static int m_init(u8 spi)
{
    gpio_init_type gpio_initstructure = {0};

    if (g_my_spi[spi].m_init_state)
    {
        log_warning("SPI has been initialized!");
        return E_OK;
    }

    switch (spi)
    {
        case MY_SPI_1:
            crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
            break;
        case MY_SPI_2:
            crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
            break;
        case MY_SPI_3:
            crm_periph_clock_enable(CRM_SPI3_PERIPH_CLOCK, TRUE);
            crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
            gpio_pin_remap_config(SPI3_GMUX_0001, TRUE);
            break;
        default:
            log_error("Invalid SPI index!");
            return E_PARAM;
    }

    gpio_default_para_init(&gpio_initstructure);

    gpio_initstructure.gpio_pins = g_cfg[spi].m_pin_sck;
    gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio_initstructure.gpio_pull = GPIO_PULL_DOWN;
    gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
    gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(g_cfg[spi].m_spi_gpio, &gpio_initstructure);

    gpio_initstructure.gpio_pins = g_cfg[spi].m_pin_miso;
    gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;		
    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_mode = GPIO_MODE_MUX;		
    gpio_init(g_cfg[spi].m_spi_gpio, &gpio_initstructure);

    gpio_initstructure.gpio_pins = g_cfg[spi].m_pin_mosi;
    gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;		
    gpio_initstructure.gpio_pull = GPIO_PULL_UP;
    gpio_initstructure.gpio_mode = GPIO_MODE_MUX;				
    gpio_init(g_cfg[spi].m_spi_gpio, &gpio_initstructure);

    spi_default_para_init(&g_my_spi[spi].m_spi_handle);
    g_my_spi[spi].m_spi_handle.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    g_my_spi[spi].m_spi_handle.master_slave_mode = SPI_MODE_MASTER;
    g_my_spi[spi].m_spi_handle.mclk_freq_division = SPI_MCLK_DIV_16;
    g_my_spi[spi].m_spi_handle.first_bit_transmission = SPI_FIRST_BIT_MSB;
    g_my_spi[spi].m_spi_handle.frame_bit_num = SPI_FRAME_8BIT;
    g_my_spi[spi].m_spi_handle.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    g_my_spi[spi].m_spi_handle.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    g_my_spi[spi].m_spi_handle.cs_mode_selection = SPI_CS_SOFTWARE_MODE;

    spi_init(g_cfg[spi].m_spi_addr, &g_my_spi[spi].m_spi_handle);
    spi_enable(g_cfg[spi].m_spi_addr, TRUE);
# if 1
	spi_i2s_dma_transmitter_enable(g_cfg[spi].m_spi_addr,TRUE) ;   /*使能SPI的DMA发送*/
	spi_i2s_dma_receiver_enable(g_cfg[spi].m_spi_addr, TRUE);		 /*使能SPI的DMA接收*/ 
		
	/*配置发送DMA*/
	dma_reset(g_cfg[spi].m_send_dma_channel);							/*重置发送通道*/
	dma_default_para_init(&g_my_spi[spi].m_tx_dma_handle); /*默认参数初始化*/   

	g_my_spi[spi].m_tx_dma_handle.buffer_size            = 0;   /*需要传输的buf大小*/   
	g_my_spi[spi].m_tx_dma_handle.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;//存储器到外设
	g_my_spi[spi].m_tx_dma_handle.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;//存储器数据长度:8位
	g_my_spi[spi].m_tx_dma_handle.memory_inc_enable =TRUE;//存储器增量模式
	g_my_spi[spi].m_tx_dma_handle.peripheral_base_addr = (uint32_t)&(g_cfg[spi].m_spi_addr->dt);//外设基本地址
	g_my_spi[spi].m_tx_dma_handle.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;//外设数据长度:8位
	g_my_spi[spi].m_tx_dma_handle.peripheral_inc_enable = FALSE;//外设非增量模式
	g_my_spi[spi].m_tx_dma_handle.priority = DMA_PRIORITY_MEDIUM;//中等优先级
	g_my_spi[spi].m_tx_dma_handle.loop_mode_enable = FALSE;//关闭循环模式
	dma_init(g_cfg[spi].m_send_dma_channel,&g_my_spi[spi].m_tx_dma_handle);//初始化DMA

	dma_interrupt_enable(g_cfg[spi].m_send_dma_channel, DMA_FDT_INT,TRUE);
	nvic_irq_enable(DMA2_Channel2_IRQn, DMA2_1_PRO, 0);//使能DMA发送中断

	/*配置接收DMA*/
	dma_reset(g_cfg[spi].m_recv_dma_channel);
	dma_default_para_init(&g_my_spi[spi].m_rx_dma_handle); 

	g_my_spi[spi].m_rx_dma_handle.buffer_size = 0;   /*需要传输的buf大小*/      
	g_my_spi[spi].m_rx_dma_handle.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;//外设到存储器
	g_my_spi[spi].m_rx_dma_handle.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;//存储器数据长度:8位
	g_my_spi[spi].m_rx_dma_handle.memory_inc_enable = FALSE;//存储器增量模式
	g_my_spi[spi].m_rx_dma_handle.peripheral_base_addr = (uint32_t)&(g_cfg[spi].m_spi_addr->dt);//外设基本地址	
	g_my_spi[spi].m_rx_dma_handle.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;//外设数据长度:8位
	g_my_spi[spi].m_rx_dma_handle.peripheral_inc_enable = FALSE;//外设非增量模式
	g_my_spi[spi].m_rx_dma_handle.priority = DMA_PRIORITY_MEDIUM;//中等优先级
	g_my_spi[spi].m_rx_dma_handle.loop_mode_enable = FALSE;//关闭循环模式
	dma_init(g_cfg[spi].m_recv_dma_channel,&g_my_spi[spi].m_rx_dma_handle);//初始化DMA

	dma_interrupt_enable(g_cfg[spi].m_recv_dma_channel, DMA_FDT_INT,TRUE);
	nvic_irq_enable(DMA2_Channel1_IRQn, DMA2_1_PRO, 1);//使能DMA接收中断
#endif
    g_my_spi[spi].m_init_state = TRUE;
    return E_OK;
}

static int m_set_speed(u8 spi, u8 speed)
{
    if (spi > MY_SPI_3 || speed > SPI_MCLK_DIV_256)
    {
        log_error("参数错误。");
        return E_PARAM;
    }
    spi_enable((spi_type*)g_cfg[spi].m_spi_addr, FALSE);         //关闭SPI
    if(speed > SPI_MCLK_DIV_256)
    {
      g_cfg[spi].m_spi_addr->ctrl2_bit.mdiv_h = 1;
      g_cfg[spi].m_spi_addr->ctrl1_bit.mdiv_l = speed & 0x7;
    }
    else
    {
      g_cfg[spi].m_spi_addr->ctrl2_bit.mdiv_h = 0;
      g_cfg[spi].m_spi_addr->ctrl1_bit.mdiv_l = speed;
    }
    spi_enable((spi_type*)g_cfg[spi].m_spi_addr, TRUE);             //使能SPI
    return E_OK;
}

//SPI 时钟极性设置函数
static int m_set_clock_polarity(u8 spi,u8 clock_polarity)
{
			/*参数检查*/
    if(MY_SPI_1 != spi && MY_SPI_2 != spi && MY_SPI_3 != spi)
    {
        log_error("Param error.");
        return E_PARAM;
    }
    /*确认spi状态*/
    if(spi_i2s_flag_get((spi_type*)g_cfg[spi].m_spi_addr,SPI_I2S_BF_FLAG) != RESET)
    {
        log_error("Spi is busy.");
        return E_ERROR;
    }
		
    spi_enable((spi_type*)g_cfg[spi].m_spi_addr, FALSE);         //关闭SPI    
		
    if(clock_polarity == SPI_CLOCK_POLARITY_HIGH)
    {
			 g_my_spi[spi].m_spi_handle.clock_polarity=SPI_CLOCK_POLARITY_HIGH;//设置串行同步时钟的空闲状态为高电平
    }
    else
    {
			g_my_spi[spi].m_spi_handle.clock_polarity=SPI_CLOCK_POLARITY_LOW;//设置串行同步时钟的空闲状态为低电平
    }
		spi_init(g_cfg[spi].m_spi_addr,&g_my_spi[spi].m_spi_handle);              /*初始化SPI*/
    spi_enable((spi_type*)g_cfg[spi].m_spi_addr, TRUE);             //使能SPI
		
	return E_OK;
}

//SPI 修改数据位宽
static int m_set_datasize(u8 spi,u8 datasize)
{
    /*参数检查*/
    if(MY_SPI_1 != spi && MY_SPI_2 != spi && MY_SPI_3 != spi)
    {
        log_error("Param error.");
        return E_PARAM;
    }
    if(8 != datasize && 16 != datasize)
    {
        log_error("Param error.");
        return E_PARAM;
    }
    /*确认spi状态*/
    if(spi_i2s_flag_get((spi_type*)g_cfg[spi].m_spi_addr,SPI_I2S_BF_FLAG) != RESET)
    {
        log_error("Spi is busy.");
        return E_ERROR;
    }
	/*必须先关闭SPI DMA使能 才能更改配置*/
    spi_enable(g_cfg[spi].m_spi_addr,FALSE);
    dma_channel_enable(g_cfg[spi].m_send_dma_channel,FALSE); 
    dma_channel_enable(g_cfg[spi].m_recv_dma_channel,FALSE); 

    if(8 == datasize)
    {
        g_my_spi[spi].m_spi_handle.frame_bit_num             = SPI_FRAME_8BIT                  ;   /*设置SPI的数据大小:SPI发送接收8位帧结构*/

        /*发送 修改为8位宽*/
        g_my_spi[spi].m_tx_dma_handle.memory_data_width      = DMA_MEMORY_DATA_WIDTH_BYTE      ;   /*存储器数据长度:8位*/
        g_my_spi[spi].m_tx_dma_handle.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_BYTE  ;   /*外设数据长度:8位*/

        /*接收 修改位8位宽*/
        g_my_spi[spi].m_rx_dma_handle.memory_data_width      = DMA_MEMORY_DATA_WIDTH_BYTE      ;   /*存储器数据长度:8位*/
        g_my_spi[spi].m_rx_dma_handle.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_BYTE  ;   /*外设数据长度:8位*/
    }
    else
    {
        g_my_spi[spi].m_spi_handle.frame_bit_num             = SPI_FRAME_16BIT                     ;  /*设置SPI的数据大小:SPI发送接收16位帧结构*/       

        /*发送 修改位16位宽*/
        g_my_spi[spi].m_tx_dma_handle.memory_data_width      = DMA_MEMORY_DATA_WIDTH_HALFWORD      ;   /*存储器数据长度:16位*/
        g_my_spi[spi].m_tx_dma_handle.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD  ;   /*外设数据长度:16位*/

        /*接收 修改位16位宽*/
        g_my_spi[spi].m_rx_dma_handle.memory_data_width      = DMA_MEMORY_DATA_WIDTH_HALFWORD      ;   /*存储器数据长度:16位*/
        g_my_spi[spi].m_rx_dma_handle.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD  ;   /*外设数据长度:16位*/
    }
    
    spi_init(g_cfg[spi].m_spi_addr,&g_my_spi[spi].m_spi_handle);              /*初始化SPI*/
    dma_init(g_cfg[spi].m_send_dma_channel,&g_my_spi[spi].m_tx_dma_handle);   /*初始化发送DMA*/
    dma_init(g_cfg[spi].m_recv_dma_channel,&g_my_spi[spi].m_rx_dma_handle);   /*初始化接收DMA*/
    spi_enable(g_cfg[spi].m_spi_addr,TRUE);                                   /*使能spi*/
		
    return E_OK;
}


static int m_transmission(u8 spi, u8 datasize, const u16* send, u16* recv, u32 len, bool incremental, TickType_t time)
{
    if (spi > MY_SPI_3 || len == 0 || (send == NULL && recv == NULL))
    {
        log_error("parameter error!");
        return E_PARAM;
    }
    if (spi_i2s_flag_get(g_cfg[spi].m_spi_addr, SPI_I2S_BF_FLAG) != RESET)
    {
        log_error("SPI busy!");
        return E_ERROR;
    }

//    spi_enable(g_cfg[spi].m_spi_addr, FALSE);

//    g_my_spi[spi].m_spi_handle.frame_bit_num = (datasize == 8) ? SPI_FRAME_8BIT : SPI_FRAME_16BIT;
//    spi_init(g_cfg[spi].m_spi_addr, &g_my_spi[spi].m_spi_handle);
//    spi_enable(g_cfg[spi].m_spi_addr, TRUE);

//    for (u32 i = 0; i < len; ++i)
//    {
//        if (send != NULL)
//        {
//            while (spi_i2s_flag_get(g_cfg[spi].m_spi_addr, SPI_I2S_TDBE_FLAG) == RESET);
//            spi_i2s_data_transmit(g_cfg[spi].m_spi_addr, send[i]);
//        }

//        if (recv != NULL)
//        {
//            while (spi_i2s_flag_get(g_cfg[spi].m_spi_addr, SPI_I2S_RDBF_FLAG) == RESET);
//            recv[i] = spi_i2s_data_receive(g_cfg[spi].m_spi_addr);
//        }
//    }
#if 1
 /*必须先关闭SPI DMA使能 才能更改配置*/
    spi_enable(g_cfg[spi].m_spi_addr,FALSE);
    dma_channel_enable(g_cfg[spi].m_send_dma_channel,FALSE); 
    dma_channel_enable(g_cfg[spi].m_recv_dma_channel,FALSE); 
    
    /*设置数据位宽*/
    if(8 == datasize)
    {
        g_my_spi[spi].m_spi_handle.frame_bit_num             = SPI_FRAME_8BIT                  ;   /*设置SPI的数据大小:SPI发送接收8位帧结构*/

        /*发送 修改为8位宽*/
        g_my_spi[spi].m_tx_dma_handle.memory_data_width      = DMA_MEMORY_DATA_WIDTH_BYTE      ;   /*存储器数据长度:8位*/
        g_my_spi[spi].m_tx_dma_handle.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_BYTE  ;   /*外设数据长度:8位*/

        /*接收 修改位8位宽*/
        g_my_spi[spi].m_rx_dma_handle.memory_data_width      = DMA_MEMORY_DATA_WIDTH_BYTE      ;   /*存储器数据长度:8位*/
        g_my_spi[spi].m_rx_dma_handle.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_BYTE  ;   /*外设数据长度:8位*/
    }
    else
    {
        g_my_spi[spi].m_spi_handle.frame_bit_num             = SPI_FRAME_16BIT                     ;  /*设置SPI的数据大小:SPI发送接收16位帧结构*/       

        /*发送 修改位16位宽*/
        g_my_spi[spi].m_tx_dma_handle.memory_data_width      = DMA_MEMORY_DATA_WIDTH_HALFWORD      ;   /*存储器数据长度:16位*/
        g_my_spi[spi].m_tx_dma_handle.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD  ;   /*外设数据长度:16位*/

        /*接收 修改位16位宽*/
        g_my_spi[spi].m_rx_dma_handle.memory_data_width      = DMA_MEMORY_DATA_WIDTH_HALFWORD      ;   /*存储器数据长度:16位*/
        g_my_spi[spi].m_rx_dma_handle.peripheral_data_width  = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD  ;   /*外设数据长度:16位*/
    }
    
    /*确认增量模式*/
    if(incremental)
    { 
        g_my_spi[spi].m_tx_dma_handle.memory_inc_enable = TRUE;     /*存储器增量模式*/       
    }
    else
    {
        g_my_spi[spi].m_tx_dma_handle.memory_inc_enable = FALSE;    /*不执行存储器增量模式*/
    }
    spi_init(g_cfg[spi].m_spi_addr,&g_my_spi[spi].m_spi_handle);    /*重新配置spi*/
    spi_enable(g_cfg[spi].m_spi_addr,TRUE);
        
		/*根据参数类型 启动相应的spi收发模式*/

		/*启动发送*/
		if(NULL != send)
		{
				g_my_spi[spi].m_tx_dma_handle.buffer_size       = len       ;       /*要传输的数据长度 */
				g_my_spi[spi].m_tx_dma_handle.memory_base_addr  = (uint32_t)send ;       /*内存地址         */

				dma_init(g_cfg[spi].m_send_dma_channel,&g_my_spi[spi].m_tx_dma_handle);  /*配置发送dma*/
				dma_channel_enable(g_cfg[spi].m_send_dma_channel,TRUE);                  /*启动发送dma*/
				while(dma_send_state==0);
						dma_send_state=0;	
		}
		/*启动接收*/
		if(NULL != recv)
		{
				g_my_spi[spi].m_rx_dma_handle.buffer_size       = len       ;       /*要接收的数据长度 */
				g_my_spi[spi].m_rx_dma_handle.memory_base_addr  = (uint32_t)recv ;       /*内存地址         */  

				dma_init(g_cfg[spi].m_recv_dma_channel,&g_my_spi[spi].m_rx_dma_handle);  /*配置接收dma*/
				dma_channel_enable(g_cfg[spi].m_recv_dma_channel,TRUE);                  /*启动接收dma*/
				while(dma_recv_state==0);
						dma_recv_state=0;	
		}	
#endif
    return E_OK;
}

int DMA2_Channel1_IRQHandler(void)
{
	    BaseType_t ret = 0;  
	    /*通道1 spi3 接收完成中断*/
    if(RESET != dma_interrupt_flag_get(DMA2_FDT1_FLAG))
    {
        dma_flag_clear(DMA2_FDT1_FLAG);                                 /*清除标志*/
        dma_channel_enable(g_cfg[MY_SPI_3].m_recv_dma_channel,FALSE);   /*关闭spi3 接收dma*/
				dma_recv_state=1;
    }
    else
    {
        dma_flag_clear(DMA2_GL1_FLAG);
    }		
	    return E_OK;
	
}

int DMA2_Channel2_IRQHandler(void)
{
    BaseType_t ret = 0;   
 
    /*通道2 spi3 发送完成中断*/
    if(RESET != dma_interrupt_flag_get(DMA2_FDT2_FLAG))
    {
        dma_flag_clear(DMA2_FDT2_FLAG);                                 /*清除标志*/
        dma_channel_enable(g_cfg[MY_SPI_3].m_send_dma_channel,FALSE);   /*关闭spi3 发送dma*/ 
				dma_send_state=1;
    }
    else
    {
        dma_flag_clear(DMA2_GL2_FLAG);
    }    
    return E_OK;

}

