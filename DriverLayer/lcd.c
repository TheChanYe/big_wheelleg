#include "lcd.h"
#include "switch.h"
#include "lcd_font.h"

#define MODULE_NAME       "lcd"

#ifdef MODE_LOG_TAG
#undef MODE_LOG_TAG
#endif
#define MODE_LOG_TAG MODULE_NAME

#define DATA_DAT          0 // 数据类型
#define DATA_CMD          1 // 命令类型
#define BUF_LEN           512 // 显示缓存数组长度

typedef struct {
    u16 m_send_buf[BUF_LEN];  // 发送缓存数组
    u16 m_brush;              // 画笔颜色
    u16 m_background;         // 背景颜色
    u8 m_spi_channal;         // SPI 通道
    c_switch m_re;            // 复位脚
    c_switch m_dc;            // 类型脚
    c_switch m_cs;            // 片选脚
    gpio_type* miso_gpio;     // SPI MISO
    uint32_t miso_pin;
} m_lcd;

// 内部使用函数
static int m_lcd_send(const c_lcd* this, u8 type, u8 datasize, const u16* send_data);
static int m_init_cmd(const c_lcd* this);
static int m_set(const c_lcd* this, u16 brush, u16 back);
static int m_lcd_address_set(const c_lcd* this, u16 x1, u16 y1, u16 x2, u16 y2);
static int m_char(const c_lcd* this, u16 x, u16 y, u8 size, char ch);
static int m_chinese(const c_lcd* this, u16 x, u16 y, u8 *s, u16 fc, u16 bc, u8 size, u8 mode);
static int m_lcd_bytes_write(const c_lcd* this, u8 type, u16 *pbuffer, u16 *recv_buff, uint32_t length);
static int  m_chinese12x12(const c_lcd* this,u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 size,u8 mode);
static int  m_chinese16x16(const c_lcd* this,u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 size,u8 mode);
static int  m_chinese24x24(const c_lcd* this,u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 size,u8 mode);
static int  m_chinese32x32(const c_lcd* this,u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 size,u8 mode);

// 对外开放的接口
static int m_fill(const c_lcd* this, u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color);
static int m_clear(const c_lcd* this, u16 color);
static int m_point(const c_lcd* this, u16 x, u16 y, u16 color);
static int m_line(const c_lcd* this, u16 x1, u16 y1, u16 x2, u16 y2, u16 color);
static int m_rectangle(const c_lcd* this, u16 x1, u16 y1, u16 x2, u16 y2, u16 color);
static int m_circle(const c_lcd* this, u16 x0, u16 y0, u16 r, u16 color);
static int m_str(const c_lcd* this, u16 x, u16 y, u8 size, char* format, ...);
static int m_pic(const c_lcd* this, u16 x, u16 y, u16 length, u16 width, u16 pic[]);

// LCD 创建
c_lcd lcd_create(u8 spi_channal, gpio_type* re_gpio, uint32_t re_pin,
                 gpio_type* dc_gpio, uint32_t dc_pin,
                 gpio_type* cs_gpio, uint32_t cs_pin) {
    c_lcd new = {0};
    m_lcd* m_this;

    // 分配内存
    new.this = pvPortMalloc(sizeof(m_lcd));
    if (new.this == NULL) {
        log_error("Out of memory");
        return new;
    }
    memset(new.this, 0, sizeof(m_lcd));
    m_this = new.this;

    // 初始化 SPI
    if (my_spi.init(spi_channal) != E_OK) {
        log_error("SPI init failed.");
        goto error_handle;
    }

    // 配置引脚
    if ((m_this->m_re = switch_create(re_gpio, re_pin)).this == NULL ||
        (m_this->m_dc = switch_create(dc_gpio, dc_pin)).this == NULL ||
        (m_this->m_cs = switch_create(cs_gpio, cs_pin)).this == NULL) {
        log_error("Switch create failed.");
        goto error_handle;
    }

    // 复位片选
    if (m_this->m_cs.set(&m_this->m_cs, SWITCH_HIGHT) != E_OK) {
        log_error("Switch set failed.");
        goto error_handle;
    }

    // 保存配置
    m_this->miso_gpio = dc_gpio;
    m_this->miso_pin = dc_pin;
    m_this->m_spi_channal = spi_channal;
    m_this->m_brush = LCD_BRUSH_DEFAULT_COLOR;
    m_this->m_background = LCD_BACK_DEFAULT_COLOR;

    // 设置接口
    new.clear = m_clear;
    new.set = m_set;
    new.str = m_str;
    new.pic = m_pic;
    new.rectangle = m_rectangle;
    new.circle = m_circle;
    new.fill = m_fill;
    new.line = m_line;
    new.point = m_point;
    new.chinese = m_chinese;

    // 发送初始化命令
    if (m_init_cmd(&new) != E_OK) {
        log_error("LCD init cmd send failed.");
        goto error_handle;
    }

    // 清屏
    if (new.clear(&new, m_this->m_background) != E_OK) {
        log_error("LCD clear failed.");
        goto error_handle;
    }

    return new;

error_handle:
    vPortFree(new.this);
    new.this = NULL;
    return new;
}

static int m_init_cmd(const c_lcd* this) {
    m_lcd* m_this;
    u16 data;

    if (this == NULL || this->this == NULL) {
        log_error("Null pointer.");
        return E_NULL;
    }
    m_this = this->this;

    // 屏幕复位
    m_this->m_re.set(&m_this->m_re, SWITCH_LOW) ;
		vTaskDelay(120),
		m_this->m_re.set(&m_this->m_re, SWITCH_HIGHT);
		vTaskDelay(120);

    // 初始化命令
    data = 0x11;
    m_lcd_send(this, DATA_CMD, 8, &data);
    vTaskDelay(120);

    data = 0x36;
    m_lcd_send(this, DATA_CMD, 8, &data);
    switch (USE_HORIZONTAL) {
        case 0:
            data = 0x00;
            break;
        case 1:
            data = 0xC0;
            break;
        case 2:
            data = 0x70;
            break;
        case 3:
            data = 0xA0;
            break;
        default:
            data = 0x00;
    }
    m_lcd_send(this, DATA_DAT, 8, &data);

    data = 0x3A;
    m_lcd_send(this, DATA_CMD, 8, &data);
    data = 0x55;
    m_lcd_send(this, DATA_DAT, 8, &data);

    // 其他初始化命令
    const u16 init_data[] = {
        0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33,
        0xB7, 0x35,
        0xBB, 0x32,
        0xC2, 0x01,
        0xC3, 0x15,
        0xC4, 0x20,
        0xC6, 0x0F,
        0xD0, 0xA4, 0xA1,
        0xE0, 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34,
        0xE1, 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31, 0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34,
        0x21, 0x29
    };

    for (int i = 0; i < sizeof(init_data) / sizeof(init_data[0]); ++i) {
        m_lcd_send(this, (i % 2 == 0) ? DATA_CMD : DATA_DAT, 8, &init_data[i]);
    }

    return E_OK;
}

static int m_lcd_send(const c_lcd* this, u8 type, u8 datasize, const u16* send_data) {
    m_lcd* m_this;
    if (this == NULL || this->this == NULL) {
        log_error("Null pointer.");
        return E_NULL;
    }
    if (type != DATA_CMD && type != DATA_DAT) {
        log_error("Param error.");
        return E_PARAM;
    }
    m_this = this->this;

    if (m_this->m_dc.set(&m_this->m_dc, (type == DATA_DAT) ? SWITCH_HIGHT : SWITCH_LOW) != E_OK ||
        m_this->m_cs.set(&m_this->m_cs, SWITCH_LOW) != E_OK ||
        (my_spi.transmission(m_this->m_spi_channal, datasize, send_data, NULL, 1, FALSE, 1000)) != E_OK ||
        m_this->m_cs.set(&m_this->m_cs, SWITCH_HIGHT) != E_OK ||
        m_this->m_dc.set(&m_this->m_dc, SWITCH_HIGHT) != E_OK) {
        log_error("Switch set failed.");
        return E_ERROR;
    }

    return E_OK;
}

static int m_lcd_bytes_write(const c_lcd* this, u8 type, u16 *send_buf, u16 *recv_buf, uint32_t length) {
    m_lcd* m_this;
    if (this == NULL || this->this == NULL) {
        log_error("Null pointer.");
        return E_NULL;
    }
    if (type != DATA_CMD && type != DATA_DAT) {
        log_error("Param error.");
        return E_PARAM;
    }
    m_this = this->this;

    if (m_this->m_dc.set(&m_this->m_dc, (type == DATA_DAT) ? SWITCH_HIGHT : SWITCH_LOW) != E_OK ||
        m_this->m_cs.set(&m_this->m_cs, SWITCH_LOW) != E_OK ||
        (my_spi.transmission(m_this->m_spi_channal, 16, send_buf, NULL, length, TRUE, 1000)) != E_OK ||
        m_this->m_cs.set(&m_this->m_cs, SWITCH_HIGHT) != E_OK) {
        log_error("Switch set failed.");
        return E_ERROR;
    }

    return E_OK;
}

static int m_lcd_address_set(const c_lcd* this, u16 x1, u16 y1, u16 x2, u16 y2) {
    u16 data;

    if (this == NULL || this->this == NULL) {
        log_error("Null pointer.");
        return E_NULL;
    }

    data = 0x2A;
    m_lcd_send(this, DATA_CMD, 8, &data);
    data = x1;
    m_lcd_send(this, DATA_DAT, 16, &data);
    data = x2;
    m_lcd_send(this, DATA_DAT, 16, &data);

    data = 0x2B;
    m_lcd_send(this, DATA_CMD, 8, &data);
    data = y1 + 20;
    m_lcd_send(this, DATA_DAT, 16, &data);
    data = y2 + 20;
    m_lcd_send(this, DATA_DAT, 16, &data);

    data = 0x2C;
    m_lcd_send(this, DATA_CMD, 8, &data);

    return E_OK;
}

static int m_point(const c_lcd* this, u16 x, u16 y, u16 color) {
    m_lcd* m_this;

    if (this == NULL || this->this == NULL) {
        log_error("Null pointer.");
        return E_NULL;
    }
    m_this = this->this;

    m_lcd_address_set(this, x, y, x, y);

    if (m_this->m_cs.set(&m_this->m_cs, SWITCH_LOW) != E_OK ||
        my_spi.transmission(m_this->m_spi_channal, 16, &color, NULL, 1, FALSE, 1000) != E_OK ||
        m_this->m_cs.set(&m_this->m_cs, SWITCH_HIGHT) != E_OK) {
        log_error("Switch set failed.");
        return E_ERROR;
    }

    return E_OK;
}

static int m_line(const c_lcd* this, u16 x1, u16 y1, u16 x2, u16 y2, u16 color) {
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;

    if (this == NULL || this->this == NULL) {
        log_error("Null pointer.");
        return E_NULL;
    }

    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;

    if (delta_x > 0) incx = 1;
    else if (delta_x == 0) incx = 0;
    else { incx = -1; delta_x = -delta_x; }

    if (delta_y > 0) incy = 1;
    else if (delta_y == 0) incy = 0;
    else { incy = -1; delta_y = -delta_y; }

    distance = (delta_x > delta_y) ? delta_x : delta_y;

    for (int t = 0; t < distance + 1; ++t) {
        m_point(this, uRow, uCol, color);
        xerr += delta_x;
        yerr += delta_y;

        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }

        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }

    return E_OK;
}

static int m_rectangle(const c_lcd* this, u16 x1, u16 y1, u16 x2, u16 y2, u16 color) {
    m_line(this, x1, y1, x2, y1, color);
    m_line(this, x1, y1, x1, y2, color);
    m_line(this, x1, y2, x2, y2, color);
    m_line(this, x2, y1, x2, y2, color);

    return E_OK;
}

static int m_circle(const c_lcd* this, u16 x0, u16 y0, u16 r, u16 color) {
    int a = 0, b = r;

    if (this == NULL || this->this == NULL) {
        log_error("Null pointer.");
        return E_NULL;
    }

    while (a <= b) {
        m_point(this, x0 - b, y0 - a, color);
        m_point(this, x0 + b, y0 - a, color);
        m_point(this, x0 - a, y0 + b, color);
        m_point(this, x0 - a, y0 - b, color);
        m_point(this, x0 + b, y0 + a, color);
        m_point(this, x0 + a, y0 - b, color);
        m_point(this, x0 + a, y0 + b, color);
        m_point(this, x0 - b, y0 + a, color);

        a++;
        if ((a * a + b * b) > (r * r)) {
            b--;
        }
    }

    return E_OK;
}

static int m_clear(const c_lcd* this, u16 color) {
    if (this == NULL || this->this == NULL) {
        log_error("Null pointer.");
        return E_NULL;
    }

    return m_fill(this, 0, 0, LCD_W, LCD_H, color);
}

static int m_fill(const c_lcd* this, u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color) {
    m_lcd* m_this;
    int num, num1 = 0;
    u16 t = 1;

    if (this == NULL || this->this == NULL) {
        log_error("Null pointer.");
        return E_NULL;
    }
    if (xend < (xsta + 1) || yend < (ysta + 1)) {
        log_error("Param error.");
        return E_PARAM;
    }
    m_this = this->this;

    num = (xend - xsta) * (yend - ysta);
    m_lcd_address_set(this, xsta, ysta, xend - 1, yend - 1);

    if (m_this->m_cs.set(&m_this->m_cs, SWITCH_LOW) != E_OK) {
        log_error("Switch set failed.");
        return E_ERROR;
    }

    while (t) {
        if (num > 65534) {
            num -= 65534;
            num1 = 65534;
        } else {
            t = 0;
            num1 = num;
        }
        my_spi.transmission(m_this->m_spi_channal, 16, &color, NULL, num1, FALSE, 1000);
    }

    if (m_this->m_cs.set(&m_this->m_cs, SWITCH_HIGHT) != E_OK) {
        log_error("Switch set failed.");
        return E_ERROR;
    }

    return E_OK;
}

/******************************************************************************
函数说明：显示单个字符
入口数据：x,y显示坐标
					num 要显示的字符
					fc 字的颜色
					bc 字的背景色
					size 字号
					mode 0非叠加模式 1叠加模式

******************************************************************************/
static int m_char(const c_lcd* this,u16 x,u16 y,u8 size,char ch)
{
	  m_lcd* m_this = NULL;
		int ret = 0;
		u8 temp,sizex,t,m=0,n=0;
		u16 i,TypefaceNum;//一个字符所占字节大小
	    /*参数检测*/
    if(NULL == this || NULL == this->this)
    {
        log_error("Null pointer.");
        return E_NULL;
    }
		sizex=size/2;
		TypefaceNum=(sizex/8+((sizex%8)?1:0))*size;
		ch=ch-' ';    //得到偏移值		
    if(12 != size && 16 != size && 24 != size && 32 != size)
    {
        log_error("Param error.");
        return E_PARAM;
    }
		m_this = this->this;
    /*清空显存*/
    memset(m_this->m_send_buf,0,TypefaceNum * 8); 
		
		for (int i = 0; i < size * sizex; i++)//填充底色
		{
			m_this->m_send_buf[i] = m_this->m_background;
		}		
		m_lcd_address_set(this,x,y,x+sizex-1,y+size-1);//设置显示范围 
		for(i=0;i<TypefaceNum;i++)
		{ 
			if(size==12)temp=ascii_1206[ch][i];		       //调用6x12字体
			else if(size==16)temp=ascii_1608[ch][i];		 //调用8x16字体
			else if(size==24)temp=ascii_2412[ch][i];		 //调用12x24字体
			else if(size==32)temp=ascii_3216[ch][i];		 //调用16x32字体
			for (t = 0; t < 8; t++)
			{
				if (temp & (0x01 << t))
				{
					m_this->m_send_buf[n * sizex + m] =m_this->m_brush;//LCD_COLOR_RED//m_this->m_brush
				}
				m++; // 记录一行中的像素点位置
				if (m % sizex == 0)
				{
					n++;   // 一行记录完成，列号加一
					m = 0; // 行号清零
					break;
				}					
			}	
		}
				/*选中片选*/
    ret = m_this->m_cs.set(&m_this->m_cs,SWITCH_LOW);
    if(E_OK != ret)
    {
        log_error("Switch set failed.");
         return E_ERROR;
    }	
		my_spi.transmission(m_this->m_spi_channal,16,m_this->m_send_buf,NULL,sizex*size,TRUE,1000); 
		/*释放片选*/
    ret = m_this->m_cs.set(&m_this->m_cs,SWITCH_HIGHT);
    if(E_OK != ret)
    {
        log_error("Switch set failed.");
         return E_ERROR;
    }	
    return E_OK;
}

	//显示字符串
static int m_str(const c_lcd* this,u16 x,u16 y,u8 size,char* format,...)
{
    u8 str_index = 0;
    va_list   argptr;
    char str_buf[25] = {0};
    int ret = 0;
    
    /*检查参数*/
    if(NULL == this || NULL == this->this)
    {
        log_error("Null pointer.");
        return E_NULL;
    }    
    /*检查坐标*/
    if(x >= LCD_W || y >= LCD_H)
    {
        log_error("Param error");
        return E_ERROR;
    }
    
    va_start( argptr, format );  // 初始化argptr	
		memset(str_buf,0,25);
		vsnprintf(str_buf,25,format,argptr);
		va_end(argptr);
		
	
    /*循环输出字符*/
    for(str_index = 0;'\0' != str_buf[str_index];++str_index)
    {
        if(x + (str_index + 1) * size / 2 >= LCD_W)
        {
            log_warning("String is too long ");  /*超长直接舍弃*/
            return E_OK;
        }

        /*输出字符串*/
        ret = m_char(this,x + str_index * size / 2,y,size,str_buf[str_index]);
        if(E_OK != ret)
        {
            log_error("Print char failed.");
            return E_ERROR;
        }
    }
    
    return E_OK;
}

//设置画笔和背景颜色
static int m_set(const c_lcd* this,u16 brush,u16 back)
{
    m_lcd*  m_this = NULL;
    /*检查参数*/
    if(NULL == this || NULL == this->this)
    {
        log_error("Null pointer.");
        return E_NULL;
    }
    m_this = this->this;

    /*设置颜色*/
    m_this->m_brush      = brush;
    m_this->m_background = back;
    return E_OK;
}


//在指定位置显示图片
static int m_pic(const c_lcd* this,u16 x,u16 y,u16 length,u16 width,u16 pic[])
{
		u8 t=1;
		u32 num,num1;
		m_lcd* m_this = NULL;
		int ret = 0;
    /*参数检测*/
    if(NULL == this || NULL == this->this)
    {
        log_error("Null pointer.");
        return E_NULL;
    }
		m_this=this->this;
		num=length*width*2;
		m_lcd_address_set(this,x,y,x+length-1,y+width-1);//设置显示范围 	
    /*选中片选*/
    ret = m_this->m_cs.set(&m_this->m_cs,SWITCH_LOW);
    if(E_OK != ret)
    {
        log_error("Switch set failed.");
         return E_ERROR;
    }
		while(t)
		{
			if(num>65534)
			{
				num-=65534;
				num1=65534;
			}
			else
			{
				t=0;
				num1=num;
			}
			ret=my_spi.transmission(m_this->m_spi_channal,8,pic,NULL,num1,TRUE,1000); 
			if(E_OK != ret)
			{
					log_error("spi send failed.");
					 return E_ERROR;
			}
			pic+=65534/2;
		}		
		/*释放片选*/
    ret = m_this->m_cs.set(&m_this->m_cs,SWITCH_HIGHT);
    if(E_OK != ret)
    {
        log_error("Switch set failed.");
         return E_ERROR;
    }
    return E_OK;
}
/******************************************************************************
函数说明：显示汉字串
入口数据：x,y显示坐标
					*s 要显示的汉字
					fc 字的颜色
					bc 字的背景色
					size 字号 可选 12 16 24 32
					mode 0非叠加模式 1叠加模式
******************************************************************************/
static int m_chinese(const c_lcd* this,u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 size,u8 mode)
{
		    /*参数检测*/
    if(NULL == this )
    {
        log_error("Null pointer.");
        return E_NULL;
    }
	  if(12 != size && 16 != size && 24 != size && 32 != size)
    {
        log_error("Param error.");
        return E_PARAM;
    }
		
	while(*s!=0)
	{
		if(size==12) m_chinese12x12(this,x,y,s,fc,bc,size,mode);
		else if(size==16) m_chinese16x16(this,x,y,s,fc,bc,size,mode);
		else if(size==24) m_chinese24x24(this,x,y,s,fc,bc,size,mode);
		else if(size==32) m_chinese32x32(this,x,y,s,fc,bc,size,mode);
		s+=2;
		x+=size;
	}
	return E_OK;
}
/******************************************************************************
函数说明：显示单个12x12汉字
入口数据：x,y显示坐标
					num 要显示的汉字
					fc 字的颜色
					bc 字的背景色
					size 字号
					mode 0非叠加模式 1叠加模式
******************************************************************************/
static int  m_chinese12x12(const c_lcd* this,u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 size,u8 mode)
{
	m_lcd* m_this = NULL;
	int ret = 0;
	u8 i,j,m=0,n=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占大小
	/*参数检测*/
	if(NULL == this || NULL == this->this)
	{
			log_error("Null pointer.");
			return E_NULL;
	}	
	m_this = this->this;	
		/*清空显存*/
	memset(m_this->m_send_buf,0,TypefaceNum * 8); 
	
	for (int i = 0; i < size * size; i++)//填充底色
	{
		m_this->m_send_buf[i] = m_this->m_background;
	}	
			/*选中片选*/
	ret = m_this->m_cs.set(&m_this->m_cs,SWITCH_LOW);
	if(E_OK != ret)
	{
			log_error("Switch set failed.");
			 return E_ERROR;
	}	
		
		
	TypefaceNum=(size/8+((size%8)?1:0))*size;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			m_lcd_address_set(this,x,y,x+size-1,y+size-1);//设置显示范围 
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{											
					if(tfont12[k].Msk[i]&(0x01<<j))
					{
						m_this->m_send_buf[n * size + m] =m_this->m_brush;
					}
					m++;
					if(m%size==0)
					{
						n++;
						m=0;
						break;
					}
				}
			}
			
							/*选中片选*/
    ret = m_this->m_cs.set(&m_this->m_cs,SWITCH_LOW);
    if(E_OK != ret)
    {
        log_error("Switch set failed.");
         return E_ERROR;
    }				
			for(i=0;i<size;i++)
			{
				m_lcd_address_set(this,x,y+i,x+size-1,y+size-1+i);//设置显示范围 
				m_lcd_bytes_write(this,DATA_DAT,m_this->m_send_buf,NULL,size);
				memset(m_this->m_send_buf, 0, size);			
				memcpy(m_this->m_send_buf,m_this->m_send_buf+size,sizeof(m_this->m_send_buf)-size);
			}
									/*选中片选*/
			ret = m_this->m_cs.set(&m_this->m_cs,SWITCH_HIGHT);
			if(E_OK != ret)
			{
					log_error("Switch set failed.");
					 return E_ERROR;
			}				
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
		return E_OK;
} 


/******************************************************************************
函数说明：显示单个16x16汉字
入口数据：x,y显示坐标
					num 要显示的汉字
					fc 字的颜色
					bc 字的背景色
					size 字号
					mode 0非叠加模式 1叠加模式
******************************************************************************/
static int  m_chinese16x16(const c_lcd* this,u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 size,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占大小
	u16 x0=x;
	u16 data = 0;
	/*参数检测*/
	if(NULL == this || NULL == this->this)
	{
			log_error("Null pointer.");
			return E_NULL;
	}
	TypefaceNum=(size/8+((size%8)?1:0))*size;
	                         
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			m_lcd_address_set(this,x,y,x+size-1,y+size-1);//设置显示范围 
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加模式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))
						{
							data=fc;m_lcd_send(this,DATA_DAT,8,&data);
						}
						else 
						{
							data=bc;m_lcd_send(this,DATA_DAT,8,&data);	
						}
						m++;
						if(m%size==0)
						{
							m=0;
							break;
						}
					}
					else//叠加模式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))							
							m_point(this,x, y, fc);//画点
						x++;
						if((x-x0)==size)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
		return E_OK;
}  

/******************************************************************************
函数说明：显示单个24x24汉字
入口数据：x,y显示坐标
					num 要显示的汉字
					fc 字的颜色
					bc 字的背景色
					size 字号
					mode 0非叠加模式 1叠加模式
******************************************************************************/
static int  m_chinese24x24(const c_lcd* this,u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 size,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占大小
	u16 x0=x;
		u16 data = 0;
	/*参数检测*/
	if(NULL == this || NULL == this->this)
	{
			log_error("Null pointer.");
			return E_NULL;
	}
	TypefaceNum=(size/8+((size%8)?1:0))*size;
	                         
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			m_lcd_address_set(this,x,y,x+size-1,y+size-1);//设置显示范围 
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加模式
					{					
						if(tfont24[k].Msk[i]&(0x01<<j))
						{
							data=fc;m_lcd_send(this,DATA_DAT,8,&data);
						}
						else 
						{
							data=bc;m_lcd_send(this,DATA_DAT,8,&data);	
						}					
						m++;
						if(m%size==0)
						{
							m=0;
							break;
						}
					}
					else//叠加模式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))							
							m_point(this,x, y,fc);//画点
						x++;
						if((x-x0)==size)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
	return E_OK;
} 
/******************************************************************************
函数说明：显示单个32x32汉字
入口数据：x,y显示坐标
					num 要显示的汉字
					fc 字的颜色
					bc 字的背景色
					size 字号
					mode 0非叠加模式 1叠加模式
******************************************************************************/
static int  m_chinese32x32(const c_lcd* this,u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 size,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占大小
	u16 x0=x;
			u16 data = 0;
	/*参数检测*/
	if(NULL == this || NULL == this->this)
	{
			log_error("Null pointer.");
			return E_NULL;
	}
	TypefaceNum=(size/8+((size%8)?1:0))*size;
	                         
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			m_lcd_address_set(this,x,y,x+size-1,y+size-1);//设置显示范围 
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加模式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))
						{
							data=fc;m_lcd_send(this,DATA_DAT,8,&data);
						}
						else 
						{
							data=bc;m_lcd_send(this,DATA_DAT,8,&data);	
						}	
						m++;
						if(m%size==0)
						{
							m=0;
							break;
						}
					}
					else//叠加模式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))							
							m_point(this,x, y,fc);//画点
						x++;
						if((x-x0)==size)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
		return E_OK;
} 



