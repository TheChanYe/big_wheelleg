#include "as5047p.h"
#include "switch.h"
#define MODULE_NAME "as5047p"

#ifdef MODE_LOG_TAG
#undef MODE_LOG_TAG
#endif
#define MODE_LOG_TAG MODULE_NAME

typedef struct __M_AS5047P
{
    gpio_type* cs_gpio;        // CS GPIO
    uint32_t cs_pin;           // CS 引脚
    u8 m_spi_channal;          // SPI 通道
} m_as5047p;

static unsigned int even_check(unsigned int x); // 偶校验函数声明
static int m_get_Data(const c_as5047p* this, float *Data); // 获取数据函数声明
static int m_get_Angle(const c_as5047p* this, float *Angle); // 获取角度函数声明
static int m_get_mech_Angle(const c_as5047p* this, float *mech_Angle); // 获取机械角度函数声明
static int m_get_Electrical_Angle(const c_as5047p* this, float *Electrical_Angle); // 获取电气角度函数声明

/* 创建 AS5047P 对象 */
c_as5047p as5047p_create(u8 spi_channal, gpio_type* cs_gpio, uint32_t cs_pin)
{
    int ret = 0; // 返回值
    c_as5047p new = {0}; // 初始化新对象
    m_as5047p* m_this = NULL; // 内部结构体指针

    /* 为新对象申请内存 */
    new.this = pvPortMalloc(sizeof(m_as5047p)); // 分配内存
    if (NULL == new.this)
    {
        log_error("Out of memory"); // 内存不足错误日志
        return new; // 返回空对象
    }
    memset(new.this, 0, sizeof(m_as5047p)); // 初始化内存
    m_this = new.this;

    /* 初始化相应的 SPI */
    ret = my_spi.init(spi_channal); // 初始化 SPI
    if (E_OK != ret)
    {
        log_error("Spi init failed."); // SPI 初始化失败日志
        goto error_handle; // 跳转到错误处理
    }

    gpio_init_type gpio_init_struct; // GPIO 初始化结构体
    gpio_default_para_init(&gpio_init_struct); // GPIO 默认参数初始化

    gpio_init_struct.gpio_pins = cs_pin; // 设置 CS 引脚
    gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL; // 推挽输出
    gpio_init_struct.gpio_pull = GPIO_PULL_UP; // 上拉
    gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT; // 输出模式
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER; // 强驱动能力
    gpio_init(cs_gpio, &gpio_init_struct); // 初始化 GPIO

    /* 保存相关配置 */
    m_this->cs_gpio = cs_gpio; // 保存 CS GPIO
    m_this->cs_pin = cs_pin; // 保存 CS 引脚
    m_this->m_spi_channal = spi_channal; // 保存 SPI 通道
    new.get_Data = m_get_Data; // 设置获取数据函数
    new.get_Angle = m_get_Angle; // 设置获取角度函数
    new.get_mech_Angle = m_get_mech_Angle; // 设置获取机械角度函数
    new.get_Electrical_Angle = m_get_Electrical_Angle; // 设置获取电气角度函数

    gpio_bits_write(m_this->cs_gpio, m_this->cs_pin, TRUE); // 设置 CS 引脚为高电平
    vTaskDelay(50); // 延时 50 ms
    return new; // 返回新对象

error_handle:

		vPortFree(new.this); // 释放内存
		new.this = NULL; // 设置指针为空
    return new; // 返回空对象
}

/* 读取数据 */
static int m_read_data(const c_as5047p* this, u16 *recv_buff)
{
    m_as5047p* m_this = NULL; // 内部结构体指针
    int ret = 0; // 返回值
    u16 data, angleValue = 0; // 数据变量
    u16 error_data = 0; // 错误数据变量

    /* 参数检测 */
    if (NULL == this || NULL == this->this)
    {
        log_error("Null pointer."); // 空指针错误日志
        return E_NULL; // 返回空指针错误
    }
    m_this = this->this;
    /* 选中片选 */
    gpio_bits_write(m_this->cs_gpio, m_this->cs_pin, FALSE); // 设置 CS 引脚为低电平
    /* 发送数据 */
    data = READ_ANGLECOM; // 读取角度命令
    ret = my_spi.transmission(m_this->m_spi_channal, 16, &data, &angleValue, 1, FALSE, 1000); // 发送 SPI 数据
    if (E_OK != ret)
    {
        log_error("spi send failed."); // SPI 发送失败日志
        gpio_bits_write(m_this->cs_gpio, m_this->cs_pin, TRUE); // 确保释放片选
        return E_ERROR; // 返回错误
    }
    /* 释放片选 */
    gpio_bits_write(m_this->cs_gpio, m_this->cs_pin, TRUE); // 设置 CS 引脚为高电平

    /* 检查错误及校验 */
    if (angleValue & (1<<14)) // 如果有错误
    {
        /* 选中片选 */
        gpio_bits_write(m_this->cs_gpio, m_this->cs_pin, FALSE); // 设置 CS 引脚为低电平
        /* 发送清除错误指令 */
        data = READ_ERRFL; // 读取错误标志命令
        ret = my_spi.transmission(m_this->m_spi_channal, 16, &data, &angleValue, 1, FALSE, 1000); // 发送 SPI 数据
        if (E_OK != ret)
        {
            log_error("spi send failed."); // SPI 发送失败日志
            gpio_bits_write(m_this->cs_gpio, m_this->cs_pin, TRUE); // 确保释放片选
            return E_ERROR; // 返回错误
        }
        error_data = angleValue & 0x0003; // 提取错误数据
        /* 释放片选 */
        gpio_bits_write(m_this->cs_gpio, m_this->cs_pin, TRUE); // 设置 CS 引脚为高电平
        log_error("read AS5047P data failed! error_code:%d", error_data); // 读取数据失败日志
        return E_ERROR; // 返回错误
    }
    else // 无错误
    {
        // 校验最高位
        if ((angleValue >> 15) == even_check(angleValue & 0x7FFF))
        {
            *recv_buff = angleValue & 0x3FFF; // 保存接收到的数据
            return E_OK; // 返回成功
        }
        else
        {
            log_error("AS5047P data Verification failed! "); // 数据校验失败日志
            return E_ERROR; // 返回错误
        }
    }
}
/* 偶校验 */
static unsigned int even_check(unsigned int x)
{
    unsigned int count = 0; // 1的个数计数
    // 计算位1个数
    while (x)
    {
        x = x & (x-1); // 去掉最低位的1
        count++;
    }
    // 奇数个1 返回1 偶数个1 返回0
    return (count % 2) == 0 ? 0 : 1;
}

/* 获取原始数据 */
static int m_get_Data(const c_as5047p* this, float *Data)
{
    /* 参数检测 */
    if (NULL == this || NULL == this->this)
    {
        log_error("Null pointer."); // 空指针错误日志
        return E_NULL; // 返回空指针错误
    }
    u16 recv_data = 0; // 接收数据变量
    int ret = m_read_data(this, &recv_data); // 读取数据
    if (ret != E_OK)
    {
        return ret; // 返回错误
    }
    *Data = (float)recv_data / 16384.0f; // 转换为浮点数
    return E_OK; // 返回成功
}

/* 获取角度 */
static int m_get_Angle(const c_as5047p* this, float *Angle)
{
    /* 参数检测 */
    if (NULL == this || NULL == this->this)
    {
        log_error("Null pointer."); // 空指针错误日志
        return E_NULL; // 返回空指针错误
    }
    u16 recv_data = 0; // 接收数据变量
    int ret = m_read_data(this, &recv_data); // 读取数据
    if (ret != E_OK)
    {
        return ret; // 返回错误
    }
    *Angle = (float)recv_data * 360.0f / 16384.0f; // 转换为角度
    return E_OK; // 返回成功
}

/* 获取机械角度 */
static int m_get_mech_Angle(const c_as5047p* this, float *mech_Angle)
{
    /* 参数检测 */
    if (NULL == this || NULL == this->this)
    {
        log_error("Null pointer."); // 空指针错误日志
        return E_NULL; // 返回空指针错误
    }
    u16 recv_data = 0; // 接收数据变量
    int ret = m_read_data(this, &recv_data); // 读取数据
    if (ret != E_OK)
    {
        return ret; // 返回错误
    }
    *mech_Angle = (float)recv_data / 16384.0f * cpr; // 转换为机械角度
    return E_OK; // 返回成功
}

/* 获取电气角度 */
static int m_get_Electrical_Angle(const c_as5047p* this, float *Electrical_Angle)
{
    /* 参数检测 */
    if (NULL == this || NULL == this->this)
    {
        log_error("Null pointer."); // 空指针错误日志
        return E_NULL; // 返回空指针错误
    }
    u16 recv_data = 0; // 接收数据变量
    int ret = m_read_data(this, &recv_data); // 读取数据POLE_PAIR_NUM
    if (ret != E_OK)
    {
        return ret; // 返回错误
    }
    *Electrical_Angle = ((float)recv_data / 16384.0f * cpr) * POLE_PAIR_NUM; // 转换为电气角度
    while (*Electrical_Angle > cpr)
    {
        *Electrical_Angle -= cpr; // 调整电气角度
    }
    return E_OK; // 返回成功
}
