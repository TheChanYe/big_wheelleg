#include "my_pid.h"
#define MODULE_NAME       "my pid"

#ifdef  MODE_LOG_TAG
#undef  MODE_LOG_TAG
#endif
#define MODE_LOG_TAG          MODULE_NAME
#define FLOAT_IS_ZERO(value)    (value > -0.000001f && value < 0.000001f)

int g_current_mode = 0; // 0: cold 1: heat
/*定义PID结构体*/
typedef struct __M_MY_PID
{
    float   m_error      ;   /*实际值与目标值的偏差*/
    float   m_last_error ;   /*上一次的偏差值 */
    float   m_kp         ;   /*比例控制系数*/
    float   m_ki         ;   /*积分控制系数*/
    float   m_kd         ;   /*微分控制系数*/
    u32     m_kt         ;   /*时间常数 */
    float   m_i_max      ;   /*积分贡献上线*/
    float   m_i_min      ;   /*积分贡献下线*/
    float   m_intergral  ;   /*积分值*/
		float 	P;
		float		I;
		float 	D;
}m_my_pid;

extern int g_current_mode;

static int m_realize(c_my_pid* this,float des_v, float real_v,float* out);
static int m_reset_param(c_my_pid* this,float kp,float ki,float kd,float i_max,float i_min);

c_my_pid my_pid_create(float kp,float ki,float kd,float i_max,float i_min)
{
    c_my_pid new = {0};
    
    /*为新对象申请内存*/
    new.this = pvPortMalloc(sizeof(m_my_pid));
    if(NULL == new.this)
    {
        log_error("Out of memory");
        return new;
    }
    memset(new.this,0,sizeof(m_my_pid));

    new.realize = m_realize;
    new.reset_param = m_reset_param;
    ((m_my_pid* )new.this)->m_kp    = kp    ;
    ((m_my_pid* )new.this)->m_ki    = ki    ;
    ((m_my_pid* )new.this)->m_kd    = kd    ;
    ((m_my_pid* )new.this)->m_i_max = i_max ;
    ((m_my_pid* )new.this)->m_i_min = i_min ;
    ((m_my_pid* )new.this)->P    = 0.0    ;
    ((m_my_pid* )new.this)->I = 0.0 ;
    ((m_my_pid* )new.this)->D = 0.0 ;
    return new;
}

static int m_reset_param(c_my_pid* this,float kp,float ki,float kd,float i_max,float i_min)
{
     m_my_pid* m_this = NULL;
    /*参数检查*/
    if(NULL == this || NULL == this->this)
    {
        log_error("Null pointer.");
        return E_NULL;
    }
    m_this = this->this;
    m_this->m_kp = kp;
    m_this->m_ki = ki;
    m_this->m_kd = kd;
    m_this->m_i_max = i_max;
    m_this->m_i_min = i_min;
    
    m_this->m_error = 0;
    m_this->m_last_error = 0;
    m_this->m_intergral = 0;
    m_this->P = 0;
    m_this->I = 0;
    m_this->D = 0;
    return E_OK;
}



static int m_realize(c_my_pid* this,float des_v, float real_v,float* out)
{
    m_my_pid* m_this = NULL;
    float output = 0;

    /*参数检查*/
    if(NULL == this || NULL == this->this || NULL == out)
    {
        log_error("Null pointer.");
        return E_NULL;
    }
    m_this = this->this;
    
    //m_this->m_error = fabs(des_v - real_v);
    if (g_current_mode == 0)
        m_this->m_error = real_v - des_v;
    else 
        m_this->m_error = des_v - real_v ;
    m_this->m_intergral += m_this->m_error;
    
    m_this->P  = m_this->m_kp * m_this->m_error;
		m_this-> I  = m_this->m_ki * m_this->m_intergral;
    m_this->D  = m_this->m_kd * (m_this->m_error - m_this->m_last_error);
    if ((g_current_mode == 0 && real_v < des_v) || (g_current_mode == 1 && real_v > des_v))
    {
        if (m_this->P + m_this->I +m_this-> D < 0)
            m_this->P = 0;
    }
    
    if(!FLOAT_IS_ZERO(m_this->m_i_max) && m_this->I > m_this->m_i_max)
    {
        m_this->I = m_this->m_i_max;
        m_this->m_intergral = m_this->I / m_this->m_ki;
    }
    else if(m_this->I < m_this->m_i_min)
    {
        m_this->I = m_this->m_i_min;
        m_this->m_intergral = m_this->I / m_this->m_ki;
    }
    
    output = m_this->P + m_this->I + m_this->D;
    // 处理两种情况：1.加热 水温已经大于目标温度 2.制冷 水温小于目标温度。 因为这两种情况算出来的I是负数，所以强制把功率改成0
    if (output < 0)
        output = 0;
    
    #if SHOW_PID_EFFECT
    log_debug("Expected  value:%.2f,Real-time value:%.2f,P = %.2f,I = %f,D = %.2f,output = %.2f",des_v,real_v,m_this->P,m_this->I,m_this->D,output);
    #endif
    m_this->m_last_error = m_this->m_error;   
    *out = output;
    
    return E_OK;
}

