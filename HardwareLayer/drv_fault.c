/**
 ******************************************************************************
 * @file     drv_fault.c
 * @brief    PC5/PC15 DRV8301 nFAULT software protection.
 ******************************************************************************
 */

#include "drv_fault.h"
#include "foc.h"
#include "motor_timr.h"
#include "motor_fault.h"

extern Motor_Data g_motor1;
extern Motor_Data g_motor2;
extern c_drv8301 drv8301_1;
extern c_drv8301 drv8301_2;
extern SemaphoreHandle_t mutex;

#define MOTOR0_NFAULT_PIN           GPIO_PINS_5
#define MOTOR1_NFAULT_PIN           GPIO_PINS_15

static volatile uint8_t g_drv_fault_pending = 0u;
static uint8_t g_motor_ready_mask = 0u;
static uint16_t g_drv_status1[2] = {0u};
static uint16_t g_drv_status2[2] = {0u};

static void DrvFault_ConfigLine(uint32_t line, gpio_pins_source_type pin_source)
{
    exint_init_type exint_init_struct;

    gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOC, pin_source);
    exint_default_para_init(&exint_init_struct);
    exint_init_struct.line_select = line;
    exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
    exint_init_struct.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
    exint_init_struct.line_enable = FALSE;
    exint_init(&exint_init_struct);
    exint_flag_clear(line);
}

int DrvFault_Init(void)
{
    gpio_init_type gpio_init_struct;

    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_pins = MOTOR0_NFAULT_PIN | MOTOR1_NFAULT_PIN;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOC, &gpio_init_struct);

    DrvFault_ConfigLine(EXINT_LINE_5, GPIO_PINS_SOURCE5);
    DrvFault_ConfigLine(EXINT_LINE_15, GPIO_PINS_SOURCE15);
    return E_OK;
}

void DrvFault_MotorReady(uint8_t motor_id)
{
    if (motor_id >= 2u)
        return;

    g_motor_ready_mask |= (uint8_t)(1u << motor_id);
    if (g_motor_ready_mask != 0x03u)
        return;

    exint_flag_clear(EXINT_LINE_5 | EXINT_LINE_15);
    nvic_irq_enable(EXINT9_5_IRQn, DRV_FAULT_5_9_PRO, 0);
    nvic_irq_enable(EXINT15_10_IRQn, DRV_FAULT_10_15_PRO, 0);
    exint_interrupt_enable(EXINT_LINE_5 | EXINT_LINE_15, TRUE);

    /* 两路驱动均已初始化后，低电平才代表真实 nFAULT。 */
    if (gpio_input_data_bit_read(GPIOC, MOTOR0_NFAULT_PIN) == RESET)
        DrvFault_NotifyFromISR(0u);
    if (gpio_input_data_bit_read(GPIOC, MOTOR1_NFAULT_PIN) == RESET)
        DrvFault_NotifyFromISR(1u);
}

void DrvFault_NotifyFromISR(uint8_t motor_id)
{
    if (motor_id == 0u)
    {
        g_drv_fault_pending |= 0x01u;
        MotorFault_EnterFromISR(0u, &g_motor1, MOTOR0_DRV_FAULT);
        if (g_motor1.tmr != NULL)
            Shut_PWM(&g_motor1);
        gpio_bits_write(GPIOB, GPIO_PINS_12, RESET);
    }
    else if (motor_id == 1u)
    {
        g_drv_fault_pending |= 0x02u;
        MotorFault_EnterFromISR(1u, &g_motor2, MOTOR1_DRV_FAULT);
        if (g_motor2.tmr != NULL)
            Shut_PWM(&g_motor2);
        gpio_bits_write(GPIOC, GPIO_PINS_13, RESET);
    }
}

uint8_t DrvFault_IsActive(uint8_t motor_id)
{
    if (motor_id == 0u)
        return gpio_input_data_bit_read(GPIOC, MOTOR0_NFAULT_PIN) == RESET;
    if (motor_id == 1u)
        return gpio_input_data_bit_read(GPIOC, MOTOR1_NFAULT_PIN) == RESET;
    return 1u;
}

void DrvFault_Process(void)
{
    uint8_t pending;

    taskENTER_CRITICAL();
    pending = g_drv_fault_pending;
    g_drv_fault_pending = 0u;
    taskEXIT_CRITICAL();

    if (pending == 0u)
        return;

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(2)) != pdTRUE)
    {
        taskENTER_CRITICAL();
        g_drv_fault_pending |= pending;
        taskEXIT_CRITICAL();
        return;
    }

    if ((pending & 0x01u) && drv8301_1.get_status_register != NULL)
        drv8301_1.get_status_register(&drv8301_1, &g_drv_status1[0],
                                      &g_drv_status2[0]);
    if ((pending & 0x02u) && drv8301_2.get_status_register != NULL)
        drv8301_2.get_status_register(&drv8301_2, &g_drv_status1[1],
                                      &g_drv_status2[1]);
    xSemaphoreGive(mutex);
}

uint16_t DrvFault_GetStatus1(uint8_t motor_id)
{
    return (motor_id < 2u) ? g_drv_status1[motor_id] : 0u;
}

uint16_t DrvFault_GetStatus2(uint8_t motor_id)
{
    return (motor_id < 2u) ? g_drv_status2[motor_id] : 0u;
}

void EXINT9_5_IRQHandler(void)
{
    if (exint_interrupt_flag_get(EXINT_LINE_5) != RESET)
    {
        exint_flag_clear(EXINT_LINE_5);
        DrvFault_NotifyFromISR(0u);
    }
}

void EXINT15_10_IRQHandler(void)
{
    if (exint_interrupt_flag_get(EXINT_LINE_15) != RESET)
    {
        exint_flag_clear(EXINT_LINE_15);
        DrvFault_NotifyFromISR(1u);
    }
}
