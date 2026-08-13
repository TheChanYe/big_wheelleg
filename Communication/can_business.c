/**
 ******************************************************************************
 * @file     can_business.c
 * @brief    CAN command handling for MOTOR0/MOTOR1.
 ******************************************************************************
 */

#include "can_business.h"
#include "my_can.h"
#include "foc.h"
#include "safety_limit.h"
#include "motor_fault.h"
#include "motor_adc.h"
#include "drv_fault.h"
#include "motor_service.h"

extern Motor_Data g_motor1;   /* physical MOTOR0: TMR1 */
extern Motor_Data g_motor2;   /* physical MOTOR1: TMR8 */

#define CAN_CMD_TIMEOUT_MS         100u
#define CAN_CMD_STALE_MS           20u
#define CAN_FAST_STATE_PERIOD_MS      10u

static WheelCommand      g_wheel_cmd = {0};
static MotorSpeedCommand g_speed_cmd = {0};
static CanMotorMode      g_motor_mode = CAN_MOTOR_MODE_CURRENT;
static CanCommState      g_comm_state = CAN_COMM_TIMEOUT;
static CanSequenceStats  g_wheel_sequence = {0};
static CanSequenceStats  g_speed_sequence = {0};
static TickType_t        g_last_new_cmd_tick = 0;
static uint8_t           g_cmd_active = 0;
static volatile uint32_t g_can_telemetry_tx_fail = 0u;

int can_business_init(void)
{
    int ret = my_can_init();
    if (ret == E_OK)
        log_inform("CAN business task started (RX 0x101/0x102)");
    return ret;
}

void can_business_process(void)
{
    uint16_t id;
    uint8_t data[8];
    uint8_t len;
    TickType_t now;

    /* 每个周期清空 FIFO，保持原任务的处理顺序。 */
    while (my_can_receive_std(&id, data, &len) == E_OK)
        can_business_process_frame(id, data, len);

    can_business_tick();
    now = xTaskGetTickCount();

    /* 1ms phase 调度：单周期最多一个 telemetry，命令 RX 始终优先。 */
    switch ((uint32_t)now % 100u)
    {
    case 0u: case 10u: case 20u: case 30u: case 40u:
    case 50u: case 60u: case 70u: case 80u: case 90u:
        can_business_send_motor0_wheel_state();
        break;
    case 5u: case 15u: case 25u: case 35u: case 45u:
    case 55u: case 65u: case 75u: case 85u: case 95u:
        can_business_send_motor1_wheel_state();
        break;
    case 2u: case 22u: case 42u: case 62u: case 82u:
        can_business_send_motor0_speed_state();
        break;
    case 7u: case 27u: case 47u: case 67u: case 87u:
        can_business_send_motor1_speed_state();
        break;
    case 3u: case 23u: case 43u: case 63u: case 83u:
        can_business_send_motor0_speed_diag();
        break;
    case 8u: case 28u: case 48u: case 68u: case 88u:
        can_business_send_motor1_speed_diag();
        break;
    case 1u:
        can_business_send_safety_diag();
        break;
    case 4u:
        can_business_send_drv_diag();
        break;
    case 6u:
        can_business_send_system_diag();
        break;
    default:
        break;
    }

}

void can_business_tx_test_process(void)
{
    static uint16_t counter = 0;
    uint8_t data[8];

    data[0] = 0xCA;
    data[1] = 0x4E;
    data[2] = (uint8_t)(counter & 0xFF);
    data[3] = (uint8_t)((counter >> 8) & 0xFF);
    data[4] = 0x11;
    data[5] = 0x22;
    data[6] = 0x33;
    data[7] = 0x44;

    if (my_can_send_std(CAN_TEST_ID, data, CAN_TEST_DLC) != E_OK)
        log_error("CAN TX failed (counter=%u)", counter);
    counter++;
}

void can_business_loopback_test_process(void)
{
    uint16_t id;
    uint8_t data[8];
    uint8_t len;

    if (my_can_receive_std(&id, data, &len) == E_OK && id == 0x124)
        my_can_send_std(0x125, data, len);
}

static int16_t get_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t to_i16_sat(float val)
{
    if (val > 32767.0f) return 32767;
    if (val < -32768.0f) return -32768;
    return (int16_t)val;
}

static uint16_t get_active_sequence(void)
{
    return (g_motor_mode == CAN_MOTOR_MODE_SPEED)
        ? g_speed_cmd.sequence : g_wheel_cmd.sequence;
}

static uint8_t accept_sequence(CanSequenceStats *stats, uint16_t sequence)
{
    uint16_t delta;

    if (!stats->valid)
    {
        stats->last_sequence = sequence;
        stats->valid = 1u;
        return 1u;
    }

    delta = (uint16_t)(sequence - stats->last_sequence);
    if (delta == 0u)
    {
        stats->duplicate_count++;
        return 0u;
    }

    if (delta >= 0x8000u)
    {
        stats->out_of_order_count++;
        return 0u;
    }

    if (delta > 1u)
        stats->drop_count += (uint32_t)(delta - 1u);

    stats->last_sequence = sequence;
    return 1u;
}

static void accept_new_command(void)
{
    g_last_new_cmd_tick = xTaskGetTickCount();
    g_comm_state = CAN_COMM_OK;
    g_cmd_active = 1u;
}

static void reset_controllers(void)
{
    Motor_Reset_Current_Controller(&g_motor1);
    Motor_Reset_Current_Controller(&g_motor2);
    Motor_Reset_Speed_Controller(&g_motor1);
    Motor_Reset_Speed_Controller(&g_motor2);
}

static void disable_command(void)
{
    g_speed_cmd.motor0_speed_ref = 0.0f;
    g_speed_cmd.motor1_speed_ref = 0.0f;
    g_motor_mode = CAN_MOTOR_MODE_CURRENT;
    SafetyLimit_ForceZeroAll();

    if (g_cmd_active)
    {
        reset_controllers();
        g_cmd_active = 0u;
    }
}

static void process_fault_clear(uint8_t flags)
{
    if (flags & CAN_CMD_FLAG_CLEAR_MOTOR0)
        MotorService_ClearFault(0u);
    if (flags & CAN_CMD_FLAG_CLEAR_MOTOR1)
        MotorService_ClearFault(1u);
}

static void send_ack(const WheelCommand *cmd)
{
    uint8_t ack[8];

    ack[0] = (uint8_t)(cmd->motor0_iq_raw & 0xFF);
    ack[1] = (uint8_t)((cmd->motor0_iq_raw >> 8) & 0xFF);
    ack[2] = (uint8_t)(cmd->motor1_iq_raw & 0xFF);
    ack[3] = (uint8_t)((cmd->motor1_iq_raw >> 8) & 0xFF);
    ack[4] = (uint8_t)(cmd->sequence & 0xFF);
    ack[5] = (uint8_t)((cmd->sequence >> 8) & 0xFF);
    ack[6] = cmd->enable;
    ack[7] = 0x00u;

    my_can_send_std(CAN_ID_WHEEL_ACK, ack, CAN_CMD_DLC);
}

const WheelCommand *can_business_get_wheel_command(void)
{
    return &g_wheel_cmd;
}

int can_business_process_frame(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (data == NULL)
        return E_PARAM;
    if ((id != CAN_ID_WHEEL_COMMAND && id != CAN_ID_MOTOR_SPEED_COMMAND)
        || len != CAN_CMD_DLC)
        return CAN_RX_EMPTY;

    if (id == CAN_ID_WHEEL_COMMAND)
    {
        WheelCommand command;

        command.motor0_iq_raw = get_i16_le(&data[0]);
        command.motor1_iq_raw = get_i16_le(&data[2]);
        command.sequence = get_u16_le(&data[4]);
        command.enable = (data[6] & CAN_CMD_FLAG_ENABLE) ? 1u : 0u;
        command.motor0_iq_ref = (float)command.motor0_iq_raw * 0.01f;
        command.motor1_iq_ref = (float)command.motor1_iq_raw * 0.01f;

        if (!command.enable)
        {
            g_wheel_cmd = command;
            g_motor_mode = CAN_MOTOR_MODE_CURRENT;
            disable_command();
            process_fault_clear(data[7]);
            g_comm_state = CAN_COMM_OK;
        }
        else if (accept_sequence(&g_wheel_sequence, command.sequence))
        {
            g_wheel_cmd = command;
            g_motor_mode = CAN_MOTOR_MODE_CURRENT;
            accept_new_command();
        }

        send_ack(&command);
        return E_OK;
    }

    {
        MotorSpeedCommand command;

        command.motor0_speed_raw = get_i16_le(&data[0]);
        command.motor1_speed_raw = get_i16_le(&data[2]);
        command.sequence = get_u16_le(&data[4]);
        command.enable = (data[6] & CAN_CMD_FLAG_ENABLE) ? 1u : 0u;
        command.motor0_speed_ref = (float)command.motor0_speed_raw * 0.1f;
        command.motor1_speed_ref = (float)command.motor1_speed_raw * 0.1f;

        if (!command.enable)
        {
            g_speed_cmd = command;
            g_motor_mode = CAN_MOTOR_MODE_SPEED;
            disable_command();
            process_fault_clear(data[7]);
            g_comm_state = CAN_COMM_OK;
        }
        else if (accept_sequence(&g_speed_sequence, command.sequence))
        {
            g_speed_cmd = command;
            g_motor_mode = CAN_MOTOR_MODE_SPEED;
            if (g_speed_cmd.motor0_speed_raw == 0)
                Motor_Reset_Speed_Controller(&g_motor1);
            if (g_speed_cmd.motor1_speed_raw == 0)
                Motor_Reset_Speed_Controller(&g_motor2);
            accept_new_command();
        }
    }

    return E_OK;
}

float can_business_get_motor0_iq_output(void) { return g_wheel_cmd.motor0_iq_ref; }
float can_business_get_motor1_iq_output(void) { return g_wheel_cmd.motor1_iq_ref; }
uint8_t can_business_current_command_is_active(void)
{
    return (g_cmd_active && g_motor_mode == CAN_MOTOR_MODE_CURRENT) ? 1u : 0u;
}
CanMotorMode can_business_get_motor_mode(void) { return g_motor_mode; }
CanCommState can_business_get_comm_state(void) { return g_comm_state; }
const CanSequenceStats *can_business_get_wheel_sequence_stats(void)
{
    return &g_wheel_sequence;
}
const CanSequenceStats *can_business_get_speed_sequence_stats(void)
{
    return &g_speed_sequence;
}
float can_business_get_motor0_speed_target(void) { return g_speed_cmd.motor0_speed_ref; }
float can_business_get_motor1_speed_target(void) { return g_speed_cmd.motor1_speed_ref; }

void can_business_tick(void)
{
    TickType_t elapsed;

    if (!g_cmd_active)
        return;

    elapsed = xTaskGetTickCount() - g_last_new_cmd_tick;
    if (elapsed > pdMS_TO_TICKS(CAN_CMD_TIMEOUT_MS))
    {
        disable_command();
        g_comm_state = CAN_COMM_TIMEOUT;
    }
    else if (elapsed > pdMS_TO_TICKS(CAN_CMD_STALE_MS))
    {
        g_comm_state = CAN_COMM_STALE;
    }
    else
    {
        g_comm_state = CAN_COMM_OK;
    }
}

static void send_speed_state(uint16_t id, Motor_Data *motor, float target)
{
    uint8_t data[8];
    float state_target = (g_motor_mode == CAN_MOTOR_MODE_SPEED) ? target : 0.0f;
    uint16_t sequence = get_active_sequence();
    int16_t cmd_raw = to_i16_sat(state_target * 10.0f);
    int16_t measured_raw = to_i16_sat(motor->velocity * 10.0f);

    data[0] = (uint8_t)(cmd_raw & 0xFF);
    data[1] = (uint8_t)((cmd_raw >> 8) & 0xFF);
    data[2] = (uint8_t)(measured_raw & 0xFF);
    data[3] = (uint8_t)((measured_raw >> 8) & 0xFF);
    /*
     * 状态帧 sequence 必须对应当前控制模式。
     * Current Mode 使用 0x101，Speed Mode 使用 0x102 的 sequence。
     */
    data[4] = (uint8_t)(sequence & 0xFF);
    data[5] = (uint8_t)((sequence >> 8) & 0xFF);
    data[6] = motor->run_state;
    data[7] = (g_cmd_active ? 0x01u : 0x00u)
            | (g_motor_mode == CAN_MOTOR_MODE_SPEED ? 0x02u : 0x00u);

    if (my_can_send_std(id, data, CAN_CMD_DLC) != E_OK)
        g_can_telemetry_tx_fail++;
}

static void send_speed_diag(uint16_t id, Motor_Data *motor)
{
    uint8_t data[8];
    int16_t iq_target_raw = to_i16_sat(motor->control.iq_current_target * 1000.0f);
    int16_t iq_feedback_raw = to_i16_sat(motor->control.iq_current_feedback * 1000.0f);
    int16_t vq_raw = to_i16_sat(motor->voltage_dq.Vq * 1000.0f);

    data[0] = (uint8_t)(iq_target_raw & 0xFF);
    data[1] = (uint8_t)((iq_target_raw >> 8) & 0xFF);
    data[2] = (uint8_t)(iq_feedback_raw & 0xFF);
    data[3] = (uint8_t)((iq_feedback_raw >> 8) & 0xFF);
    data[4] = (uint8_t)(vq_raw & 0xFF);
    data[5] = (uint8_t)((vq_raw >> 8) & 0xFF);
    data[6] = Motor_Is_Speed_Startup_Boost_Active(motor) ? 0x01u : 0x00u;
    data[7] = motor->control.speed_startup_failed ? 0x01u : 0x00u;

    if (my_can_send_std(id, data, CAN_CMD_DLC) != E_OK)
        g_can_telemetry_tx_fail++;
}

void can_business_send_motor0_speed_state(void)
{
    send_speed_state(CAN_ID_MOTOR0_SPEED_STATE, &g_motor1,
                     g_speed_cmd.motor0_speed_ref);
}

void can_business_send_motor1_speed_state(void)
{
    send_speed_state(CAN_ID_MOTOR1_SPEED_STATE, &g_motor2,
                     g_speed_cmd.motor1_speed_ref);
}

void can_business_send_motor0_speed_diag(void)
{
    send_speed_diag(CAN_ID_MOTOR0_SPEED_DIAG, &g_motor1);
}

void can_business_send_motor1_speed_diag(void)
{
    send_speed_diag(CAN_ID_MOTOR1_SPEED_DIAG, &g_motor2);
}

void can_business_send_safety_diag(void)
{
    uint8_t data[8];
    uint16_t bus_raw = (uint16_t)to_i16_sat(Motor_ADC_GetBusVoltage() * 100.0f);
    MotorFaultBits fault_bits = MotorFault_GetBits();
    int16_t temp0_raw = to_i16_sat(g_motor1.mos_temp * 10.0f);
    int16_t temp1_raw = to_i16_sat(g_motor2.mos_temp * 10.0f);

    data[0] = (uint8_t)(temp0_raw & 0xFF);
    data[1] = (uint8_t)((temp0_raw >> 8) & 0xFF);
    data[2] = (uint8_t)(temp1_raw & 0xFF);
    data[3] = (uint8_t)((temp1_raw >> 8) & 0xFF);
    data[4] = (uint8_t)(bus_raw & 0xFF);
    data[5] = (uint8_t)((bus_raw >> 8) & 0xFF);
    data[6] = (uint8_t)(fault_bits & 0xFFu);
    data[7] = (uint8_t)((fault_bits >> 8) & 0xFFu);

    if (my_can_send_std(CAN_ID_SAFETY_DIAG, data, CAN_CMD_DLC) != E_OK)
        g_can_telemetry_tx_fail++;
}

void can_business_send_drv_diag(void)
{
    uint8_t data[8];
    uint16_t status0_1 = DrvFault_GetStatus1(0u);
    uint16_t status0_2 = DrvFault_GetStatus2(0u);
    uint16_t status1_1 = DrvFault_GetStatus1(1u);
    uint16_t status1_2 = DrvFault_GetStatus2(1u);

    data[0] = (uint8_t)(status0_1 & 0xFFu);
    data[1] = (uint8_t)(status0_1 >> 8);
    data[2] = (uint8_t)(status0_2 & 0xFFu);
    data[3] = (uint8_t)(status0_2 >> 8);
    data[4] = (uint8_t)(status1_1 & 0xFFu);
    data[5] = (uint8_t)(status1_1 >> 8);
    data[6] = (uint8_t)(status1_2 & 0xFFu);
    data[7] = (uint8_t)(status1_2 >> 8);

    if (my_can_send_std(CAN_ID_DRV_DIAG, data, CAN_CMD_DLC) != E_OK)
        g_can_telemetry_tx_fail++;
}

void can_business_send_system_diag(void)
{
    uint8_t data[8];
    MotorFaultBits fault_bits = MotorFault_GetBits();

    data[0] = (uint8_t)(fault_bits & 0xFFu);
    data[1] = (uint8_t)((fault_bits >> 8) & 0xFFu);
    data[2] = (uint8_t)((fault_bits >> 16) & 0xFFu);
    data[3] = (uint8_t)((fault_bits >> 24) & 0xFFu);
    data[4] = (uint8_t)g_comm_state;
    data[5] = g_motor1.run_state;
    data[6] = g_motor2.run_state;
    data[7] = (g_cmd_active ? 0x01u : 0x00u)
        | (g_motor_mode == CAN_MOTOR_MODE_SPEED ? 0x02u : 0x00u)
        | (Motor_ADC_BusVoltageValid() ? 0x04u : 0x00u)
        | (g_can_telemetry_tx_fail ? 0x08u : 0x00u);

    if (my_can_send_std(CAN_ID_SYSTEM_DIAG, data, CAN_CMD_DLC) != E_OK)
        g_can_telemetry_tx_fail++;
}

static void send_wheel_state(uint16_t id, Motor_Data *motor)
{
    uint8_t data[8];
    float direction = (motor->tmr == TMR1) ? MOTOR0_COMMAND_DIRECTION
                                            : MOTOR1_COMMAND_DIRECTION;
    int32_t position_raw = (int32_t)((motor->filter_angle
        + (float)motor->circle_num * cpr) * direction * 1000.0f);
    int16_t velocity_raw = to_i16_sat(motor->velocity * direction * 10.0f);
    int16_t iq_raw = to_i16_sat(motor->control.iq_current_feedback
        * direction * 100.0f);

    data[0] = (uint8_t)(position_raw & 0xFF);
    data[1] = (uint8_t)((position_raw >> 8) & 0xFF);
    data[2] = (uint8_t)((position_raw >> 16) & 0xFF);
    data[3] = (uint8_t)((position_raw >> 24) & 0xFF);
    data[4] = (uint8_t)(velocity_raw & 0xFF);
    data[5] = (uint8_t)((velocity_raw >> 8) & 0xFF);
    data[6] = (uint8_t)(iq_raw & 0xFF);
    data[7] = (uint8_t)((iq_raw >> 8) & 0xFF);
    if (my_can_send_std(id, data, CAN_CMD_DLC) != E_OK)
        g_can_telemetry_tx_fail++;
}

void can_business_send_motor0_wheel_state(void)
{
    send_wheel_state(CAN_ID_MOTOR0_WHEEL_STATE, &g_motor1);
}

void can_business_send_motor1_wheel_state(void)
{
    send_wheel_state(CAN_ID_MOTOR1_WHEEL_STATE, &g_motor2);
}
