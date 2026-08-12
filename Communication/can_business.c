/**
 ******************************************************************************
 * @file     can_business.c
 * @brief    CAN command handling for MOTOR0/MOTOR1.
 ******************************************************************************
 */

#include "can_business.h"
#include "my_can.h"
#include "foc.h"
#include "pid.h"

extern Motor_Data g_motor1;   /* physical MOTOR0: TMR1 */
extern Motor_Data g_motor2;   /* physical MOTOR1: TMR8 */

#define CAN_WHEEL_IQ_TEST_LIMIT_A  0.3f
#define CAN_CMD_TIMEOUT_MS         100u

static WheelCommand      g_wheel_cmd = {0};
static MotorSpeedCommand g_speed_cmd = {0};
static volatile float    g_motor0_iq_output = 0.0f;
static volatile float    g_motor1_iq_output = 0.0f;
static CanMotorMode      g_motor_mode = CAN_MOTOR_MODE_CURRENT;
static TickType_t        g_last_cmd_tick = 0;
static uint8_t           g_cmd_active = 0;

static inline float clamp_f(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
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

static void reset_controllers(void)
{
    Motor_Reset_Current_Controller(&g_motor1);
    Motor_Reset_Current_Controller(&g_motor2);
    PID_Reset(&g_motor1.speed_pid);
}

static void disable_command(void)
{
    g_motor0_iq_output = 0.0f;
    g_motor1_iq_output = 0.0f;
    g_speed_cmd.motor0_speed_ref = 0.0f;
    g_speed_cmd.motor1_speed_ref = 0.0f;
    g_motor_mode = CAN_MOTOR_MODE_CURRENT;

    if (g_cmd_active)
    {
        reset_controllers();
        g_cmd_active = 0u;
    }
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

    g_last_cmd_tick = xTaskGetTickCount();

    if (id == CAN_ID_WHEEL_COMMAND)
    {
        g_wheel_cmd.motor0_iq_raw = get_i16_le(&data[0]);
        g_wheel_cmd.motor1_iq_raw = get_i16_le(&data[2]);
        g_wheel_cmd.sequence = get_u16_le(&data[4]);
        g_wheel_cmd.enable = (data[6] & CAN_CMD_FLAG_ENABLE) ? 1u : 0u;
        g_wheel_cmd.motor0_iq_ref = (float)g_wheel_cmd.motor0_iq_raw * 0.01f;
        g_wheel_cmd.motor1_iq_ref = (float)g_wheel_cmd.motor1_iq_raw * 0.01f;
        g_motor_mode = CAN_MOTOR_MODE_CURRENT;

        if (g_wheel_cmd.enable)
        {
            g_motor0_iq_output = clamp_f(g_wheel_cmd.motor0_iq_ref,
                                         -CAN_WHEEL_IQ_TEST_LIMIT_A,
                                          CAN_WHEEL_IQ_TEST_LIMIT_A);
            g_motor1_iq_output = clamp_f(g_wheel_cmd.motor1_iq_ref,
                                         -CAN_WHEEL_IQ_TEST_LIMIT_A,
                                          CAN_WHEEL_IQ_TEST_LIMIT_A);
            g_cmd_active = 1u;
        }
        else
        {
            disable_command();
        }

        send_ack(&g_wheel_cmd);
        return E_OK;
    }

    g_speed_cmd.motor0_speed_raw = get_i16_le(&data[0]);
    g_speed_cmd.motor1_speed_raw = get_i16_le(&data[2]);
    g_speed_cmd.sequence = get_u16_le(&data[4]);
    g_speed_cmd.enable = (data[6] & CAN_CMD_FLAG_ENABLE) ? 1u : 0u;
    g_speed_cmd.motor0_speed_ref = (float)g_speed_cmd.motor0_speed_raw * 0.1f;
    g_speed_cmd.motor1_speed_ref = (float)g_speed_cmd.motor1_speed_raw * 0.1f;
    g_motor_mode = CAN_MOTOR_MODE_SPEED;

    if (g_speed_cmd.enable)
    {
        g_cmd_active = 1u;
    }
    else
    {
        disable_command();
    }

    return E_OK;
}

float can_business_get_motor0_iq_output(void) { return g_motor0_iq_output; }
float can_business_get_motor1_iq_output(void) { return g_motor1_iq_output; }
CanMotorMode can_business_get_motor_mode(void) { return g_motor_mode; }
float can_business_get_motor0_speed_target(void) { return g_speed_cmd.motor0_speed_ref; }
float can_business_get_motor1_speed_target(void) { return g_speed_cmd.motor1_speed_ref; }

void can_business_tick(void)
{
    if ((xTaskGetTickCount() - g_last_cmd_tick) > pdMS_TO_TICKS(CAN_CMD_TIMEOUT_MS))
        disable_command();
}

void can_business_send_motor0_speed_state(void)
{
    uint8_t data[8];
    int16_t cmd_raw = to_i16_sat(g_speed_cmd.motor0_speed_ref * 10.0f);
    int16_t measured_raw = to_i16_sat(g_motor1.velocity * 10.0f);

    data[0] = (uint8_t)(cmd_raw & 0xFF);
    data[1] = (uint8_t)((cmd_raw >> 8) & 0xFF);
    data[2] = (uint8_t)(measured_raw & 0xFF);
    data[3] = (uint8_t)((measured_raw >> 8) & 0xFF);
    data[4] = (uint8_t)(g_speed_cmd.sequence & 0xFF);
    data[5] = (uint8_t)((g_speed_cmd.sequence >> 8) & 0xFF);
    data[6] = g_motor1.run_state;
    data[7] = (g_cmd_active ? 0x01u : 0x00u)
            | (g_motor_mode == CAN_MOTOR_MODE_SPEED ? 0x02u : 0x00u);

    my_can_send_std(CAN_ID_MOTOR0_SPEED_STATE, data, CAN_CMD_DLC);
}

void can_business_send_motor0_speed_diag(void)
{
    uint8_t data[8];
    int16_t iq_target_raw = to_i16_sat(g_motor1.control.iq_current_target * 1000.0f);
    int16_t iq_feedback_raw = to_i16_sat(g_motor1.control.iq_current_feedback * 1000.0f);
    int16_t vq_raw = to_i16_sat(g_motor1.voltage_dq.Vq * 1000.0f);

    data[0] = (uint8_t)(iq_target_raw & 0xFF);
    data[1] = (uint8_t)((iq_target_raw >> 8) & 0xFF);
    data[2] = (uint8_t)(iq_feedback_raw & 0xFF);
    data[3] = (uint8_t)((iq_feedback_raw >> 8) & 0xFF);
    data[4] = (uint8_t)(vq_raw & 0xFF);
    data[5] = (uint8_t)((vq_raw >> 8) & 0xFF);
    data[6] = Motor_Is_Speed_Startup_Boost_Active(&g_motor1) ? 0x01u : 0x00u;
    data[7] = 0x00u;

    my_can_send_std(CAN_ID_MOTOR0_SPEED_DIAG, data, CAN_CMD_DLC);
}
