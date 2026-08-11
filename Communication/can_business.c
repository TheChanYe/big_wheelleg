/**
 ******************************************************************************
 * @file     can_business.c
 * @brief    CAN business layer implementation
 * @note     Protocol: 0x101 WheelCommand → parse → 0x201 ACK
 ******************************************************************************
 */

#include "can_business.h"
#include "my_can.h"
#include "foc.h"

extern Motor_Data g_motor1;   /* left wheel  */
extern Motor_Data g_motor2;   /* right wheel */

#define MODULE_NAME     "can_biz"

#ifdef MODE_LOG_TAG
#undef MODE_LOG_TAG
#endif
#define MODE_LOG_TAG    MODULE_NAME

/* ==========================  safety  ========================== */

#define CAN_WHEEL_IQ_TEST_LIMIT_A   0.3f    /* first-stage validation clamp    */
#define CAN_CMD_TIMEOUT_MS          100u    /* command-loss timeout            */

/* ==========================  static state  ========================== */

static WheelCommand g_wheel_cmd = {0};

/*  outputs actually delivered to FOC (clamped, timeout-gated)  */
static volatile float  g_left_iq_output   = 0.0f;
static volatile float  g_right_iq_output  = 0.0f;
static TickType_t      g_last_cmd_tick    = 0;
static uint8_t         g_cmd_active       = 0;    /* edge-triggered: 1=receiving valid cmd */

/* ==========================  helpers  ========================== */

static inline float clamp_f(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ==========================  Little-Endian helpers  ========================== */

static int16_t get_i16_le(const uint8_t *p)
{
    return (int16_t)(
        ((uint16_t)p[0]) |
        ((uint16_t)p[1] << 8)
    );
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)(
        ((uint16_t)p[0]) |
        ((uint16_t)p[1] << 8)
    );
}

/* ==========================  ACK sender  ==========================
 *
 *  Builds 0x201 from raw values (no float → int round-trip).
 *  byte6 = enable flag, byte7 = result (0x00 = success).
 */

static void send_ack(const WheelCommand *cmd)
{
    uint8_t ack[8];

    ack[0] = (uint8_t)(cmd->left_iq_raw & 0xFF);
    ack[1] = (uint8_t)((cmd->left_iq_raw >> 8) & 0xFF);

    ack[2] = (uint8_t)(cmd->right_iq_raw & 0xFF);
    ack[3] = (uint8_t)((cmd->right_iq_raw >> 8) & 0xFF);

    ack[4] = (uint8_t)(cmd->sequence & 0xFF);
    ack[5] = (uint8_t)((cmd->sequence >> 8) & 0xFF);

    ack[6] = cmd->enable & 0x01u;       /* flags: only bit0 used */
    ack[7] = 0x00u;                     /* result: success        */

    my_can_send_std(CAN_ID_WHEEL_ACK, ack, CAN_CMD_DLC);
}

/* ==========================  public API  ========================== */

const WheelCommand *can_business_get_wheel_command(void)
{
    return &g_wheel_cmd;
}

int can_business_process_frame(uint16_t id,
                               const uint8_t *data,
                               uint8_t len)
{
    /*  Parameter check  */
    if (data == NULL)
    {
        return E_PARAM;
    }

    /*  Only 0x101 with DLC==8  */
    if (id != CAN_ID_WHEEL_COMMAND)
    {
        return CAN_RX_EMPTY;   /* not ours, caller skips  */
    }
    if (len != CAN_CMD_DLC)
    {
        return CAN_RX_EMPTY;   /* malformed, ignore       */
    }

    /*  Parse raw values (little-endian)  */
    g_wheel_cmd.left_iq_raw  = get_i16_le(&data[0]);
    g_wheel_cmd.right_iq_raw = get_i16_le(&data[2]);
    g_wheel_cmd.sequence     = get_u16_le(&data[4]);
    g_wheel_cmd.enable       = (data[6] & CAN_CMD_FLAG_ENABLE) ? 1u : 0u;

    /*  Convert to engineering values: 1 LSB = 0.01 A  */
    g_wheel_cmd.left_iq_ref  = (float)g_wheel_cmd.left_iq_raw  * 0.01f;
    g_wheel_cmd.right_iq_ref = (float)g_wheel_cmd.right_iq_raw * 0.01f;

    /*  Update command timestamp (even if disabled — proves we heard it)  */
    g_last_cmd_tick = xTaskGetTickCount();

    /*  Apply safety clamp + enable gate  */
    if (g_wheel_cmd.enable)
    {
        g_left_iq_output  = clamp_f(g_wheel_cmd.left_iq_ref,
                                    -CAN_WHEEL_IQ_TEST_LIMIT_A,
                                     CAN_WHEEL_IQ_TEST_LIMIT_A);
        g_right_iq_output = clamp_f(g_wheel_cmd.right_iq_ref,
                                    -CAN_WHEEL_IQ_TEST_LIMIT_A,
                                     CAN_WHEEL_IQ_TEST_LIMIT_A);
        g_cmd_active = 1u;   /* receiving valid enabled commands */
    }
    else
    {
        g_left_iq_output  = 0.0f;
        g_right_iq_output = 0.0f;
        /* edge: ACTIVE → DISABLED — reset PI once */
        if (g_cmd_active)
        {
            Motor_Reset_Current_Controller(&g_motor1);
            Motor_Reset_Current_Controller(&g_motor2);
            g_cmd_active = 0u;
        }
    }

    /*  Send ACK with raw values (always reflects request, not clamped output)  */
    send_ack(&g_wheel_cmd);

    return E_OK;
}

/* ==========================  output accessors  ========================== */

float can_business_get_left_iq_output(void)
{
    return g_left_iq_output;
}

float can_business_get_right_iq_output(void)
{
    return g_right_iq_output;
}

/* ==========================  timeout monitor  ==========================
 *
 *  Called every poll cycle from the business task.
 *  If no valid 0x101 frame arrives within CAN_CMD_TIMEOUT_MS,
 *  both outputs are forced to zero.
 *  Edge-triggered: only resets PI controllers on the FIRST tick
 *  that crosses the timeout threshold (ACTIVE → TIMEOUT).
 */

void can_business_tick(void)
{
    TickType_t now = xTaskGetTickCount();

    if ((now - g_last_cmd_tick) > pdMS_TO_TICKS(CAN_CMD_TIMEOUT_MS))
    {
        g_left_iq_output  = 0.0f;
        g_right_iq_output = 0.0f;

        /* edge: ACTIVE → TIMEOUT — reset PI once */
        if (g_cmd_active)
        {
            Motor_Reset_Current_Controller(&g_motor1);
            Motor_Reset_Current_Controller(&g_motor2);
            g_cmd_active = 0u;
        }
    }
}

/* ==========================  right-wheel FOC diagnostic  ==========================
 *
 *  CAN ID 0x202, DLC=8, 50 Hz — control-chain health.
 *  byte0-1: iq_current_target   (int16 LE, 1 LSB = 0.001 A)
 *  byte2-3: iq_current_feedback (int16 LE, 1 LSB = 0.001 A)
 *  byte4-5: Vq                  (int16 LE, 1 LSB = 0.001 V)
 *  byte6:   run_state
 *  byte7:   bit0 = CAN enable, rest reserved = 0
 */

static int16_t to_i16_sat(float val)
{
    if (val > 32767.0f) return 32767;
    if (val < -32768.0f) return -32768;
    return (int16_t)val;
}

void can_business_send_right_wheel_state(void)
{
    uint8_t data[8];
    int16_t iq_tgt_raw;
    int16_t iq_fb_raw;
    int16_t vq_raw;

    iq_tgt_raw = to_i16_sat(g_motor2.control.iq_current_target   * 1000.0f);
    iq_fb_raw  = to_i16_sat(g_motor2.control.iq_current_feedback * 1000.0f);
    vq_raw     = to_i16_sat(g_motor2.voltage_dq.Vq               * 1000.0f);

    data[0] = (uint8_t)(iq_tgt_raw & 0xFF);
    data[1] = (uint8_t)((iq_tgt_raw >> 8) & 0xFF);

    data[2] = (uint8_t)(iq_fb_raw & 0xFF);
    data[3] = (uint8_t)((iq_fb_raw >> 8) & 0xFF);

    data[4] = (uint8_t)(vq_raw & 0xFF);
    data[5] = (uint8_t)((vq_raw >> 8) & 0xFF);

    data[6] = g_motor2.run_state;
    data[7] = g_wheel_cmd.enable ? 0x01u : 0x00u;

    my_can_send_std(CAN_ID_RIGHT_WHEEL_STATE, data, 8);
}

/* ==========================  right-wheel current diagnostic  ==========================
 *
 *  CAN ID 0x203, DLC=8, 50 Hz — phase & dq-axis current health.
 *  byte0-1: Ia (int16 LE, 1 LSB = 0.001 A)
 *  byte2-3: Ib (int16 LE, 1 LSB = 0.001 A)
 *  byte4-5: Id (int16 LE, 1 LSB = 0.001 A)
 *  byte6-7: Iq (int16 LE, 1 LSB = 0.001 A)
 */

void can_business_send_right_wheel_current_diag(void)
{
    uint8_t data[8];
    int16_t ia_raw;
    int16_t ib_raw;
    int16_t id_raw;
    int16_t iq_raw;

    ia_raw = to_i16_sat(g_motor2.current_abc.Ia * 1000.0f);
    ib_raw = to_i16_sat(g_motor2.current_abc.Ib * 1000.0f);
    id_raw = to_i16_sat(g_motor2.current_dq.Id  * 1000.0f);
    iq_raw = to_i16_sat(g_motor2.current_dq.Iq  * 1000.0f);

    data[0] = (uint8_t)(ia_raw & 0xFF);
    data[1] = (uint8_t)((ia_raw >> 8) & 0xFF);

    data[2] = (uint8_t)(ib_raw & 0xFF);
    data[3] = (uint8_t)((ib_raw >> 8) & 0xFF);

    data[4] = (uint8_t)(id_raw & 0xFF);
    data[5] = (uint8_t)((id_raw >> 8) & 0xFF);

    data[6] = (uint8_t)(iq_raw & 0xFF);
    data[7] = (uint8_t)((iq_raw >> 8) & 0xFF);

    my_can_send_std(CAN_ID_RIGHT_WHEEL_CURRENT_DIAG, data, 8);
}
