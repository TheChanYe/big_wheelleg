/**
 ******************************************************************************
 * @file     can_business.c
 * @brief    CAN business layer implementation
 * @note     Protocol: 0x101 WheelCommand → parse → 0x201 ACK
 ******************************************************************************
 */

#include "can_business.h"
#include "my_can.h"

#define MODULE_NAME     "can_biz"

#ifdef MODE_LOG_TAG
#undef MODE_LOG_TAG
#endif
#define MODE_LOG_TAG    MODULE_NAME

/* ==========================  static state  ========================== */

static WheelCommand g_wheel_cmd = {0};

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

    /*  Send ACK with raw values  */
    send_ack(&g_wheel_cmd);

    return E_OK;
}
