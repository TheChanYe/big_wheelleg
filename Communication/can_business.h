/**
 ******************************************************************************
 * @file     can_business.h
 * @brief    CAN business layer - command/ACK protocol on top of my_can
 * @note     Receives 0x101 (WheelCommand), responds 0x201 (ACK).
 *           Does NOT control FOC. Protocol validation only.
 ******************************************************************************
 */

#ifndef __CAN_BUSINESS_H__
#define __CAN_BUSINESS_H__

#include "main.h"

/* ==========================  CAN ID definition  ========================== */

#define CAN_ID_WHEEL_COMMAND    0x101u   /* host → slave: wheel iq command   */
#define CAN_ID_WHEEL_ACK        0x201u   /* slave → host: parsed ACK          */

#define CAN_CMD_DLC              8u      /* fixed DLC for command & ACK       */

/* ==========================  flag bits  ========================== */

#define CAN_CMD_FLAG_ENABLE     0x01u    /* bit0: enable (1) / disable (0)    */

/* ==========================  WheelCommand  ==========================
 *
 *  Holds both CAN raw values and converted engineering values,
 *  1 LSB = 0.01 A (int16_t, little-endian on wire).
 */

typedef struct
{
    /*  raw CAN payload (exactly as received)  */
    int16_t   left_iq_raw;
    int16_t   right_iq_raw;

    /*  converted engineering values  */
    float     left_iq_ref;         /*  A   */
    float     right_iq_ref;        /*  A   */

    /*  protocol fields  */
    uint16_t  sequence;
    uint8_t   enable;              /*  0 = disable, 1 = enable  */
} WheelCommand;

/* ==========================  public API  ========================== */

/*  read-only access to last parsed command (caller must not free)  */
const WheelCommand *can_business_get_wheel_command(void);

/*  safety-limited output Iq (clamped by CAN_WHEEL_IQ_TEST_LIMIT_A)
 *  — what the motor tasks should actually use via Current_loop  */
float can_business_get_left_iq_output(void);
float can_business_get_right_iq_output(void);

/*  call every poll cycle; zeros outputs when CAN command times out  */
void can_business_tick(void);

/*  process one received CAN frame
 *  return: E_OK = frame was consumed (matched & ACK-ed)
 *          E_PARAM = NULL pointer
 *          CAN_RX_EMPTY = ignored (not our ID or malformed)  */
int can_business_process_frame(uint16_t id,
                               const uint8_t *data,
                               uint8_t len);

#endif /* __CAN_BUSINESS_H__ */
