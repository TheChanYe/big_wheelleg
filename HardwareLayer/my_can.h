/**
 ******************************************************************************
 * @file     my_can.h
 * @brief    CAN1 driver header - TX and polling RX
 * @note     AT32F403ARGT7, CAN1 on PB8(RX)/PB9(TX), MAX3051 transceiver
 ******************************************************************************
 */

#ifndef __MY_CAN_H__
#define __MY_CAN_H__

#include "main.h"

/*  Task enable switches  */
#define ENABLE_CAN_TX_TEST        0   /* 周期发送 0x123 (已验证, 关闭)      */
#define ENABLE_CAN_LOOPBACK_TEST  0   /* 回环测试 (已验证, 关闭)            */
#define ENABLE_CAN_BUSINESS       1   /* 业务层: 0x101 cmd → 0x201 ACK    */

/*  CAN1 TX test frame definitions  */
#define CAN_TEST_ID               0x123u
#define CAN_TEST_DLC              8u

/*  RX poll status (cannot use E_OK=0, so use positive value)  */
#define CAN_RX_EMPTY              1   /* FIFO 为空, 无新帧 */

/*  Public interface  */
int my_can_init(void);
int my_can_send_std(uint16_t id, const uint8_t *data, uint8_t len);
int my_can_receive_std(uint16_t *id, uint8_t *data, uint8_t *len);

#endif /* __MY_CAN_H__ */
