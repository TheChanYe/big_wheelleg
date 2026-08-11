/**
 ******************************************************************************
 * @file     my_can.c
 * @brief    CAN1 driver - TX and polling RX
 * @note     AT32F403ARGT7, CAN1 on PB8(RX)/PB9(TX), MAX3051 transceiver
 *           SYSCLK=240MHz, APB1=120MHz, 500Kbps baud rate
 ******************************************************************************
 */

#include "my_can.h"

#define MODULE_NAME     "my_can"

#ifdef MODE_LOG_TAG
#undef MODE_LOG_TAG
#endif
#define MODE_LOG_TAG    MODULE_NAME

/* ==========================  CAN1 初始化  ==========================
 *
 *  GPIO   : PB8 = CAN1_RX, PB9 = CAN1_TX
 *  MUX    : CAN_MUX_10
 *  Mode   : Normal, auto-retransmit, no loopback, no silent
 *  Baud   : 500 Kbps
 *
 *  120MHz / (12 × (1 + 15 + 4)) = 500Kbps
 *  sample point = (1+15)/20 = 80%
 */

int my_can_init(void)
{
    gpio_init_type       gpio_init_struct;
    can_base_type         can_base_struct;
    can_baudrate_type     can_baudrate_struct;

    /* ---- 1. Enable clocks ---- */
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);  /* GPIOB           */
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);  /* IOMUX           */
    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);   /* CAN1            */

    /* ---- 2. GPIO config: PB8 = CAN1_RX (input pull-up) ---- */
    gpio_default_para_init(&gpio_init_struct);
    gpio_init_struct.gpio_mode  = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pins  = GPIO_PINS_8;
    gpio_init_struct.gpio_pull  = GPIO_PULL_UP;
    gpio_init(GPIOB, &gpio_init_struct);

    /* ---- 3. GPIO config: PB9 = CAN1_TX (AF push-pull) ---- */
    gpio_init_struct.gpio_mode        = GPIO_MODE_MUX;
    gpio_init_struct.gpio_pins        = GPIO_PINS_9;
    gpio_init_struct.gpio_out_type    = GPIO_OUTPUT_PUSH_PULL;
    gpio_init_struct.gpio_pull        = GPIO_PULL_NONE;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
    gpio_init(GPIOB, &gpio_init_struct);

    /* ---- 4. Remap CAN1 to PB8/PB9  (CAN_MUX_10) ---- */
    gpio_pin_remap_config(CAN_MUX_10, TRUE);

    /* ---- 5. CAN1 reset ---- */
    can_reset(CAN1);

    /* ---- 6. CAN1 base init: normal mode, auto re-send enabled ---- */
    can_default_para_init(&can_base_struct);
    can_base_struct.mode_selection = CAN_MODE_COMMUNICATE;  /* normal mode          */
    can_base_struct.ttc_enable     = FALSE;                 /* no time-triggered    */
    can_base_struct.aebo_enable    = TRUE;                  /* auto exit bus-off    */
    can_base_struct.aed_enable     = FALSE;                 /* no auto exit doze    */
    can_base_struct.prsf_enable    = FALSE;                 /* auto retransmit ON   */
    can_base_struct.mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED;
    can_base_struct.mmssr_selection  = CAN_SENDING_BY_REQUEST;

    if (can_base_init(CAN1, &can_base_struct) != SUCCESS)
    {
        log_error("CAN1 base init failed");
        return E_ERROR;
    }

    /* ---- 7. Baud rate: 500 Kbps ---- */
    can_baudrate_default_para_init(&can_baudrate_struct);
    can_baudrate_struct.baudrate_div = 12;           /* 120MHz / 12 / (1+15+4) = 500Kbps */
    can_baudrate_struct.rsaw_size    = CAN_RSAW_1TQ;
    can_baudrate_struct.bts1_size    = CAN_BTS1_15TQ;
    can_baudrate_struct.bts2_size    = CAN_BTS2_4TQ;

    if (can_baudrate_set(CAN1, &can_baudrate_struct) != SUCCESS)
    {
        log_error("CAN1 baud rate set failed");
        return E_ERROR;
    }

    /* ---- 8. RX filter: Filter 0, 32-bit mask mode, FIFO0, accept all ---- */
    {
        can_filter_init_type  rx_filter;

        can_filter_default_para_init(&rx_filter);
        rx_filter.filter_activate_enable = TRUE;
        rx_filter.filter_mode            = CAN_FILTER_MODE_ID_MASK;
        rx_filter.filter_bit             = CAN_FILTER_32BIT;
        rx_filter.filter_fifo            = CAN_FILTER_FIFO0;
        rx_filter.filter_number          = 0;
        rx_filter.filter_id_high         = 0;
        rx_filter.filter_id_low          = 0;
        rx_filter.filter_mask_high       = 0;
        rx_filter.filter_mask_low        = 0;

        can_filter_init(CAN1, &rx_filter);
    }

    log_inform("CAN1 init OK  (PB8/PB9, 500Kbps, RX filter enable)");
    return E_OK;
}

/* ==========================  CAN1 标准帧发送  ==========================
 *
 *  Standard CAN, 11-bit ID, Data Frame, 0~8 bytes.
 *  No blocking wait — returns immediately.
 */

int my_can_send_std(uint16_t id, const uint8_t *data, uint8_t len)
{
    can_tx_message_type tx_msg;
    uint8_t             mailbox_num;

    /*  Parameter check  */
    if (len > 8)
    {
        return E_PARAM;
    }
    if (len > 0 && data == NULL)
    {
        return E_NULL;
    }

    /*  Fill TX message  */
    tx_msg.standard_id = (uint32_t)id;
    tx_msg.extended_id = 0;
    tx_msg.id_type     = CAN_ID_STANDARD;
    tx_msg.frame_type  = CAN_TFT_DATA;
    tx_msg.dlc         = len;

    if (len > 0)
    {
        uint8_t i;
        for (i = 0; i < len; i++)
        {
            tx_msg.data[i] = data[i];
        }
    }

    /*  Transmit - returns CAN_TX_STATUS_NO_EMPTY (0x04) if no free mailbox  */
    mailbox_num = can_message_transmit(CAN1, &tx_msg);

    if (mailbox_num == CAN_TX_STATUS_NO_EMPTY)
    {
        return E_ERROR;    /*  no free mailbox  */
    }

    return E_OK;           /*  submitted to mailbox  */
}

/* ==========================  CAN1 标准帧轮询接收  ==========================
 *
 *  Non-blocking poll of FIFO0.
 *
 *  Return:  E_OK         — one frame received
 *           CAN_RX_EMPTY — FIFO0 empty
 *           E_PARAM      — NULL pointer
 *
 *  Accepts only Standard ID + Data Frame + DLC ≤ 8.
 *  Extended ID / Remote frames are silently dropped (caller sees CAN_RX_EMPTY).
 */

int my_can_receive_std(uint16_t *id, uint8_t *data, uint8_t *len)
{
    can_rx_message_type  rx_msg;

    /*  Parameter check  */
    if (id == NULL || data == NULL || len == NULL)
    {
        return E_PARAM;
    }

    /*  FIFO0 pending?  */
    if (can_receive_message_pending_get(CAN1, CAN_RX_FIFO0) == 0)
    {
        return CAN_RX_EMPTY;
    }

    /*  Read message from FIFO0  */
    can_message_receive(CAN1, CAN_RX_FIFO0, &rx_msg);
    can_receive_fifo_release(CAN1, CAN_RX_FIFO0);

    /*  Accept only Standard ID + Data Frame + DLC ≤ 8  */
    if (rx_msg.id_type != CAN_ID_STANDARD)
    {
        return CAN_RX_EMPTY;   /* drop extended frames silently */
    }
    if (rx_msg.frame_type != CAN_TFT_DATA)
    {
        return CAN_RX_EMPTY;   /* drop remote frames silently  */
    }
    if (rx_msg.dlc > 8)
    {
        return CAN_RX_EMPTY;   /* malformed, drop              */
    }

    /*  Output  */
    *id  = (uint16_t)rx_msg.standard_id;
    *len = rx_msg.dlc;

    if (rx_msg.dlc > 0)
    {
        uint8_t i;
        for (i = 0; i < rx_msg.dlc; i++)
        {
            data[i] = rx_msg.data[i];
        }
    }

    return E_OK;
}
