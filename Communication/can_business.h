/**
 ******************************************************************************
 * @file     can_business.h
 * @brief    CAN motor command and MOTOR0 speed-state protocol.
 ******************************************************************************
 */

#ifndef __CAN_BUSINESS_H__
#define __CAN_BUSINESS_H__

#include "main.h"

#define CAN_ID_WHEEL_COMMAND          0x101u
#define CAN_ID_MOTOR_SPEED_COMMAND    0x102u
#define CAN_ID_WHEEL_ACK              0x201u
#define CAN_ID_MOTOR0_SPEED_STATE     0x202u
#define CAN_ID_MOTOR0_SPEED_DIAG      0x203u

#define CAN_CMD_DLC                   8u
#define CAN_CMD_FLAG_ENABLE           0x01u

typedef struct
{
    int16_t  motor0_iq_raw;
    int16_t  motor1_iq_raw;
    float    motor0_iq_ref;
    float    motor1_iq_ref;
    uint16_t sequence;
    uint8_t  enable;
} WheelCommand;

typedef struct
{
    int16_t  motor0_speed_raw;
    int16_t  motor1_speed_raw;
    float    motor0_speed_ref;
    float    motor1_speed_ref;
    uint16_t sequence;
    uint8_t  enable;
} MotorSpeedCommand;

typedef enum
{
    CAN_MOTOR_MODE_CURRENT = 0,
    CAN_MOTOR_MODE_SPEED
} CanMotorMode;

const WheelCommand *can_business_get_wheel_command(void);
float can_business_get_motor0_iq_output(void);
float can_business_get_motor1_iq_output(void);
CanMotorMode can_business_get_motor_mode(void);
float can_business_get_motor0_speed_target(void);
float can_business_get_motor1_speed_target(void);

void can_business_tick(void);
void can_business_send_motor0_speed_state(void);
void can_business_send_motor0_speed_diag(void);

int can_business_process_frame(uint16_t id,
                               const uint8_t *data,
                               uint8_t len);

#endif /* __CAN_BUSINESS_H__ */
