/**
 ******************************************************************************
 * @file     safety_limit.c
 * @brief    Safety-layer Iq clamp and slew limiter.
 ******************************************************************************
 */

#include "safety_limit.h"

static float g_effective_iq[MOTOR_COMMAND_COUNT] = {0.0f};
static float g_iq_limit[MOTOR_COMMAND_COUNT] = {
    MOTOR_COMMAND_IQ_LIMIT_A, MOTOR_COMMAND_IQ_LIMIT_A
};
static TickType_t g_iq_update_tick[MOTOR_COMMAND_COUNT] = {0};
static uint8_t g_iq_tick_valid[MOTOR_COMMAND_COUNT] = {0};

static float clamp_f(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void SafetyLimit_Init(void)
{
    uint8_t motor_id;

    for (motor_id = 0u; motor_id < MOTOR_COMMAND_COUNT; motor_id++)
        g_iq_limit[motor_id] = MOTOR_COMMAND_IQ_LIMIT_A;

    SafetyLimit_ForceZeroAll();
}

float MotorCommand_UpdateIq(float current, float target, float max_delta)
{
    if (target > current + max_delta)
        return current + max_delta;
    if (target < current - max_delta)
        return current - max_delta;
    return target;
}

float SafetyLimit_UpdateIq(uint8_t motor_id, float command_iq,
                           uint8_t command_enabled)
{
    TickType_t now;
    TickType_t elapsed;
    float max_delta;

    if (motor_id >= MOTOR_COMMAND_COUNT)
        return 0.0f;

    now = xTaskGetTickCount();
    if (!command_enabled)
    {
        g_effective_iq[motor_id] = 0.0f;
        g_iq_update_tick[motor_id] = now;
        return 0.0f;
    }

    command_iq = clamp_f(command_iq, -MOTOR_COMMAND_IQ_LIMIT_A,
                         MOTOR_COMMAND_IQ_LIMIT_A);
    command_iq = clamp_f(command_iq, -g_iq_limit[motor_id],
                         g_iq_limit[motor_id]);
    if (!g_iq_tick_valid[motor_id])
    {
        g_iq_update_tick[motor_id] = now;
        g_iq_tick_valid[motor_id] = 1u;
        return g_effective_iq[motor_id];
    }

    elapsed = now - g_iq_update_tick[motor_id];
    g_iq_update_tick[motor_id] = now;
    max_delta = (float)elapsed * (float)portTICK_PERIOD_MS * 0.001f
        * MOTOR_COMMAND_IQ_SLEW_A_PER_S;

    g_effective_iq[motor_id] = MotorCommand_UpdateIq(g_effective_iq[motor_id],
                                                       command_iq, max_delta);
    return g_effective_iq[motor_id];
}

void SafetyLimit_SetIqLimit(uint8_t motor_id, float limit_a)
{
    if (motor_id >= MOTOR_COMMAND_COUNT)
        return;

    g_iq_limit[motor_id] = clamp_f(limit_a, 0.0f,
                                   MOTOR_COMMAND_IQ_LIMIT_A);
}

void SafetyLimit_ForceZero(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COMMAND_COUNT)
        return;

    g_effective_iq[motor_id] = 0.0f;
    g_iq_update_tick[motor_id] = xTaskGetTickCount();
    g_iq_tick_valid[motor_id] = 0u;
}

void SafetyLimit_ForceZeroFromISR(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COMMAND_COUNT)
        return;

    g_effective_iq[motor_id] = 0.0f;
    g_iq_tick_valid[motor_id] = 0u;
}

void SafetyLimit_ForceZeroAll(void)
{
    uint8_t motor_id;

    for (motor_id = 0u; motor_id < MOTOR_COMMAND_COUNT; motor_id++)
        SafetyLimit_ForceZero(motor_id);
}
