#ifndef LIFT_CONTROL_H
#define LIFT_CONTROL_H

#include "remote_state.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LIFT_STOPPED = 0,
    LIFT_DESCENDING,
    LIFT_ASCENDING,
    LIFT_STALLED,
    LIFT_AT_LIMIT,
    LIFT_READY,
    LIFT_CALIBRATING_UP,
    LIFT_CALIBRATING_BACKOFF
} LiftControl_State_t;

typedef enum
{
    LIFT_WAIT_NONE = 0,
    LIFT_WAIT_REMOTE,
    LIFT_WAIT_YAW,
    LIFT_WAIT_MOTOR,
    LIFT_WAIT_CHASSIS_SPEED,
    LIFT_WAIT_FAULT
} LiftControl_WaitReason_t;

typedef struct
{
    bool valid;
    LiftControl_State_t direction; /* 堵转时的运动方向。 */
    uint16_t encoder;              /* 堵转瞬间单圈编码器值。 */
    int32_t encoder_total;         /* 堵转瞬间上电累计计数。 */
    float rotor_turns;             /* encoder_total / 8192，转子圈数。 */
    uint32_t time_ms;
} LiftControl_StallSnapshot_t;

extern volatile LiftControl_State_t lift_control_state;
extern volatile LiftControl_WaitReason_t lift_wait_reason;
extern volatile bool lift_calibrated;
extern volatile int32_t lift_top_encoder_total;
extern volatile int32_t lift_bottom_encoder_total;

void LiftControl_Init(void);
void LiftControl_Update(const RemoteState_t *remote);
bool LiftControl_GetStallSnapshot(LiftControl_StallSnapshot_t *snapshot);

#endif /* LIFT_CONTROL_H */
