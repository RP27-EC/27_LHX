#ifndef SHOOT_CONTROL_H
#define SHOOT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "remote_state.h"

typedef enum
{
    SHOOT_DIAL_IDLE = 0,       /* 保持当前位置，等待单发触发。 */
    SHOOT_DIAL_FEED,           /* 向累计单发目标正向供弹。 */
    SHOOT_DIAL_CONTINUOUS,     /* 速度闭环连续供弹。 */
    SHOOT_DIAL_STUCK_REVERSE,  /* 堵转后沿反方向退让。 */
    SHOOT_DIAL_STUCK_RELOAD    /* 退让后重新追踪原供弹目标。 */
} ShootDialState_t;

extern volatile ShootDialState_t shoot_dial_state; /* 当前拨盘状态机状态。 */
extern volatile uint32_t shoot_single_count;       /* 已完成的单发次数。 */
extern volatile uint32_t shoot_dial_stuck_count;  /* 拨盘堵转恢复次数。 */

void ShootControl_Init(void);

/* 每 1 ms 调用；单发仅在右拨杆进入上档的边沿触发。 */
void ShootControl_Update(RemoteShoot_t mode, bool right_up);

#ifdef __cplusplus
}
#endif

#endif /* SHOOT_CONTROL_H */
