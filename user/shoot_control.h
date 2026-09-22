#ifndef SHOOT_CONTROL_H
#define SHOOT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SHOOT_DIAL_IDLE = 0,
    SHOOT_DIAL_FEED,
    SHOOT_DIAL_STUCK_REVERSE,
    SHOOT_DIAL_STUCK_RELOAD
} ShootDialState_t;

extern volatile ShootDialState_t shoot_dial_state;
extern volatile uint32_t shoot_single_count;
extern volatile uint32_t shoot_dial_stuck_count;

void ShootControl_Init(void);

/* 每 1 ms 调用；单发命令在 false->true 上升沿触发一次。 */
void ShootControl_Update(bool control_enabled, bool single_shot_command);

#ifdef __cplusplus
}
#endif

#endif /* SHOOT_CONTROL_H */
