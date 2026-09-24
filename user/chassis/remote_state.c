#include "remote_state.h"
#include "parameter.h"
#include <string.h>

static RemoteState_t remote_state; /* 遥控任务发布给底盘任务的状态快照。 */

void RemoteState_Init(void)
{
    RemoteState_Update(NULL, false);
}

void RemoteState_Update(const RC_ctrl_t *control, bool online)
{
    RemoteState_t next;
    uint32_t primask;

    memset(&next, 0, sizeof(next));
    next.mode = REMOTE_MODE_DISABLED;
    if (online && control != NULL)
    {
        next.online = true;
        memcpy(next.channel, control->rc.ch, sizeof(next.channel));

        /* 左下档始终按云台坐标移动；只有右上档提供自旋目标。 */
        if (control->rc.s[0] == CHASSIS_SPIN_SWITCH_0_POSITION)
        {
            next.mode = REMOTE_MODE_SPIN;
            next.spin_enabled =
                control->rc.s[1] == CHASSIS_SPIN_SWITCH_1_POSITION;
        }
        else if (control->rc.s[0] == CHASSIS_FOLLOW_SWITCH_POSITION)
        {
            next.mode = REMOTE_MODE_FOLLOW;
        }
        else if (control->rc.s[0] == CHASSIS_MECHANICAL_SWITCH_POSITION)
        {
            next.mode = REMOTE_MODE_MECHANICAL;
        }
        else
        {
            next.online = false;
            memset(next.channel, 0, sizeof(next.channel));
        }
    }

    primask = __get_PRIMASK();
    __disable_irq();
    remote_state = next;
    __set_PRIMASK(primask);
}

void RemoteState_Get(RemoteState_t *state)
{
    uint32_t primask;

    if (state == NULL) { return; }
    primask = __get_PRIMASK();
    __disable_irq();
    *state = remote_state;
    __set_PRIMASK(primask);
}
