#include "remote_state.h"
#include <string.h>

static RemoteState_t remote_state; /* 通信任务发布给各控制任务的遥控状态。 */
static bool shoot_switch_seen; /* 是否已记录本次上线后的首个右拨杆档位。 */
static uint8_t shoot_previous_switch; /* 上一次有效遥控快照的右拨杆档位。 */
static bool shoot_armed; /* 右拨杆切换后锁存，断联时清除。 */

void RemoteState_Init(void)
{
    RemoteState_Update(NULL, false);
}

void RemoteState_Update(const Communication_RcControl_t *control, bool online)
{
    RemoteState_t next;
    uint32_t primask;

    memset(&next, 0, sizeof(next));
    next.mode = REMOTE_MODE_DISABLED;
    next.shoot = REMOTE_SHOOT_OFF;
    if (online && control != NULL)
    {
        next.online = true;
        memcpy(next.channel, control->rc.ch, sizeof(next.channel));

        /* 左下档始终为小陀螺模式，右上档才开启实际自旋。 */
        if (control->rc.s[0] == COMM_RC_SW_DOWN)
        {
            next.mode = REMOTE_MODE_SPIN;
            next.spin_enabled = control->rc.s[1] == COMM_RC_SW_UP;
        }
        else if (control->rc.s[0] == COMM_RC_SW_MID)
        {
            next.mode = REMOTE_MODE_MECHANICAL;
        }
        else if (control->rc.s[0] == COMM_RC_SW_UP)
        {
            next.mode = REMOTE_MODE_FOLLOW;
        }
        else
        {
            next.online = false;
            memset(next.channel, 0, sizeof(next.channel));
        }

        if (next.online &&
            (control->rc.s[1] == COMM_RC_SW_UP ||
             control->rc.s[1] == COMM_RC_SW_MID ||
             control->rc.s[1] == COMM_RC_SW_DOWN))
        {
            next.right_up = control->rc.s[1] == COMM_RC_SW_UP;
            if (next.mode == REMOTE_MODE_SPIN)
            {
                /* 左下档全保险；离开小陀螺后必须重新拨动右拨杆。 */
                shoot_switch_seen = false;
                shoot_armed = false;
            }
            else
            {
                if (shoot_switch_seen &&
                    control->rc.s[1] != shoot_previous_switch)
                {
                    shoot_armed = true;
                }
                shoot_switch_seen = true;
                next.shoot_armed = shoot_armed;
                if (shoot_armed && control->rc.s[1] == COMM_RC_SW_MID)
                {
                    next.shoot = REMOTE_SHOOT_READY;
                }
                else if (shoot_armed && next.right_up)
                {
                    next.shoot = next.mode == REMOTE_MODE_FOLLOW ?
                        REMOTE_SHOOT_CONTINUOUS : REMOTE_SHOOT_SINGLE;
                }
            }
            shoot_previous_switch = control->rc.s[1];
        }
    }

    if (!next.online ||
        (control != NULL && control->rc.s[1] != COMM_RC_SW_UP &&
         control->rc.s[1] != COMM_RC_SW_MID &&
         control->rc.s[1] != COMM_RC_SW_DOWN))
    {
        /* 初次上线只记住档位，不能把离线默认值当作真实拨杆动作。 */
        shoot_switch_seen = false;
        shoot_armed = false;
        shoot_previous_switch = 0U;
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
