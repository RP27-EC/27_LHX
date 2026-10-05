#include "upper_tasks.h"
#include "cmsis_os2.h"
#include "application_config.h"
#include "cloud_terrace.h"
#include "communication.h"
#include "dial_motor.h"
#include "imu.h"
#include "lift_control.h"
#include "motor2006.h"
#include "motor3508.h"
#include "remote_state.h"
#include "shoot_control.h"

volatile UpperTaskTimingState upper_task_timing;
volatile UpperShootBlockReason upper_shoot_block_reason;

static void UpperTasks_WaitPeriod(uint32_t *next_tick, uint32_t period_ticks,
                                  volatile uint32_t *overruns)
{
    if (period_ticks == 0U) { period_ticks = 1U; }
    *next_tick += period_ticks;
    if (osDelayUntil(*next_tick) != osOK)
    {
        (*overruns)++;
        // 超期后至少让出一个 tick，避免高优先级任务立即重跑。
        (void)osDelay(1U);
        *next_tick = osKernelGetTickCount();
    }
}

static void UpperTasks_CommunicationStep(void)
{
    Communication_RcControl_t remote_control;

    board_link.process();
    RemoteState_Update(&remote_control,
                       board_link.rc_get(&remote_control));
}

static void UpperTasks_ShootStep(void)
{
    static bool shoot_rearm_required = true; // 许可丢失后等待新的手动操作。
    static bool last_input_valid; // 上周期遥控输入有效。
    static bool last_keyboard_active; // 上周期输入来源。
    static bool last_shoot_armed; // 上周期键鼠摩擦轮布防状态。
    static uint8_t last_right_switch; // 上周期物理右拨杆位置。
    bool friction_motors_online;
    bool dial_motor_online;
    bool manual_rearm = false;
    bool shoot_allowed;
    LiftSafetyState_t safety;
    RemoteState_t remote;
    RemoteShoot_t shoot_mode;

    motor3508.heartbeat();
    dial_motor.heartbeat();
    friction_motors_online =
        motor3508.online_check(SHOOT_LEFT_FRIC_MOTOR_ID) &&
        motor3508.online_check(SHOOT_RIGHT_FRIC_MOTOR_ID);
    dial_motor_online = dial_motor.online_check();
    RemoteState_Get(&remote);

    shoot_allowed = LiftControl_SafetyGet(&safety) && safety.shoot_allowed &&
                    (remote.mode.chassis != REMOTE_MODE_SPIN || remote.input.keyboard_active);
    if (last_input_valid && remote.safety.online &&
        remote.input.keyboard_active == last_keyboard_active)
    {
        if (remote.input.keyboard_active)
        { manual_rearm = !last_shoot_armed && remote.safety.shoot_armed; }
        else
        {
            // 与遥控解析一致：许可恢复后重新拨动右杆，保险档不启动。
            manual_rearm = remote.input.right_switch != last_right_switch &&
                           (remote.input.right_switch == COMM_RC_SW_MID ||
                            remote.input.right_switch == COMM_RC_SW_UP) &&
                           remote.safety.shoot_armed;
        }
    }
    if (!shoot_allowed || !remote.safety.online ||
        (last_input_valid &&
         last_keyboard_active != remote.input.keyboard_active))
    { shoot_rearm_required = true; }
    else if (manual_rearm)
    { shoot_rearm_required = false; }
    last_input_valid = remote.safety.online;
    last_keyboard_active = remote.input.keyboard_active;
    last_shoot_armed = remote.safety.shoot_armed;
    last_right_switch = remote.input.right_switch;

    if (!remote.safety.online) { upper_shoot_block_reason = UPPER_SHOOT_BLOCK_REMOTE; }
    else if (remote.mode.chassis == REMOTE_MODE_SPIN && !remote.input.keyboard_active)
    { upper_shoot_block_reason = UPPER_SHOOT_BLOCK_SPIN; }
    else if (!shoot_allowed) { upper_shoot_block_reason = UPPER_SHOOT_BLOCK_LIFT; }
    else if (!remote.safety.shoot_armed || shoot_rearm_required)
    { upper_shoot_block_reason = UPPER_SHOOT_BLOCK_REARM; }
    else if (!friction_motors_online)
    { upper_shoot_block_reason = UPPER_SHOOT_BLOCK_FRICTION; }
    else if (!dial_motor_online) { upper_shoot_block_reason = UPPER_SHOOT_BLOCK_DIAL; }
    else { upper_shoot_block_reason = UPPER_SHOOT_BLOCK_NONE; }

    shoot_mode = remote.safety.online && remote.safety.shoot_armed &&
        shoot_allowed && !shoot_rearm_required && friction_motors_online ?
        remote.mode.shooting : REMOTE_SHOOT_OFF;
    // 拨盘离线时只允许摩擦轮待发，不允许供弹。
    if (!dial_motor_online && shoot_mode != REMOTE_SHOOT_OFF)
    { shoot_mode = REMOTE_SHOOT_READY; }

    ShootControl_SetIdleHoldEnabled(remote.safety.online && dial_motor_online);

    if (remote.input.keyboard_active)
    {
        if (!dial_motor_online)
        {
            ShootControl_ResetKeyboard(remote.event.shoot_single_request_count);
            ShootControl_Update(shoot_mode, false);
        }
        else
        {
            ShootControl_UpdateKeyboard(shoot_mode,
                                        remote.event.shoot_single_request_count);
        }
    }
    else
    {
        ShootControl_ResetKeyboard(remote.event.shoot_single_request_count);
        ShootControl_Update(shoot_mode, remote.input.right_up);
    }
}

static void UpperTasks_ControlStep(void)
{
    RemoteState_t remote;

    // 一次遥控快照先判安全，再供云台和升降使用；C1 不会落后一整周期。
    (void)gimbal_imu_driver.update();
    motor2006.heartbeat();
    RemoteState_Get(&remote);
    LiftControl_SafetyUpdate(&remote);
    CloudTerrace_Update(&remote);
    LiftControl_Update(&remote);
}

void UpperTasks_RunCommunication(void)
{
    uint32_t next_tick;

    RemoteState_Init();
    next_tick = osKernelGetTickCount();
    for (;;)
    {
        UpperTasks_CommunicationStep();
        UpperTasks_WaitPeriod(&next_tick,
                              upper_communication_task_config.period_ticks,
                              &upper_task_timing.communication_overruns);
    }
}

void UpperTasks_RunControl(void)
{
    uint32_t next_tick;

    CloudTerrace_Init();
    LiftControl_Init();
    next_tick = osKernelGetTickCount();
    for (;;)
    {
        UpperTasks_ControlStep();
        UpperTasks_WaitPeriod(&next_tick, cloud_config.control_period_ticks,
                              &upper_task_timing.gimbal_overruns);
    }
}

void UpperTasks_RunShoot(void)
{
    uint32_t next_tick;

    ShootControl_Init();
    next_tick = osKernelGetTickCount();
    for (;;)
    {
        UpperTasks_ShootStep();
        UpperTasks_WaitPeriod(&next_tick, shoot_config.control_period_ticks,
                              &upper_task_timing.shoot_overruns);
    }
}
