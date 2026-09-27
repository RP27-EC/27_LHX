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

    Communication_Process();
    RemoteState_Update(&remote_control,
                       Communication_RC_Get(&remote_control));
}

static void UpperTasks_ShootStep(void)
{
    bool friction_motors_online;
    bool dial_motor_online;
    RemoteState_t remote;
    RemoteShoot_t shoot_mode;

    Motor3508_Heartbeat();
    DialMotor_Heartbeat();
    friction_motors_online =
        Motor3508_OnlineCheck(SHOOT_LEFT_FRIC_MOTOR_ID) &&
        Motor3508_OnlineCheck(SHOOT_RIGHT_FRIC_MOTOR_ID);
    dial_motor_online = DialMotor_OnlineCheck();
    RemoteState_Get(&remote);

    shoot_mode = remote.safety.online && remote.safety.shoot_armed &&
        friction_motors_online ? remote.mode.shooting : REMOTE_SHOOT_OFF;
    // 拨盘离线时只允许摩擦轮待发，不允许供弹。
    if (!dial_motor_online && shoot_mode != REMOTE_SHOOT_OFF)
    { shoot_mode = REMOTE_SHOOT_READY; }

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

    // 升降依据本周期云台位置更新。
    (void)GimbalImu_Update();
    CloudTerrace_Update();
    Motor2006_Heartbeat();
    RemoteState_Get(&remote);
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
