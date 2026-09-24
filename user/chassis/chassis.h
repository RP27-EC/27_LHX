#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "motor3508.h"
#include "parameter.h"
#include <stdbool.h>

/* Keil Watch：当前逻辑正方向和拨轮调头停车锁存状态。 */
extern volatile bool chassis_front_reversed;
extern volatile bool chassis_turnaround_pending;

void Chassis_MecanumInverse(float front,float left,float cycle);
/* 机械模式平移方向跟随云台朝向；旋转摇杆仍直接控制底盘转向。 */
void Chassis_MechanicalUpdate(float front, float left, float cycle);
/* 调头请求由下板本地拨轮边沿产生；返回 true 时底盘所有运动应停止。 */
bool Chassis_TurnaroundUpdate(uint32_t request_count);
void Chassis_TurnaroundReset(uint32_t request_count);

/* 左拨杆上档的云台带动底盘跟随；角度帧失效时自动停轮。 */
void Chassis_FollowUpdate(float front, float left, float yaw_input);

/*电机行进速度清零*/
void Chassis_FollowReset(void);

/*小陀螺行进控制*/
void Chassis_SpinUpdate(float gimbal_front, float gimbal_left,
                        bool spin_enabled);

/*电机小陀螺旋转速度清零*/
void Chassis_SpinReset(void);




#endif
