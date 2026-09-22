#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "motor3508.h"
#include "parameter.h"

void Chassis_MecanumInverse(float front,float left,float cycle);
/* 左拨杆上档的云台带动底盘跟随；角度帧失效时自动停轮。 */
void Chassis_FollowUpdate(float front, float left, float yaw_input);
void Chassis_FollowReset(void);
void Chassis_SpinUpdate(float gimbal_front, float gimbal_left);
void Chassis_SpinReset(void);




#endif
