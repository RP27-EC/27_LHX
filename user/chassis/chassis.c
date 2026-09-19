#include "chassis.h"


static float Chassis_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Chassis_Max4(float value_0,float value_1,float value_2,float value_3)
{
    float maximum = value_0;

    if (value_1 > maximum) { maximum = value_1; }
    if (value_2 > maximum) { maximum = value_2; }
    if (value_3 > maximum) { maximum = value_3; }

    return maximum;
}

void Chassis_MecanumInverse(float front,float left,float cycle)
{
    static float motor[4];
    uint32_t i;

    motor[0] =  front - left - cycle;
    motor[1] =  front + left - cycle;
    motor[2] = -front - left - cycle;
    motor[3] = -front + left - cycle;

    float max_abs = Chassis_Max4(Chassis_Abs(motor[0]),
                                 Chassis_Abs(motor[1]),
                                 Chassis_Abs(motor[2]),
                                 Chassis_Abs(motor[3]));

    if (max_abs > CHASSIS_MAX_MOTOR_RPM)
    {
        float scale = CHASSIS_MAX_MOTOR_RPM / max_abs;

        for (i = 0; i < 4; i++)
        {
            motor[i] *= scale;
        }
    }

    (void)Motor3508_control(1U,
                            (int16_t)motor[0],
                            (int16_t)motor[1],
                            (int16_t)motor[2],
                            (int16_t)motor[3]);
}

