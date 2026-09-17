
/**
 * @file        rp_math.c
 * @author      RobotPilots
 * @Version     v1.1
 * @brief       RobotPilots Robots' Math Libaray.
 * @update
 *              v1.0(11-September-2020)
 *              v1.1(13-November-2021)
 *                  1.增加位操作函数
 */

/* Includes ------------------------------------------------------------------*/
#include "rp_math.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief  低通滤波,K∈(0,1)，K 越大，滤波效果越弱
 * @param  上次的滤波输出X_last  ，新的输入X_new ，滤波系数K
 * @return  滤波后数值
 */
float Lowpass(float X_last, float X_new, float K)
{
	return (X_last + (X_new - X_last) * K);
}

/**
 *	@brief	过半圈处理 angle：源数据 cycle:数据范围
 */
float motor_half_cycle(float angle, float max)
{
	if (abs(angle) > (max / 2))
	{
		if (angle >= 0)
			angle += -max;
		else
			angle += max;
	}
	return angle;
}

int16_t RampInt(int16_t final, int16_t now, int16_t ramp)
{
	int32_t buffer = 0;

	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}

float RampFloat(float final, float now, float ramp)
{
	float buffer = 0;

	buffer = final - now;
	if (buffer > 0)
	{
		if (buffer > ramp)
			now += ramp;
		else
			now += buffer;
	}
	else
	{
		if (buffer < -ramp)
			now += -ramp;
		else
			now += buffer;
	}

	return now;
}




float DeathZoom(float input, float center, float death)
{
	if (abs(input - center) < death)
		return center;
	return input;
}
/**
  * @name   Time_Trigger_inloop
  * @brief  在循环里定时触发
  * @note   ①需要外部定义变量存时间、第一次不直接触发标志位,防止多处调用共用时间或标志位导致错误
  *         ②要么自己外部清标志位，要么执行的内容要紧跟这个函数后面，不然容易错过触发时间
  *         ③*ignore_first_trigger_flag需要初始为0
			 ④*ignore_first_trigger_flag 退出长按后要清零
  * @param  private_flag: 标志位指针，用于指示是否触发（1为触发）
  * @param  last_trigger_tick: 用于存储上次触发的时间（外部变量，需初始化为0）
  * @param  ignore_first_trigger_flag: 是否忽略第一次触发的标志位（外部变量，需初始化为0,退出长按后要清零）
  * @param  delay_tick: 触发的时间间隔（单位：毫秒）
  * @param  if_ignore_first: 是否忽略第一次触发（1为忽略，0为不忽略）
  * @author HERMIT_PURPLE
  */
void Time_Trigger_inloop(Time_trigger_t *Time_trigger_struct)
{
	if (Time_trigger_struct->private_flag == NULL || Time_trigger_struct->last_trigger_tick == NULL || Time_trigger_struct->ignore_first_trigger_flag == NULL)
	{
		return;
	}

	// 如果进来这个函数不直接触发一次,就先赋值一次上次的时间
	if (Time_trigger_struct->if_ignore_first == 1 && Time_trigger_struct->ignore_first_trigger_flag == 0)
	{
		*Time_trigger_struct->ignore_first_trigger_flag = 1;
		*Time_trigger_struct->last_trigger_tick = HAL_GetTick();
	}

	if ((HAL_GetTick() - *Time_trigger_struct->last_trigger_tick > Time_trigger_struct->delay_tick) && (Time_trigger_struct->ignore_first_trigger_flag != 0 || Time_trigger_struct->if_ignore_first == 0))
	{
		*Time_trigger_struct->private_flag = 1;
		*Time_trigger_struct->last_trigger_tick = HAL_GetTick();
	}
	/*内部清标志位*/
	else
	{
		*Time_trigger_struct->private_flag = Time_trigger_struct->flag_before_trigger;
	}
}


uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  将uint转为float，并对正负做处理
  * @param
  * @retval 
  */
float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief 步进式限幅滤波函数
 * @param new_value 当前采样值
 * @param last_value 上一次的滤波输出值
 * @param max_step 最大步进值（死区阈值）
 * @return 本次滤波后的值
 */
float step_limit_filter(float new_value, float last_value, float max_step)
{
    float filtered_value;
    float difference = new_value - last_value;

    // 如果变化量超过最大步进值，则进行限幅步进处理
    if (fabs(difference) > max_step) {
        filtered_value = last_value + sgn(difference) * max_step;
    } else {
        // 变化量在允许范围内，直接采用新采样值
        filtered_value = new_value;
    }
    
    return filtered_value; 
}

