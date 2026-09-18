#ifndef __PID_H
#define __PID_H
#include <stdint.h>


typedef struct pid_ctrl {
	float	  target;
	float 	measure;
	float 	err;
	float 	last_err;
	float	  kp;
	float 	ki;
	float 	kd;
	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;
	float  	last_dout;
	float	  integral;
	float 	integral_max;
	float 	out_max;
    float ff_last_target; /* 每个实例独立的前馈目标历史。 */
    uint8_t ff_initialized; /* 首次前馈调用不产生目标跳变项。 */

} pid_ctrl_t;

/* PID_Init 清零全部运行状态，并设置增益、积分累计量限幅和输出限幅。
 * 限幅参数取绝对值；传入 NULL 无操作；不改变 PID 运算公式。
 */
void PID_Init(pid_ctrl_t *pid, float kp, float ki, float kd,
              float integral_max, float out_max);

/* 参数初始化：调用 PID_Init 设置 kp/ki/kd、integral_max/out_max。
 * 限幅参数应非负；本库按调用次数积分和差分，不包含 dt。
 * single_pid_ctrl 必须由调用方先设置 err。
 * all_pid_calc：out=NULL 时为单速度环；双环误差为外环输出 + mea_in*inner_kp，
 * 因此常规负反馈使用 inner_kp=-1。err_cal_mode 0 不处理、1/2 编码器、3 角度制。
 */
void integral_to_zero(pid_ctrl_t *pid);
void single_pid_ctrl(pid_ctrl_t *pid);
float  all_pid_calc (pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);
float feedforward_pid_calc(float K_ff,pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode);

#endif
