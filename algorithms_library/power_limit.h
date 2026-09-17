#ifndef __POWER_LIMIT_H
#define __POWER_LIMIT_H


#include "rp_config.h"
#include "rp_math.h"

#include "judge.h"

#define CHAS_SP_MAX_OUT 8000.f



typedef struct{
	int16_t   current_raw;
	int16_t   encoder_speed;
	float     current;
	float     speed;


}Power_Rx_Info_t;


typedef struct{
  float power_err;
	uint32_t power_err_cnt;
  uint32_t root_err_cnt;


}Power_Err_t;


typedef struct{
	float     power;


}Power_Predict_t;


typedef struct{
	float                 fitting_coefficient[6];
	Power_Rx_Info_t       info;
  Power_Err_t           err;
  Power_Predict_t       predict;

}Power_Uint_t;



typedef struct Power_Limit_Struct_t{
  Power_Uint_t    uint[4];
	
	void (*ctrl)(struct Power_Limit_Struct_t* limit);

}Power_Limit_t;














void Chassis_Motor_Power_Limit(int16_t *data);

#endif

