#include "Power_Limit.h"
#include "judge.h"


///*-祖传功率-*/
//void Chassis_Motor_Power_Limit(int16_t *data)
//{
//	float buffer = judge.pkt->buffer_energy;
//	float heat_rate, Limit_k, CHAS_LimitOutput, CHAS_TotalOutput;
//	
//	float OUT_MAX = 0.f;
//	
//	OUT_MAX = CHAS_SP_MAX_OUT * 4.f;
//	
//	if(buffer > 60.f)buffer = 60.f;//防止飞坡之后缓冲250J变为正增益系数
//	
//	Limit_k = buffer / 60.f;
//	
//	if(buffer < 25.f)
//		Limit_k = Limit_k * Limit_k ;// * Limit_k; //3方
//	else
//		Limit_k = Limit_k;// * str->Limit_k; //平方
//	
//	if(buffer < 60.f)
//		CHAS_LimitOutput = Limit_k * OUT_MAX;
//	else 
//		CHAS_LimitOutput = OUT_MAX;    
//	
//	CHAS_TotalOutput = abs(data[0]) + abs(data[1]) + abs(data[2]) + abs(data[3]) ;
//	
//	heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;
//	
//  if(CHAS_TotalOutput >= CHAS_LimitOutput)
//  {
//		for(char i = 0 ; i < 4 ; i++)
//		{	
//			data[i] = (int16_t)(data[i] * heat_rate);	
//		}
//	}
//}



//static void Chassis_Power_Limit(Chassis_t * chassis)
//{
//	  static float last_buffer = 0;
//		float limit_output_speed[4];
//	
//		float buffer = (float)judge.pkt->buffer_energy;
//		float heat_rate;//输出电流缩放比例
//		float Limit_k; //轮组速度和缩放比例
//		float CHAS_LimitOutput;//缩放后轮组最大速度之和
//		float CHAS_TotalOutput;//轮组电流之和
//		
//		//获取理想的底盘输出
//		for(uint8_t i = 0;i<4;i++)
//		{
//			limit_output_speed[i] = chassis->wheel->motor[i]->rx_info->speed;
//		}
//		
//		float OUT_MAX = 0;
//	
//		OUT_MAX = CHASSIS_MAX_SPEED * 4;//最大速度之和
//		
//		if(buffer > 60.f)
//		{
//			buffer = 60.f;//防止飞坡之后缓冲250J变为正增益系数
//		}
//		
//		Limit_k = buffer / 60.f;  //最大为1，飞坡后底盘一直最大速度运行
//		
//		if(buffer < 25.f)
//		{
//			Limit_k = Limit_k * Limit_k ;//缓冲没多小就更慢一点
//		}
//		else
//		{
//			Limit_k = Limit_k;// 缓冲能量还有比较多就限制一点
//		}
//			
//		if(buffer < 60.f)
//		{
//			CHAS_LimitOutput = Limit_k * OUT_MAX; //只要缓冲能量没满才限制
//		}
//		else 
//		{
//			CHAS_LimitOutput = OUT_MAX;    //缓冲能量满的就全速前进
//		}
//			
//		CHAS_TotalOutput = fabs(limit_output_speed[0]) + fabs(limit_output_speed[1]) + fabs(limit_output_speed[2]) + fabs(limit_output_speed[3]) ;
//		
//		if(CHAS_TotalOutput >= CHAS_LimitOutput)
//		{
//			heat_rate = CHAS_LimitOutput / CHAS_TotalOutput;//电流缩放比例 = 利用现在剩余缓冲能量算出的速度和限制比例 * 轮组最大速度和 / 解算出的理想轮组速度和
//		}
//		else{
//		  heat_rate = 1.f;
//		}
//		
//		for(uint8_t i = 0 ; i < 4 ; i++) 
//		{	
//			chassis->out.wheel_powerd_out[i] = (float)(chassis->out.wheel_initial_out[i] * heat_rate);	
//		}
//		
//		if(buffer <= 0 && last_buffer > 0)
//		{
//			power_fail ++;
//		}
//		
//		last_buffer = buffer;
//		
//}
