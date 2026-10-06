#ifndef RLS2_H
#define RLS2_H
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float theta[2]; // 两项待辨识系数。
    float covariance[2][2]; // 参数估计协方差。
    float innovation; // 本次量测与预测的差值。
    uint32_t updates; // 成功更新次数。
} Rls2;

bool Rls2_Init(Rls2 *state, const float initial[2], float covariance);
// 输入 y=theta[0]*x[0]+theta[1]*x[1]，失败时保留上次状态。
bool Rls2_Update(Rls2 *state, const float x[2], float y, float forgetting);

typedef struct
{
    bool (*init)(Rls2 *state, const float initial[2], float covariance); // 建立参数和协方差初值。
    bool (*update)(Rls2 *state, const float x[2], float y, float forgetting); // 递推辨识两项线性系数。
} Rls2Ops;
typedef struct { Rls2Ops ops; } Rls2Algorithm;
extern const Rls2Algorithm rls2_algorithm; // 多实例共用算法入口。
#endif
