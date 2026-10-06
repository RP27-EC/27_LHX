#include "rls2.h"
#include <float.h>
#include <stddef.h>
#include <string.h>

#define RLS_COVARIANCE_MAX 1000000.0f // 限制弱激励下的协方差增长。
#define RLS_COVARIANCE_MIN 0.00000001f // 防止单精度舍入使对角项归零。

static bool finite_value(float value)
{ return value >= -FLT_MAX && value <= FLT_MAX; }

// 清除辨识历史，为两个系数建立独立初始不确定度。
bool Rls2_Init(Rls2 *state, const float initial[2], float covariance)
{
    if (state == NULL || initial == NULL || !finite_value(initial[0]) ||
        !finite_value(initial[1]) || !(covariance > 0.0f && covariance <= RLS_COVARIANCE_MAX))
    { return false; }
    memset(state, 0, sizeof(*state));
    state->theta[0] = initial[0]; state->theta[1] = initial[1];
    state->covariance[0][0] = state->covariance[1][1] = covariance;
    return true;
}

bool Rls2_Update(Rls2 *state, const float x[2], float y, float forgetting)
{
    Rls2 next;
    float prior[2][2], gain[2], transform[2][2], temp[2][2], denominator;
    unsigned i, j, k;
    if (state == NULL || x == NULL || !finite_value(x[0]) || !finite_value(x[1]) ||
        !finite_value(y) || !(forgetting >= 0.9f && forgetting <= 1.0f) ||
        (x[0] == 0.0f && x[1] == 0.0f)) { return false; }
    next = *state;
    for (i = 0; i < 2; ++i)
    {
        for (j = 0; j < 2; ++j) { prior[i][j] = state->covariance[i][j] / forgetting; }
        gain[i] = prior[i][0] * x[0] + prior[i][1] * x[1];
    }
    denominator = 1.0f + x[0] * gain[0] + x[1] * gain[1];
    if (!(denominator > 0.0f && denominator <= FLT_MAX)) { return false; }
    next.innovation = y - x[0] * state->theta[0] - x[1] * state->theta[1];
    for (i = 0; i < 2; ++i)
    {
        gain[i] /= denominator;
        next.theta[i] += gain[i] * next.innovation;
        if (!finite_value(next.theta[i])) { return false; }
        for (j = 0; j < 2; ++j) { transform[i][j] = (i == j ? 1.0f : 0.0f) - gain[i] * x[j]; }
    }
    // Joseph 形式避免较大初始协方差发生相减消精度。
    for (i = 0; i < 2; ++i)
        for (j = 0; j < 2; ++j)
        {
            temp[i][j] = 0.0f;
            for (k = 0; k < 2; ++k) { temp[i][j] += transform[i][k] * prior[k][j]; }
        }
    for (i = 0; i < 2; ++i)
        for (j = 0; j < 2; ++j)
        {
            float value = gain[i] * gain[j];
            for (k = 0; k < 2; ++k) { value += temp[i][k] * transform[j][k]; }
            if (!finite_value(value)) { return false; }
            next.covariance[i][j] = value;
        }
    next.covariance[0][1] = next.covariance[1][0] =
        0.5f * (next.covariance[0][1] + next.covariance[1][0]);
    for (i = 0; i < 2; ++i)
    {
        float value = next.covariance[i][i];
        if (value < RLS_COVARIANCE_MIN) { value = RLS_COVARIANCE_MIN; }
        if (value > RLS_COVARIANCE_MAX) { value = RLS_COVARIANCE_MAX; }
        next.covariance[i][i] = value;
    }
    // 限幅后的互协方差仍须满足半正定条件。
    {
        float product = next.covariance[0][0] * next.covariance[1][1];
        float cross = next.covariance[0][1];
        if (!(cross * cross <= product)) { return false; }
    }
    next.updates++;
    *state = next;
    return true;
}

const Rls2Algorithm rls2_algorithm = {.ops = {.init = Rls2_Init, .update = Rls2_Update}};
