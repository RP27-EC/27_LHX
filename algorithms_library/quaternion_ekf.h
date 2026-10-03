#ifndef QUATERNION_EKF_H
#define QUATERNION_EKF_H

#include <stdbool.h>
#include <stdint.h>

// 六状态姿态 EKF：四元数及 X、Y 轴零偏，参照模板 Wang Hongxi 的 QuaternionEKF。
typedef struct
{
    float quaternion_noise; // 四元数过程噪声，越大越容易接受加速度修正。
    float bias_noise; // 零偏过程噪声，决定零偏估计的变化速度。
    float accel_noise; // 归一化加速度量测噪声，越大越少依赖加速度。
    float fading; // 零偏协方差遗忘系数，取正值且不超过一。
    float chi_square_threshold; // 加速度残差门槛，收敛后用于拒绝运动干扰。
} QuaternionEkfConfig;

typedef struct
{
    float q[4]; // 单位四元数，w、x、y、z。
    float bias[3]; // 在线估计的剩余零偏，rad/s；Z 轴不可观测。
    float gyro[3]; // 本周期去除在线零偏后的角速度，rad/s。
    float covariance[36]; // 四元数与零偏的协方差。
    float chi_square; // 当前加速度残差检验值。
    uint32_t reject_count; // 连续稳定状态下的量测拒绝次数。
    bool converged; // 残差是否已进入收敛范围。
    bool accel_used; // 本周期是否接受加速度修正。

    // 矩阵工作区随实例保存，避免占用控制任务栈或动态申请内存。
    float f[36], predicted_p[36], temp[36];
    float h[18], pht[18], gain[18];
    float s[9], inverse_s[9];
    float predicted_x[6], correction[6];
} QuaternionEkf;

void QuaternionEkf_Init(QuaternionEkf *filter, const float quaternion[4]);
// gyro：rad/s；accel：m/s²；dt：s。无有效加速度时只做陀螺预测。
bool QuaternionEkf_Update(QuaternionEkf *filter, const QuaternionEkfConfig *config,
                          const float gyro[3], const float accel[3], float dt);

#endif // QUATERNION_EKF_H
