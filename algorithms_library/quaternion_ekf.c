// 参照模板 bmi_EKF.c（Wang Hongxi，V1.2.0）的六状态模型、残差检验和自适应增益。
// 固定矩阵存储，加入量测无效处理、归一化及协方差数值保护。
#include "quaternion_ekf.h"
#include <math.h>
#include <string.h>

#define EKF_GRAVITY 9.80665f // 地球重力加速度，m/s²。
#define EKF_BIAS_STEP_RAD_S2 0.01f // 收敛后的零偏每秒修正上限。
#define EKF_RECOVER_SAMPLES 50U // 稳定状态下持续拒绝后重新允许收敛。

static bool ekf_finite(float value)
{
    return value >= -3.402823466e38f && value <= 3.402823466e38f;
}

static float ekf_clamp(float value, float limit)
{
    if (value > limit) { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}

static bool ekf_normalize(float q[4])
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    uint32_t i;
    if (!ekf_finite(norm) || norm < 1e-6f) { return false; }
    for (i = 0U; i < 4U; i++) { q[i] /= norm; }
    return true;
}

// 对称正定三阶矩阵的 Cholesky 逆，避免直接行列式求逆的精度损失。
static bool ekf_inverse(const float s[9], float inverse[9])
{
    float l[9] = {0}, y[3], x[3], value;
    int i, j, k, column;
    for (i = 0; i < 3; i++) {
        for (j = 0; j <= i; j++) {
            value = s[i*3+j];
            for (k = 0; k < j; k++) { value -= l[i*3+k]*l[j*3+k]; }
            if (i == j) {
                if (!ekf_finite(value) || value <= 0.0f) { return false; }
                l[i*3+j] = sqrtf(value);
            } else { l[i*3+j] = value/l[j*3+j]; }
        }
    }
    for (column = 0; column < 3; column++) {
        for (i = 0; i < 3; i++) {
            value = i == column ? 1.0f : 0.0f;
            for (k = 0; k < i; k++) { value -= l[i*3+k]*y[k]; }
            y[i] = value/l[i*3+i];
        }
        for (i = 2; i >= 0; i--) {
            value = y[i];
            for (k = i+1; k < 3; k++) { value -= l[k*3+i]*x[k]; }
            x[i] = value/l[i*3+i];
            inverse[i*3+column] = x[i];
        }
    }
    return true;
}

void QuaternionEkf_Init(QuaternionEkf *filter, const float quaternion[4])
{
    uint32_t i;
    if (filter == NULL) { return; }
    memset(filter, 0, sizeof(*filter));
    if (quaternion != NULL) { memcpy(filter->q, quaternion, sizeof(filter->q)); }
    if (!ekf_normalize(filter->q)) {
        memset(filter->q, 0, sizeof(filter->q));
        filter->q[0] = 1.0f;
    }
    // 初始方差沿用模板；四元数未知程度高于已经静止标定过的零偏。
    for (i = 0U; i < 6U; i++) {
        filter->covariance[i*6+i] = i < 4U ? 100000.0f : 100.0f;
    }
}

static void ekf_commit_prediction(QuaternionEkf *filter)
{
    memcpy(filter->q, filter->predicted_x, sizeof(filter->q));
    filter->bias[0] = filter->predicted_x[4];
    filter->bias[1] = filter->predicted_x[5];
    memcpy(filter->covariance, filter->predicted_p, sizeof(filter->covariance));
}

bool QuaternionEkf_Update(QuaternionEkf *e, const QuaternionEkfConfig *c,
                          const float gyro[3], const float accel[3], float dt)
{
    float q0, q1, q2, q3, hx, hy, hz, norm, gyro_norm;
    float residual[3], direction[3];
    float value, scale = 1.0f, diagonal_noise;
    uint32_t i, j, k;
    bool stable;

    if (e == NULL || c == NULL || gyro == NULL || accel == NULL ||
        !(dt > 0.0f && dt <= 0.02f) ||
        !(c->quaternion_noise >= 0.0f && c->quaternion_noise <= 1e8f) ||
        !(c->bias_noise >= 0.0f && c->bias_noise <= 1e8f) ||
        !(c->accel_noise > 0.0f && c->accel_noise <= 1e12f) ||
        !(c->fading > 0.0f && c->fading <= 1.0f) ||
        !(c->chi_square_threshold > 0.0f && c->chi_square_threshold <= 1e8f))
    { return false; }
    for (i = 0U; i < 3U; i++) {
        if (!ekf_finite(gyro[i])) { return false; }
        e->gyro[i] = gyro[i] - e->bias[i];
    }
    e->accel_used = false;
    e->chi_square = 0.0f;
    q0=e->q[0]; q1=e->q[1]; q2=e->q[2]; q3=e->q[3];
    hx=0.5f*e->gyro[0]*dt; hy=0.5f*e->gyro[1]*dt; hz=0.5f*e->gyro[2]*dt;
    e->predicted_x[0] = q0-q1*hx-q2*hy-q3*hz;
    e->predicted_x[1] = q1+q0*hx+q2*hz-q3*hy;
    e->predicted_x[2] = q2+q0*hy-q1*hz+q3*hx;
    e->predicted_x[3] = q3+q0*hz+q1*hy-q2*hx;
    e->predicted_x[4] = e->bias[0];
    e->predicted_x[5] = e->bias[1];
    if (!ekf_normalize(e->predicted_x)) { return false; }

    // F 的四元数部分由角速度给出，末两列是对 X、Y 零偏的偏导。
    memset(e->f, 0, sizeof(e->f));
    for (i=0U; i<6U; i++) { e->f[i*6+i]=1.0f; }
    e->f[1]=-hx; e->f[2]=-hy; e->f[3]=-hz;
    e->f[6]=hx; e->f[8]=hz; e->f[9]=-hy;
    e->f[12]=hy; e->f[13]=-hz; e->f[15]=hx;
    e->f[18]=hz; e->f[19]=hy; e->f[20]=-hx;
    q0=e->predicted_x[0]; q1=e->predicted_x[1];
    q2=e->predicted_x[2]; q3=e->predicted_x[3];
    e->f[4]=q1*dt*0.5f; e->f[5]=q2*dt*0.5f;
    e->f[10]=-q0*dt*0.5f; e->f[11]=q3*dt*0.5f;
    e->f[16]=-q3*dt*0.5f; e->f[17]=-q0*dt*0.5f;
    e->f[22]=q2*dt*0.5f; e->f[23]=-q1*dt*0.5f;
    e->covariance[28]=fminf(e->covariance[28]/c->fading, 10000.0f);
    e->covariance[35]=fminf(e->covariance[35]/c->fading, 10000.0f);
    for (i=0U; i<6U; i++) {
        for (j=0U; j<6U; j++) {
            value=0.0f;
            for (k=0U; k<6U; k++) { value+=e->f[i*6+k]*e->covariance[k*6+j]; }
            e->temp[i*6+j]=value;
        }
    }
    for (i=0U; i<6U; i++) {
        for (j=0U; j<6U; j++) {
            value=0.0f;
            for (k=0U; k<6U; k++) { value+=e->temp[i*6+k]*e->f[j*6+k]; }
            diagonal_noise=i<4U ? c->quaternion_noise : c->bias_noise;
            e->predicted_p[i*6+j]=value+(i==j ? diagonal_noise*dt : 0.0f);
        }
    }
    norm=sqrtf(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    if (!ekf_finite(norm) || norm<0.1f) {
        ekf_commit_prediction(e);
        return true;
    }
    gyro_norm=sqrtf(e->gyro[0]*e->gyro[0]+e->gyro[1]*e->gyro[1]+e->gyro[2]*e->gyro[2]);
    stable=gyro_norm<2.0f && fabsf(norm-EKF_GRAVITY)<0.5f;
    direction[0]=2.0f*(q1*q3-q0*q2);
    direction[1]=2.0f*(q0*q1+q2*q3);
    direction[2]=q0*q0-q1*q1-q2*q2+q3*q3;
    for (i=0U; i<3U; i++) {
        residual[i]=accel[i]/norm-direction[i];
    }
    memset(e->h, 0, sizeof(e->h));
    e->h[0]=-2*q2; e->h[1]=2*q3; e->h[2]=-2*q0; e->h[3]=2*q1;
    e->h[6]=2*q1; e->h[7]=2*q0; e->h[8]=2*q3; e->h[9]=2*q2;
    e->h[12]=2*q0; e->h[13]=-2*q1; e->h[14]=-2*q2; e->h[15]=2*q3;
    // P H'、S = H P H' + R。
    for (i=0U; i<6U; i++) {
        for (j=0U; j<3U; j++) {
            value=0.0f;
            for (k=0U; k<6U; k++) { value+=e->predicted_p[i*6+k]*e->h[j*6+k]; }
            e->pht[i*3+j]=value;
        }
    }
    for (i=0U; i<3U; i++) {
        for (j=0U; j<3U; j++) {
            value=0.0f;
            for (k=0U; k<6U; k++) { value+=e->h[i*6+k]*e->pht[k*3+j]; }
            e->s[i*3+j]=value+(i==j ? c->accel_noise : 0.0f);
        }
    }
    if (!ekf_inverse(e->s, e->inverse_s)) {
        ekf_commit_prediction(e);
        return true;
    }
    for (i=0U; i<3U; i++) {
        for (j=0U; j<3U; j++) {
            e->chi_square+=residual[i]*e->inverse_s[i*3+j]*residual[j];
        }
    }
    if (e->chi_square<0.5f*c->chi_square_threshold) { e->converged=true; }
    if (e->converged && e->chi_square>c->chi_square_threshold) {
        e->reject_count=stable ? e->reject_count+1U : 0U;
        if (e->reject_count<=EKF_RECOVER_SAMPLES) {
            ekf_commit_prediction(e);
            return true;
        }
        e->converged=false;
    } else if (e->converged && e->chi_square>0.1f*c->chi_square_threshold) {
        scale=(c->chi_square_threshold-e->chi_square)/(0.9f*c->chi_square_threshold);
    }
    e->reject_count=0U;
    for (i=0U; i<6U; i++) {
        for (j=0U; j<3U; j++) {
            value=0.0f;
            for (k=0U; k<3U; k++) { value+=e->pht[i*3+k]*e->inverse_s[k*3+j]; }
            // 接近重力轴的零偏可观测性较弱，沿用模板的方向权重。
            if (i>=4U) { value*=acosf(fminf(fabsf(direction[i-4U]),1.0f))/1.570796327f; }
            e->gain[i*3+j]=value*scale;
        }
        value=0.0f;
        for (j=0U; j<3U; j++) { value+=e->gain[i*3+j]*residual[j]; }
        if (i>=4U && e->converged) { value=ekf_clamp(value,EKF_BIAS_STEP_RAD_S2*dt); }
        e->correction[i]=value;
    }
    // Joseph 形式更新 P，缩小自适应增益后仍保持协方差的对称性。
    for (i=0U; i<6U; i++) {
        for (j=0U; j<6U; j++) {
            value=i==j ? 1.0f : 0.0f;
            for (k=0U; k<3U; k++) { value-=e->gain[i*3+k]*e->h[k*6+j]; }
            e->f[i*6+j]=value;
        }
    }
    for (i=0U; i<6U; i++) {
        for (j=0U; j<6U; j++) {
            value=0.0f;
            for (k=0U; k<6U; k++) { value+=e->f[i*6+k]*e->predicted_p[k*6+j]; }
            e->temp[i*6+j]=value;
        }
    }
    for (i=0U; i<6U; i++) {
        for (j=0U; j<6U; j++) {
            value=0.0f;
            for (k=0U; k<6U; k++) { value+=e->temp[i*6+k]*e->f[j*6+k]; }
            for (k=0U; k<3U; k++) { value+=c->accel_noise*e->gain[i*3+k]*e->gain[j*3+k]; }
            e->covariance[i*6+j]=value;
        }
    }
    for (i=0U; i<6U; i++) { e->predicted_x[i]+=e->correction[i]; }
    if (!ekf_normalize(e->predicted_x)) { return false; }
    memcpy(e->q,e->predicted_x,sizeof(e->q));
    e->bias[0]=e->predicted_x[4]; e->bias[1]=e->predicted_x[5];
    e->bias[2]=0.0f; // 重力量测无法估计绕重力轴的零偏。
    e->accel_used=true;
    return true;
}
