#include "kalman_filter.h"

// 私有实现类
class KalmanFilterImpl
{
public:
    float x; // 状态估计
    float P; // 误差协方差
    float Q; // 过程噪声协方差
    float R; // 测量噪声协方差
    float K; // 卡尔曼增益
};

// 公共接口实现
KalmanFilter::KalmanFilter(float initial_x, float initial_P, float process_noise, float measurement_noise)
    : impl(new KalmanFilterImpl{initial_x, initial_P, process_noise, measurement_noise}) {}

KalmanFilter::~KalmanFilter()
{
    delete impl;
}

void KalmanFilter::predict()
{
    // 预测状态
    impl->x = impl->x;
    // 预测误差协方差
    impl->P = impl->P + impl->Q;
}

void KalmanFilter::update(float measurement)
{
    // 计算卡尔曼增益
    impl->K = impl->P / (impl->P + impl->R);

    // 更新状态估计
    impl->x = impl->x + impl->K * (measurement - impl->x);

    // 更新状态估计误差协方差
    impl->P = (1 - impl->K) * impl->P;
}

float KalmanFilter::get_val(void)
{
    return impl->x;
}