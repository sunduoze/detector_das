#include "kalman_filter.h"
// // 初始化Kalman Filter
// void kalman_filter_init(KalmanFilter *kf, float initial_x, float initial_P, float process_noise, float measurement_noise)
// {
//     kf->x = initial_x;
//     kf->P = initial_P;
//     kf->Q = process_noise;
//     kf->R = measurement_noise;
// }

// // 卡尔曼滤波预测步骤
// void kalman_predict(KalmanFilter *kf)
// {
//     // 预测状态
//     kf->x = kf->x;
//     // 预测误差协方差
//     kf->P = kf->P + kf->Q;
// }

// // 卡尔曼滤波更新步骤
// void kalman_update(KalmanFilter *kf, float measurement)
// {
//     // 计算卡尔曼增益
//     kf->K = kf->P / (kf->P + kf->R);

//     // 更新状态估计
//     kf->x = kf->x + kf->K * (measurement - kf->x);

//     // 更新状态估计误差协方差
//     kf->P = (1 - kf->K) * kf->P;
// }
