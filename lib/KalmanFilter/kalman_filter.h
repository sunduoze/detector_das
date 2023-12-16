#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilterImpl;

class KalmanFilter
{
public:
    KalmanFilter(float initial_x, float initial_P, float process_noise, float measurement_noise);
    ~KalmanFilter(); // 析构函数

    void predict();                 // 预测步骤
    void update(float measurement); // 更新步骤
    float get_val(void);            //

private:
    KalmanFilterImpl *impl; // 指向实现的指针
};

#endif // KALMAN_FILTER_H
