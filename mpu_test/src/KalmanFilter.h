
#ifndef KalmanFilter_h
#define KalmanFilter_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

class KalmanFilter {
public:
    float Yiorderfilter(float est_angle,float angle_m, float gyro_m, float dt, float K1);
    void  Kalman_Filter(double angle_m, double gyro_m, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0);
    void  Angletest(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, float dt, float K1);
    float Gyro_x = 0;   // 俯仰角 角速度 度/秒
    float Gyro_y = 0;   // 翻滚角 角速度 度/秒
    float angle  = 0;   //卡尔曼滤波得到的最终俯仰角angle，初始猜测为0度
    float angle6 = 0;   //一阶滤波得到的最终翻滚翻滚角angle6，
    float m_angle = 0;
    float m_angle6= 0;

private:
    float angle_err, q_bias;
    float Pdot[4] = { 0, 0, 0, 0 };
    float P[2][2] = { { 1, 0 }, { 0, 1 } };
    float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
    float angle_dot;
};
#endif
