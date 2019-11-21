
#ifndef KalmanFilter_h
#define KalmanFilter_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <Kalman.h>

class KalmanFilter : private Kalman {
public:
    double angle1, angle6;      // 俯仰角，翻转角 单位 度
    double gyro_x, gyro_y;      // 俯仰角速度，翻转角速度 单位 度/s
    double m_angle, m_angle6;   // 俯仰角原始数据，翻转角原始数据
    KalmanFilter();
    // 更新数据
    void update(double ax, double ay, double az, double gx, double gy, double gz);

private:
    double dt;   //中断间隔时间
    double k1;   // 一阶滤波的测量值置信度
};
#endif
