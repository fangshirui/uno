#include "KalmanFilter.h"

// 使用默认的基类构造函数
KalmanFilter::KalmanFilter()  
{
    angle1=angle6=gyro_x=gyro_y=0;
    m_angle = m_angle6 = 0;
    dt = 0.005;
}

void KalmanFilter::update(double ax,double ay, double az,double gx,double gy, double gz)
{
    m_angle = atan2(ay, az) * 57.3 + 1.2; //加速度计计算倾角原始数据
    gyro_x = gx / 131.0;
    angle1 = getAngle(m_angle,gyro_x,dt);

    m_angle6 = atan2(-ax, az) * 57.3 + 2.7;
    gyro_y= gy / 131.0; // 角速度换算
    double k1 = 0.05;
    angle6 = m_angle6 * k1 + (1 - k1) * (angle6 + gyro_y * dt); 
  

}

