
#include "./KalmanFilter.h"

float KalmanFilter::Yiorderfilter(float est_angle ,float me_angle, float m_gyro, float dt, float K1)
{
    est_angle = K1 * me_angle + (1 - K1) * (est_angle + m_gyro * dt);
    return est_angle;
}

void KalmanFilter::Kalman_Filter(double m_angle, double m_gyro, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0)
{
    angle += (m_gyro - q_bias) * dt;
    angle_err = m_angle - angle;
    Pdot[0]   = Q_angle - P[0][1] - P[1][0];
    Pdot[1]   = -P[1][1];
    Pdot[2]   = -P[1][1];
    Pdot[3]   = Q_gyro;
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E     = R_angle + C_0 * PCt_0;
    K_0   = PCt_0 / E;
    K_1   = PCt_1 / E;
    t_0   = PCt_0;
    t_1   = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    angle += K_0 * angle_err;
    q_bias += K_1 * angle_err;
    angle_dot = m_gyro - q_bias;
}

void KalmanFilter::Angletest(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, float dt, float K1)
{
    // 卡尔曼滤波相关参数
    float Q_angle = 0.001;
    float Q_gyro  = 0.005;
    float R_angle = 0.5;
    float C_0     = 1;

    // 角度计算公式，rad 1 = 57.3度,m_angle 为测量俯仰角
    m_angle = atan2(ay, az) * 57.3 + 1.6;


    //卡尔曼滤波得到实时俯仰角，查看卡尔曼函数，最终俯仰角记为angle，使用前一时刻的角速度
    Kalman_Filter(m_angle, Gyro_x, dt, Q_angle, Q_gyro, R_angle, C_0);
    // angle = Yiorderfilter(angle,m_angle, Gyro_x, dt, K1);

    //角度转换,32768对应+250度,由此计算比例为131倍，这里减128.1的目的可能是为了减去初始的误差 ，Gyro_x为俯仰角速度 单位为 度/秒
    Gyro_x = (gx) / 131;  // 更新当前角速度

    // m_angle6 为测量的翻滚角
    m_angle6 = atan2(ax, az) * 57.3 - 4.8;

    //使用前一时刻的角速度
    angle6         = Yiorderfilter(angle6, m_angle6, Gyro_y, dt, K1);
    Gyro_y         = -gy / 131.0;   //翻滚角速度更新
}
