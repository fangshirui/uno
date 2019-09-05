/********
FangShiRui 修改于2019.3
*********/


#include "./KalmanFilter.h"

//////////////////////////yijielvbo////////////////////
void KalmanFilter::Yiorderfilter(float angle_m, float gyro_m,float dt,float K1)
{
  angle6 = K1 * angle_m + (1 - K1) * (angle6 + gyro_m * dt);    // angle_m这里是翻滚角
 // return angle6;
}


////////////////////////kalman/////////////////////////

void KalmanFilter::Kalman_Filter(double angle_m, double gyro_m,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0)
{
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err; 
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias; 
}

////////////////////////kalman/////////////////////////


///////////////////////////// Angle test/////////////////////////////////
void KalmanFilter::Angletest(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,
									float R_angle,float C_0,float K1)
{
  float Angle = atan2(ay , az) * 57.3;           // 角度计算公式，rad 1 = 57.3度,Angle为实时俯仰倾角
  // Gyro_x = (gx - 128.1) / 131;              //角度转换,32768对应+250度,由此计算比例为131倍，这里减128.1的目的可能是为了减去初始的误差 ，Gyro_x为俯仰角速度
  Gyro_x = gx / 131;              //角度转换,32768对应+250度,由此计算比例为131倍，这里减128.1的目的可能是为了减去初始的误差 ，Gyro_x为俯仰角速度
  Kalman_Filter(Angle, Gyro_x, dt, Q_angle, Q_gyro,R_angle,C_0);            //卡尔曼滤波得到实时俯仰角，查看卡尔曼函数，最终俯仰角记为angle

  // if (gz > 32768) gz -= 65536;              //强制转换2g-1g    不知道其根本作用
  Gyro_z = -gz / 131;                      //z轴参数转换            将z轴旋转参数化为单位度每秒
  // accelz = az / 16.4;                         // 不知作用，且好像写错了，应该是除以1670左右的数，得到加速度标准单位m/s^2
  // accelz = az / 1670;                         // 不知作用，且好像写错了，应该是除以1670左右的数，得到加速度标准单位m/s^2

  float angleAx = atan2(ax, az) * 180 / PI; //计算翻滚角
   Gyro_y = -gy / 131.00;                        // 计算翻滚角速度
  Yiorderfilter(angleAx, Gyro_y, dt, K1); //一阶互补滤波，得到翻滚角，查看一阶滤波函数，最终翻滚角记为angle6

}
