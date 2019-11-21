
#ifndef BalanceCar_h   // 防止二次调用这个文件
#define BalanceCar_h

#include <Arduino.h>

class BalanceCar {
public:
    void get_angleout(double kp, double ks, double kd, float angle, float gyro_x);
    void get_speedout(double kps, double kis, double kds, double p0 = 0);
    void get_pwm(float angle1, float angle6, int Pin1, int Pin2, int Pin3, int Pin4, int PinPWMA, int PinPWMB);
    void go_forward();
    void go_back();

    int pulseright
        = 0;                       // 40ms周期统计的右侧旋转脉冲数量
    int    pulseleft = 0;          // 40ms周期统计的左侧旋转脉冲数量
    double pwm_r = 0, pwm_l = 0;   // 右侧pwm(+-255)和 左侧pwm(+-255) 输出,最后应用要转成(0-255)

private:
    double angleoutput = 0;   // 角度环输出
    double speedoutput = 0;   // 速度环输出

    float          speeds_filterold;   // 速度滤波
    volatile float positions;          // 位置
};
#endif
