
#ifndef BalanceCar_h // 防止二次调用这个文件
#define BalanceCar_h

#include <Arduino.h>

class BalanceCar {
public:
    void get_angleout(double kp, double ks, double kd, float angle, float gyro_x);
    void get_speedout(double kps, double kis, double kds, double p0 = 0);
    void pwma(float angle, float angle5, int Pin1, int Pin2, int Pin3, int Pin4, int PinPWMA, int PinPWMB);

    int pulseright = 0;
    int pulseleft = 0;
    double pwm_r = 0, pwm_l = 0;

private:
    double angleoutput = 0; // 角度环输出
    double speedoutput = 0; // 速度环输出

    float speeds_filterold; // 速度滤波
    float positions; // 位置
    int turnmax = 0;
    int turnmin = 0;
    float turnout = 0;
    int flag1 = 0;
    int flag2 = 0;
    int flag3 = 0;
    int flag4 = 0;
};
#endif
