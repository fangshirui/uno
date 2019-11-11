/********
FangShiRui 修改于2019.3
*********/

#ifndef BalanceCar_h // 防止二次调用这个文件
#define BalanceCar_h

/////////////////添加根本库函数//////////////////////////////////////////////////
#include <Arduino.h>
///////////////////////////////////////////////////////////////////////////////

class BalanceCar {
public:
    void speedpiout(double kps, double kis, double kds, double p0);
    void pwma(float angle, float angle5, int Pin1, int Pin2, int Pin3, int Pin4, int PinPWMA, int PinPWMB);

    int pulseright = 0;
    int pulseleft = 0;
    double angleoutput = 0, speedoutput = 0;
    double pwm1 = 0, pwm2 = 0;

private:
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
