
//FangShiRui 修改于2019.3*

#include "BalanceCar.h"

void BalanceCar::speedpiout(double kps, double kis, double kds, double p0)
{
    double speeds = (pulseleft + pulseright) * 1.0;
    pulseright = pulseleft = 0; //每次进行速度pi计算，都会给脉冲数置零，也即每隔40ms

    //* 一阶互补滤波
    speeds_filterold *= 0.7;
    double speeds_filter = speeds_filterold + speeds * 0.3;
    speeds_filterold = speeds_filter;

    // 位置量即所有周期（40ms）速度量的累加
    positions += speeds_filter;
    // 未知量幅值限制
    positions = constrain(positions, -3550, 3550);

    speedoutput = kis * (p0 - positions) + kps * (p0 - speeds_filter);
}

void BalanceCar::pwma(float angle, float angle6, int Pin1, int Pin2, int Pin3, int Pin4, int PinPWMA, int PinPWMB)
{

    pwm1 = -angleoutput - speedoutput; //左侧电机控制量
    pwm2 = -angleoutput - speedoutput; //右侧电机控制量

    //设置幅值限制
    if (pwm1 > 255)
        pwm1 = 255;
    if (pwm1 < -255)
        pwm1 = -255;
    if (pwm2 > 255)
        pwm2 = 255;
    if (pwm2 < -255)
        pwm2 = -255;

    //设置停转俯仰角和停转翻转角
    if (angle > 30 || angle < -30 || angle6 < -10 || angle6 > 10) {
        pwm1 = 0;
        pwm2 = 0;
    }

    if (pwm1 >= 0) {
        digitalWrite(Pin2, 0);
        digitalWrite(Pin1, 1);
        analogWrite(PinPWMA, pwm1);
    } else {
        digitalWrite(Pin2, 1);
        digitalWrite(Pin1, 0);
        analogWrite(PinPWMA, -pwm1);
    }

    if (pwm2 >= 0) {
        digitalWrite(Pin4, 0);
        digitalWrite(Pin3, 1);
        analogWrite(PinPWMB, pwm2);
    } else {
        digitalWrite(Pin4, 1);
        digitalWrite(Pin3, 0);
        analogWrite(PinPWMB, -pwm2);
    }
}