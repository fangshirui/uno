
//FangShiRui 修改于2019.3*

#include "BalanceCar.h"

void BalanceCar::pwma(float angle, float angle6, int Pin1, int Pin2, int Pin3, int Pin4, int PinPWMA, int PinPWMB)
{

    pwm1 = -angleoutput; //左侧电机控制量
    pwm2 = -angleoutput; //右侧电机控制量

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
    if (angle > 30 || angle < -30 || angle6 < -10 || angle6 > 10)
    {
        pwm1 = 0;
        pwm2 = 0;
    }

    if (pwm1 >= 0)
    {
        digitalWrite(Pin2, 0);
        digitalWrite(Pin1, 1);
        analogWrite(PinPWMA, pwm1);
    }
    else
    {
        digitalWrite(Pin2, 1);
        digitalWrite(Pin1, 0);
        analogWrite(PinPWMA, -pwm1);
    }

    if (pwm2 >= 0)
    {
        digitalWrite(Pin4, 0);
        digitalWrite(Pin3, 1);
        analogWrite(PinPWMB, pwm2);
    }
    else
    {
        digitalWrite(Pin4, 1);
        digitalWrite(Pin3, 0);
        analogWrite(PinPWMB, -pwm2);
    }
}
