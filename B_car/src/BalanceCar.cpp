/*
 * @Author: FangShiRui
 * @Date: 2019-08-10 09:11:06
 * @LastEditors: FangShiRui
 * @LastEditTime: 2019-11-22 11:50:12
 * @Description: 平衡车pid 输出 pwm 计算
 */

#include "BalanceCar.h"

void BalanceCar::get_angleout(double kp, double ki, double kd, float angle, float gyro_x)
{
    angleoutput = kp * angle + kd * gyro_x;
}

void BalanceCar::get_speedout(double kps, double kis, double kds, double p0)
{
    double speeds = (pulseleft + pulseright) * 1.0;
    pulseright = pulseleft = 0;   //每次进行速度pi计算，都会给脉冲数置零，也即每隔40ms

    //* 一阶互补滤波
    speeds_filterold *= 0.7;
    double speeds_filter = speeds_filterold + speeds * 0.3;
    speeds_filterold     = speeds_filter;

    // 位置量即所有周期（40ms）速度量的累加
    positions += speeds_filter;

    // 位置量幅值限制
    positions = constrain(positions, -3550, 3550);

    speedoutput = kis * (p0 - positions) + kps * (p0 - speeds_filter);
}

void BalanceCar::get_pwm(float angle, float angle6, int R_IN1, int R_IN2, int L_IN1, int L_IN2, int R_PWM, int L_PWM)
{

    pwm_r = angleoutput - speedoutput;   //右侧电机控制量
    pwm_l = angleoutput - speedoutput;   //左侧电机控制量

    //设置幅值限制
    if (pwm_r > 255)
        pwm_r = 255;
    if (pwm_r < -255)
        pwm_r = -255;
    if (pwm_l > 255)
        pwm_l = 255;
    if (pwm_l < -255)
        pwm_l = -255;

    //设置停转俯仰角和停转翻转角
    if (angle > 30 || angle < -30 || angle6 < -10 || angle6 > 10) {
        pwm_r = 0;
        pwm_l = 0;
    }

    if (pwm_r >= 0) {
        digitalWrite(R_IN2, 0);
        digitalWrite(R_IN1, 1);
        analogWrite(R_PWM, pwm_r);
    } else {
        digitalWrite(R_IN2, 1);
        digitalWrite(R_IN1, 0);
        analogWrite(R_PWM, -pwm_r);
    }

    if (pwm_l >= 0) {
        digitalWrite(L_IN2, 0);
        digitalWrite(L_IN1, 1);
        analogWrite(L_PWM, pwm_l);
    } else {
        digitalWrite(L_IN2, 1);
        digitalWrite(L_IN1, 0);
        analogWrite(L_PWM, -pwm_l);
    }
}

void BalanceCar::go_forward()
{
    positions -= 250;
}

void BalanceCar::go_back()
{
    positions += 250;
}