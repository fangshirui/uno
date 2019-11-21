#include "BalanceCar.h"
#include "KalmanFilter.h"
#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <MsTimer2.h>       //定时中断库
#include <PinChangeInt.h>   //外部中断拓展库
#include <Wire.h>

//TB6612FNG驱动模块控制信号端口
#define R_IN1 7    //右电机控制端口 IN1
#define R_IN2 6    //右电机控制端口 IN2
#define L_IN1 13   //左电机控制端口 IN1
#define L_IN2 12   //左电机控制端口 IN2, 注意在驱动板上已经默认了两个电机是反向的，也即左右电机均接收 in1 0 in2 1 时，两个电机是同方向转的！
#define R_PWM 9    //右电机PWM端口
#define L_PWM 10   //左电机PWM端口
#define STBY 8     //standby 端口，置零则电机停止

// 码盘中断端口号
#define R_INTR 2   // 自带的外部中断0 ，优先级很高，端口在D2
#define L_INTR 4   // 用户设定外部中断， 端口在D4

MPU6050      mpu;          //实例化一个 MPU6050对象，对象名称为 mpu
BalanceCar   balancecar;   // 实例化一个名为balancecar 的对象
KalmanFilter kalmanfilter;
int16_t      ax, ay, az;   // x,y,z方向的加速度
int16_t      gx, gy, gz;   // x,y,z方向的角加速度，

//* pid的设定值
double kp = 30, ki = 0.0, kd = 0.58;                        //平衡pid值，可调
double kp_speed = 3.5, ki_speed = 0.1058, kd_speed = 0.0;   // 速度pid值

//* 脉冲计算相关参数
volatile long count_right = 0;   // 右侧脉冲数量，一个周期(5ms)归零，
volatile long count_left  = 0;   // 左侧脉冲数量，一个周期(5ms)归零，
int           rpluse      = 0;   // 带正负号的当前周期右侧脉冲数量
int           lpluse      = 0;   // 带正负号的当前周期左侧脉冲数量
int           speedcc     = 0;   // 速度环时间周期计数量，累积到8 则40ms周期

//* 报告上位机数据
String upload;
String str_angle, str_angle6, str_m_angle, str_m_angle6;

void countpulse();

//* 中断定时 5ms
void inter()
{

    //意义应该是打开所有的中断。理解为中断的初始化。若不用该函数，会影响mpu的数据获取
    sei();

    // 计算脉冲
    countpulse();

    //IIC获取MPU6050六轴数据 ax ay az gx gy gz
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //得到经过卡尔曼滤波的俯仰角angle，和基本的俯仰角角速度
    kalmanfilter.update(ax, ay, az, gx, gy, gz);

    //平衡车角度环计算
    balancecar.get_angleout(kp, ki, kd, kalmanfilter.angle1, kalmanfilter.gyro_x);

    //平衡车速度环计算
    speedcc++;
    if (speedcc >= 8) {
        balancecar.get_speedout(kp_speed, ki_speed, kd_speed);
        speedcc = 0;
    }

    //小车总PWM输出
    balancecar.pwma(kalmanfilter.angle1, kalmanfilter.angle6, R_IN1, R_IN2, L_IN1, L_IN2, R_PWM, L_PWM);
}

void code_left()
{
    count_left++;
}

void code_right()
{
    count_right++;
}

String dToStr(double raw_data)
{
    char char_data[7];
    dtostrf(raw_data, 3, 2, char_data);
    return (String)char_data;
}

void setup()
{
    // TB6612FNGN驱动模块控制信号初始化

    pinMode(R_IN1, OUTPUT);   //控制右侧电机的方向，01为正转，10为反转
    pinMode(R_IN2, OUTPUT);
    pinMode(L_IN1, OUTPUT);   //控制左侧电机的方向，01为正转，10为反转
    pinMode(L_IN2, OUTPUT);
    pinMode(R_PWM, OUTPUT);   //右电机PWM
    pinMode(L_PWM, OUTPUT);   //左电机PWM
    pinMode(STBY, OUTPUT);    //TB6612FNG使能

    //初始化电机驱动模块
    digitalWrite(R_IN1, 0);
    digitalWrite(R_IN2, 1);
    digitalWrite(L_IN1, 1);
    digitalWrite(L_IN2, 0);
    digitalWrite(STBY, 1);
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);

    // 测速码盘的设定
    pinMode(R_INTR, INPUT);
    pinMode(L_INTR, INPUT);

    // 加入I2C总线
    Wire.begin();          //加入 I2C 总线序列
    Serial.begin(38400);   //开启串口，设置波特率为38400
    delay(1500);
    mpu.initialize();   //初始化MPU6050
    delay(50);

    //5ms定时中断设置  使用timer2    注意：使用timer2会对pin3 pin11的PWM输出有影响，因为PWM使用的是定时器控制占空比，所以在使用timer的时候要注意查看对应timer的pin口。
    MsTimer2::set(5, inter);   // 设置定时中断，5ms执行一次inter函数
    MsTimer2::start();         //定时中断开始计时

    // 循环检测及叠加脉冲，测定小车车速
    attachInterrupt(0, code_right, CHANGE);                // 使用自带0号中断,右侧
    attachPinChangeInterrupt(L_INTR, code_left, CHANGE);   // 使用自定义端口的中断，左侧
}

void loop()
{
    str_angle    = dToStr(kalmanfilter.angle1);
    str_angle6   = dToStr(kalmanfilter.angle6);
    str_m_angle  = dToStr(kalmanfilter.m_angle);
    str_m_angle6 = dToStr(kalmanfilter.m_angle6);

    upload = str_angle + "," + str_angle6 + "," + str_m_angle + "," + str_m_angle6;

    Serial.println(upload);
    delay(8);
}

//* 脉冲计算
void countpulse()
{
    lpluse = count_left;
    rpluse = count_right;

    // 将当前周期脉冲数归零，进行下一周期的统计
    count_left  = 0;
    count_right = 0;

    if (balancecar.pwm_r < 0) {
        rpluse = -rpluse;
    }
    if (balancecar.pwm_l < 0) {
        lpluse = -lpluse;
    }

    //每5ms进入中断时，脉冲数叠加
    balancecar.pulseright += rpluse;
    balancecar.pulseleft += lpluse;
}